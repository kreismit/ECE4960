#!/usr/bin/python3

# Automatically reload changes in python modules
#%load_ext autoreload
#%autoreload 2

# Import classes
#from robot_interface_tim import *
from robot_interface import *

import time
import numpy as np
import rospy
from Traj import Trajectory


# Start points for each line segment describing the map
start_points = np.genfromtxt("StartPoints.csv", delimiter=",")
#start_points = start_points + np.array([-1,-2])

# End points for each line segment describing the map
end_points = np.genfromtxt("EndPoints.csv", delimiter=",")
#end_points = end_points + np.array([-1,-2])

# Check if map described by start_points and end_points is valid
if(not is_map_valid(start_points, end_points)):
    raise Exception("The definitions of start_points and end_points are not valid. Please make sure the number of points are equal.")


import commander # code for interacting with bot over Bluetooth
commander.resetCache() # make sure no odometry data is left over from last time

class RealRobot(BaseRobot):
    """A class to interact with the real robot
    """

    def __init__(self):
        super().__init__()
        print("Initializing Real Robot")

    def get_pose(self):
        """Get the latest odometry pose data in the map frame.
        
        Do NOT change the arguments or return values of this function.
        
        Returns:
            (x, y, a) (float, float, float): A tuple with latest odometry pose in the map frame 
                                             in the format (x, y, a) with units (meters, meters, degrees)

        """
        return commander.returnPose()
    
    def perform_observation_loop(self, observation_count, rot_vel):
        """ Implement a Bluetooth command, that tells your robot to 
        start an anti-clockwise, rotational scan using PID control on 
        the gyroscope. The scan needs to be a full 360 degree rotation with 
        at least 18 readings from the TOF sensor, with the first reading taken 
        at the current heading of the robot. At the end of the scan, 
        have your robot send back the TOF measurements via Bluetooth. 
        
        If you haven't already, write an automated script to pair down your 
        measurements to 18 approximately equally spaced readings such that 
        the first reading was taken at the heading angle of the robot.
        Use a reasonable rotational speed to achieve this behavior.
        
        Do NOT change the arguments or return values of the function since it will 
        break the localization code. This function is called by the member function 
        "get_obseration_data" in the Localization class (robot_interface.py), 
        with observation_count = 18 and rot_vel = 30. 
        You may choose to ignore the values in the arguments.

        Args:
            observation_count (integer): Number of observations to record
            rot_vel (integer): Rotation speed (in degrees/s)

        Returns:
            obs_range_data (ndarray): 1D array of 'float' type containing observation range data
        """
        readings = commander.observationLoop()
        if len(readings)<18:
            print("Warning: lost some observation readings.")
        #else:
        #    readings.insert(0, readings.pop(-1)) # don't need this anymore
        # old algorithm returned readings from 20 to 360 instead of 0 to 340
        return np.array(readings)
    
    def set_vel(self, v, w):
        """Set a linear and an angular velocity for your robot.
        
        You will use this function to move the robot.
        It is not used by the Localization class and so you
        may change the arguments and/or return types as needed.
        
        The move() function already takes care of this. Limitations of
        my Bluetooth implementation make setting velocity separately
        from commander.move() impractical.

        Args:
            v (integer): Linear velocity
            w (integer): Angular velocity
        """
        pass
        
    def get_gt_pose(self):
        # Do not change this function
        """Get the latest ground truth pose data
        
        Do NOT change the arguments or return values of this function.

        Returns:
            (x, y, a) (float, float, float): A tuple with latest ground truth pose 
                                             in the format (x, y, a) with units (meters, meters, degress)

        """
        # Since there is no mechanism to find out the ground truth pose of your real robot,
        # it simply returns the odometry pose.
        # This function exists to comply with the design model of the Localization class
        return self.get_pose()

# Instantiate RealRobot to communicate with the real robot
robot = RealRobot()

# Instantiate Mapper
# Requires a RealRobot object as input
mapper = Mapper(min_x=-0.4, max_x=2, min_y=-0.3, max_y=5, min_a=-180, max_a=180,
                cell_size_x=0.2, cell_size_y=0.2, cell_size_a=20,
                max_cells_x=12, max_cells_y=26, max_cells_a=18,
                ray_length=8, lines=[start_points, end_points], obs_per_cell=18, 
                robot=robot)

# Instantiate Localization
odom_trans_sigma = 0.33
odom_rot_sigma = 15
sensor_sigma = 0.11

# Requires a RealRobot object and a Mapper object as inputs
loc = Localization(robot, mapper, sensor_sigma, odom_trans_sigma, odom_rot_sigma)

# Visualize the map (described using line segments) in the plotter
loc.plotter.visualize_map()

# Peform raycasting and pre-cache the values
mapper.populate_views()

#  Reset the plot, initializes the belief with a uniform distribution, 
# performs the rotation behavior, and runs the update step
def init_bayes_filter():
    # Reset Plots
    loc.plotter.reset_plot()

    # Initiize belief
    loc.init_uniform_distribution()

    # Get Observation Data by executing a 360 degree rotation motion
    loc.get_observation_data()

    # Update Step
    loc.update_step()
    #loc.bel = loc.update_step(loc.bel_bar, loc.obs_range_data)
    loc.print_update_stats(plot_data=True)

# One iteration of the Bayes filter algorithm
def step_bayes_filter(current_odom, prev_odom):
    # Prediction Step
    loc.prediction_step(current_odom, prev_odom)
    #loc.bel_bar = loc.prediction_step(current_odom, prev_odom, loc.bel)
    loc.print_prediction_stats(plot_data=True)

    # Get Observation Data by executing a 360 degree rotation behavior
    loc.get_observation_data()

    # Update Step
    loc.update_step()
    #loc.bel = loc.update_step(loc.bel_bar, loc.obs_range_data)
    loc.print_update_stats(plot_data=True)
    

# Records the odom before a robot motion, 
# moves the robot, and records the odom again after motion
def move_robot(linSpeed, angSpeed, t):
    '''Inputs:  linSpeed (float): linear velocity target; 0 for point turn
                angSpeed (float): angular velocity target; 0 for straight line
                t (float): time to move at this speed
        Outputs: curr_odom (float tuple): x, y, and theta after moving
                prev_odom (float tuple): x, y, and theta before moving
    '''
    prev_odom = robot.get_pose()

    # Code to move your robot goes here
    curr_odom = commander.move(linSpeed, angSpeed, t)
    # the move function both moves the robot and outputs the pose
    #curr_odom = robot.get_pose()
    
    return curr_odom, prev_odom


steps = [[0.5,0,1]] # one step: drive straight and stop

init_bayes_filter()
for i in range(len(steps)):
    currOdom, prevOdom = move_robot(*steps[i])
    step_bayes_filter(currOdom, prevOdom)
