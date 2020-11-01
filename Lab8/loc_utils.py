import numpy as np
import math
import time
from os.path import expanduser
from rospy import ROSInterruptException, wait_for_service

class Pose():
    """A class to store the odometry and ground truth data
    """

    def __init__(self):
        # Odom values
        self.x = 0
        self.y = 0
        self.a = 0

        # Ground Truth values
        self.gt_x = 0
        self.gt_y = 0
        self.gt_a = 0

# https://stackoverflow.com/questions/5469286/how-to-get-the-index-of-a-maximum-element-in-a-numpy-array-along-one-axis
def get_max(a):
    argmax = (np.unravel_index(a.argmax(), a.shape), a.max())
    # print(argmax)
    return argmax

# Get simulator model configuration for map based on line segments
def get_sim_model(start_point, end_point):
    d = math.hypot(end_point[1]-start_point[1],
                   end_point[0]-start_point[0])
    slope = math.degrees(math.atan2(end_point[1]-start_point[1],
                                    end_point[0]-start_point[0]))
    half_dist_vector = (end_point - start_point)/2.0
    center_point = start_point + half_dist_vector
    print ("model(pose [{0} {1} 0 {2}] size [{3} 0.002 0.5])".format(
        center_point[0], center_point[1], slope, d))

# Check if the world file is valid
def is_world_file_valid():
    search_str = "turtlebot(pose [ 0.000 0.000 0.000 0.000 ])"
    found = False
    with open(expanduser("~") + "/catkin_ws/src/lab8/world/playground.world") as lines:
        for i, line in enumerate(lines):
            line = line.rstrip()  # remove '\n' at end of line
            if search_str == line:
                found = True

    if(found == False):
        print("The Robot is not initialized at pose (0,0,0) in the simulator.")
        print("Please open the file \"world/playground.world\") and replace the line starting with \"turtlebot(...\" with the below line: \n")
        print(search_str)
    
    return found

# Check if map defined by start_points and end_poitns is valid
def is_map_valid(start_points, end_points):
    if (start_points.shape[0] != end_points.shape[0]):
        return False
    
    try:
        if((start_points.shape[1] != 2)  or (end_points.shape[1] != 2)):
            return False
    except:
        return False

    return True

def wait_for_service_wrapper(service_name, timeout=3):
    success = False
    try:
        wait_for_service(service_name, timeout)
    except ROSInterruptException:
        print("Caught rospy shutdown Exception")
        success = True
    

class TestFunctions():
    """Test functions
    """

    @staticmethod
    def plot_views(loc):
        loc.robot.reset()
        loc.plotter.reset_plot()
        pose = loc.robot.get_gt_pose()
        robot_idx = loc.mapper.to_map(*pose)

        a_delta = loc.mapper.RAY_TRACING_ANGLE_INCREMENT

        rx, ry, ra = loc.mapper.from_map(*robot_idx)
        cur_a = ra

        for view in loc.mapper.obs_views[robot_idx]:
            x = rx + view*np.cos(np.radians(cur_a))
            y = ry + view*np.sin(np.radians(cur_a))
            print(x,y)
            loc.plotter.plot_point(x,y,0)
            
            cur_a = cur_a + a_delta
            time.sleep(1)
    
    @staticmethod
    def plot_obs(loc):
        pose = loc.robot.get_gt_pose()
        loc.get_observation_data()

        for obs in range(0, len(loc.obs_range_data) ):
            x = pose[0] + loc.obs_range_data[obs]*np.cos(loc.obs_bearing_data[obs])
            y = pose[1] + loc.obs_range_data[obs]*np.sin(loc.obs_bearing_data[obs])
            print(x,y)
            loc.plotter.plot_point(x,y,1)
            
            time.sleep(1)
            
            

    # Plot observation data from robot
    # TODO: Need to incorporate rotation
    @staticmethod
    def plot_obs_data(robot, mapper, loc, obs_range_data):
        pose = robot.get_gt_pose()
        pose_in_rad = math.radians(pose[2])
        for i in range(0, len(obs_range_data)):
            x = pose[0] + loc.obs_range_data[i] * \
                math.cos(pose_in_rad + loc.obs_bearing_data[i])
            y = pose[1] + loc.obs_range_data[i] * \
                math.sin(pose_in_rad + loc.obs_bearing_data[i])
            mapper.send_to_plot(x, y, 0)
            time.sleep(0.01)

    # Plot views from alls cell in RED
    # This will plot a lot of values,
    # and might make the plotter unresponsive after the first 100K points
    @staticmethod
    def plot_all_views(robot, mapper, loc):
        for cx in range(0, mapper.MAX_CELLS_X):
            for cy in range(0, mapper.MAX_CELLS_Y):
                for ca in range(0, mapper.MAX_CELLS_A):
                    x_s = mapper.obs_points_x[cx, cy, ca]
                    y_s = mapper.obs_points_y[cx, cy, ca]
                    for i in range(0, len(x_s)):
                        mapper.send_to_plot(x_s[i], y_s[i], 0)
                        time.sleep(0.01)

    # Plot observation and sensor data from robot one-by-one
    # TODO: Need to incorporate rotation
    @staticmethod
    def plot_obs_views(robot, mapper, loc, obs_range_data):
        pose = robot.get_gt_pose()
        pose_in_rad = math.radians(pose[2])
        robot_idx = mapper.to_map(*pose)

        x_s = mapper.obs_points_x[robot_idx]
        y_s = mapper.obs_points_y[robot_idx]

        for i in range(0, len(obs_range_data)):
            x = pose[0] + obs_range_data[i] * \
                math.cos(pose_in_rad + loc.obs_bearing_data[i])
            y = pose[1] + obs_range_data[i] * \
                math.sin(pose_in_rad + loc.obs_bearing_data[i])
            mapper.send_to_plot(x, y, 0)
            mapper.send_to_plot(
                x_s[len(obs_range_data)-1-i], y_s[len(obs_range_data)-1-i], 1)
            time.sleep(5)
