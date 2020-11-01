#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import numpy as np
from lab8.msg import PlotPoint, SimPose
import math
import sys
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from copy import deepcopy
from loc_utils import Pose, get_max, get_sim_model, is_world_file_valid, is_map_valid, wait_for_service_wrapper
from lab8.srv import ProbDist, ProbDistRequest, ProbDistResponse
from lab8.srv import MapInit, MapInitRequest, MapInitResponse

ODOM = 0
GT = 1
BEL = 2


class BaseRobot():
    """A base class to interact with the real/virtual robot
    """

    def __init__(self):
        print ("Using python version:", sys.version, "\n")

        # ROS Publishers
        self.vel_pub = None
        self.traj_pub = None

        print("Initializing Node")

        rospy.init_node('lab8_robot_controller', anonymous=True)

        self.traj_pub = rospy.Publisher('/traj_points',
                                        PlotPoint,
                                        queue_size=300)


class VirtualRobot(BaseRobot):
    """A class to interact with the virtual robot
    """

    def __init__(self):
        super().__init__()

        if(is_world_file_valid() == False):
            raise Exception("Please fix the world configuration file.")

        self.pose = Pose()

        self.laser_ranges = []

        # Temp variables
        self.temp_pose = Pose()

        print ("Initializing Virtual Robot")

        rospy.Subscriber("/sim_pose",
                         SimPose,
                         self.odom_callback,
                         queue_size=300)

        rospy.Subscriber("/base_scan",
                         LaserScan,
                         self.laser_callback,
                         queue_size=300)

        self.vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist,
                                       queue_size=1)

    def odom_callback(self, data):
        self.temp_pose.x = data.odom_x
        self.temp_pose.y = data.odom_y
        self.temp_pose.a = math.degrees(data.odom_a)
        self.temp_pose.gt_x = data.gt_x
        self.temp_pose.gt_y = data.gt_y
        self.temp_pose.gt_a = math.degrees(data.gt_a)

        self.pose = self.temp_pose

    def set_pose_freq(self, freq):
        self.subscriber_pose_time_delay = 1.0/freq

    def laser_callback(self, data):
        self.laser_range = data.ranges[0]

    def set_vel(self, v, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w

        self.vel_pub.publish(cmd_vel)

    def get_pose(self):
        return (self.pose.x, self.pose.y, self.pose.a)

    def get_gt_pose(self):
        return (self.pose.gt_x, self.pose.gt_y, self.pose.gt_a)

    def get_laser_data(self):
        return self.laser_range

    # Execute the rotation behavior and return the observation data
    def perform_observation_loop(self, observation_count, rot_vel):
        print(" | Executing Observation Loop at:", rot_vel, "deg/s")
        rot_vel = math.radians(rot_vel)
        total_rot_duration = 2*math.pi/rot_vel
        loop_rot_duration = total_rot_duration/observation_count
        angle_before = self.get_gt_pose()[2]

        obs_range_data = np.zeros((observation_count))
        obs_bearing_data = np.zeros((observation_count))
        for i in range(0, len(obs_bearing_data)):
            obs_bearing_data[i] = math.degrees(rot_vel*loop_rot_duration*i)

        self.set_vel(0, rot_vel)
        for i in range(0, observation_count):
            obs_range_data[i] = self.get_laser_data()
            time.sleep(loop_rot_duration)

        self.set_vel(0, 0)

        angle_diff = self.get_gt_pose()[2] - angle_before
        # print("     | Angle diff after obs loop:{:.3f} deg".format(angle_diff))

        return obs_range_data, obs_bearing_data

    # Resets the pose of the virtual robot in the simulator
    def reset(self):

        self.set_vel(0, 0)

        print(" | Resetting Robot pose")
        wait_for_service_wrapper('/reset_positions')
        try:
            reset_positions = rospy.ServiceProxy('/reset_positions', Empty)
            reset_positions()
            time.sleep(1.5)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


class Mapper():
    """A class to perform various mapping-related processing required for grid localization
    """

    def __init__(self, min_x, max_x, min_y, max_y, min_a, max_a,
                 cell_size_x, cell_size_y, cell_size_a,
                 max_cells_x, max_cells_y, max_cells_a,
                 ray_length, lines, obs_per_cell, robot):

        # mapper limits
        self.MIN_X = min_x
        self.MAX_X = max_x
        self.MIN_Y = min_y
        self.MAX_Y = max_y
        self.MIN_A = min_a
        self.MAX_A = max_a

        self.CELL_SIZE_X = cell_size_x
        self.HALF_CELL_SIZE_X = cell_size_x / 2.0
        self.CELL_SIZE_Y = cell_size_y
        self.HALF_CELL_SIZE_Y = cell_size_y / 2.0
        self.CELL_SIZE_A = cell_size_a
        self.HALF_CELL_SIZE_A = cell_size_a / 2.0

        self.MAX_CELLS_X = max_cells_x
        self.MAX_CELLS_Y = max_cells_y
        self.MAX_CELLS_A = max_cells_a

        self.CENTER_CELL_X = int(max_cells_x/2)
        self.CENTER_CELL_Y = int(max_cells_y/2)
        self.CENTER_CELL_A = int(max_cells_a/2)

        # Ray tracing parameters
        self.OBS_PER_CELL = int(obs_per_cell)
        self.RAY_TRACING_ANGLE_INCREMENT = 360/self.OBS_PER_CELL

        # Map Cells
        self.cells = np.zeros((self.MAX_CELLS_X,
                               self.MAX_CELLS_Y,
                               self.MAX_CELLS_A))
        # Map rays for each cell
        self.obs_views = np.zeros((self.MAX_CELLS_X,
                                   self.MAX_CELLS_Y,
                                   self.MAX_CELLS_A,
                                   self.OBS_PER_CELL
                                   ))
        # Ray Intersection points based on the map
        self.obs_points_x = np.zeros((self.MAX_CELLS_X,
                                      self.MAX_CELLS_Y,
                                      self.MAX_CELLS_A,
                                      self.OBS_PER_CELL
                                      ))
        self.obs_points_y = np.zeros((self.MAX_CELLS_X,
                                      self.MAX_CELLS_Y,
                                      self.MAX_CELLS_A,
                                      self.OBS_PER_CELL
                                      ))

        # x, y and a indices in real world coordinates for each cell index
        self.x_values = np.zeros(
            (self.MAX_CELLS_X, self.MAX_CELLS_Y, self.MAX_CELLS_A))
        self.y_values = np.zeros(
            (self.MAX_CELLS_X, self.MAX_CELLS_Y, self.MAX_CELLS_A))
        self.a_values = np.zeros(
            (self.MAX_CELLS_X, self.MAX_CELLS_Y, self.MAX_CELLS_A))

        self.lines = lines

        self.RAY_LENGTH = ray_length

        self.robot = robot

    # Ref: https://stackoverflow.com/questions/2320986/easy-way-to-keeping-angles-between-179-and-180-degrees
    # https://stackoverflow.com/questions/36717163/python-numpy-radians-to-degrees-in-0-360
    def normalize_angle(self, a):
        new_a = a
        while (new_a < -180):
            new_a = new_a + 360
        while (new_a >= 180):
            new_a = new_a - 360
        return new_a

    # Return the continuous world coordinates (x,y,z) [in (m,m,deg)] of the center of the grid cell (cx, cy, cz) 
    def from_map(self, cx, cy, ca):
        x = cx*self.CELL_SIZE_X + self.MIN_X + self.HALF_CELL_SIZE_X
        y = cy*self.CELL_SIZE_Y + self.MIN_Y + self.HALF_CELL_SIZE_Y
        a = ca*self.CELL_SIZE_A + self.MIN_A + self.HALF_CELL_SIZE_A
        return x, y, self.normalize_angle(a)

    # Return the grid cell index (cx,cy,cz) of the point (x, y, a) [in (m,m,deg)] in the continuous world frame
    def to_map(self, x, y, a):
        a = self.normalize_angle(a)
        cx = (x/self.CELL_SIZE_X) + self.CENTER_CELL_X
        cy = (y/self.CELL_SIZE_Y) + self.CENTER_CELL_Y
        ca = (a/self.CELL_SIZE_A) + self.CENTER_CELL_A
        return int(cx), int(cy), int(ca)

    def cross_product_single_multiple(self, v1, v2):
        return v1[0]*v2[:, 1] - v1[1]*v2[:, 0]

    def cross_product_multiple(self, v1, v2):
        return v1[:, 0]*v2[:, 1] - v1[:, 1]*v2[:, 0]

    def cross_product_multiple_single(self, v1, v2):
        return v1[:, 0]*v2[1] - v1[:, 1]*v2[0]

    def backup_cross_product(self, v1, v2):
        return v1[0]*v2[1] - v1[1]*v2[0]

    def get_intersection(self, ray, pose):
        try:
            with np.errstate(divide='ignore'):
                # print "\nTracing ray: \n", ray

                denom = self.cross_product_single_multiple(ray[1] - ray[0],
                                                           self.lines[1] - self.lines[0])

                t = self.cross_product_multiple(self.lines[0] - ray[0],
                                                self.lines[1] - self.lines[0])
                t = t / denom

                # print "\nt: ", t.shape, "\n", t

                u = self.cross_product_multiple_single(self.lines[0] - ray[0],
                                                       ray[1] - ray[0]) / denom

                # print "\n u: \n", u

                t[(t < 0) | (t > 1)] = np.nan
                u[(u < 0) | (u > 1)] = np.nan

                tt = np.copy(t)
                tt[np.isnan(t) | np.isnan(u)] = np.nan

                uu = np.copy(u)
                uu[np.isnan(t) | np.isnan(u)] = np.nan

                intersections_tt = ray[0] + tt[:, np.newaxis]*(ray[1]-ray[0])
                # intersections_uu = self.lines[0] + uu[:, np.newaxis]*(self.lines[1]-self.lines[0])

                # print "\nintersections_tt: ", intersections_tt.shape, "\n", intersections_tt
                # print "\nintersections_uu: ", intersections_uu.shape, "\n", intersections_uu

                distance_intersections_tt = np.hypot(ray[0][1]-intersections_tt[:, 1],
                                                     ray[0][0]-intersections_tt[:, 0])

                # print "\ndistance_intersections: ", distance_intersections.shape, "\n", distance_intersections

                return np.nanmin(distance_intersections_tt), intersections_tt[np.nanargmin(distance_intersections_tt)]
                # return np.nanmin(distance_intersections_tt), intersections_uu[np.nanargmin(distance_intersections_tt)]
        except Exception as ex:
            print ("ERROR -> Pose: \n", pose)

            # ray_start = np.array([pose[0], pose[1]])

            # unit_ray = np.array([1, 0])
            # c, s = np.cos(np.radians(pose[2])), np.sin(np.radians(pose[2]))
            # # R = np.array(((c, -s), (s, c)))
            # R_T = np.array(((c, s), (-s, c)))

            # print "\nR: \n", R_T
            # # print "Unit_ray: \n", unit_ray
            # # print "Unit_ray: \n", unit_ray[:, np.newaxis]
            # unit_ray = unit_ray.dot(R_T)
            # print "\nR*Unit_ray: \n", unit_ray

            # print "\nTracing ray: \n", ray

            # print "\nTracing ray (calc): \n", np.array([ray_start, ray_start+(self.RAY_LENGTH*unit_ray)])

            # print "\nself.lines[1] - self.lines[0]: \n", self.lines[1] - self.lines[0]

            # print "\nDenom: ", denom.shape, "\n", denom

            # # print "\ntt: ", t.shape, "\n", tt
            # # print "\nuu: ", t.shape, "\n", uu
            # print ex

    def get_tracing_rays(self, pose_x, pose_y, pose_angles):
        ray_start = np.array([pose_x, pose_y])
        ray_start = np.repeat(ray_start[:, np.newaxis],
                              pose_angles.shape[0], axis=1)

        unit_ray = np.array([1, 0])
        c, s = np.cos(np.radians(pose_angles)), np.sin(np.radians(pose_angles))
        # R = np.array((c, -s), (s, c))mapper.MAX_CELLS_X -

        R_T = np.array(((c, -s), (s, c)))
        unit_ray = unit_ray.dot(R_T)

        # print "R: \n", R
        # print "Unit_ray: \n", unit_ray
        # unit_ray = R * unit_ray
        # print "R*Unit_ray: \n", unit_ray

        return np.array([ray_start, ray_start+(self.RAY_LENGTH*unit_ray)])

    def populate_views(self):
        print (" | Precaching Views...")
        start_time = time.time()
        for cx in range(0, self.MAX_CELLS_X):
            for cy in range(0, self.MAX_CELLS_Y):
                for ca in range(0, self.MAX_CELLS_A):
                    pose = np.array(self.from_map(cx, cy, ca))

                    # Populate x, y and a values for each cell
                    self.x_values[cx, cy, ca] = pose[0]
                    self.y_values[cx, cy, ca] = pose[1]
                    self.a_values[cx, cy, ca] = pose[2]

                    # Calculate bearings and tracing rays
                    bearings = np.arange(
                        0, 360, self.RAY_TRACING_ANGLE_INCREMENT) + pose[2]

                    # print "Pose: \n", pose
                    tracing_rays = self.get_tracing_rays(pose[0],
                                                         pose[1],
                                                         bearings)

                    # print(tracing_rays)
                    # print(tracing_rays[:,:,0].shape)
                    # print(tracing_rays[:,:,0])

                    # For each tracing ray, find the point of intersection and range
                    view = None
                    point = None
                    for i in range(0, self.OBS_PER_CELL):
                        view, point = self.get_intersection(
                            tracing_rays[:, :, i], pose)
                        self.obs_views[cx, cy, ca, i] = view
                        self.obs_points_x[cx, cy, ca, i] = point[0]
                        self.obs_points_y[cx, cy, ca, i] = point[1]

        print(" | Precaching Time: ", time.time() - start_time)

    def get_views(self, cx, cy, ca):
        return self.obs_views[cx, cy, ca]


class BaseLocalization():
    """A base class to perform grid localization
    """

    def __init__(self, robot, mapper):
        self.robot = robot
        self.mapper = mapper

        self.bel_bar = np.zeros((self.mapper.MAX_CELLS_X,
                                 self.mapper.MAX_CELLS_Y,
                                 self.mapper.MAX_CELLS_A))
        self.bel = np.zeros((self.mapper.MAX_CELLS_X,
                             self.mapper.MAX_CELLS_Y,
                             self.mapper.MAX_CELLS_A))

        # Intialize Pose
        self.init_pose(0, 0, 0)

        # Current data collected robot
        self.obs_range_data = None
        self.obs_bearing_data = None

        # Plotter
        self.plotter = Plotter(self)

        # Noise Parameters
        self.sensor_sigma = 0.11
        self.odom_trans_sigma = 0.33
        self.odom_rot_sigma = 15

    # Initialize Grid Belief
    def init_pose(self, x, y, a):
        self.initial_pose = self.mapper.to_map(x, y, a)
        print("Initial Pose: ", self.initial_pose)

        # Point mass distribution
        print("Initializing belief with a Point mass Distribution at: ",
              self.initial_pose)
        self.bel = np.zeros((self.mapper.MAX_CELLS_X,
                             self.mapper.MAX_CELLS_Y,
                             self.mapper.MAX_CELLS_A))
        self.bel[self.initial_pose] = 1

        # Uniform distribution
        # print("Initializing belief with a Uniform Distribution")
        # self.bel.fill(1 / (self.bel.size))
        # print("Uniform Belief with each cell value: ",
        #       self.bel[self.initial_pose])

    # Gassian Function
    def gaussian(self, x, mu, sigma):
        return np.exp(-np.power(x - mu, 2) / (2*np.power(sigma, 2)))

    # Execute the rotation behavior to measure observations
    def get_observation_data(self, rot_vel=30):
        self.obs_range_data, self.obs_bearing_data = self.robot.perform_observation_loop(self.mapper.OBS_PER_CELL,
                                                                                         rot_vel)

    # Print prior belief statistics (for after prediction step) and plot data in the plotter
    def print_prediction_stats(self, plot_data=True):
        print('\n---------- PREDICTION STATS -----------')
        current_odom = self.robot.get_pose()
        current_gt = self.robot.get_gt_pose()

        gt_index = self.mapper.to_map(*current_gt)
        argmax_bel_bar = get_max(self.bel_bar)
        current_prior_belief = self.mapper.from_map(*argmax_bel_bar[0])
        pos_error = np.array(current_gt) - np.array(current_prior_belief)

        # Print prob as a string to prevent rounding
        print("GT index            : ", gt_index)
        print("Prior Bel index     : ({},{},{}) with prob = {:.4f}".format(
              *argmax_bel_bar[0], argmax_bel_bar[1]))
        print("POS ERROR      : ({:.3f}, {:.3f}, {:.3f})".format(*pos_error))

        # Plot data
        if(plot_data == True):
            self.plotter.plot_point(current_gt[0],
                                    current_gt[1],
                                    GT)
            self.plotter.plot_point(current_odom[0],
                                    current_odom[1],
                                    ODOM)
            self.plotter.visualize_prior_bel()
        print('---------- PREDICTION STATS -----------')
        return pos_error

    # Print belief statistics (for after update step) and plot data in the plotter
    def print_update_stats(self, plot_data=True):
        print('\n---------- UPDATE STATS -----------')
        current_gt = self.robot.get_gt_pose()

        gt_index = self.mapper.to_map(*current_gt)
        argmax_bel = get_max(self.bel)
        current_belief = self.mapper.from_map(*argmax_bel[0])
        pos_error = np.array(current_gt) - np.array(current_belief)

        # Print prob as a string to prevent rounding
        print("GT index      : ", gt_index)
        
        print("Bel index     : ({},{},{}) with prob = {:.4f}".format(
              *argmax_bel[0], argmax_bel[1]))
        print("Bel_bar prob at index = {:.4f}".format(self.bel_bar[argmax_bel[0]]))

        print("\nGT     : ({:.3f}, {:.3f}, {:.3f})".format(*current_gt))
        print("Belief   : ({:.3f}, {:.3f}, {:.3f})".format(*current_belief))
        print("POS ERROR : ({:.3f}, {:.3f}, {:.3f})".format(*pos_error))

        # Plot data
        if(plot_data == True):
            self.plotter.plot_point(current_belief[0], current_belief[1], BEL)

        print('---------- UPDATE STATS -----------')

        return pos_error


class Plotter():
    def __init__(self, loc):
        self.loc = loc

    # Send a point (x,y) to the plotter
    def plot_point(self, x, y, plot_type):
        """Send data to plotter for scatter plot

        Args:
            x (float): x value of Point
            y (float): y value of Point
            plot_type (integer): It may be one of: 
                                    0 (ODOM) for blue color
                                    1 (GT) for green color  
                                    2 (BEL) for cyan color
        """
        self.loc.robot.traj_pub.publish(x, y, plot_type)
        time.sleep(0.01)

    # Reset Plot in plotter
    def reset_plot(self):
        wait_for_service_wrapper('plot_reset', timeout=3)
        try:
            plot_reset_client = rospy.ServiceProxy('plot_reset',
                                                   Empty)
            resp1 = plot_reset_client()
            time.sleep(0.3)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # Clear Map in plotter
    def reset_map(self):
        wait_for_service_wrapper('map_reset', timeout=3)
        try:
            map_reset_client = rospy.ServiceProxy('map_reset', Empty)
            resp1 = map_reset_client()
            time.sleep(0.3)

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # Plot a 2D grid prob distribution with a serialized
    # data of size equal to number of total grid cells
    def plot_prob_dist(self, data):
        wait_for_service_wrapper('plot_prob_dist', timeout=3)
        try:
            plot_prob_dist_client = rospy.ServiceProxy('plot_prob_dist',
                                                       ProbDist)
            resp1 = plot_prob_dist_client(data)
            time.sleep(0.3)
            return resp1.result
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # Visualize the map in the plotter based on
    # the lines member variable of mapper
    def visualize_map(self):
        wait_for_service_wrapper('map_init', timeout=3)
        try:
            map_init_client = rospy.ServiceProxy('map_init', MapInit)
            resp1 = map_init_client(self.loc.mapper.lines[0][:, 0], self.loc.mapper.lines[0][:, 1],
                                    self.loc.mapper.lines[1][:, 0], self.loc.mapper.lines[1][:, 1])
            time.sleep(0.3)
            # return resp1.result
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    # Visualize Prior Bel
    def visualize_prior_bel(self):
        data = np.sum(self.loc.bel_bar, axis=2)
        if(np.sum(data) != 0):
            data = (data/np.sum(data))
        # data = np.flip(data, 1)
        data = data.reshape(self.loc.mapper.MAX_CELLS_X *
                            self.loc.mapper.MAX_CELLS_Y)
        self.plot_prob_dist(data)

    # Visualize Bel
    def visualize_bel(self):
        data = np.sum(self.loc.bel, axis=2)
        data = (data/np.sum(data))
        data = data.reshape(self.loc.mapper.MAX_CELLS_X *
                            self.loc.mapper.MAX_CELLS_Y)
        self.plot_prob_dist(data)


def init_map(robot):
    # Initializer the mapper module
    # Start points for each line segment describing the map
    start_points = np.array([[-1, -1],
                             [1, 0],
                             [-1.5, 1],
                             [-2, 2],
                             [2, 2],
                             [2, -2],
                             [-2, -2]])
    # End points for each line segment describing the map
    end_points = np.array([[0, -1],
                           [0, 1],
                           [-1, 2],
                           [2, 2],
                           [2, -2],
                           [-2, -2],
                           [-2, 2]])

    if(not is_map_valid(start_points, end_points)):
        raise Exception(
            "The definitions of start_points and end_points are not valid. Please make sure the sizes are equal.")

    lines = [start_points, end_points]

    # Units are meters and degrees
    print ("Initializing Mapper")
    mapper = Mapper(min_x=-2, max_x=2, min_y=-2, max_y=2, min_a=-180, max_a=180,
                    cell_size_x=0.2, cell_size_y=0.2, cell_size_a=20,
                    max_cells_x=20, max_cells_y=20, max_cells_a=18,
                    ray_length=6, lines=lines, obs_per_cell=18, robot=robot)

    print(" | Number of observations per grid cell: ", mapper.OBS_PER_CELL)
    mapper.populate_views()

    return mapper
