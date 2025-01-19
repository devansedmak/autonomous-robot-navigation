#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import warnings
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
from core_grad_nav_ctrl import grad_nav_tools
from core_proj_nav_ctrl import proj_nav_tools
from group5_tue4tm00_assignment1 import tools
from group5_tue4tm00_assignment2 import tools_2
from core_path_follow_ctrl import path_follow_tools
import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

import numpy as np
import time 
from scipy.spatial import ConvexHull
from group5_tue4tm00_assignment1 import tools


class Visualization(Node):
    def __init__(self):
        super().__init__('visualization')

       # If needed in your design, define your node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position
        self.path_goal = np.zeros((1,2)) # Path goal position
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.scan_polygon = np.zeros((2,2)) # Scan polygon vertices
        self.scan_pose_x = 0.0 # scan x-position
        self.scan_pose_y = 0.0 # scan y-position
        self.scan_pose_a = 0.0 # scan yaw angle
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.scan_polygon = np.zeros((2,2)) # Scan polygon vertices
        self.path = np.zeros((0,2)) # Path
        self.convex_interior = np.zeros((0, 2))  # Interior of the convex hull
        self.points_plot = np.zeros((0, 2)) # Initialize as an empty array
        self.r = 0.22 # Robot radius

        self.real_pose_x = 0.0
        self.real_pose_y = 0.0
        self.real_pose_a = 0.0
        self.pose1_x_list = [] # x-position list
        self.pose1_y_list = [] # y-position list
        self.pose1_a_list = [] # yaw-angle list

        self.number_of_poses1 = 10                
        
        self.check = False # Check for initialization issues
        self.positions = np.empty((0, 2)) # Contains all the positions that the robot takes

        # Node parameter for update rate
        self.rate = 10.0 
        
        self.figure_options = {'figwidth': 8.0, 'figheight': 8.0} # Refer to **kwargs of matplotlib.figure.Figure
        self.axes_options = {'aspect': 'equal', 'xlim': (-14.0,14.0), 'ylim':(-9.5, 9.5)} # Refer to **kwargs of matplotlib.axes.Axes
        self.grid_options = {'visible': True, 'linewidth': 0.5} # Refer to **kwargs of matplotlib.axes.Axes.grid
        self.plot_options = {'color': 'r', 'marker': 'o', 'linestyle': '', 'markersize': 2.0} # Refer to **kwargs of matplotlib.pyplot.plot
        self.quiver_options = {'scale': 1.0, 'angles': 'xy', 'scale_units': 'xy'} # Refer to **kwargs of matplotlib.pyplot.quiver
        self.patch_options = {'facecolor':'b', 'edgecolor': None, 'alpha': 0.3} # Refer to **kwargs of matplotlib.patches.Patch
        self.scatter_options = {'s': 50, 'color': 'blue'} # # Refer to **kwargs of matplotlib.pyplot.scatter
        self.corridor_patch_options = {'facecolor':'y', 'edgecolor': None, 'alpha': 0.5} # Refer to **kwargs of matplotlib.patches.Patch
        self.path_plot_options = {'color': 'b', 'linewidth':1.0} # Refer to **kwargs of matplotlib.pyplot.plot
        self.path_goal_plot_options = {'color': 'cyan', 'marker': 'o', 'linestyle': '', 'markersize': 2.0} # Refer to **kwargs of matplotlib.pyplot.plot
        figure_options = self.get_parameters_by_prefix('figure_options')
        self.figure_options = {key: param.value for key, param in figure_options.items()} if figure_options else self.figure_options
        axes_options = self.get_parameters_by_prefix('axes_options')
        self.axes_options = {key: param.value for key, param in axes_options.items()} if axes_options else self.axes_options  
        grid_options = self.get_parameters_by_prefix('grid_options')
        self.grid_options = {key: param.value for key, param in grid_options.items()} if grid_options else self.grid_options
        plot_options = self.get_parameters_by_prefix('plot_options')
        self.plot_options = {key: param.value for key, param in plot_options.items()} if plot_options else self.plot_options
        quiver_options = self.get_parameters_by_prefix('quiver_options')
        self.quiver_options = {key: param.value for key, param in quiver_options.items()} if quiver_options else self.quiver_options
        patch_options = self.get_parameters_by_prefix('patch_options')
        self.patch_options = {key: param.value for key, param in patch_options.items()} if patch_options else self.patch_options
        scatter_options = self.get_parameters_by_prefix('scatter_options')
        self.scatter_options = {key: param.value for key, param in scatter_options.items()} if scatter_options else self.scatter_options
        corridor_patch_options = self.get_parameters_by_prefix('corridor_patch_options')
        self.corridor_patch_options = {key: param.value for key, param in corridor_patch_options.items()} if corridor_patch_options else self.corridor_patch_options
        path_plot_options = self.get_parameters_by_prefix('path_plot_options')
        self.path_plot_options = {key: param.value for key, param in path_plot_options.items()} if path_plot_options else self.path_plot_options
        path_goal_plot_options = self.get_parameters_by_prefix('path_goal_plot_options')
        self.path_goal_plot_options = {key: param.value for key, param in path_goal_plot_options.items()} if path_goal_plot_options else self.path_goal_plot_options
        self.positions_plot_options = {'color': 'g', 'linewidth':1.0} # Refer to **kwargs of matplotlib.pyplot.plot
        positions_plot_options = self.get_parameters_by_prefix('positions_plot_options')
        self.positions_plot_options = {key: param.value for key, param in positions_plot_options.items()} if positions_plot_options else self.positions_plot_options
        
        self.plot_options1 = {'color': 'r', 'marker': 'o', 'linestyle': '', 'markersize': 2.0} # Refer to **kwargs of matplotlib.pyplot.plot
        plot_options1 = self.get_parameters_by_prefix('plot_options1')
        self.plot_options1 = {key: param.value for key, param in plot_options1.items()} if plot_options1 is not None else self.plot_options1

        self.quiver_options1 = {'scale': 1.0, 'angles': 'xy', 'scale_units': 'xy'} # Refer to **kwargs of matplotlib.pyplot.quiver        
        quiver_options1 = self.get_parameters_by_prefix('quiver_options1')
        self.quiver_options1 = {key: param.value for key, param in quiver_options1.items()} if quiver_options1 else self.quiver_options1

        # If needed in your design, create a subscriber to the pose topic
        self.pose_subscriber = self.create_subscription(PoseStamped, 'odom_pose', self.pose_callback, 1)
        self.pose_msg = PoseStamped()

        # If needed in your design, create a subscriber to the goal topic
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)
        self.goal_msg = PoseStamped()

        # If needed in your design, create a subscriber to the scan topic
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.scan_msg = LaserScan()

        # If needed in your design, create a subscriber to the path topic of message type nav_msgs.msg.Path
        path_qos_profile = QoSProfile(depth=1)
        path_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_subscriber = self.create_subscription(Path, '/turtlebot/path', self.path_callback, qos_profile=path_qos_profile)
        self.path_msg = Path()

        
        # Create a subscriber for the input pose topic of type geometry_msgs.msg.PoseStamped
        pose_in_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.pose_in_subscriber = self.create_subscription(PoseStamped, 'pose_in', self.pose_in_callback, qos_profile=pose_in_qos_profile)
        self.pose_in_subscriber  # prevent unused variable warning
        self.pose_in_msg = PoseStamped()

        # Subscribers
        self.create_subscription(Float32MultiArray, 'convex_interior', self.convex_interior_callback, 10)
        self.create_subscription(Float32MultiArray, 'path_goal', self.path_goal_callback, 10)

        # Start the plot
        self.plot_start()

        # Start the plot update loop
        self.create_timer(1.0/self.rate, self.timer_callback)
    
    def convex_interior_callback(self, msg):
        """
        Callback function for the convex_interior topic, handling messages of type std_msgs.msg.Float32MultiArray
        """
        if msg.data and len(msg.data) % 2 == 0:
            self.convex_interior = np.array(msg.data).reshape(-1, 2)
            if self.convex_interior is not None and len(self.convex_interior) > 2:
                hull = ConvexHull(self.convex_interior)
                self.points_plot=self.convex_interior
                self.convex_interior = self.convex_interior[hull.vertices]
                pose_temp = np.array([self.pose_x, self.pose_y])
                self.convex_interior= tools.trova_intersezione(self.convex_interior, pose_temp)
        else:
            # Set convex_interior to an empty array to avoid using invalid data
            self.convex_interior = np.array([])

    def path_goal_callback(self, msg):
        """
        Callback function for the path_goal topic, handling messages of type std_msgs.msg.Float32MultiArray
        """
        self.path_goal = np.array(msg.data).reshape(-1, 2) if len(msg.data) > 0 else None

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.pose_msg = msg
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
        new_position=np.array([[self.pose_x, self.pose_y]])
        self.positions = np.vstack([self.positions, new_position])

    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.goal_msg = msg
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
    
    def scan_callback(self, scan_msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """        
        self.scan_pose_x = self.pose_x
        self.scan_pose_y = self.pose_y
        self.scan_pose_a = self.pose_a

        scan_ranges = np.array(scan_msg.ranges)
        scan_angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, scan_ranges.size)
        scan_valid = np.logical_not(np.isinf(scan_ranges))
        scan_ranges[~scan_valid] = scan_msg.range_max

        scan_rotation_matrix = np.array([[np.cos(self.scan_pose_a), -np.sin(self.scan_pose_a)],
                                         [np.sin(self.scan_pose_a), np.cos(self.scan_pose_a)]])
        scan_points = np.column_stack((scan_ranges*np.cos(scan_angles), scan_ranges*np.sin(scan_angles)))
        
        scan_points = np.dot(scan_points, np.transpose(scan_rotation_matrix)) + np.array([[self.scan_pose_x, self.scan_pose_y]])

        self.scan_points = scan_points[scan_valid,:]
        self.scan_polygon = scan_points

        self.scan_msg = scan_msg
    
    def path_callback(self, path_msg):
        """
        Callback function for the path topic, handling messages of type nav_msgs.msg.Path
        """
        self.path_msg = path_msg  
        path_points = []
        for pose_stamped in path_msg.poses:
            path_points.append([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
        path = np.asarray(path_points)
        if not np.array_equal(path, self.path):  # Check if the path is different than the previous one
            self.positions = np.empty((0, 2))   # Reset the positions array
            self.positions_plot.set_data([], [])
            self.pose1_x_list = [] # x-position list
            self.pose1_y_list = [] # y-position list
            self.pose1_a_list = []
            self.pose_list_plot1.set_data([], [])
        self.path = np.asarray(path_points)

    def plot_start(self):
        """
        Initialize the plot
        """
        # Create figure for visualization
        plt.ion()
        self.fig = plt.figure()
        self.fig.set(**self.figure_options)
        self.ax = self.fig.add_subplot(1,1,1)
        self.ax.set(**self.axes_options)
        self.ax.grid(**self.grid_options)

        self.pose_list_plot1, = self.ax.plot(self.pose1_x_list, self.pose1_y_list, linestyle='', marker='o', color='r', **self.plot_options1)
        self.pose_scatter = self.ax.scatter([], [], c='red', s=50)  # Add a scatter plot for the robot pose

        self.scan_patch = patches.Polygon(self.scan_polygon, **self.patch_options)
        self.ax.add_patch(self.scan_patch)
        self.corridor_patch = patches.Polygon(self.scan_polygon, **self.corridor_patch_options)
        self.ax.add_patch(self.corridor_patch)    
        self.path_plot, = self.ax.plot([], [], **self.path_plot_options)
        self.path_goal_plot, = self.ax.plot([], [], **self.path_goal_plot_options)
        self.scan_plot, = self.ax.plot([], [], **self.plot_options)
        self.nearest_scan_scatter = self.ax.scatter([],[], **self.scatter_options)
        self.quiver = self.ax.quiver([0,0], [0,0], **self.quiver_options)

        self.positions_plot, = self.ax.plot([], [], **self.positions_plot_options)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def pose_in_callback(self, msg):
        """
        Callback function for the input pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.real_pose_x = msg.pose.position.x
        self.real_pose_y = msg.pose.position.y
        self.real_pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
        self.pose1_x_list.append(self.real_pose_x) 
        self.pose1_y_list.append(self.real_pose_y)
        self.pose1_a_list.append(self.real_pose_a)
    
    def timer_callback(self):
        # Update figure
        if self.path_goal is not None:
            self.path_goal_plot.set_data(self.path_goal[0, 0], self.path_goal[0, 1])
        else:
            self.path_goal_plot.set_data([], [])

        self.pose1_x_list.append(self.real_pose_x)
        self.pose1_y_list.append(self.real_pose_y)
        self.pose1_a_list.append(self.real_pose_a)

        self.pose_list_plot1.set_data(self.pose1_x_list, self.pose1_y_list)
        # Combine x and y positions into a single array
        offsets = np.column_stack((self.pose1_x_list, self.pose1_y_list))
        self.pose_scatter.set_offsets(offsets)
        
        self.scan_plot.set_data(self.scan_points[:,0], self.scan_points[:,1])
        self.path_plot.set_data(self.path[:,0], self.path[:,1])
        
        self.positions_plot.set_data(self.positions[:,0], self.positions[:,1])
        
        self.scan_patch.set_xy(self.scan_polygon)

        if self.convex_interior is not None and len(self.convex_interior) > 0:
            self.corridor_patch.set_xy(self.convex_interior)

        self.quiver.set_UVC(U=[np.cos(self.scan_pose_a), -np.sin(self.scan_pose_a)], 
                            V=[np.sin(self.scan_pose_a), np.cos(self.scan_pose_a)])
        self.quiver.set(offsets=(self.scan_pose_x,self.scan_pose_y))
        self.nearest_scan_scatter.set_offsets(np.c_[self.points_plot[:,0], self.points_plot[:,1]])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    

def main(args=None):
    rclpy.init(args=args)
    node = Visualization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
