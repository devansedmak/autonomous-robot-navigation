#!/usr/bin/env python3
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

class SafePathFollower(Node):
    
    def __init__(self):
        super().__init__('safe_path_follower', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # Node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position
        self.path_goal = np.zeros((1,2))
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.scan_polygon = np.zeros((2,2)) # Scan polygon vertices
        self.scan_pose_x = 0.0 # scan x-position
        self.scan_pose_y = 0.0 # scan y-position
        self.scan_pose_a = 0.0 # scan yaw angle
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.scan_polygon = np.zeros((2,2)) # Scan polygon vertices
        self.path = np.zeros((0,2)) # Path
        
        self.r = 0.22 # Robot radius
        self.lin_gain=1.0 # Linear gain
        lin_gain = self.get_parameter('lin_gain').value
        self.lin_gain = lin_gain if lin_gain is not None else self.lin_gain
        self.ang_gain=2.0 # Angular gain
        ang_gain = self.get_parameter('ang_gain').value
        self.ang_gain = ang_gain if ang_gain is not None else self.ang_gain

        self.check = False # Check for initialization issues
        self.positions = np.empty((0, 2)) # Is an array that contains all the positions that the robot takes

        # Node parameter for update rate
        self.rate = 10.0 
        rate = self.get_parameter('rate').value
        self.rate = rate if rate is not None else self.rate 
        
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
        
        # Create a subscriber to the pose topic
        self.pose_subscriber = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)
        self.pose_msg = PoseStamped()

        # If needed in your design, create a subscriber to the goal topic
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)
        self.goal_msg = PoseStamped()

        # If needed in your design, create a subscriber to the scan topic
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.scan_msg = LaserScan()

        # If needed in your design, create a subscriber to the map topic with the QoS profile of transient_local durability
        map_qos_profile = QoSProfile(depth=1)
        map_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile=map_qos_profile)
        self.map_msg = OccupancyGrid()

        # If needed in your design, create a subscriber to the costmap topic of message type nav_msgs.msg.OccupancyGrid
        costmap_qos_profile = QoSProfile(depth=1)
        costmap_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'costmap', self.costmap_callback, qos_profile=costmap_qos_profile)
        self.costmap_msg = OccupancyGrid()

        # If needed in your design, create a subscriber to the path topic of message type nav_msgs.msg.Path
        path_qos_profile = QoSProfile(depth=1)
        path_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_subscriber = self.create_subscription(Path, '/turtlebot/path', self.path_callback, qos_profile=path_qos_profile)
        self.path_msg = Path()

        # Create a publisher for the cmd_vel topic of message type geometry_msgs.msg.Twist
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_vel = Twist()

        self.convex_interior_pub = self.create_publisher(Float32MultiArray, 'convex_interior', 10)
        self.path_goal_pub = self.create_publisher(Float32MultiArray, 'path_goal', 10)

        # If needed in your design, create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # If need in your design, crease a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.pose_msg = msg
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]

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

    def map_callback(self, msg):
        """
        Callback function for the map topic, handling messages of type nav_msgs.msg.OccupancyGrid
        """
        self.map_msg = msg

    def costmap_callback(self, msg):
        """
        Callback function for the costmap topic, handling messages of type nav_msgs.msg.OccupancyGrid
        """    
        self.costmap_msg = msg  

    def path_callback(self, path_msg):
        """
        Callback function for the path topic, handling messages of type nav_msgs.msg.Path
        """
        self.path_msg = path_msg  
        path_points = []
        for pose_stamped in path_msg.poses:
            path_points.append([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
        self.path = np.asarray(path_points)          
           
    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        pose_temp = np.array([self.pose_x, self.pose_y])
        scan_polygon_temp1= self.scan_polygon
        nearest_points = proj_nav_tools.local_nearest(self.scan_points, pose_temp)
        self.convex_interior = tools.polygon_convex_interior_safe_new(scan_polygon_temp1,nearest_points, pose_temp, self.r)
        self.path_goal = path_follow_tools.path_goal_support_corridor(self.path, pose_temp, self.convex_interior)
        
        # If close to goal or no path, stop the robot
        if self.path is None or self.path_goal is None or len(self.path_msg.poses) == 0 or np.linalg.norm(self.path_goal - pose_temp) < 0.2 :
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.angular.z = 0.0
        else:
            position = np.expand_dims([float(self.pose_x), float(self.pose_y)], axis=0)
            # Compute the gradient of the navigation potential
            gradient = -grad_nav_tools.gradient_navigation_potential_attractive(position, (self.path_goal).astype(float), strength=1.0)
            # Compute the unicycle control inputs
            lin_vel, ang_vel = grad_nav_tools.unicycle_gradient_ctrl_2D(gradient, self.pose_a, lin_gain=self.lin_gain, ang_gain=self.ang_gain)
            
            # Limit the linear and angular velocities
            if np.abs(lin_vel[0]) > 0.26:
                self.cmd_vel.linear.x = 0.26*lin_vel[0]/np.abs(lin_vel[0])
            else:
                self.cmd_vel.linear.x = lin_vel[0]
            self.cmd_vel.linear.y = 0.0
            if np.abs(ang_vel[0]) > 0.5:
                self.cmd_vel.angular.z = 0.5*ang_vel[0]/np.abs(ang_vel[0])
            else:
                self.cmd_vel.angular.z = ang_vel[0]
        
        # Publish the control inputs
        self.cmd_vel_pub.publish(self.cmd_vel)

        # Publish convex interior
        convex_interior_msg = Float32MultiArray(data=self.convex_interior.flatten())
        self.convex_interior_pub.publish(convex_interior_msg)

        # Publish path goal
        path_goal_msg = Float32MultiArray(data=self.path_goal.flatten() if self.path_goal is not None else [])
        self.path_goal_pub.publish(path_goal_msg)

def main(args=None):
    rclpy.init(args=args)
    safe_path_follower_node = SafePathFollower()
    try: 
        rclpy.spin(safe_path_follower_node)
    except KeyboardInterrupt:
        pass 


if __name__ == '__main__':
    main()
