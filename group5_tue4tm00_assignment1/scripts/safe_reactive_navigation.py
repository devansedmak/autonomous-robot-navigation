#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from core_proj_nav_ctrl import proj_nav_tools
from core_grad_nav_ctrl import grad_nav_tools
from group5_tue4tm00_assignment1 import tools
import matplotlib.pyplot as plt
import matplotlib.patches as patches

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

import numpy as np
import time 

class SafeReactiveNavigation(Node):
    
    def __init__(self):
        super().__init__('safe_reactive_navigation', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # time.sleep(5.0) # Some Delays
        # Node Properties
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.r = 0.2 # radius
        self.scan_pose_x = 0.0 # scan x-position
        self.scan_pose_y = 0.0 # scan y-position
        self.scan_pose_a = 0.0 # scan yaw angle
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.scan_polygon = np.zeros((2,2)) # Scan polygon vertices
        self.convex_interior = np.zeros((2,2))
        self.goal_proj_x = 0.0 # projected goal x-position
        self.goal_proj_y = 0.0 # projected goal y-position
        self.goal_x = 0.0   # goal x-position
        self.goal_y = 0.0   # goal y-position
        
        # Node parameters
        default_rate = Parameter('rate', Parameter.Type.DOUBLE, 10.0) 
        self.rate = self.get_parameter_or('rate', default_rate).value

        self.figure_options = {'figwidth': 4.0, 'figheight': 4.0} # Refer to **kwargs of matplotlib.figure.Figure
        self.axes_options = {'aspect': 'equal', 'xlim': (-3.0,3.0), 'ylim':(-3.0, 3.0)} # Refer to **kwargs of matplotlib.axes.Axes
        self.grid_options = {'visible': True, 'linewidth': 0.5} # Refer to **kwargs of matplotlib.axes.Axes.grid
        self.plot_options = {'color': 'r', 'marker': 'o', 'linestyle': '', 'markersize': 2.0} # Refer to **kwargs of matplotlib.pyplot.plot
        self.quiver_options = {'scale': 1.0, 'angles': 'xy', 'scale_units': 'xy'} # Refer to **kwargs of matplotlib.pyplot.quiver
        self.patch_options = {'facecolor':'b', 'edgecolor': None, 'alpha': 0.3} # Refer to **kwargs of matplotlib.patches.Patch
        self.scatter_options = {'s': 50, 'color': 'blue'} # # Refer to **kwargs of matplotlib.pyplot.scatter
        self.corridor_patch_options = {'facecolor':'y', 'edgecolor': None, 'alpha': 0.5} # Refer to **kwargs of matplotlib.patches.Patch
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
        corridor_patch_options = self.get_parameters_by_prefix('corridor_patch_options')
        self.corridor_patch_options = {key: param.value for key, param in corridor_patch_options.items()} if corridor_patch_options else self.corridor_patch_options

        # Create a subcriber to the pose topic
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)

        # Create a subcriber to the goal topic
        self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)

        # Create a subcriber to the scan topic
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)

        # Create a publisher for the cmd_vel topic
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_vel = Twist()

        # Create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Start visualization
        self.plot_start()

        # Create a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]


    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
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

    def plot_start(self):
        # Create figure for visualization
        plt.ion()
        self.fig = plt.figure()
        self.fig.set(**self.figure_options)
        self.ax = self.fig.add_subplot(1,1,1)
        self.ax.set(**self.axes_options)
        self.ax.grid(**self.grid_options)

        self.scan_patch = patches.Polygon(self.scan_polygon, **self.patch_options)
        self.ax.add_patch(self.scan_patch)
        self.corridor_patch = patches.Polygon(self.scan_polygon, **self.corridor_patch_options)
        self.ax.add_patch(self.corridor_patch)    
        self.scan_plot, = self.ax.plot([], [], **self.plot_options)
        self.nearest_scan_scatter = self.ax.scatter([],[], **self.scatter_options)
        self.quiver = self.ax.quiver([0,0], [0,0], **self.quiver_options)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def plot_update(self):
        # Update figure

        self.nearest_points = proj_nav_tools.local_nearest(self.scan_points, [self.scan_pose_x, self.scan_pose_y])
        # Compute the Local Free Space
        self.convex_interior = tools.polygon_convex_interior_safe(self.scan_polygon, [self.scan_pose_x, self.scan_pose_y], self.r)

        self.scan_plot.set_data(self.scan_points[:,0], self.scan_points[:,1])
        self.scan_patch.set_xy(self.scan_polygon)
        self.corridor_patch.set_xy(self.convex_interior)
        self.quiver.set_UVC(U=[np.cos(self.scan_pose_a), -np.sin(self.scan_pose_a)], 
                            V=[np.sin(self.scan_pose_a), np.cos(self.scan_pose_a)])
        self.quiver.set(offsets=(self.scan_pose_x,self.scan_pose_y))
        # Plot the goal projection
        self.nearest_scan_scatter.set_offsets(np.c_[self.goal_proj_x, self.goal_proj_y])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        # Project the goal on the Local Free Space boundary
        d, self.goal_proj_x, self.goal_proj_y = proj_nav_tools.point_to_polygon_distance(float(self.goal_x), float(self.goal_y), self.convex_interior[:,0].astype(float), self.convex_interior[:,1].astype(float))

        # Check if the poligon is correctly computed (only a problem when the simulator is started)
        if self.convex_interior.shape[0] >= 3:
            # If the goal is inside the Local Free Space don't use the projection
            if proj_nav_tools.inpolygon(self.convex_interior, [float(self.goal_x), float(self.goal_y)]):
                self.goal_proj_x = self.goal_x
                self.goal_proj_y = self.goal_y
        
        # Update the plot
        self.plot_update()
        
        # Define correctly the parameters and variables
        ctrl_gain = 0.1
        position = np.expand_dims([float(self.pose_x), float(self.pose_y)], axis=0)
        const_ang_vel = 0.0

        # Compute the gradient and scale it by the negative gain
        gradient = -ctrl_gain*grad_nav_tools.gradient_navigation_potential(position, [float(self.goal_proj_x), float(self.goal_proj_y)], self.nearest_points, attractive_strength=1, repulsive_tolerance=0.0, repulsive_threshold_decay=3.0)
        
        # Transform velocity
        velocity_body = grad_nav_tools.velocity_world_to_body_2D(gradient, self.pose_a)
        
        self.cmd_vel.linear.x = float(velocity_body.flatten()[0])
        self.cmd_vel.linear.y = float(velocity_body.flatten()[1])
        self.cmd_vel.angular.z = const_ang_vel

        # Publish velocity
        self.cmd_vel_pub.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    safe_reactive_navigation_node = SafeReactiveNavigation()
    rclpy.spin(safe_reactive_navigation_node)
    safe_reactive_navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
