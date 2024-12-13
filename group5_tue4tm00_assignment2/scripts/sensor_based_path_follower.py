#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped
from core_occupancy_grid_costmap import occupancy_grid_costmap
from core_search_path_planner import search_based_path_planning
from core_grad_nav_ctrl import grad_nav_tools
from core_proj_nav_ctrl import proj_nav_tools
from core_path_follow_ctrl import path_follow_tools
from group5_tue4tm00_assignment1 import tools

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

import matplotlib.pyplot as plt 
import matplotlib.patches as patches

import numpy as np
import time 

import numpy as np
from core_proj_nav_ctrl import proj_nav_tools

def gradient(x, p, r):
    # Compute the gradient
    grad_norm = np.linalg.norm(x - p)
    if grad_norm == 0:
        return (x-p)
    else:
        return (x-p)*(1/grad_norm)

def U_repulsive(polygon, center, r):
    # Repulsive field
    nearest_points = proj_nav_tools.local_nearest(polygon, center)
   
    U =  [0.0,0.0]
    U = np.asarray(U)
    for point in nearest_points:
        U1=  [0.0,0.0]
        if np.linalg.norm(center-point) <= 2*r:
        
            if np.linalg.norm(center-point) <= (r):
                U1 =  1.4 * gradient(center, point, r)
            else:
                #U1=[2*r,2*r]-(center-point)
                U1 = gradient(center, point, r) * (1.4*np.linalg.norm(center-point)/r+2.8) 
        else:
           U1=[0.0,0.0]
        U=U+U1 # Sum all the repulsive gradients
        if np.linalg.norm(U)>=1.4:
            U=U/np.linalg.norm(U)*1.4
    return U

def polygon_convex_interior_safe(polygon, center, r):
    # Computes the Free Local Space
    polygon = np.asarray(polygon)
    center = np.asarray(center)

    nearest_points = proj_nav_tools.local_nearest(polygon, center)

    convex_interior = polygon
    for point in nearest_points:
        point1=point
        point = safe_point(center, point, r) # Get the new point
        if np.linalg.norm(center-point) <= r:
            center1 =safe_point1(center, point1, r)
            convex_interior = proj_nav_tools.polygon_intersect_halfplane(convex_interior, point, center1-point)
        else:
            convex_interior = proj_nav_tools.polygon_intersect_halfplane(convex_interior, point, center-point)

    return convex_interior

def safe_point(x, p, r):
    # Computes the new shifted point  
    grad = gradient(x,p,r)
    d = grad*r+p
    return d

def safe_point1(x, p, r):
    # Finds a point that is 2*r distant from the border in the direction for the gradient (x-p)
    grad = gradient(x,p,r)
    d = grad*2*r+p
    return d

def path_goal_support_corridor_safe(path, center, boundary, r):

    path = np.asarray(path)
    if path.ndim < 2:
        path = path.reshape((1,-1))

    center = np.asarray(center).reshape((-1, path.shape[1]))
    boundary = np.asarray(boundary).reshape((-1, path.shape[1]))

    xline_start = path[:-1]
    xline_end = path[1:]
    for point in boundary:
        point1=point
        point = safe_point(center, point, r) # Get the new point
        if np.linalg.norm(center-point) <= r:
            center1 = safe_point1(center, point1, r)
            xline_start, xline_end = path_follow_tools.line_intersect_halfplane(xline_start, xline_end, point, center1-point)
        else:
            xline_start, xline_end = path_follow_tools.line_intersect_halfplane(xline_start, xline_end, point, center-point)

    if xline_end.shape[0] > 0:
        goal = xline_end[-1]
    else:
        goal = None

    return goal

class SafePathFollower(Node):
    
    def __init__(self):
        super().__init__('safe_path_follower', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # If needed in your design, define your node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position
        self.scan_pose_x = 0.0 # scan x-position
        self.scan_pose_y = 0.0 # scan y-position
        self.scan_pose_a = 0.0 # scan yaw angle
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.scan_polygon = np.zeros((2,2)) # Scan polygon vertices
        self.path = np.zeros((0,2)) # Path
        self.path_goal = np.zeros((1,2))
        self.r = 0.2
        
        # Node parameters
        self.rate = 10.0 
        rate = self.get_parameter('rate').value
        self.rate = rate if rate is not None else self.rate

        self.figure_options = {'figwidth': 4.0, 'figheight': 4.0} # Refer to **kwargs of matplotlib.figure.Figure
        self.axes_options = {'aspect': 'equal', 'xlim': (-3.0,3.0), 'ylim':(-3.0, 3.0)} # Refer to **kwargs of matplotlib.axes.Axes
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

        
        # If needed in your design, create a subscriber to the pose topic
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
        self.path_subscriber = self.create_subscription(Path, 'path', self.path_callback, qos_profile=path_qos_profile)
        self.path_msg = Path()

        # Create a publisher for the cmd_vel topic of message type geometry_msgs.msg.Twist
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.cmd_vel = Twist()

        # If needed in your design, create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Start visualization
        self.plot_start()

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
        path_points = []
        for pose_stamped in path_msg.poses:
            path_points.append([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
        self.path = np.asarray(path_points)

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
        self.path_plot, = self.ax.plot([], [], **self.path_plot_options)
        self.path_goal_plot, = self.ax.plot([], [], **self.path_goal_plot_options)
        self.scan_plot, = self.ax.plot([], [], **self.plot_options)
        self.nearest_scan_scatter = self.ax.scatter([],[], **self.scatter_options)
        self.quiver = self.ax.quiver([0,0], [0,0], **self.quiver_options)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def plot_update(self):
        # Update figure

        self.nearest_points = proj_nav_tools.local_nearest(self.scan_points, [self.scan_pose_x, self.scan_pose_y])
        self.convex_interior = polygon_convex_interior_safe(self.scan_polygon, [self.scan_pose_x, self.scan_pose_y], self.r)
        self.path_goal = path_goal_support_corridor_safe(self.path, [self.scan_pose_x, self.scan_pose_y], self.nearest_points, self.r)

        if self.path_goal is not None:
            self.path_goal_plot.set_data(self.path_goal[0], self.path_goal[1])
        else:
            self.path_goal_plot.set_data([], [])

        self.scan_plot.set_data(self.scan_points[:,0], self.scan_points[:,1])
        self.path_plot.set_data(self.path[:,0], self.path[:,1])
        self.scan_patch.set_xy(self.scan_polygon)
        self.corridor_patch.set_xy(self.convex_interior)
        self.quiver.set_UVC(U=[np.cos(self.scan_pose_a), -np.sin(self.scan_pose_a)], 
                            V=[np.sin(self.scan_pose_a), np.cos(self.scan_pose_a)])
        self.quiver.set(offsets=(self.scan_pose_x,self.scan_pose_y))
        self.nearest_scan_scatter.set_offsets(np.c_[self.nearest_points[:,0], self.nearest_points[:,1]])
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()            

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """        
        # Update the plot
        self.plot_update()

        # Define correctly the parameters and variables
        ctrl_gain = 0.1
        position = np.expand_dims([float(self.pose_x), float(self.pose_y)], axis=0)
        const_ang_vel = 0.0

        # Compute the gradient and scale it by the negative gain
        if self.path_goal is None:
            gradient = np.array([0, 0])
        else:
            gradient = -ctrl_gain*grad_nav_tools.gradient_navigation_potential(position, (self.path_goal).astype(float), self.nearest_points, attractive_strength=1, repulsive_tolerance=0.0, repulsive_threshold_decay=3.0)

        # Transform velocity
        velocity_body = grad_nav_tools.velocity_world_to_body_2D(gradient, self.pose_a)
        
        self.cmd_vel.linear.x = float(velocity_body.flatten()[0])
        self.cmd_vel.linear.y = float(velocity_body.flatten()[1])
        self.cmd_vel.angular.z = const_ang_vel

        # Publish velocity
        self.cmd_vel_pub.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    safe_path_follower_node = SafePathFollower()
    try: 
        rclpy.spin(safe_path_follower_node)
    except KeyboardInterrupt:
        pass 
    # finally:   
    #     navigation_costmap_node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
