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
import numpy as np
# from group5_tue4tm00_assignment1 import tools

import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

def gradient(x, p, r):
    grad_norm = np.linalg.norm(x - p)
    if grad_norm == 0:
        return (x-p)
    else:
        return (x-p)*(1/grad_norm)

def U_repulsive(polygon, center, r):
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
        U=U+U1
        if np.linalg.norm(U)>=1.4:
            U=U/np.linalg.norm(U)*1.4
    return U

def safe_point(x, p, r):
    grad = gradient(x,p,r)
    d = grad*r+p
    return d

def safe_point1(x, p, r):
    grad = gradient(x,p,r)
    d = grad*2*r+p
    return d

def polygon_convex_interior_safe(polygon, center, r):

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

class SafeTwistTeleop2D(Node):
    
    def __init__(self):
        super().__init__('safe_twist_teleop_2D', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
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
        # If needed in your design, get a node parameter for update rate
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

        # If needed in your design, create a subscriber to the input cmd_vel topic
        self.create_subscription(Twist, 'cmd_vel_in', self.cmd_vel_in_callback, 1)

        # If needed in your design, create a subcriber to the scan topic
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)

        # If needed in your design, create a subcriber to the pose topic
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)

        # If needed in your design, create a publisher for the output cmd_vel topic
        self.cmd_vel_out_pub = self.create_publisher(Twist, 'cmd_vel_out', 1)
        self.cmd_vel_out = Twist()

        # If needed in your design, create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Start visualization
        self.plot_start()

        # If need in your design, crease a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

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

    def pose_callback(self, pose_msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.pose_x = pose_msg.pose.position.x
        self.pose_y = pose_msg.pose.position.y
        self.pose_a = euler_from_quaternion([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])[2]
        pass

    def cmd_vel_in_callback(self, msg):
        """
        Callback function for the input cmd_vel topic, handling messages of type geometry_msgs.msg.LaserScan
        """
        
        # For example, a simple direct input-to-output cmd_vel mapping without safety check
        self.nearest_points = proj_nav_tools.local_nearest(self.scan_points, [self.scan_pose_x, self.scan_pose_y])
        #gradient = 0.001 * grad_nav_tools.gradient_navigation_potential_repulsive([self.pose_x, self.pose_y], self.nearest_points)
        gradient = U_repulsive(self.convex_interior, [self.pose_x, self.pose_y], self.r)
        #gradient = gradient*grad_nav_tools.navigation_potential_repulsive([self.pose_x, self.pose_y], self.nearest_points)
        velocity_body = grad_nav_tools.velocity_world_to_body_2D(gradient, self.pose_a)
        
        print(msg.linear.x)
        print(msg.linear.y)
        print(velocity_body[0])
        print(velocity_body[1])

        self.cmd_vel_out.linear.x = velocity_body[0] + msg.linear.x
        self.cmd_vel_out.linear.y = velocity_body[1] + msg.linear.y
        self.cmd_vel_out.angular.z = 0.0
        
        # self.cmd_vel_out = msg
        
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
        self.convex_interior = polygon_convex_interior_safe(self.scan_polygon, [self.scan_pose_x, self.scan_pose_y], self.r)

        self.scan_plot.set_data(self.scan_points[:,0], self.scan_points[:,1])
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
        self.plot_update()
        """
        if (proj_nav_tools.inpolygon(self.convex_interior, [self.scan_pose_x, self.scan_pose_y])):
            '''
            # Set linear velocity (forward)
            self.cmd_vel_out.linear.x = 0.0  # Forward velocity in m/s
            self.cmd_vel_out.linear.y = 0.0
            self.cmd_vel_out.linear.z = 0.0

            # Set angular velocity (rotation)
            self.cmd_vel_out.angular.x = 0.0
            self.cmd_vel_out.angular.y = 0.0
            self.cmd_vel_out.angular.z = 0.0  # Rotation in rad/s
            '''
            print("safe")
        else: print("non safe")
        """
        '''
        # For example, a simple direct input-to-output cmd_vel mapping without safety check
        # self.nearest_points = proj_nav_tools.local_nearest(self.scan_points, [self.scan_pose_x, self.scan_pose_y])
        gradient = grad_nav_tools.gradient_navigation_potential_repulsive([self.pose_x, self.pose_y], self.nearest_points)
        #gradient = gradient*grad_nav_tools.navigation_potential_repulsive([self.pose_x, self.pose_y], self.nearest_points)
        velocity_body = grad_nav_tools.velocity_world_to_body_2D(gradient, self.pose_a)
        
        print(self.cmd_vel_out.linear.x)
        print(self.cmd_vel_out.linear.y)
        print(velocity_body[0])
        print(velocity_body[1])

        self.cmd_vel_out.linear.x = velocity_body[0] + self.cmd_vel_out.linear.x
        self.cmd_vel_out.linear.y = velocity_body[1] + self.cmd_vel_out.linear.y
        self.cmd_vel_out.angular.z = 0.0
        '''
        # For example, publish the output cmd_vel message
        self.cmd_vel_out_pub.publish(self.cmd_vel_out)


def main(args=None):
    rclpy.init(args=args)
    safe_twist_teleop_2d_node = SafeTwistTeleop2D()
    rclpy.spin(safe_twist_teleop_2d_node)
    safe_twist_teleop_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
