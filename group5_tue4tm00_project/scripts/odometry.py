#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
from core_odometry_tools import odometry
import numpy as np


class Odometry(Node):
    
    def __init__(self):
        super().__init__('odometry', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
         # If needed in your design, define your node parameters
        self.pose_x = None # x-position in meters
        self.pose_y = None # y-position in meters
        self.pose_a = None # yaw angle in radians
        self.lin_vel = 0.0 # linear velocity in m/s
        self.ang_vel = 0.0 # angular velocity in m/s

        self.scan_pose_x = 0.0 # scan x-position
        self.scan_pose_y = 0.0 # scan y-position
        self.scan_pose_a = 0.0 # scan yaw angle
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.scan_polygon = np.zeros((2,2)) # Scan polygon vertices
       
        self.scan_points_old = np.zeros((2,2))
        self.chek_real_position_rrived = False
       
        # Default Parameters
        self.rate = 10.0
        rate = self.get_parameter('rate').value
        self.rate = rate if rate is not None else self.rate

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

        # If needed in your design, create a subscriber for the command velocity topic of type geometry_msgs.msg.Twist
        cmd_vel_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile=cmd_vel_qos_profile)
        self.cmd_vel_subscriber  # prevent unused variable warning

        # If needed in your design, create a subscriber to the scan topic
        scan_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=scan_qos_profile)
        self.scan_subscriber  # prevent unused variable warning
        self.scan_msg = LaserScan()

        # If needed in your design, create a subscriber to the map topic with the QoS profile of transient_local durability
        map_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.map_subscriber = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos_profile=map_qos_profile)
        self.map_msg = OccupancyGrid()

        # If needed in your design, create a publisher for the output pose topic of type geometry_msgs.msg.PoseStamped
        pose_out_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.pose_out_publisher = self.create_publisher(PoseStamped, 'pose_out', qos_profile=pose_out_qos_profile)
        self.pose_out_msg = PoseStamped()

        # Create a timer for the odometry updates
        self.create_timer(1.0/self.rate, self.timer_update)

    def pose_in_callback(self, msg):
        """
        Callback function for the input pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
        self.pose_out_msg = msg
        self.chek_real_position_rrived = True

    def cmd_vel_callback(self, msg):
        """
        Callback function for the input control velocity topic, handling messages of type geometry_msgs.msg.Twist
        """
        #TODO: If needed, use the cmd_vel topic messages in your design
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z
    
    def scan_callback(self, scan_msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        #TODO: If needed, use the scan topic messages in your design 
        #self.scan_msg = msg
        if self.pose_x is None:
            return
        
        self.scan_points_old=self.scan_points

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
        #TODO: If needed, use the map topic messages in your design
        self.map_msg = msg

    def scan_matching(self, P, Q, max_iterations=50, tolerance=1e-6):
        """
        Perform Scan Matching algorithm.
        
        Parameters:
            P: numpy array of shape (n, d), the first set of points
            Q: numpy array of shape (m, d), the second set of points
            max_iterations: int, maximum number of iterations
            tolerance: float, stopping criterion for convergence
            
        Returns:
            R: numpy array of shape (d, d), rotation matrix
            t: numpy array of shape (d,), translation vector
            c: list, correspondence indices from Q to P
        """
        n, d = P.shape
        m, _ = Q.shape
        
        # Initialize rotation R, translation t, and correspondence c
        R = np.eye(d)
        t = np.zeros(d)
        c = np.zeros(n, dtype=int)
        
        for k in range(max_iterations):
            # Step 1: Find closest point correspondences
            for i in range(n):
                distances = np.linalg.norm(P[i] - (Q @ R.T + t), axis=1)
                c[i] = np.argmin(distances)
            
            # Step 2: Compute translation t
            Qc = Q[c]
            t_new = np.mean(P - Qc @ R.T, axis=0)
            
            # Step 3: Compute covariance matrices
            PP = np.zeros((d, d))
            QQ = np.zeros((d, d))
            QP = np.zeros((d, d))
            
            for i in range(n):
                pi = P[i] - t_new
                qi = Qc[i]
                PP += np.outer(pi, pi)
                QQ += np.outer(qi, qi)
                QP += np.outer(qi, pi)
            
            # Step 4: Compute SVD for rotation matrix
            U, _, Vt = np.linalg.svd(PP + QQ @ np.linalg.inv(QP))
            R_new = U @ Vt
            
            # Step 5: Check convergence
            error = np.mean(np.linalg.norm(P - (Qc @ R_new.T + t_new), axis=1))
            if error <= tolerance:
                break
            
            R, t = R_new, t_new
        
        return R, t, c


    def timer_update(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design 
        
        # Update odometry
        if self.pose_x is None:
            return
        if self.chek_real_position_rrived:
            self.chek_real_position_rrived = False
        else:
            if np.abs(self.ang_vel) <= 0.5:
                self.pose_x, self.pose_y, self.pose_a = odometry.unicycle_odometry_Euler(self.pose_x, self.pose_y, self.pose_a, self.lin_vel, self.ang_vel, delta_t = 1.0/self.rate)
                #self.pose_x, self.pose_y, self.pose_a = odometry.unicycle_odometry(self.pose_x, self.pose_y, self.pose_a, self.lin_vel, self.ang_vel, delta_t = 1.0/self.rate)
                #self.pose_x, self.pose_y, self.pose_a = odometry.unicycle_odometry_RungeKutta(self.pose_x, self.pose_y, self.pose_a, self.lin_vel, self.ang_vel, delta_t = 1.0/self.rate)
            else:
                if self.scan_pose_a is not None:
                    print("Scan Matching")
                    R, t, c =self.scan_matching(self.scan_points_old, self.scan_points , max_iterations=5, tolerance=1e-6)
                    original_position = np.array([self.pose_x, self.pose_y])
                    new_position = original_position + t
                    self.pose_x, self.pose_y = new_position
                    delta_theta = np.arctan2(R[1, 0], R[0, 0])
                    self.pose_a += delta_theta
                    self.pose_a=(self.pose_a + np.pi) % (2 * np.pi) - np.pi
                    print(delta_theta)
                    print(t)


        self.pose_out_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_out_msg.pose.position.x = self.pose_x
        self.pose_out_msg.pose.position.y = self.pose_y
        self.pose_out_msg.pose.position.z = 0.0
        q = quaternion_from_euler(0.0,0.0,self.pose_a, axes='sxyz')
        self.pose_out_msg.pose.orientation.x = q[0]
        self.pose_out_msg.pose.orientation.y = q[1]
        self.pose_out_msg.pose.orientation.z = q[2]
        self.pose_out_msg.pose.orientation.w = q[3]

        self.pose_out_publisher.publish(self.pose_out_msg)


def main(args=None):
    rclpy.init(args=args)
    odometry_node = Odometry()
    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()