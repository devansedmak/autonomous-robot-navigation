#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rclpy.qos
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion, quaternion_from_euler 
from core_odometry_tools import odometry

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

    def cmd_vel_callback(self, msg):
        """
        Callback function for the input control velocity topic, handling messages of type geometry_msgs.msg.Twist
        """
        #TODO: If needed, use the cmd_vel topic messages in your design
        self.lin_vel = msg.linear.x
        self.ang_vel = msg.angular.z
    
    def scan_callback(self, msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        #TODO: If needed, use the scan topic messages in your design 
        self.scan_msg = msg

    def map_callback(self, msg):
        """
        Callback function for the map topic, handling messages of type nav_msgs.msg.OccupancyGrid
        """
        #TODO: If needed, use the map topic messages in your design
        self.map_msg = msg

    def timer_update(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design 
        
        # Update odometry
        if self.pose_x is None:
            return
        
        self.pose_x, self.pose_y, self.pose_a = odometry.unicycle_odometry_Euler(
            self.pose_x, self.pose_y, self.pose_a, self.lin_vel, self.ang_vel, delta_t = 1.0/self.rate)
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