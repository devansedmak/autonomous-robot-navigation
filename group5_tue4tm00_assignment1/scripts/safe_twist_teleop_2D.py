#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros


class SafeTwistTeleop2D(Node):
    
    def __init__(self):
        super().__init__('safe_twist_teleop_2D', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        
        # If needed in your design, get a node parameter for update rate
        default_rate = Parameter('rate', Parameter.Type.DOUBLE, 10.0) 
        self.rate = self.get_parameter_or('rate', default_rate).value
                
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

        # If need in your design, crease a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def scan_callback(self, msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        #TODO: If needed, use the scan topic messages in your design
        pass

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        #TODO: If needed, use the pose topic messages in your design
        pass

    def cmd_vel_in_callback(self, msg):
        """
        Callback function for the input cmd_vel topic, handling messages of type geometry_msgs.msg.LaserScan
        """
        #TODO: If needed, use the input cmd_vel topic messages in your design
        
        # For example, a simple direct input-to-output cmd_vel mapping without safety check
        self.cmd_vel_out = msg                  
        

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design 
        
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
