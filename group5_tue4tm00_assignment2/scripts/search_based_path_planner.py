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
import networkx as nx

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros

import numpy as np
import time 

class SearchBasedPathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # If needed in your design, define your node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position
        
        # If needed in your design, get a node parameter for update rate
        self.rate = 1.0 
        rate = self.get_parameter('rate').value
        self.rate = rate if rate is not None else self.rate

        # Node Parameters
        self.max_cost = 100.0 # Maximum (unsafe) cost in the range of [0, 100]
        max_cost = self.get_parameter('max_cost').value
        self.max_cost = max_cost if max_cost is not None else self.max_cost
        
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

        # Create a subscriber to the costmap topic of message type nav_msgs.msg.OccupancyGrid
        costmap_qos_profile = QoSProfile(depth=1)
        costmap_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_subscriber = self.create_subscription(OccupancyGrid, 'costmap', self.costmap_callback, qos_profile=costmap_qos_profile)
        self.costmap_msg = OccupancyGrid()

        # Create a publisher for the path topic of message type nav_msgs.msg.Path
        path_qos_profile = QoSProfile(depth=1)
        path_qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_publisher = self.create_publisher(Path, 'path', qos_profile=path_qos_profile)
        self.path_msg = Path()

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
    
    def scan_callback(self, msg):
        """
        Callback function for the scan topic, handling messages of type sensor_msgs.msg.LaserScan
        """
        self.scan_msg = msg

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

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        if (self.pose_msg is None) or (self.goal_msg is None) or (self.costmap_msg is None):
            return
        
        self.get_logger().info('Path is being searched...')
        
        start_position = np.asarray([self.pose_x, self.pose_y])
        goal_position = np.asarray([self.goal_x, self.goal_y])

        costmap_origin = np.asarray([self.costmap_msg.info.origin.position.x, self.costmap_msg.info.origin.position.y])
        costmap_resolution = self.costmap_msg.info.resolution 
        costmap_matrix = np.array(self.costmap_msg.data).reshape(self.costmap_msg.info.height, self.costmap_msg.info.width)
        costmap_matrix = np.float64(costmap_matrix)
        costmap_matrix[costmap_matrix>=self.max_cost] = -1

        start_cell = search_based_path_planning.world_to_grid(start_position, origin=costmap_origin, resolution=costmap_resolution)[0]
        goal_cell = search_based_path_planning.world_to_grid(goal_position, origin=costmap_origin, resolution=costmap_resolution)[0]

        try:
            # Attempt to find the shortest path
            path_grid = search_based_path_planning.shortest_path_networkx(
                costmap_matrix, start_cell, goal_cell, diagonal_connectivity=True
            )
            path_world = search_based_path_planning.grid_to_world(path_grid, costmap_origin, costmap_resolution)

            path_msg = Path()
            path_msg.header.frame_id = 'world'
            path_msg.header.stamp = self.get_clock().now().to_msg()

            if path_world.size > 0:
                path_msg.poses.append(self.pose_msg)
                for waypoint in path_world:
                    pose_msg = PoseStamped()
                    pose_msg.header = path_msg.header
                    pose_msg.pose.position.x = waypoint[0]
                    pose_msg.pose.position.y = waypoint[1]
                    path_msg.poses.append(pose_msg)
                path_msg.poses.append(self.goal_msg)

            self.path_publisher.publish(path_msg)
            self.get_logger().info('Path is published!')

        except nx.NodeNotFound as e:
            # Handle the case where the graph does not contain the required nodes
            self.get_logger().warn(f"A safe path does not exist!")
        except ValueError as e:
            self.get_logger().warn(f"Cost map still loadeding!")


def main(args=None):
    rclpy.init(args=args)
    path_planner_node = SearchBasedPathPlanner()
    try: 
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass 
    # finally:   
    #     navigation_costmap_node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
