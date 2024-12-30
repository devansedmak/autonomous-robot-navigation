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
        #TODO: If needed, use the pose topic messages in your design
        self.pose_msg = msg
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]

    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        #TODO: If needed, use the pose topic messages in your design
        self.goal_msg = msg
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
    
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

    def costmap_callback(self, msg):
        """
        Callback function for the costmap topic, handling messages of type nav_msgs.msg.OccupancyGrid
        """
        #TODO: If needed, use the costmap topic messages in your design
        self.costmap_msg = msg    

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        #TODO: If needed, use the timer callbacks in your design 
        
        # For example, publish the straight path between the pose and the goal messages 
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(self.pose_msg)
        self.path_msg.poses.append(self.goal_msg)
        self.path_publisher.publish(self.path_msg)


class MotionGraph:
    def __init__(self):
        self.V = set()  # Set of vertices
        self.E = set()  # Set of edges

    def add_vertex(self, vertex):
        self.V.add(vertex)

    def add_edge(self, edge):
        self.E.add(edge)

    def __repr__(self):
        return f"Vertices: {self.V}\nEdges: {self.E}"


def weighted_posterior_sampling(costmap, num_samples):
    """
    Perform Weighted Posterior Sampling on a costmap.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        num_samples (int): The number of samples to generate.

    Returns:
        sampled_indices (np.ndarray): Array of sampled indices (row, col) based on weighted posterior sampling.
    """
    # Ensure the costmap is a numpy array
    costmap = np.asarray(costmap)

    # Calculate importance weights as the inverse of the costmap
    importance_weights = 1.0 / (costmap + 1e-9)  # Add small value to avoid division by zero

    # Normalize weights to form a probability distribution
    importance_weights /= np.sum(importance_weights)

    # Flatten the costmap and weights for sampling
    flattened_weights = importance_weights.flatten()
    flattened_indices = np.arange(flattened_weights.shape[0])

    # Perform weighted sampling with replacement
    sampled_flat_indices = np.random.choice(flattened_indices, size=num_samples, replace=True, p=flattened_weights)

    # Convert flattened indices back to 2D indices
    sampled_indices = np.unravel_index(sampled_flat_indices, costmap.shape)

    # Stack the indices into (row, col) format
    sampled_indices = np.stack(sampled_indices, axis=1)

    # Remove indices where costmap value is 0
    sampled_indices = np.array([idx for idx in sampled_indices if costmap[idx[0], idx[1]] != 0])

    return sampled_indices



def safety_verification_brehensam(costmap, idx1, idx2):
    """
    Verify if the given indices are safe using Brehensam's Line Algorithm.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        idx1 (tuple): First index (row, col) as a tuple.
        idx2 (tuple): Second index (row, col) as a tuple.

    Returns:
        bool: True if safe, False otherwise.
    """
    # Calculate n using the formula in Brehensam's Line Algorithm
    n = max(abs(idx1[0] - idx2[0]), abs(idx1[1] - idx2[1])) + 1

    # Check if the path is safe by ensuring all intermediate points have non-zero cost
    for step in range(n):
        row = int(idx1[0] + step * (idx2[0] - idx1[0]) / (n - 1))
        col = int(idx1[1] + step * (idx2[1] - idx1[1]) / (n - 1))
        if costmap[row, col] == 0:
            return False

    return True



def optimal_rrt(costmap, start_point, n):
    """
    Implement Optimal Rapidly Exploring Random Trees (RRT) algorithm.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        start_point (tuple): Starting point as (row, col).
        n (int): Number of iterations.

    Returns:
        MotionGraph: The motion graph G = (V, E).
    """
    G = MotionGraph()
    G.add_vertex(start_point)

    sampled_indices = weighted_posterior_sampling(costmap, n)

    for x_rand in sampled_indices:
        x_nearest = min(G.V, key=lambda v: np.linalg.norm(np.array(v) - np.array(x_rand)))
        #anche x_nearest è un indice mi sembra
        x_new = tuple((np.array(x_nearest) + np.array(x_rand)) // 2)
        #da definire come trovare x_new che dovrebbe essere un punto non un indice

        if safety_verification_brehensam(costmap, x_new, x_nearest):
            x_min = x_nearest
            mincost = np.linalg.norm(np.array(start_point) - np.array(x_nearest)) + np.linalg.norm(np.array(x_nearest) - np.array(x_new))
            #il costo è da riscrivere perchè stiamo usando sampled_indices che sono indici quindi bisogna
            #trasformare da indice a mondo  
            for x_near in G.V:
                tempcost = np.linalg.norm(np.array(start_point) - np.array(x_near)) + np.linalg.norm(np.array(x_near) - np.array(x_new))
                if tempcost < mincost and safety_verification_brehensam(costmap, x_near, x_new):
                    x_min, mincost = x_near, tempcost

            G.add_vertex(x_new)
            G.add_edge((x_min, x_new))

            for x_near in G.V:
                tempcost = np.linalg.norm(np.array(start_point) - np.array(x_new)) + np.linalg.norm(np.array(x_new) - np.array(x_near))
                if tempcost < np.linalg.norm(np.array(start_point) - np.array(x_near)) and safety_verification_brehensam(costmap, x_new, x_near):
                    G.E.discard((x_nearest, x_near))
                    G.add_edge((x_new, x_near))

    return G




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
