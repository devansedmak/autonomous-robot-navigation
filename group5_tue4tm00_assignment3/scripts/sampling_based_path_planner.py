#!/usr/bin/env python3

import math
import random
from core_search_path_planner import search_based_path_planning
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import TransformStamped
import networkx as nx

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros
from core_proj_nav_ctrl import proj_nav_tools
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
        self.max_cost = 90 #max value of cost map
        self.d_parameter = 10 # Parameter for adaptive selection of neibor size
        self.n = 100 

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

        graph = optimal_rrt(costmap_matrix, start_cell, self.n, self.d_parameter, self.max_cost)

        try:
            # Attempt to find the shortest path

            path_grid = dijkstra(graph, costmap_matrix, start_cell, random.choice(list(graph.nodes()))) # Da sostituire con goal_cell
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

        except nx.NodeNotFound:
            # Handle the case where the graph does not contain the required nodes
            self.get_logger().warn(f"A safe path does not exist!")
        except ValueError:
            self.get_logger().warn(f"Cost map still loadeding!")

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

def safety_verification_brehensam(costmap, idx1, idx2, max_cost):
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
        if costmap[row, col] >= max_cost:
            return False
    
    return True

def local_cost(x, y, costmap):
    n = max(abs(x[0] - y[0]), abs(x[1] - y[1])) + 1

    touched_cells = []

    for step in range(n):
        row = int(x[0] + step * (y[0] - x[0]) / (n - 1))
        col = int(x[1] + step * (y[1] - x[1]) / (n - 1))
        touched_cells.append((row, col))

    distance = np.linalg.norm(np.array(x) - np.array(y))

    values = [costmap[row][col] for row, col in touched_cells]
    media_cost=sum(values) / len(values) if values else 0
    return distance*media_cost

def dijkstra(graph, costmap_matrix, source_node, target_node):
    try:
        path = nx.dijkstra_path(graph, source_node, target_node)
        path = np.unravel_index(path, costmap_matrix.shape)
        path = np.column_stack((path[0], path[1]))
    except nx.exception.NetworkXNoPath: 
        path = np.zeros((0,2))

    return path

def point_projected(point, center, obstacles):
    convex_interior = proj_nav_tools.polygon_convex_interior(obstacles , center)
    _, proj_x, proj_y = proj_nav_tools.point_to_polygon_distance(point[0],point[1], convex_interior[:,0].astype(float), convex_interior[:,1].astype(float))
    new_point = np [ round(proj_x), round(proj_y)]
    return new_point

def points_within_radius(points, center, radius):
    """
    Return a list of points that have a distance of exactly `radius` from the center.

    Parameters:
        points (list of tuples): List of points as (x, y) coordinates.
        center (tuple): The center point as (x, y).
        radius (float): The radius to check.

    Returns:
        list: List of points within the radius.
    """
    result = []
    for point in points:
        distance = np.linalg.norm(np.array(point) - np.array(center))
        if np.isclose(distance, radius):
            result.append(point)
    return result

def optimal_rrt(costmap, start_point, n, d_parameter, max_cost):
    """
    Implement Optimal Rapidly Exploring Random Trees (RRT) algorithm.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        start_point (tuple): Starting point as (row, col).
        n (int): Number of iterations.

    Returns:
        MotionGraph: The motion graph G = (V, E).

    """
    max_cost_points = np.argwhere(costmap == max_cost)
    G = nx.Graph()
    G.add_node(start_point)

    sampled_indices = weighted_posterior_sampling(costmap, n)
    i=0

    for x_rand in sampled_indices:
        i=i+1
        x_nearest = min(G.nodes, key=lambda v: np.linalg.norm(np.array(v) - np.array(x_rand)))
        #anche x_nearest è un indice mi sembra
        x_new = point_projected(x_rand, x_nearest, max_cost_points)
        #da definire come trovare x_new che dovrebbe essere un punto non un indice
        radius = (math.log(i) / n) ** (1 / d_parameter)

        if safety_verification_brehensam(costmap, x_new, x_nearest):
            x_min = x_nearest
            mincost = dijkstra(costmap, start_point, x_nearest) + local_cost(x_nearest, x_new, costmap)
            #il costo è da riscrivere perchè stiamo usando sampled_indices che sono indici quindi bisogna
            #trasformare da indice a mondo
            x_neighbor = points_within_radius(G.nodes, x_new, radius) 
            for x_near in x_neighbor:
                tempcost = dijkstra(costmap, start_point, x_near) + local_cost(x_near, x_new, costmap)
                if tempcost < mincost and safety_verification_brehensam(costmap, x_near, x_new, max_cost):
                    x_min, mincost = x_near, tempcost
            G.add_node(x_new)
            G.add_edge(x_min, x_new, weight = local_cost(x_min, x_new))

            x_neighbor = points_within_radius(G.nodes, x_new, radius)
            for x_near in x_neighbor:
                tempcost = dijkstra(costmap, start_point, x_new) + local_cost(x_near, x_new, costmap)
                if tempcost < dijkstra(costmap, start_point, x_near) and safety_verification_brehensam(costmap, x_new, x_near):
                    x_parent = parent(G, x_near, start_point, radius, costmap)
                    G.remove_edge(x_parent, x_near)
                    G.add_edge(x_new, x_near, weight = local_cost(x_new, x_near))
    return G

def parent(G, x, x_ancestor, radius, costmap_matrix):
    """
    Find the parent of a vertex x with respect to its ancestor x_ancestor.

    Parameters:
        x (tuple): The vertex for which the parent is to be found.
        x_ancestor (tuple): The ancestor vertex.

    Returns:
        tuple: The parent vertex of x.
    """
    neighbors = points_within_radius(G.nodes, x, radius)
    min_cost = float('inf')
    parent_vertex = None

    for neighbor in neighbors:
        cost = dijkstra(G, costmap_matrix, neighbor, x_ancestor)
        if cost < min_cost:
            min_cost = cost
            parent_vertex = neighbor

    return parent_vertex

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
