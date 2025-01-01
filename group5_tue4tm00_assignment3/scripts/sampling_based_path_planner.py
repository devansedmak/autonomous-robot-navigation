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
import heapq

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
        self.n = 500 

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
            self.get_logger().warn("Pose, goal, or costmap messages are not yet received. Skipping...")
            return

        self.get_logger().info('Path is being searched...')

        start_position = np.asarray([self.pose_x, self.pose_y])
        goal_position = np.asarray([self.goal_x, self.goal_y])

        try:
            # Check if the costmap has the necessary data
            if (self.costmap_msg.info.origin is None or
                self.costmap_msg.info.resolution is None or
                not self.costmap_msg.data):
                raise ValueError("Costmap message is incomplete or still computing.")

            # Extract costmap parameters
            costmap_origin = np.asarray([self.costmap_msg.info.origin.position.x,
                                            self.costmap_msg.info.origin.position.y])
            costmap_resolution = self.costmap_msg.info.resolution
            costmap_matrix = np.array(self.costmap_msg.data).reshape(
                self.costmap_msg.info.height, self.costmap_msg.info.width
            )
            costmap_matrix = np.float64(costmap_matrix)
            costmap_matrix[costmap_matrix < 0] = self.max_cost
    

        except ValueError as e:
            self.get_logger().warn(f"Costmap processing failed: {str(e)}")
            return
        except AttributeError as e:
            self.get_logger().warn(f"Costmap attributes missing or malformed: {str(e)}")
            return

        # Convert start and goal positions from world to grid
        start_cell = search_based_path_planning.world_to_grid(
        start_position, origin=costmap_origin, resolution=costmap_resolution)[0]
        goal_cell = search_based_path_planning.world_to_grid(
        goal_position, origin=costmap_origin, resolution=costmap_resolution)[0]
        
        # Perform RRT path planning
        graph = optimal_rrt(costmap_matrix, start_cell, self.n, self.d_parameter, self.max_cost)
        
        node_list = list(graph.nodes)
       #prendo momentaneamente un punto del grafo
        #goal_cell=np.array(node_list[15])
        #start_position=np.array(node_list[15])
        
        try:
        # Attempt to find the shortest path
           

            x_nearest = min(graph.nodes, key=lambda v: np.linalg.norm(np.array(v) - goal_cell))
            

            path_grid, length= dijkstra_path(graph, tuple(start_cell), tuple(x_nearest))
            path_world = search_based_path_planning.grid_to_world(
                path_grid, costmap_origin, costmap_resolution
            )

            # Construct Path message
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
                
                if safety_verification_brehensam(costmap_matrix, goal_cell, x_nearest, self.max_cost):
                    path_msg.poses.append(self.goal_msg)
                else:
                    print("not safe connection")
                print(path_world)
            else:
                print("path_world null")
                print(path_world)
            self.path_publisher.publish(path_msg)
            self.get_logger().info('Path is published!')

        except nx.NodeNotFound:
            self.get_logger().warn("A safe path does not exist!")
        except ValueError as e:
            self.get_logger().warn(f"Path planning failed: {str(e)}")
    

def weighted_posterior_sampling(costmap, num_samples, max_cost):
    """
    Perform Weighted Posterior Sampling on a costmap, ensuring valid probabilities.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        num_samples (int): The number of samples to generate.

    Returns:
        sampled_indices (np.ndarray): Array of sampled indices (row, col) based on weighted posterior sampling.
    """
    # Ensure the costmap is a numpy array
    costmap = np.asarray(costmap)

    # Replace negative or NaN values with a high cost (optional: np.inf)
    costmap = np.where((costmap < 0) | (np.isnan(costmap)), np.inf, costmap)

    # Calculate importance weights as the inverse of the costmap
    importance_weights = 1.0 / (costmap + 1e-9)  # Add small value to avoid division by zero

    # Normalize weights to form a probability distribution
    total_weight = np.sum(importance_weights)
    if total_weight <= 0:
        raise ValueError("Costmap weights cannot be normalized: sum is zero or negative.")
    importance_weights /= total_weight

    # Flatten the costmap and weights for sampling
    flattened_weights = importance_weights.flatten()
    flattened_indices = np.arange(flattened_weights.shape[0])

    # Ensure probabilities are non-negative
    if np.any(flattened_weights < 0):
        raise ValueError("Probabilities are not non-negative after normalization.")

    # Perform weighted sampling with replacement
    sampled_flat_indices = np.random.choice(flattened_indices, size=num_samples, replace=True, p=flattened_weights)

    # Convert flattened indices back to 2D indices
    sampled_indices = np.unravel_index(sampled_flat_indices, costmap.shape)
    sampled_indices = np.stack(sampled_indices, axis=1)

    # Filter out indices where costmap value is 0
    valid_indices = [idx for idx in sampled_indices if costmap[idx[0], idx[1]] != 0]

    # Repeat sampling until enough valid indices are collected
    while len(valid_indices) < num_samples:
        additional_samples = num_samples - len(valid_indices)
        sampled_flat_indices = np.random.choice(flattened_indices, size=additional_samples, replace=True, p=flattened_weights)
        sampled_indices = np.unravel_index(sampled_flat_indices, costmap.shape)
        sampled_indices = np.stack(sampled_indices, axis=1)
        valid_indices.extend([idx for idx in sampled_indices if costmap[idx[0], idx[1]] < max_cost])

    return np.array(valid_indices[:num_samples])


def dijkstra_path(graph, source, target):
    # Priority queue per i nodi da esplorare
    priority_queue = []
    distances = {node: float('inf') for node in graph}
    previous_nodes = {node: None for node in graph}
    distances[source] = 0
    heapq.heappush(priority_queue, (0, source))

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node == target:
            break

        for neighbor, attributes in graph[current_node].items():
            # Estrai il peso correttamente
            weight = attributes.get('weight', 1)  # Default a 1 se il peso manca
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    path = []
    current = target
    while current is not None:
        path.insert(0, current)
        current = previous_nodes[current]

    if distances[target] == float('inf'):
        return None, float('inf')

    return path, distances[target]




def bresenham(x1, y1, x2, y2):
    
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1))  # add the current point to the list
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return points

def safety_verification_brehensam(costmap, idx1, idx2, max_cost):
    """
    Verifica se il percorso tra idx1 e idx2 è sicuro usando l'algoritmo di Bresenham.

    Parameters:
        costmap (np.ndarray): La mappa dei costi.
        idx1 (tuple): Indice iniziale (riga, colonna).
        idx2 (tuple): Indice finale (riga, colonna).
        max_cost (float): Costo massimo consentito.

    Returns:
        bool: True se il percorso è sicuro, False altrimenti.
    """
    # Usa l'algoritmo di Bresenham per trovare i punti sulla linea tra idx1 e idx2
    line_points = list(bresenham(int(idx1[0]), int(idx1[1]), int(idx2[0]), int(idx2[1])))

    # Verifica i valori nella costmap lungo il percorso
    for row, col in line_points:
        if costmap[row, col] >= max_cost:
            return False  # Il percorso non è sicuro

    return True  # Il percorso è sicuro

def local_cost(x, y, costmap):
    

    touched_cells = bresenham(int(x[0]), int(x[1]), int(y[0]), int(y[1]))
    
    

    distance = np.linalg.norm(np.array(x) - np.array(y))

    values = [costmap[row][col] for row, col in touched_cells]
    media_cost=sum(values) / len(values) if values else 0
    return distance*media_cost
"""
def dijkstra(graph, costmap_matrix, source_node, target_node):
    source_node = tuple(source_node)
    target_node = tuple(target_node)
    try:
        path = nx.dijkstra_path(graph, source_node, target_node)
        path = np.unravel_index(path, costmap_matrix.shape)
        path = np.column_stack((path[0], path[1]))
    except nx.exception.NetworkXNoPath: 
        path = np.zeros((0,2))

    return path
"""
def point_projected(point, center, obstacles):
    #print(obstacles)
    convex_interior = proj_nav_tools.polygon_convex_interior(obstacles , center)
    _, proj_x, proj_y = proj_nav_tools.point_to_polygon_distance(point[0],point[1], convex_interior[:,0].astype(float), convex_interior[:,1].astype(float))
    #new_point = np.array([np.round(proj_x), np.round(proj_y)])
    new_point = (int(np.round(proj_x)), int(np.round(proj_y)))
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
    # Trova punti con costo massimo nella costmap
    max_cost_points = np.argwhere(costmap >= max_cost)
    print("eccomi qua")
   # print(max_cost_points)

    # Crea il grafo
    G = nx.Graph()
    G.add_node(tuple(start_point))

    # Genera punti campionati
    sampled_indices = weighted_posterior_sampling(costmap, n, max_cost)

    for i, x_rand in enumerate(sampled_indices, start=1):
        # Converte x_rand in tupla se necessario
        x_rand = tuple(x_rand)

        # Trova il nodo più vicino a x_rand nel grafo
        x_nearest = min(G.nodes, key=lambda v: np.linalg.norm(np.array(v) - np.array(x_rand)))

        # Proietta il punto x_rand verso x_nearest
        x_new = point_projected(x_rand, x_nearest, max_cost_points)
        x_new = tuple(x_new)  # Converti in tupla hashable

        # Calcola il raggio per il rewire
        radius = (math.log(i) / n) ** (1 / d_parameter)

        # Verifica la sicurezza del collegamento
        if safety_verification_brehensam(costmap, x_new, x_nearest, max_cost):
            x_min = x_nearest
            path, cost = dijkstra_path(G, tuple(start_point), tuple(x_nearest)) 
            mincost = cost + local_cost(x_nearest, tuple(x_new), tuple(costmap))

            # Trova nodi vicini a x_new
            x_neighbors = points_within_radius(G.nodes, x_new, radius)

            for x_near in x_neighbors:
                # Converte x_near in tupla se necessario
                x_near = tuple(x_near)

                path, tempcost = dijkstra_path(G, tuple(start_point), tuple(x_near)) + local_cost(x_near, x_new, costmap)
                if tempcost < mincost and safety_verification_brehensam(costmap, x_near, x_new, max_cost):
                    x_min, mincost = x_near, tempcost

            # Aggiungi il nodo e il collegamento al grafo
            G.add_node(tuple(x_new))
            G.add_edge(x_min, x_new, weight=local_cost(x_min, x_new, costmap))

            # Aggiorna i collegamenti dei vicini
            for x_near in x_neighbors:
                # Converte x_near in tupla se necessario
                x_near = tuple(x_near)

                path, tempcost = dijkstra_path(G,  tuple(start_point), tuple(x_new)) + local_cost(x_near, x_new, costmap)
                path, length = dijkstra_path(G, tuple(start_point), tuple(x_near))
                if tempcost < length and safety_verification_brehensam(costmap, x_new, x_near, max_cost):
                    x_parent = parent(G, x_near, start_point, radius, costmap)
                    G.remove_edge(x_parent, x_near)
                    G.add_edge(x_new, x_near, weight=local_cost(x_new, x_near, costmap))

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
        path, cost = dijkstra_path(G, tuple(neighbor), tuple(x_ancestor))
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
