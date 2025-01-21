#!/usr/bin/env python3

import warnings
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
from group5_tue4tm00_assignment3 import tools3
from core_path_follow_ctrl import path_follow_tools
from group5_tue4tm00_assignment1 import tools
import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros
import numpy as np
from group5_tue4tm00_assignment2 import tools_2
from core_proj_nav_ctrl import proj_nav_tools
import matplotlib.pyplot as plt
import networkx as nx
import matplotlib.pyplot as plt



class PathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # Define the node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position

        self.path_goal = np.zeros((1,2)) # Projection of the goal on the Local Free Space
        self.path = np.zeros((0,2)) # Path points
        self.scan_pose_x = 0.0 # scan x-position
        self.scan_pose_y = 0.0 # scan y-position
        self.scan_pose_a = 0.0 # scan yaw angle
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.r = 0.22 # Radius of the robot
        self.scan_polygon = np.zeros((2,2)) 

        self.check_goal = True # Check if the goal has changed
        self.check_graph = True # Check if the graph has been created
        self.check = True # Check if the path has been found
        self.check_pose = False #Check if the position has been given
        self.max_cost = 90 # Max value of cost map
        self.d_parameter = 10 # Parameter for adaptive selection of neibor size
        self.n = 400 # Number of iterations of RRT* 
        
        # Node parameter for update rate
        self.rate = 1.0 
        rate = self.get_parameter('rate').value
        self.rate = rate if rate is not None else self.rate 
        
        # If needed in your design, create a subscriber to the pose topic
        pose_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.VOLATILE, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.pose_subscriber = self.create_subscription(PoseStamped, 'odom_pose', self.pose_callback, qos_profile=pose_qos_profile)
        self.pose_subscriber  # prevent unused variable warning
        self.pose_msg = PoseStamped()

        # If needed in your design, create a subscriber to the goal topic
        goal_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, # BEST_EFFORT | RELIABLE
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL, # TRANSIENT_LOCAL | VOLATILE
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, # KEEP_LAST | KEEP ALL
            depth=1, # Integer queue size
        )
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, qos_profile=goal_qos_profile)
        self.goal_subscriber  # prevent unused variable warning
        self.goal_msg = PoseStamped()

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

        # Create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

    def pose_callback(self, msg):
        """
        Callback function for the pose topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.pose_msg = msg
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        self.pose_a = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])[2]
        self.check_pose = True # Check if the position has been given

    def goal_callback(self, msg):
        """
        Callback function for the goal topic, handling messages of type geometry_msgs.msg.PoseStamped
        """
        self.goal_msg = msg
        if(self.goal_x != msg.pose.position.x or self.goal_y != msg.pose.position.y):
            self.check_goal = True # Check if the goal has changed
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
    
    def plot_graph(self, G):
        """
        Plots the vertices and edges of a networkx graph.

        Parameters:
        G (networkx.Graph): The graph to be plotted.

        Returns:
        None
        """
        pos = {node: node for node in G.nodes()}

        # Draw nodes (vertices)
        nx.draw_networkx_nodes(G, pos, node_size=5, node_color="skyblue", edgecolors="black")
        
        # Draw edges
        nx.draw_networkx_edges(G, pos, edge_color="red")

        # Display the graph
        plt.title("Graph Plot")
        plt.axis("equal")  # Ensure equal scaling for accurate spatial representation
        plt.show()

    def plot_graph_path(self, G, path):
        """
        Plots the graph with edges and highlights the path.

        Parameters:
        G (networkx.Graph): The graph to be plotted.
        path (list): The path as a list of nodes.

        Returns:
        None
        """
        import matplotlib.pyplot as plt
        import networkx as nx

        # Convert nodes to tuples if necessary
        pos = {tuple(node): tuple(node) for node in G.nodes()}

        # Convert path nodes to tuples
        path = [tuple(node) for node in path]

        # Validate that all path nodes are in the graph
        for node in path:
            if node not in pos:
                print(f"Warning: Node {node} is not in the graph!")
                return

        # Draw nodes (vertices)
        nx.draw_networkx_nodes(G, pos, node_size=5, node_color="skyblue", edgecolors="black")

        # Draw edges (all in red)
        nx.draw_networkx_edges(G, pos, edge_color="red")

        # Highlight the path if provided
        if path:
            path_edges = list(zip(path[:-1], path[1:]))
            nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="blue", width=2)

        plt.title("Graph Plot with Path")
        plt.axis("equal")  # Ensure equal scaling for accurate spatial representation
        plt.show()

    def timer_callback(self):
        """
        Callback function for peridic timer updates
        """
        
        if (self.pose_msg is None) or (self.goal_msg is None) or (self.costmap_msg is None):
            self.get_logger().warn("Pose, goal, or costmap messages are not yet received. Skipping...")
            return

        start_position = np.asarray([self.pose_x, self.pose_y])
        goal_position = np.asarray([self.goal_x, self.goal_y])
        
        pose_temp = np.array([self.pose_x, self.pose_y])
        scan_polygon_temp1= self.scan_polygon
        nearest_points = proj_nav_tools.local_nearest(self.scan_points, pose_temp) # Find the nearest points
        self.convex_interior = tools.polygon_convex_interior_safe_new(scan_polygon_temp1,nearest_points, pose_temp, self.r) # Find the convex interior
        self.path_goal = path_follow_tools.path_goal_support_corridor(self.path, pose_temp, self.convex_interior) # Find the projection of the goal on the Local Free Space
        
        if np.array_equal(goal_position, np.asarray([0, 0])):
            print("Goal still not loaded properly")
            return

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
        
        if self.check_graph:
            # Perform RRT path planning
            self.graph = tools3.Optimal_Probabilistic_Roadmap(costmap_matrix, self.n, self.d_parameter, self.max_cost)
            self.check_graph = False # Check if the graph has been created
            #self.plot_graph(self.graph) # Plot the graph

        if (self.check_goal or self.check or self.path_goal is None) and self.check_pose:
            print('Path is being searched...')
            if self.check_goal:
                self.check_goal=False
            try:
                # Attempt to find the shortest path
                x_nearest = min(self.graph.nodes, key=lambda v: np.linalg.norm(np.array(v) - goal_cell))

                # Construct Path message
                path_msg = Path()
                path_msg.header.frame_id = 'world'
                path_msg.header.stamp = self.get_clock().now().to_msg()
                
                # Check if the start cell is in the graph
                if tuple(start_cell) not in self.graph.nodes:
                    #x_nearest_start = min(self.graph.nodes, key=lambda v: np.linalg.norm(np.array(v) - start_cell))
                    #Connect the start cell to all its neighboring cells by establishing possible safe connections between them.
                    radius=100
                    x_neighbors = tools3.points_within_radius(self.graph.nodes, start_cell, radius)
                    connection_found = False
                    for x_near in x_neighbors:
                        if tools3.safety_verification_brehensam(costmap_matrix, start_cell, x_near, self.max_cost):
                            self.graph.add_node(tuple(start_cell))
                            self.graph.add_edge(tuple(x_near), tuple(start_cell), weight=tools3.local_cost(x_near, start_cell, costmap_matrix))
                            connection_found = True

                    if not connection_found:
                        if costmap_matrix[start_cell[1], start_cell[0]] == self.max_cost:
                            raise nx.NodeNotFound
                        #self.graph = tools3.informed_optimal_rrt(costmap_matrix, x_nearest_start, (self.n+500), self.d_parameter, self.max_cost, start_cell, self.graph)

                        if self.n < 1600:
                            
                            self.check_graph = True
                            self.n=self.n*4
                            self.timer_callback()
                        else: 
                            
                            pose_msg = PoseStamped()
                            pose_msg.header = path_msg.header
                            self.path_publisher.publish(path_msg)
                            
                            return
                            
                
                # Check if the goal cell is in the graph
                #Connect the goal cell to all its neighboring cells by establishing possible safe connections between them.
                radius=10
                x_neighbors = tools3.points_within_radius(self.graph.nodes, goal_cell, radius)
                connection_found = False
                for x_near in x_neighbors:
                    if tools3.safety_verification_brehensam(costmap_matrix, goal_cell, x_near, self.max_cost):
                        self.graph.add_node(tuple(goal_cell))
                        self.graph.add_edge(tuple(x_near), tuple(goal_cell), weight=tools3.local_cost(x_near, goal_cell, costmap_matrix))
                        connection_found = True
                
                if connection_found:
                    try:
                        path_grid, length = tools3.dijkstra_path(self.graph, tuple(start_cell), tuple(goal_cell))
                    except KeyError:
                        raise nx.NodeNotFound
                else:
                    if costmap_matrix[goal_cell[1], goal_cell[0]] == self.max_cost:
                        raise nx.NodeNotFound
                    #self.graph = tools3.informed_optimal_rrt(costmap_matrix, x_nearest, (self.n+500), self.d_parameter, self.max_cost, goal_cell, self.graph)
                    #path_grid, length = tools3.dijkstra_path(self.graph, tuple(start_cell), tuple(goal_cell))
                    if self.n < 1600:
                        
                        self.check_graph = True
                        self.n=self.n*4
                        self.timer_callback()
                    else: 
                        
                        pose_msg = PoseStamped()
                        pose_msg.header = path_msg.header
                        #pose_msg.pose.position.x = []
                        #pose_msg.pose.position.y = []
                        #path_msg.poses.append(pose_msg)
                        self.path_publisher.publish(path_msg)
                        
                        return
                
                # If a path is found, restart the algorithm only one time
                if path_grid is None and self.n < 1600:
                    
                    self.check_graph = True
                    self.n=self.n*4
                    self.timer_callback()
                
                if path_grid is None and self.n >= 1600:
                    
                    pose_msg = PoseStamped()
                    pose_msg.header = path_msg.header
                    #pose_msg.pose.position.x = []
                    #pose_msg.pose.position.y = []
                    #path_msg.poses.append(pose_msg)
                    self.path_publisher.publish(path_msg)
                    
                    return
                
                # Convert the path from grid to world coordinates
                path_world = search_based_path_planning.grid_to_world(path_grid, costmap_origin, costmap_resolution)  
                print(path_world)
                self.path=path_world
                #self.plot_graph(self.graph) # Plot the graph
                #self.plot_graph_path(self.graph,path_grid) # Plot the path

                for waypoint in path_world:
                    pose_msg = PoseStamped()
                    pose_msg.header = path_msg.header
                    pose_msg.pose.position.x = waypoint[0]
                    pose_msg.pose.position.y = waypoint[1]
                    path_msg.poses.append(pose_msg)

                if path_world.size > 0:
                    self.get_logger().info('Path is published!')
                    self.path_publisher.publish(path_msg)
                else:
                    print("There is no safe path, waiting for a new goal or new start position...")

                self.check = False
            except nx.NodeNotFound:
                self.get_logger().warn("A safe path does not exist!")
                self.path_publisher.publish(path_msg) # Publish empty path
            except ValueError as e:
                self.get_logger().warn(f"Path planning failed: {str(e)}")  

def main(args=None):
    rclpy.init(args=args)
    path_planner_node = PathPlanner()
    try: 
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass 
    # finally:   
    #     navigation_costmap_node.destroy_node()
    #     rclpy.shutdown()


if __name__ == '__main__':
    main()
