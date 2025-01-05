#!/usr/bin/env python3

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

import rclpy.time
from tf_transformations import euler_from_quaternion
import tf2_ros
import numpy as np
from group5_tue4tm00_assignment2 import tools_2
from core_proj_nav_ctrl import proj_nav_tools
class SearchBasedPathPlanner(Node):
    
    def __init__(self):
        super().__init__('path_planner', allow_undeclared_parameters=True, 
                         automatically_declare_parameters_from_overrides=True)
        
        # Define the node parameters
        self.pose_x = 0.0 # robot x-position
        self.pose_y = 0.0 # robot y-position
        self.pose_a = 0.0 # robot yaw angle
        self.goal_x = 0.0 # goal x-position
        self.goal_y = 0.0 # goal y-position

        self.check_goal = True # Check if the goal has changed
        self.check_graph = True # Check if the graph has been created
        self.check = True # Check if the path has been found

        self.max_cost = 90 # Max value of cost map
        self.d_parameter = 10 # Parameter for adaptive selection of neibor size
        self.n = 300 # Number of iterations of RRT* 
        
        # Node parameter for update rate
        self.rate = 1.0 
        rate = self.get_parameter('rate').value
        self.rate = rate if rate is not None else self.rate 
        
        # Create a subscriber to the pose topic
        self.pose_subscriber = self.create_subscription(PoseStamped, 'pose', self.pose_callback, 1)
        self.pose_msg = PoseStamped()

        # Create a subscriber to the goal topic
        self.goal_subscriber = self.create_subscription(PoseStamped, 'goal', self.goal_callback, 1)
        self.goal_msg = PoseStamped()

        # Create a subscriber to the scan topic
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.scan_msg = LaserScan()

        # Create a subscriber to the map topic with the QoS profile of transient_local durability
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

        # Create a buffer and listener to the /tf topic 
        # to get transformations via self.tf_buffer.lookup_transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a timer for periodic updates
        self.create_timer(1.0 / self.rate, self.timer_callback)

        self.path_goal = np.zeros((1,2))
        self.path = np.zeros((0,2)) # Path
        self.scan_pose_x = 0.0 # scan x-position
        self.scan_pose_y = 0.0 # scan y-position
        self.scan_pose_a = 0.0 # scan yaw angle
        self.scan_points = np.zeros((2,2)) # Valid scan points
        self.r = 0.2
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
        if(self.goal_x != msg.pose.position.x or self.goal_y != msg.pose.position.y):
            self.check_goal = True
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

    def timer_callback(self):
        """
        Callback function for periodic timer updates
    
        """

        if (self.pose_msg is None) or (self.goal_msg is None) or (self.costmap_msg is None):
            self.get_logger().warn("Pose, goal, or costmap messages are not yet received. Skipping...")
            return

        start_position = np.asarray([self.pose_x, self.pose_y])
        goal_position = np.asarray([self.goal_x, self.goal_y])

        self.nearest_points = proj_nav_tools.local_nearest(self.scan_points, [self.scan_pose_x, self.scan_pose_y])
        self.path_goal = tools_2.path_goal_support_corridor_safe(self.path, [self.scan_pose_x, self.scan_pose_y], self.nearest_points, self.r)


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
            self.graph = tools3.optimal_rrt(costmap_matrix, start_cell, self.n, self.d_parameter, self.max_cost)
            self.check_graph = False
            print("fatto rrt")

        if self.check_goal or self.check or self.path_goal is None:
            self.get_logger().info('Path is being searched...')
            if self.check_goal:
                self.check_goal=False
            try:
                # Attempt to find the shortest path
                x_nearest = min(self.graph.nodes, key=lambda v: np.linalg.norm(np.array(v) - goal_cell))
                print("trovato nearest " )
                print(search_based_path_planning.grid_to_world( x_nearest, costmap_origin, costmap_resolution))
                
                #path_world = search_based_path_planning.grid_to_world(path_grid, costmap_origin, costmap_resolution)
                

                # Construct Path message
                path_msg = Path()
                path_msg.header.frame_id = 'world'
                path_msg.header.stamp = self.get_clock().now().to_msg()
                
                if tuple(start_cell) not in self.graph.nodes:
                    x_nearest_start = min(self.graph.nodes, key=lambda v: np.linalg.norm(np.array(v) - start_cell))
                    if tools3.safety_verification_brehensam(costmap_matrix, start_cell, x_nearest_start, self.max_cost):
                        #path_msg.poses.append(self.goal_msg)
                        self.graph.add_node(tuple(start_cell))
                        self.graph.add_edge(tuple(x_nearest_start), tuple(start_cell), weight=tools3.local_cost(x_nearest_start, start_cell, costmap_matrix))
                        #path_grid, length = tools3.dijkstra_path(self.graph, tuple(x_nearest_start), tuple(start_cell))
                        #path_grid.append(goal_cell)
                        print("ho trovato un gollegamento diretto tra la start cell e la cella più vicina")
                        #print(path_world)
                    else:
                        
                        #print("i used informed rrt!!!!!!!!!!!!")
                        self.graph = tools3.informed_optimal_rrt(costmap_matrix, x_nearest_start, (self.n+500), self.d_parameter, self.max_cost, start_cell, self.graph)
                        print("ho fatto rrt informed ho trovato un gollegamento diretto tra la start cell e la cella più vicina")
                        #path_grid, length = tools3.dijkstra_path(self.graph, tuple(start_cell), tuple(x_nearest_start))
                        
                        #print(path_world)




                if tools3.safety_verification_brehensam(costmap_matrix, goal_cell, x_nearest, self.max_cost):
                    #path_msg.poses.append(self.goal_msg)
                    self.graph.add_node(tuple(goal_cell))
                    self.graph.add_edge(tuple(x_nearest), tuple(goal_cell), weight=tools3.local_cost(x_nearest, goal_cell, costmap_matrix))
                    path_grid, length = tools3.dijkstra_path(self.graph, tuple(start_cell), tuple(goal_cell))
                    #path_grid.append(goal_cell)
                    print()
                    #print(path_world)
                else:
                        
                    print("i used informed rrt!!!!!!!!!!!!")
                    self.graph = tools3.informed_optimal_rrt(costmap_matrix, x_nearest, (self.n+500), self.d_parameter, self.max_cost, goal_cell, self.graph)
                    print("ho fatto rrt informed")
                    path_grid, length = tools3.dijkstra_path(self.graph, tuple(start_cell), tuple(goal_cell))
                        
                    #print(path_world)
                path_world = search_based_path_planning.grid_to_world(path_grid, costmap_origin, costmap_resolution)  
                self.path=path_world
                for waypoint in path_world:
                    pose_msg = PoseStamped()
                    pose_msg.header = path_msg.header
                    pose_msg.pose.position.x = waypoint[0]
                    pose_msg.pose.position.y = waypoint[1]
                    path_msg.poses.append(pose_msg)

                if path_world.size > 0:
                    #path_msg.poses.append(self.pose_msg)
                    print(path_world)

                else:
                    print("path_world null")
                    print(path_world)
                self.path_publisher.publish(path_msg)
                self.get_logger().info('Path is published!')
                self.check = False
            except nx.NodeNotFound:
                self.get_logger().warn("A safe path does not exist!")
            except ValueError as e:
                self.get_logger().warn(f"Path planning failed: {str(e)}")  

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
