import warnings
import numpy as np
import heapq
import networkx as nx
import math
from core_proj_nav_ctrl import proj_nav_tools

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


def goal_weighted_sampling(costmap, goal_position, num_samples, max_cost):
    """
    Perform sampling on a costmap with higher density near the goal position.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        goal_position (tuple): The (row, col) coordinates of the goal position.
        num_samples (int): The number of samples to generate.
        max_cost (float): Maximum allowable cost for sampling.

    Returns:
        sampled_indices (np.ndarray): Array of sampled indices (row, col).
    """
    # Ensure the costmap is a numpy array
    costmap = np.asarray(costmap)

    # Replace negative or NaN values with a high cost
    costmap = np.where((costmap < 0) | (np.isnan(costmap)), np.inf, costmap)

    # Create a grid of coordinates
    rows, cols = costmap.shape
    grid_y, grid_x = np.meshgrid(np.arange(rows), np.arange(cols), indexing='ij')

    # Calculate distance from each cell to the goal
    goal_row, goal_col = goal_position
    #distances = np.sqrt((grid_y - goal_row) ** 2 + (grid_x - goal_col) ** 2)
    distances = (grid_y - goal_row) ** 2 + (grid_x - goal_col) ** 2

    # Compute weights: inverse of the costmap and proximity to the goal
    importance_weights =( 1.0 / (distances + 1e-9))**(1/4)

    # Mask out cells with costs higher than max_cost
    importance_weights[costmap > max_cost] = 0

    # Normalize weights to form a probability distribution
    total_weight = np.sum(importance_weights)
    if total_weight <= 0:
        raise ValueError("Costmap weights cannot be normalized: sum is zero or negative.")
    importance_weights /= total_weight

    # Flatten the weights and costmap for sampling
    flattened_weights = importance_weights.flatten()
    flattened_indices = np.arange(flattened_weights.shape[0])

    # Perform initial sampling
    sampled_flat_indices = np.random.choice(flattened_indices, size=num_samples, replace=True, p=flattened_weights)
    sampled_indices = np.unravel_index(sampled_flat_indices, costmap.shape)
    sampled_indices = np.stack(sampled_indices, axis=1)

    # Filter out indices where costmap value is 0 or exceeds max_cost
    valid_indices = [idx for idx in sampled_indices if costmap[idx[0], idx[1]] < max_cost]

    # Repeat sampling until enough valid indices are collected
    while len(valid_indices) < num_samples:
        additional_samples = num_samples - len(valid_indices)
        sampled_flat_indices = np.random.choice(flattened_indices, size=additional_samples, replace=True, p=flattened_weights)
        sampled_indices = np.unravel_index(sampled_flat_indices, costmap.shape)
        sampled_indices = np.stack(sampled_indices, axis=1)
        valid_indices.extend([idx for idx in sampled_indices if costmap[idx[0], idx[1]] < max_cost])

    # Return exactly the required number of samples
    return np.array(valid_indices[:num_samples])


def dijkstra_path(graph, source, target):
    """
    Find the shortest path between two nodes in a graph using Dijkstra's algorithm.

    Parameters:
        graph (nx.Graph): The graph to search.
        source (hashable): The starting node.
        target (hashable): The ending node.

    Returns:
        tuple: A tuple containing the path as a list of nodes and the total distance.
    """
    # Priority queue for Dijkstra's algorithm
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
            # Extract the weight of the edge
            weight = attributes.get('weight', 1)  # Default weight is 1
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
    """
    Bresenham's line algorithm.

    Parameters:
        x1 (int): The x-coordinate of the starting point.
        y1 (int): The y-coordinate of the starting point.
        x2 (int): The x-coordinate of the ending point.
        y2 (int): The y-coordinate of the ending point.

    Returns:
        list: A list of points on the line between (x1, y1) and (x2, y2).
    """
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        points.append((x1, y1))  # Add the current point to the list
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
    Verify the safety of a path between two points using Bresenham's line algorithm.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        idx1 (tuple): The starting point as (row, col).
        idx2 (tuple): The ending point as (row, col).
        max_cost (float): Maximum allowable cost for the path.

    Returns:
        bool: True if the path is safe, False otherwise.
    """
    # Compute the points along the line between idx1 and idx2
    line_points = list(bresenham(int(idx1[0]), int(idx1[1]), int(idx2[0]), int(idx2[1])))

    # Verify the safety of the path
    for row, col in line_points:
        if costmap[row, col] >= max_cost:
            return False  # The path is not safe

    return True  # The path is safe

def local_cost(x, y, costmap):
    """
    Compute the local cost of moving from point x to point y.

    Parameters:
        x (tuple): The starting point as (row, col).
        y (tuple): The ending point as (row, col).
        costmap (np.ndarray): A 2D numpy array representing the cost map.

    Returns:
        float: The local cost of moving from x to y.
    """
    touched_cells = bresenham(int(x[0]), int(x[1]), int(y[0]), int(y[1]))
    distance = np.linalg.norm(np.array(x) - np.array(y))

    values = [costmap[row][col] for row, col in touched_cells]
    media_cost=sum(values) / len(values) if values else 0
    return distance*media_cost

def point_projected(point, center, obstacles):
    """
    Project a point onto the convex interior of a polygon.

    Parameters:
        point (tuple): The point to project as (x, y).

    Returns:
        tuple: The projected point as (x, y).
    """
    obstacles=obstacles.astype(float)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        convex_interior = proj_nav_tools.polygon_convex_interior(obstacles, center)
    distance , proj_x, proj_y = proj_nav_tools.point_to_polygon_distance(point[0],point[1], convex_interior[:,0].astype(float), convex_interior[:,1].astype(float))
    if distance <0 : 
        return point
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

        #if  distance < radius*10 and distance!=0:
        if  distance < radius*10 :
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
        MotionGraph: The graph G = (V, E).
    """
    # Find points with maximum cost in the costmap
    max_cost_points = np.argwhere(costmap >= max_cost)

    # Create the graph
    G = nx.Graph()
    G.add_node(tuple(start_point))

    # Generate sampled points
    sampled_indices = weighted_posterior_sampling(costmap, n, max_cost)

    for i, x_rand in enumerate(sampled_indices, start=1):
        x_rand = tuple(x_rand)  # Convert x_rand to tuple
        # Find the nearest node to x_rand in the graph
        x_nearest = min(G.nodes, key=lambda v: np.linalg.norm(np.array(v) - np.array(x_rand)))
        
        # Project the point x_rand towards x_nearest
        x_new = point_projected(x_rand, x_nearest, max_cost_points)
        x_new = tuple(x_new)  # Convert to tuple

        radius = (math.log(i) / i) ** (1 / d_parameter) # Calculate radius for rewire

        # Verify the safety of the connection
        if safety_verification_brehensam(costmap, x_new, x_nearest, max_cost):
            x_min = x_nearest
            path, cost = dijkstra_path(G, tuple(start_point), tuple(x_nearest)) 
            mincost = cost + local_cost(x_nearest, tuple(x_new), tuple(costmap))

            # Find neighbors of x_new
            x_neighbors = points_within_radius(G.nodes, x_new, radius)

            for x_near in x_neighbors:
                x_near = tuple(x_near) # Convert x_near to tuple
                # Calculate the cost of the path
                path, tempcost = dijkstra_path(G, tuple(start_point), tuple(x_near)) 
                tempcost = tempcost+ local_cost(x_near, x_new, costmap)
                if tempcost < mincost and safety_verification_brehensam(costmap, x_near, x_new, max_cost):
                    x_min, mincost = x_near, tempcost

            # Add the node and edge to the graph
            G.add_node(tuple(x_new))
            G.add_edge(x_min, x_new, weight=local_cost(x_min, x_new, costmap))
            x_neighbors = points_within_radius(G.nodes, x_new, radius)
            # Update the connections of the neighbors
            for x_near in x_neighbors:
                x_near = tuple(x_near) # Convert x_near to tuple
                # Calculate the cost of the path
                path, tempcost = dijkstra_path(G,  tuple(start_point), tuple(x_new)) 
                tempcost=tempcost+local_cost(x_near, x_new, costmap)
                path, length = dijkstra_path(G, tuple(start_point), tuple(x_near))
                if tempcost < length and safety_verification_brehensam(costmap, x_new, x_near, max_cost):
                    x_parent = parent(G, x_near, start_point, radius)
                    G.remove_edge(tuple(x_parent), tuple(x_near))
                    G.add_edge(tuple(x_new), tuple(x_near), weight=local_cost(x_new, x_near, costmap))
    return G

def informed_optimal_rrt(costmap, start_point, n, d_parameter, max_cost, goal_position, graph):
    """
    Implement Informed Optimal Rapidly Exploring Random Trees (RRT) algorithm.

    Parameters:
        costmap (np.ndarray): A 2D numpy array representing the cost map.
        start_point (tuple): Starting point as (row, col).
        n (int): Number of iterations.
        d_parameter (float): Parameter for adaptive selection of neighbor size.
        max_cost (float): Maximum allowable cost for sampling.
        goal_position (tuple): The goal position as (row, col).
        graph (nx.Graph): The existing graph.

    Returns:
        MotionGraph: The graph G = (V, E).
    """
    # Find points with maximum cost in the costmap
    max_cost_points = np.argwhere(costmap >= max_cost)
    G=graph
    
    # Generate sampled points
    sampled_indices = goal_weighted_sampling(costmap, goal_position, n, max_cost)
    for i, x_rand in enumerate(sampled_indices, start=1):
        # Converte x_rand in tupla se necessario
        x_rand = tuple(x_rand)

        # Find the nearest node to x_rand in the graph
        x_nearest = min(G.nodes, key=lambda v: np.linalg.norm(np.array(v) - np.array(x_rand)))
        # Project the point x_rand towards x_nearest
        x_new = point_projected(x_rand, x_nearest, max_cost_points)
        x_new = tuple(x_new)  # Converts to tuple

        radius = (math.log(i) / n) ** (1 / d_parameter) # Calculate radius for rewire

        # Verify the safety of the connection
        if safety_verification_brehensam(costmap, x_new, x_nearest, max_cost):
            x_min = x_nearest
            # Calculate the cost of the path
            path, cost = dijkstra_path(G, tuple(start_point), tuple(x_nearest)) 
            mincost = cost + local_cost(x_nearest, tuple(x_new), tuple(costmap))

            # Find neighbors of x_new
            x_neighbors = points_within_radius(G.nodes, x_new, radius)

            for x_near in x_neighbors:
                x_near = tuple(x_near) # Convert x_near to tuple
                # Calculate the cost of the path
                path, tempcost = dijkstra_path(G, tuple(start_point), tuple(x_near)) 
                tempcost = tempcost+ local_cost(x_near, x_new, costmap)
                if tempcost < mincost and safety_verification_brehensam(costmap, x_near, x_new, max_cost):
                    x_min, mincost = x_near, tempcost
            
            # Add the node and edge to the graph
            G.add_node(tuple(x_new))
            G.add_edge(x_min, x_new, weight=local_cost(x_min, x_new, costmap))

            # Update the connections of the neighbors
            for x_near in x_neighbors:
                x_near = tuple(x_near) # Convert x_near to tuple
                # Calculate the cost of the path
                path, tempcost = dijkstra_path(G,  tuple(start_point), tuple(x_new)) 
                tempcost = tempcost+local_cost(x_near, x_new, costmap)
                path, length = dijkstra_path(G, tuple(start_point), tuple(x_near))
                if tempcost < length and safety_verification_brehensam(costmap, x_new, x_near, max_cost):
                    x_parent = parent(G, x_near, start_point, radius)
                    G.remove_edge(x_parent, x_near)
                    G.add_edge(x_new, x_near, weight=local_cost(x_new, x_near, costmap))
            if safety_verification_brehensam(costmap, goal_position, x_new, max_cost):
                G.add_node(tuple(goal_position))
                G.add_edge(tuple(goal_position), x_new, weight=local_cost(goal_position, x_new, costmap))
                return G 
    return G
def Optimal_Probabilistic_Roadmap(costmap, n, d_parameter, max_cost):
    # Create the graph
    G = nx.Graph()
    

    # Generate sampled points
    sampled_indices = weighted_posterior_sampling(costmap, n, max_cost)

    for i, x_rand in enumerate(sampled_indices, start=1):
        x_rand = tuple(x_rand)  # Convert x_rand to tuple
        radius = (math.log(i) / i) ** (1 / d_parameter) # Calculate radius for rewire
        G.add_node(tuple(x_rand))
        
        x_neighbors = points_within_radius(G.nodes, x_rand, radius*10)

        for x_near in x_neighbors:
            x_near = tuple(x_near) # Convert x_near to tuple
            # Calculate the cost of the path
            
            if safety_verification_brehensam(costmap, x_near, x_rand, max_cost):
                G.add_edge(x_near, x_rand, weight=local_cost(x_near, x_rand, costmap))

         
    return G
def parent(G, x, x_ancestor, radius):
    """
    Find the parent vertex of a node x in the graph G.

    Parameters:
        G (nx.Graph): The graph.
        x (tuple): The node x.
        x_ancestor (tuple): The ancestor node of x.
        radius (float): The radius for rewiring.

    Returns:
        tuple: The parent vertex of x.
    """
    #neighbors = points_within_radius(G.nodes, x, radius)
    neighbors= G.neighbors(x)
    min_cost = float('inf')
    parent_vertex = None

    for neighbor in neighbors:
        path, cost = dijkstra_path(G, tuple(neighbor), tuple(x_ancestor))
        if cost < min_cost:
            min_cost = cost
            parent_vertex = neighbor

    return parent_vertex