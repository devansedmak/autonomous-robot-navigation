import numpy as np
from core_proj_nav_ctrl import proj_nav_tools

def gradient(x, p, r):
    # Compute the gradient
    grad_norm = np.linalg.norm(x - p)
    if grad_norm == 0:
        return (x-p)
    else:
        return (x-p)*(1/grad_norm)

def U_repulsive(polygon, center, r):
    # Repulsive field
    nearest_points = proj_nav_tools.local_nearest(polygon, center)
   
    U =  [0.0,0.0]
    U = np.asarray(U)
    for point in nearest_points:
        U1=  [0.0,0.0]
        if np.linalg.norm(center-point) <= 2*r:
        
            if np.linalg.norm(center-point) <= (r):
                U1 =  1.4 * gradient(center, point, r)
            else:
                #U1=[2*r,2*r]-(center-point)
                U1 = gradient(center, point, r) * (1.4*np.linalg.norm(center-point)/r+2.8) 
        else:
           U1=[0.0,0.0]
        U=U+U1 # Sum all the repulsive gradients
        if np.linalg.norm(U)>=1.4:
            U=U/np.linalg.norm(U)*1.4
    return U

def safe_point(x, p, r):
    # Computes the new shifted point  
    grad = gradient(x,p,r)
    d = grad*r+p
    return d

def safe_point1(x, p, r):
    # Finds a point that is 2*r distant from the border in the direction for the gradient (x-p)
    grad = gradient(x,p,r)
    d = grad*2*r+p
    return d

def polygon_convex_interior_safe(polygon, center, r):
    # Computes the Free Local Space
    polygon = np.asarray(polygon)
    center = np.asarray(center)

    nearest_points = proj_nav_tools.local_nearest(polygon, center)

    convex_interior = polygon
    for point in nearest_points:
        point1=point
        point = safe_point(center, point, r) # Get the new point
        if np.linalg.norm(center-point) <= r:
            center1 =safe_point1(center, point1, r)
            convex_interior = proj_nav_tools.polygon_intersect_halfplane(convex_interior, point, center1-point)
        else:
            convex_interior = proj_nav_tools.polygon_intersect_halfplane(convex_interior, point, center-point)

    return convex_interior

def polygon_convex_interior_safe_new(polygon, nearest_points, center, r):
    # Computes the Free Local Space
    polygon = np.asarray(polygon)
    center = np.asarray(center)

    convex_interior = polygon
    for point in nearest_points:
        point1=point
        point = safe_point(center, point, r) # Get the new point
        if np.linalg.norm(center-point) <= r:
            center1 =safe_point1(center, point1, r)
            convex_interior = intersect_polygon_halfplane(convex_interior, point, center1-point)
        else:
            convex_interior = intersect_polygon_halfplane(convex_interior, point, center-point)
        convex_interior = np.array(convex_interior)
    
    num_nearest_points = nearest_points.shape[0]
    num_vertices = convex_interior.shape[0]

    if (num_vertices-num_nearest_points) > 6:
        sampling_interval = int(num_vertices / 6)  # Ensure at least 6 points are sampled

        final_convex = []
        for i in range(6):
            final_convex.append(safe_point(center, convex_interior[i*sampling_interval, :], r))  # Select the

        for point in nearest_points:
            point1=point
            point = safe_point(center, point, r) # Get the new point
            if np.linalg.norm(center-point) <= r:
                print("Too close to wall")
                center1 =safe_point1(center, point1, r)
                point = safe_point(center1, point, r)
                final_convex.append(point)
            else:
                final_convex.append(point)
    else:
        final_convex = []
        for point in nearest_points:
            point1=point
            point = safe_point(center, point, r) # Get the new point
            if np.linalg.norm(center-point) <= r:
                print("Too close to wall")
                center1 =safe_point1(center, point1, r)
                point = safe_point(center1, point, r)
                final_convex.append(point)
            else:
                final_convex.append(point)
        for i in range(num_vertices-num_nearest_points):
            final_convex.append(safe_point(center, convex_interior[i, :], r))  # Select the


    return np.asarray(final_convex)



def find_intersection(polygon, center):
    
    """
    Find the intersection of the lines perpendicular to the polygon's edges,
    passing through the polygon's vertices.

    Args:
        polygon (list of tuple): List of points representing the polygon's vertices (x, y).
        center (tuple): Central point (x, y).

    Returns:
        tuple: Intersection point (x, y) of the perpendicular lines.
    """
    intersection=[]
    def perpendicular_line(segment, point):
        (x1, y1), (x2, y2) = segment
        if x2 - x1 != 0:  # Avoid division by zero
            m_segment = (y2 - y1) / (x2 - x1)
            m_perpendicular = -1 / m_segment
        else:  # Vertical segment -> perpendicular line is horizontal
            m_perpendicular = 0
        
        x0, y0 = point
        q = y0 - m_perpendicular * x0
        return m_perpendicular, q

    n = len(polygon)
    if n < 2:
        raise ValueError("The polygon must have at least two points.")

    
    n = len(polygon)

    for i in range(n):
        point1 = polygon[i]
        point2 = polygon[(i + 1) % n]  # Use modulo to close the polygon
        # Create segments connecting the center to two consecutive points
        segment1 = (point1, center)
        segment2 = (point2, center)
        line1 = perpendicular_line(segment1, point1)
        line2 = perpendicular_line(segment2, point2)
        line1 = perpendicular_line(segment1, point1)
        line2 = perpendicular_line(segment2, point2)
        intersection.append( intersect_line(line1, line2) )

    
    return intersection



def intersect_polygon_halfplane(polygon, point, normal):
    """
    Calculate the intersection between a polygon and a half-plane.

    Args:
        polygon (list of list/tuple): Vertices of the polygon (in clockwise or counterclockwise order).
        point (array-like): A point on the half-plane.
        normal (array-like): The normal vector of the half-plane.

    Returns:
        list: Vertices of the new intersected polygon.
 
    """
    def is_inside(vertex):
        """Determines whether a point is inside the half-plane"""
        return np.dot(vertex - point, normal) >= 0

    def line_intersection(p1, p2, plane_point, plane_normal):
        """Calculate the intersection point between a segment and the edge of the half-plane."""
        d = p2 - p1
        t = np.dot(plane_point - p1, plane_normal) / np.dot(d, plane_normal)
        return p1 + t * d

    polygon = np.array(polygon)
    point = np.array(point)
    normal = np.array(normal)

    new_polygon = []
    for i in range(len(polygon)):
        current_vertex = polygon[i]
        next_vertex = polygon[(i + 1) % len(polygon)]

        current_inside = is_inside(current_vertex)
        next_inside = is_inside(next_vertex)

        if current_inside:
            new_polygon.append(current_vertex.tolist())

        if current_inside != next_inside:
            # Calculate the intersection point with the edge of the half-plane
            intersection = line_intersection(current_vertex, next_vertex, point, normal)
            new_polygon.append(intersection.tolist())

    return new_polygon


def intersect_line(line1, line2):
    """
    Find the intersection point between two lines given by the equations y = m1*x + q1 and y = m2*x + q2.

    Args:
        line1 (tuple): The first line in the form (m1, q1).
        line2 (tuple): The second line in the form (m2, q2).

    Returns:
        tuple: Intersection point (x, y).
    """
    m1, q1 = line1
    m2, q2 = line2

    if m1 == m2:
        raise ValueError("The lines are parallel and have no intersection.")

    # Calculate the intersection point
    x = (q2 - q1) / (m1 - m2)
    y = m1 * x + q1
    return x, y




