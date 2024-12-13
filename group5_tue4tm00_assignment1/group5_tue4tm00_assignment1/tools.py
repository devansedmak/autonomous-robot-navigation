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
'''
def nearest_points_safe(nearest_points, center, r):
    # Computes the Free Local Space
    center = np.asarray(center)

    for point in nearest_points:
        point = safe_point(center, point, r) # Get the new point
    return nearest_points
'''