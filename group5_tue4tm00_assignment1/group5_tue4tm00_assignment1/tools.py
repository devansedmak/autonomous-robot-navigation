import numpy as np
from core_proj_nav_ctrl import proj_nav_tools

def gradient(x, p, r):
    if np.linalg.norm(x - p) >= r:
        return 2 * (x - p)
    else:
        return 0

def safe_point(x, p, r):
    grad=gradient(x,p,r)
    d=(grad/np.linalg.norm(grad))*r+p
    return d

def polygon_convex_interior_safe(polygon, center, r):

    polygon = np.asarray(polygon)
    center = np.asarray(center)

    nearest_points = proj_nav_tools.local_nearest(polygon, center)

    convex_interior = polygon
    for point in nearest_points:
        point = safe_point(center, point, r) # Get the new point
        convex_interior = proj_nav_tools.polygon_intersect_halfplane(convex_interior, point, center-point)
    print("ciao")
    return convex_interior