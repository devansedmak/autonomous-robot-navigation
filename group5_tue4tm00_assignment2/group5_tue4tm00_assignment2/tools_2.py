import numpy as np
from group5_tue4tm00_assignment1 import tools
from core_path_follow_ctrl import path_follow_tools


def path_goal_support_corridor_safe(path, center, boundary, r):

    path = np.asarray(path)
    if path.ndim < 2:
        path = path.reshape((1,-1))

    center = np.asarray(center).reshape((-1, path.shape[1]))
    boundary = np.asarray(boundary).reshape((-1, path.shape[1]))

    xline_start = path[:-1]
    xline_end = path[1:]
    for point in boundary:
        point1=point
        point = tools.safe_point(center, point, r) # Get the new point
        if np.linalg.norm(center-point) <= r:
            center1 = tools.safe_point1(center, point1, r)
            xline_start, xline_end = path_follow_tools.line_intersect_halfplane(xline_start, xline_end, point, center1-point)
        else:
            xline_start, xline_end = path_follow_tools.line_intersect_halfplane(xline_start, xline_end, point, center-point)

    if xline_end.shape[0] > 0:
        goal = xline_end[-1]
    else:
        goal = None

    return goal