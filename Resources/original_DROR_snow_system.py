from scipy.spatial import cKDTree
import math
import numpy as np
import time

"""
This system is meant for use with the multi-row lidar scanner.
"""
global min_dist_away
global rect_dist
global rect_left
global rect_right
global rect_bot
global rect_top
global dist_bt_rows
global max_xpt_separation

min_dist_away = 1 # meters

rect_dist = 1 # meter distance in front of scanner
rect_left = -0.25 # meters
rect_right = 0.25
rect_bot = 0.0
rect_top = 0.1

dist_bt_rows = 0.0345 # meters; depends on rect_dist
max_xpt_separation = 0.01 # meters -- HORIZONTALLY within every max_xpt_separation meters,
                          # there's at least point (no obstacle with this width); depends
                          # on rect_dist

################################################################################

class Certificate:
    def __init__(self, points, action):
        self.action = action
        self.points = points # np.ndarray of 3D points (shape ix3)

################################################################################

def distance(pt, pt2=(0,0,0)):
    """Distance of a 3D point from another 3D point, or the origin"""
    x, y, z = pt
    x2, y2, z2 = pt2
    return ((x-x2)**2 + (y-y2)**2 + (z-z2)**2)**0.5

################################################################################

def snow_removed(pts):
    min_search_rad = 0.04 # tune! (this value from "Denoising of LiDAR point cloud" paper)
    min_neighbors = 3 # must be >1 (includes the point itself)
    beta_mult = 3 # 6 removes ~the same number of points as the SOR filter (but paper uses 3)

    points_per_row = 1550 # approximately
    angular_res = 2*math.pi/points_per_row # horizontal angular resolution of the lidar

    tree = cKDTree(pts)

    search_radii = np.array([np.clip(beta_mult * angular_res * np.linalg.norm(pts, axis=1), \
                            min_search_rad, None)]).T

    global index 
    index = 0
    def is_not_snow(pt):
        global index
        # with Scipy 1.3.0+ (which requires Python 3.5+), can use the 
        # query_ball_point return_length tag
        result = len(tree.query_ball_point(pt, search_radii[index][0])) >= min_neighbors
        index += 1
        return result

    return np.apply_along_axis(is_not_snow, 1, pts)

################################################################################

def filtered(pts):
    is_not_snow = snow_removed(pts)

    def is_far_enough(pt):
        return distance(pt) >= min_dist_away
    is_far_enough_mask = np.apply_along_axis(is_far_enough, 1, pts) # bool mask

    return pts[np.logical_and(is_not_snow, is_far_enough_mask)]

def controller(lidar_points):
    """
    Produces a Certificate of points if deemed safe to proceed forward;
    otherwise produces an empty Certificate with action False.

    lidar_points is a numpy array of 3D points
    """

    proj_lidar_points = rect_dist * lidar_points / np.linalg.norm(lidar_points, axis=1)[:,None]

    lidar_points = lidar_points[np.multiply(np.multiply(np.logical_and(rect_left <= proj_lidar_points[:,1], 
                                                                       proj_lidar_points[:,1] <= rect_right), 
                                                        proj_lidar_points[:,0] >= 0), 
                                            np.logical_and(rect_bot <= proj_lidar_points[:,2],
                                                            proj_lidar_points[:,2]  <= rect_top))]

    npoints_before_filtering = len(lidar_points)
    lidar_points = filtered(lidar_points)

    # check that at least 80% of points weren't snow outliers
    # or too close -- indicates maybe too much snow filtering or
    # an obstacle (In reality, we'd need a much smarter object
    # detection neural net controller.)
    if len(lidar_points) < 0.8*npoints_before_filtering:
        return Certificate(lidar_points, False)

    # certificate contains the far enough, non-snow points
    return Certificate(lidar_points, True)

################################################################################

def interlock(certificate):
    """Returns whether the controller is allowed to keep going.
       Specially designed for the very striped nature of lidar scanner!!!
    """
    if not certificate.action:
        return False

    pts = certificate.points

    # 0. check no point is too close
    for pt in pts:
        if distance(pt) < min_dist_away:
            return 'False--a point is too close' # False

    # 1. project the points
    def projected_flattened_pt(pt):
        mag = distance(pt)
        distance_wanted = rect_dist # makes it a flat (not
                                    # slightly curved) rectangle
        return  distance_wanted/mag*pt[1], \
                distance_wanted/mag*pt[2]

    flat_pts = np.array([projected_flattened_pt(row) for row in pts])

    # 2. identify rows of data
    data = {} # maps height (from ground) to the "row" (scan) 
              # list of horizontal-only (1D) data
    for pt in flat_pts:
        for row_height in data:
            if abs(pt[1]-row_height) < dist_bt_rows/2:
                data[row_height].append([pt[0]])
                break
        else:
            data[pt[1]] = [[pt[0]]]

    #3. check there are enough rows
    if not len(data) >= int((rect_top-rect_bot) / dist_bt_rows)-2:
        # not enough rows of data in the certificate
        return False 

    #4. check each row has enough x-density of points
    for _, subdata in data.items():
        subdata = sorted(subdata)
        for pt1, pt2 in zip(subdata, subdata[1:]):
            if pt1[0] < rect_left or pt2[0] > rect_right: 
                # outside the lane
                continue
            if abs(pt1[0]-pt2[0]) > max_xpt_separation:
                return False

    return True
