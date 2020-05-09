import numpy as np
import on_ground
from scipy.spatial import cKDTree

NUM_LOW_ROW_PTS = 50 # minimum number of LiDAR points from the lowest scanner 
                     # row which must lie on the ground plane for the certificate
                     # to be accepted by the interlock

# Parameters describing characteristics of an allowable ground plane
# More detail in on_ground.py
max_tilt = 0.1 
max_height = -.7

# Parameters for the required size and density of the high plane ("patch" square) points
patch_edge_size = 0.5 # meters
max_xpt_separation = 0.01 # meters - within every max_xpt_separation meters,
                          # there's at least one point

################################################################################

class Certificate:
    """
    Certificate for ground plane includes the plane itself (an instance of
    Plane, defined in on_ground.py), some points which are on the lowest
    LiDAR scanner row (to be confirmed to lie on the ground), and a representation
    of a "high patch," as a dictionary containing keys 'pts' and 'top_left', 
    the first of which is a list of points which lie on a dense region on the 
    ground plane above the lowest row (to be confirmed to lie on the ground 
    plane, densely), and the second of which is the tuple coordinate of the 
    top left of the rectangular patch.
    """

    def __init__(self, ground_plane, low_row_points, high_patch):
        self.ground_plane = ground_plane
        self.low_row_points = low_row_points
        self.high_patch = high_patch

################################################################################

def controller(lidar_points, pts_to_row, testing=False):
    """
    lidar_points: numpy array of 3D points of shape (number_of_points, 3)
    pts_to_row: dictionary mapping 3D points (tuples) to the row index of 
                the LiDAR scanner (0 is bottom)
    """
    ground_detector = on_ground.GroundDetector(lidar_points, max_height=max_height, max_tilt=max_tilt)
    ground_plane = ground_detector.plane

    def from_low_row(pt):
        """
        Determines whether this LiDAR point is a signed point
        from the lowest scanner row. Placeholder logic here
        for now--real LiDAR data should be signed and indicate
        its row.
         
        Copied in the interlock (bad coding practice, but indicates
        the eventual separation of the components)
        """
        return pts_to_row[tuple(pt)] == 0

    plane_points = [pt for pt in lidar_points if ground_plane.contains(pt)]
    low_row_points = [pt for pt in plane_points if from_low_row(pt)]

    ## get a dense patch of points on the ground plane ##

    # parameters for where the controller should look for the patch
    # (x and y refer to the LiDAR unit/robot's coordinate system)
    # call y the "forward/back" direction
    # call x the "left/right" direction
    patch_search_min_x = -10 # meters
    patch_search_max_x = 10
    patch_search_min_y = -10
    patch_search_max_y = 10

    
    # in the search region defined above, try candidate centers of 
    # a possible dense patch. Keep the most dense one. 
    
    tree = cKDTree(plane_points)

    # radius of a circle inscribed with a square with edge size patch_edge_size
    radius = patch_edge_size/(2**0.5)
    best_center, best_pts = None, []
    center_x = patch_search_min_x

    # average height used as z coordinate of candidate patch centers
    heights = [p[2] for p in plane_points]
    avg_height = sum(heights)/len(heights)

    while center_x < patch_search_max_x:
        center_y = patch_search_min_y
        while center_y < patch_search_max_y:
            # see how many points are near this candidate patch center
            near_pts = tree.query_ball_point([center_x, center_y, avg_height], radius)
            if len(near_pts) > len(best_pts):
                best_pts = near_pts
                best_center = center_x, center_y
            center_y += patch_edge_size/2
        center_x += patch_edge_size/2

    cert = Certificate(ground_plane, low_row_points, {'pts': [plane_points[i] for i in best_pts], 
                                                      'top_left': (center_x - patch_edge_size/2, 
                                                                   center_y - patch_edge_size/2)})
    if testing:
        return cert, plane_points
    return cert

################################################################################

def interlock(certificate):
    """
    Returns whether or not the certificate certifies a valid ground plane.
    """
    ground_plane, low_row_points, high_patch = certificate.ground_plane, certificate.low_row_points, certificate.high_patch

    ### CHECK THE PLANE IS REASONABLE

    if not ground_plane.small_angle(max_tilt):
        return False # too tilted
    if not ground_plane.point[0][2] < max_height:
        return False # too high


    ### CHECK THE LOW-LYING POINTS

    def from_low_row(pt):
        """Confirms that this LiDAR point is a signed point
        from the lowest scanner row. Placeholder here
        for now--real LiDAR data should be signed and indicate
        its row.
         
        Copied in the controller (bad coding practice, but indicates
        the eventual separation of the components)
        """
        return True

    # check that the low points are on the plane
    count_on_ground = 0
    for pt in low_row_points:
        if from_low_row(pt) and ground_plane.contains(pt):
            count_on_ground += 1


    ### CHECK THE PATCH

    pts, top_left = high_patch['pts'], high_patch['top_left']
    rect_left, rect_top = top_left
    rect_right = rect_left + patch_edge_size
    rect_bot = rect_left + patch_edge_size

    # check the patch points are all on the ground plane
    if not all(ground_plane.contains(pt) for pt in pts):
        return False

    # check sufficient horizontal density on the patch
    def check_patch_density(pts):
        # 1. project the points onto the flat plane
        def projected_pt(pt):
            # simply ignore the tilt of the ground plane
            # (it is sufficiently close to horizontal)
            return  pt[1], pt[2]

        flat_pts = [projected_pt(row) for row in pts]

        # 2. identify rows of data
        data = {} # maps coordinate to the "row" (scan) list 
                  # of horizontal-only (1D) data
        dist_bt_rows = 0.0345 # meters -- approximate estimated distance
                              # between the LiDAR
        for pt in flat_pts:
            for row_height in data:
                if abs(pt[1]-row_height) < dist_bt_rows/2:
                    data[row_height].append([pt[0]])
                    break
            else:
                data[pt[1]] = [[pt[0]]]

        #3. check each row has enough x-density of points
        for _, subdata in data.items():
            subdata = sorted(subdata)
            for pt1, pt2 in zip(subdata, subdata[1:]):
                if pt1[0] < rect_left or pt2[0] > rect_right: 
                    # outside the considered region
                    continue
                if abs(pt1[0]-pt2[0]) > max_xpt_separation:
                    return False

        return True

    return count_on_ground >= NUM_LOW_ROW_PTS and check_patch_density(pts)
            

    