import random 

"""Generates fake distance-from-scanner data"""

def two_dim_points(num_points=4000, max_dist=5, percent_outliers=0.05):
    """Interesting 2-dimensionsal points with some outliers"""
    points = []
    for _ in range(num_points):
        point = 3*max_dist/4.0 + random.random()*max_dist/4.0
        random_condition = random.random() < percent_outliers
        if random_condition:
            point = random.random()*max_dist # could be an outlier
        points.append(point)
    return points

def three_dim_points(num_pts_per_row=1000, num_rows=24, max_dist=5, percent_outliers=0.05):
    """Interesting 3-dimensionsal points with some outliers"""
    points = []
    for _ in range(num_rows):
        row = two_dim_points(num_pts_per_row, max_dist, percent_outliers)
        points.append(row)
    return points

def two_dim_points_pedestrian(num_points=4000, max_dist=5, percent_outliers=0.05, ped_dist=3, left=1600, right=2400):
    """Interesting 2-dimensionsal points with some outliers"""
    points = []
    for i in range(num_points):
        if left < i < right:
            # point = ped_dist
            point = 3*ped_dist/4.0 + random.random()*ped_dist/4.0
            random_condition = random.random() < percent_outliers
            if random_condition:
                point = random.random()*ped_dist # could be an outlier
        else:
            point = 3*max_dist/4.0 + random.random()*max_dist/4.0
            random_condition = random.random() < percent_outliers
            if random_condition:
                point = random.random()*max_dist # could be an outlier
        points.append(point)
    return points

def three_dim_points_pedestrian(num_pts_per_row=1000, num_rows=24, max_dist=5, percent_outliers=0.05, top=15, bottom=3, ped_dist=2):
    """Interesting 3-dimensionsal points with some outliers"""
    points = []
    for row_i in range(num_rows):
        if bottom < row_i < top:
            left = num_pts_per_row/3.0 - 1/16.0*num_pts_per_row
            right = num_pts_per_row/3.0 + 1/16.0*num_pts_per_row
            row = two_dim_points_pedestrian(num_pts_per_row, max_dist, percent_outliers, ped_dist, left, right)
        else:
            row = two_dim_points(num_pts_per_row, max_dist, percent_outliers)
        points.append(row)
    return points