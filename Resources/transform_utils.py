import math

def rad(deg):
    return deg*math.pi/180

im_theta_y = rad(16)
im_theta_x = rad(33) # degrees
lidar_theta_x = rad(180) # degrees
lidar_theta_y = rad(15) # degrees
LIDAR_ANGLE_SPANS = (lidar_theta_x, lidar_theta_y)

def pt_to_lidar_angle(im_point, im_dims):
    """
    im_point : (x, y) where x is the number of pixels to the
                right of the image's top left, y is the number
                of pixels from the image's top--(0,0) is TL corner
    im_dims : (width, height) in number of pixels
    """
    x, y = im_point
    width, height = im_dims

    def get_angle(half_dim, pt, im_angle):
        dim_ratio = float(pt - half_dim)/half_dim
        return math.atan2(dim_ratio * math.tan(im_angle), 1)

    return get_angle(width/2.0, x, im_theta_x), \
           get_angle(height/2.0, y, im_theta_y)
           

def lidar_angle_to_dist(theta_x, theta_y, angle_spans, lidar):
    x_span, y_span = angle_spans

    def angle_to_bucket(theta, n_buckets, angle_span):
        """
        Finds closest bucket that this angle falls into. 
        'Bucket' is a row or column. Returns 0 for the 
        top or leftmost bucket.

        angle_span is the angle, in degrees, away from 
        the center point to which the n_buckets buckets 
        are equally spaced. In other words, the angle
        from the center of the outermost bucket.
        """
        return int(round((angle_span + theta) * (n_buckets) / (2.0 * angle_span)))

    row = min(len(lidar) - 1, 
              max(0, len(lidar) - angle_to_bucket(theta_y, 
                                                  len(lidar), 
                                                  y_span)))

    def get_center_i(row):
        # get the index of the lidar point IN FRONT OF YOU (x>0),
        # whose y value is smallest (the actual center)
        center_y, center = float('inf'), 0
        for i, (x, y, _) in enumerate(row):
            if abs(y) < abs(center_y) and x > 0:
                center_y = y
                center = i
        return center

    center_i = get_center_i(lidar[row])
    shift = int(center_i - (len(lidar[row])-1)/2.0)

    col = angle_to_bucket(theta_x, len(lidar[row]), x_span) + shift

    return lidar[row][col]

def transform(pt, im_dims, lidar):
    temp = pt_to_lidar_angle(pt, im_dims)
    return lidar_angle_to_dist(temp[0], temp[1], LIDAR_ANGLE_SPANS, lidar)