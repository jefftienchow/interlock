import math

"""
Utility to produce the image-coordinate y-values of the lidar
sensor row scans.
"""

def rad(deg):
    return deg*math.pi/180.0

lidar_theta_x = rad(180) # Velodyne puck spans -180 to 180 degrees horizontally
lidar_theta_y = rad(15) # Velodyne puck spans -15 to 15 degrees vertically
LIDAR_ANGLE_SPANS = (lidar_theta_x, lidar_theta_y)


def get_lidar_ys(self, im_dims, nlidar_rows):
    """Returns list of y-values in the image which best correspond 
       to lidar data rows
    """
    lidar_angle = {i: rad(15-30/16.0*i) for i in range(16)}
    def get_y_from_ang(ang):
        return 240* 0.297 * math.tan(ang) / (0.297*math.tan(rad(16)))
    return [int(get_y_from_ang(ang)+240) for ang in lidar_angle.values()]


if __name__ == '__main__':

    # Example use of transformation from image to lidar
    im_dims = (480,640) # pixels
    y_vals = get_lidar_ys(im_dims,16)
