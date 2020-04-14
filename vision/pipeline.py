import numpy as np

from lanefilter import LaneFilter
from conv import convolve
from shape import shape

SCALE_FACTOR = .5
_PATH_SIZE = 192.0
_PATH_XD = np.arange(192.0)

def get_binary(img, thresholds):
    """
    Applies transformations to process the given image
    :param img: the given image
    :param birds_eye: the bird's eye transformation object
    :param thresholds: thresholds used for vision filters
    :return: the processed image
    """
    lane_filter = LaneFilter(thresholds)
    binary = lane_filter.apply(img)
    return binary.astype(np.uint8)

def draw_lines(img, uv_model_dots):
    for i, j  in ((-1, 0), (0, -1), (0, 0), (0, 1), (1, 0)): # makes it look like stars
        img[uv_model_dots[:, 1] + i, uv_model_dots[:, 0] + j] = (0, 255,0) 

def warp_points(pt_s, warp_matrix):
    # pt_s are the source points, nxm array.
    pt_d = np.dot(warp_matrix[:, :-1], pt_s.T) + warp_matrix[:, -1, None]

    # Divide by last dimension for representation in image space.
    return (pt_d[:-1, :] / pt_d[-1, :]).T

def get_pts(img, path, calibration):

    uv_model_real = warp_points(np.column_stack((_PATH_XD, path)), calibration)
    uv_model = np.round(uv_model_real).astype(int)
    uv_model_dots = uv_model[np.logical_and.reduce((np.all(  # pylint: disable=no-member
        uv_model > 0, axis=1), uv_model[:, 0] < img.shape[1]/SCALE_FACTOR - 1, uv_model[:, 1] <
                                                    img.shape[0]/SCALE_FACTOR - 1))]
    return uv_model_dots

def interlock(img, lines, calibration, thresholds):
    '''
    runs the actual vision interlock; the parameters are the components that create the `certificate`
    :param img: image from the controller
    :param birds_eye: bird's eye transformation object from the controller
    :param thresholds: thresholds used for vision filters
    :param lines: proposed lane lines
    :return: the result of each vision interlock test
    '''
    wb = get_binary(img, thresholds)

    left_fit = np.polyfit(_PATH_XD, lines[0],2)
    right_fit = np.polyfit(_PATH_XD, lines[1],2)

    left_pts = (get_pts(img, lines[0], calibration) * SCALE_FACTOR).astype(int)
    right_pts = (get_pts(img, lines[1], calibration) * SCALE_FACTOR).astype(int)

    draw_lines(img, left_pts)
    draw_lines(img, right_pts)

    shape_result = shape(left_fit, right_fit, _PATH_SIZE)

    left_trans_fit = np.polyfit(left_pts[:,1], left_pts[:,0], 2) # :,1 is the y axis
    right_trans_fit = np.polyfit(right_pts[:,1], right_pts[:,0], 2)

    left_result = convolve(True, left_trans_fit, wb)
    right_result = convolve(False, right_trans_fit, wb)

    return shape_result, left_result, right_result
