import matplotlib.pyplot as plt
import cv2
import numpy as np
import pickle

from birdseye import BirdsEye
from lanefilter import LaneFilter
from curves import Curves
from helpers import roi
import sys
from scipy import signal
import random
import math

scale_factor = 1.0 / 4

MAX_Y = 145
MIN_Y = 30
def convolve(left, fit, img): # ypts from 145 to 26
    # plt.imshow(img)
    # plt.show()
    filter_fit = np.copy(fit)
    h = MAX_Y - MIN_Y - 1
    
    ypts = np.arange(MIN_Y, MAX_Y)

    filter_p = np.poly1d(filter_fit)
    xpts = filter_p(ypts)

    min_x = np.amin(xpts)
    w = int(np.amax(xpts) - min_x)
    # print('ypts', ypts)
    # print('xpts', xpts)

    # filter values are heavier towards the bottom of the filter
    filter_values = np.arange(100, 255, 155.0/(h-1))

    padding = 10
    fill_val = -1
    filter = np.full((h + padding, w + padding), fill_val)
    # print(filter)
    for i in range(len(filter_values)):
        # subtracts min_x to account for offset given by fit
        filter[(ypts[i] + padding/2.0 - MIN_Y).astype(int), (xpts[i] - min_x + padding/2.0).astype(int)] = filter_values[i]

    blur = np.full([4, 4], 1.0 / 16)
    filter = signal.convolve2d(filter, blur, fillvalue=fill_val)

    plt.title('Filter')
    plt.imshow(filter)
    plt.show()

    # print('boundary is at: ',int(img.shape[1]/2))

    half_width = int(img.shape[1]/2)
    if left:
        img = img[:, :half_width]
    else:
        img = img[:, half_width + 1:]
    #
    plt.imshow(img)
    plt.show()

    grad = signal.correlate2d(filter, img, mode='same', fillvalue=fill_val)

    result = np.unravel_index(grad.argmax(), grad.shape)
    print('result is: ', result)

    print('max val is: ', grad[result[0]][result[1]])
    print('location of max for solid is: ', result)

    p = np.poly1d(fit)
    plt.title('Correlational Output')
    plt.imshow(grad, cmap='gray')
    plt.show()

    image_y = result[0] - padding//2 + MIN_Y
    image_x = result[1] - padding//2 + min_x
    # offset = int(p(result[0]) - image_x)
    offset = int(filter.shape[1] / 2) - (filter_p(image_y) - min_x + padding / 2)

    if left:
        # print('actual middle x is: ', result[1])
        # print("filter shape is: ", filter.shape)
        # print('xpts[result[0]] is: ', xpts[result[0]])
        actual_x = image_x #* (1/scale_factor)
        expected_x = filter_p(image_y)#* (1/scale_factor))
    else:
        actual_x = (image_x)#* (//scale_factor)
        expected_x = filter_p(image_y)#* (1/scale_factor))
    print('image_y', image_y)
    print('actual',actual_x)
    print('expected',expected_x)
    print('offset', offset)
    # plt.imshow(img)
    # plt.show()

    if abs(actual_x - expected_x) < 25 and grad[result[0]][result[1]] > 1250:
        # print('its solid')
        return "pass"
    return False