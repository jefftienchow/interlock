import random 
import numpy as np

"""Generates fake (x,y,z) coordinate data"""

def points():
    pts = []
    for x in np.linspace(2, 2.5, 3): # depth from scanner
        for y in np.linspace(-1,1,500): # horizontal from scanner
            for z in np.linspace(0,2,14): # vertical from scanner
                pts.append([x, y, z])

    num_random_outliers = int(0.03 * len(pts))
    for _ in range(num_random_outliers):
        x = random.randrange(0, 5)
        y = random.randrange(-2, 2)
        z = random.randrange(0, 3)
        pts.append([x, y, z])

    return np.array(pts)