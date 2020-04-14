import numpy as np
import math
import statistics

def shape(left_fit, right_fit, h):
    # ax**2 + bx + c
    la, lb, lc = left_fit
    ra, rb, rc = right_fit
    x = np.arange(0, h - 1.0)
    lxs = la * (x ** 2.0) + lb * x + lc
    rxs = ra * (x ** 2.0) + rb * x + rc

    leftpoints = lxs
    rightpoints = rxs
    x_coords = x

    # check which point from right lane corresponds to the normal line created by left lane points
    lnormalslope = -1/(2.0*la*x+lb)
    diffs = []
    for xcoord,ycoord, m in zip(x_coords, leftpoints, lnormalslope):
        b = ycoord - m*xcoord # slope intercept for normal line
        
        newa = ra
        newb = rb-m
        newc = rc-b

        xsoln1 = int((-newb + math.sqrt(newb**2.0 - 4*newa*newc)) / (2.0 * newa))
        xsoln2 = int((-newb - math.sqrt(newb**2.0 - 4*newa*newc)) / (2.0 * newa))
        ysoln1 = ra*xsoln1**2.0 + rb*xsoln1 + rc
        ysoln2 = ra*xsoln2**2.0 + rb*xsoln2 + rc
        firstdiff = math.sqrt((xcoord - xsoln1)**2.0 + (ycoord - ysoln1)**2.0)
        seconddiff = math.sqrt((xcoord - xsoln2)**2.0 + (ycoord - ysoln2)**2.0)
        diff = min(firstdiff, seconddiff)
        diffs.append(diff)
    comparednum = len(diffs)
    stdev = statistics.stdev(diffs)
    mean = statistics.mean(diffs)

    if stdev > 1.0 or mean < 3.0 or mean > 6.0:
        return False
    return True
