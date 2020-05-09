# Only works for Python 3.6 or above
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import time

def plot(points):
    fig = plt.figure()
    plt.axis('equal')
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter([p[0] for p in points], 
               [p[1] for p in points],
               [p[2] for p in points])
    plt.savefig(f'pics/{time.time()}.png')
    plt.show()

def double_plot(all_pts, base_pts, to_highlight_pts):
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')
    ax._axis3don = False

    alpha = 0.5   
    red = [1., alpha, alpha]
    green = [alpha, 1., alpha]
    blue = [alpha, alpha, 1.]

    to_highlight_pts = [list(pt) for pt in to_highlight_pts]
    base_pts = [list(pt) for pt in base_pts if not list(pt) in to_highlight_pts]

    for pts, color, alpha, s in [(all_pts, green, 0.05, 1),
                                 (base_pts, red, 0.2, 1),
                                 (to_highlight_pts, blue, 1, 1)
                                ]:
      ax.scatter([p[0] for p in pts], 
                 [p[1] for p in pts],
                 [p[2] for p in pts], marker='o', color=color, alpha=alpha, s=s)

    plt.savefig(f'pics/{time.time()}.png')
    plt.show()
