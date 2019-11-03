#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos


#!/usr/bin/env python
from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys
from math import sin, cos


# Plot a path in R3 with a unit square obstacle centered at the origin
def plotR2(path):
    fig = plt.figure()
    ax = fig.gca()

    # Plotting the path
    X = [p[0] for p in path]
    Y = [p[1] for p in path]
    ax.plot(X, Y)

    x = -1.0
    y = 0.8
    w = 2
    h = 0.2
    ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20'))
    x = -1
    y = -0.5
    w = 1.2
    h = 1.0
    ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20'))
    x = 0.5
    y = -0.5
    w = 0.5
    h = 1.0
    ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20'))
    x = -1.0
    y = -1.0
    w = 2.0
    h = 0.2
    ax.add_patch(patches.Polygon([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], fill=True, color='0.20'))
    ax.set_ylim([-1, 1])
    ax.set_xlim([-1, 1])
    plt.show()




# Read the cspace definition and the path from filename
def readPath(filename):
    lines = [line.rstrip() for line in open(filename) if len(line.rstrip()) > 0]

    if len(lines) == 0:
        print "That file's empty!"
        sys.exit(1)


    data = [[float(x) for x in line.split(' ')] for line in lines[1:]]
    return data

if __name__ == '__main__':
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = 'car_path.txt'

    path = readPath(filename)
    plotR2(path)


