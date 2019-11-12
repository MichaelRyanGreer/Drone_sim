import numpy as np
import math
import time

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from plane import Plane

matplotlib.interactive(True)

pi = math.radians(180)

# state
a       = [0,0,0]
adot    = [0,0,0]
p       = [0,0,0]
pdot    = [0,0,0]

myplane = Plane()

print(myplane.getX())

print(myplane.getPoints())

myplane.setX(a, adot, p, pdot)

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

ax.set_xlim3d(-5,5)
ax.set_ylim3d(-5,5)
ax.set_zlim3d(-5,5)

drawing = None;


for i in np.linspace(0,2*pi,100):

    a = [i,0,0]
    p = [math.sin(i),0,0]

    if (drawing):
        ax.collections.remove(drawing)

    myplane.setX(a, adot, p, pdot)

    plot_points = np.array(myplane.getPoints())
    
    drawing = ax.scatter(plot_points[0],plot_points[1],plot_points[2],color='b')

    plt.pause(0.01)

for i in np.linspace(0,2*pi,100):

    a = [0,i,0]
    p = [0,math.sin(i),0]

    if (drawing):
        ax.collections.remove(drawing)

    myplane.setX(a, adot, p, pdot)

    plot_points = np.array(myplane.getPoints())
    
    drawing = ax.scatter(plot_points[0],plot_points[1],plot_points[2],color='b')

    plt.pause(0.01)

for i in np.linspace(0,2*pi,100):

    a = [0,0,i]
    p = [0,0,math.sin(i)]

    if (drawing):
        ax.collections.remove(drawing)

    myplane.setX(a, adot, p, pdot)

    plot_points = np.array(myplane.getPoints())
    
    drawing = ax.scatter(plot_points[0],plot_points[1],plot_points[2],color='b')

    plt.pause(0.01)

