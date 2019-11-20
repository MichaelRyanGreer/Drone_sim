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
a       = [0,-1*pi/4,0]
adot    = [1,0,1]
p       = [0,0,90]
pdot    = [0,0,10]

myplane = Plane(20)

print(myplane.getX())

print(myplane.getPoints())

myplane.setX(a, adot, p, pdot)
myplane.set_controls(0,0,0,0)

fig = plt.figure()
ax = fig.add_subplot(111,projection='3d')

ax.set_xlim3d(-50,50)
ax.set_ylim3d(-50,50)
ax.set_zlim3d(0,100)

#ax.set_xlim3d(-10,10)
#ax.set_ylim3d(-10,10)
#ax.set_zlim3d(0,20)



drawing = None;

simlen = 100

#prev_states = np.zeros(3,simlen)


while(1):

    if (drawing):
        ax.collections.remove(drawing)
        ax.collections.remove(axis)

    # Run physics step
    
    myplane.physics_step(t_step=0.05)

    plot_points = np.array(myplane.getPoints())
    
    drawing = ax.scatter(plot_points[0],plot_points[1],plot_points[2],color='b')

    #center = ax.scatter(plot_points[0][0],plot_points[1][0],plot_points[2][0],color='r')

    a, adot, p, pdot = myplane.getX()

    rot_axis = np.dot(myplane.getRot(), np.transpose(adot)) + p

    axis = ax.scatter(rot_axis[0],rot_axis[1],rot_axis[2],color='g')

    # add to list of previous points

    plt.pause(0.01)

