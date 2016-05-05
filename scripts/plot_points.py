from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot
import numpy as np
from cv2 import cv

fig = pyplot.figure()
ax = Axes3D(fig)

filepath = "/home/sparus/.ros/calibration.yaml"
P = np.array(cv.Load(filepath, cv.CreateMemStorage(),"laser_plane"))
points = np.array(cv.Load(filepath, cv.CreateMemStorage(),"points"))


ax.scatter(points[:,0],points[:,1],points[:,2])
i = 0
x = np.arange(0.005,0.025,0.005)
y = np.arange(-0.1,0.1,0.05)
x,y = np.meshgrid(x,y)
z = - (P[i,0]*x + P[i,1]*y + P[i,3])/P[i,2]
ax.plot_surface(x, y, z,  rstride=4, cstride=4)

pyplot.show()
