from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot
import numpy as np
import os
import yaml
import re
def opencv_matrix(loader, node):
    mapping = loader.construct_mapping(node, deep=True)
    mat = np.array(mapping["data"])
    mat.resize(mapping["rows"], mapping["cols"])
    return mat
yaml.add_constructor(u"tag:yaml.org,2002:opencv-matrix", opencv_matrix)
def readYAMLFile(fileName):
  ret = {}
  skip_lines=1    # Skip the first line which says "%YAML:1.0". Or replace it with "%YAML 1.0"
  with open(fileName) as fin:
    for i in range(skip_lines):
      fin.readline()
    yamlFileOut = fin.read()
    myRe = re.compile(r":([^ ])")   # Add space after ":", if it doesn't exist. Python yaml requirement
    yamlFileOut = myRe.sub(r': \1', yamlFileOut)
    ret = yaml.load(yamlFileOut)
  return ret

file_name = os.environ['HOME'] + "/.ros/calibration.yaml"
result = readYAMLFile(file_name)
P = result['laser_plane']
points = result['points']

fig = pyplot.figure()
ax = Axes3D(fig)
ax.scatter(points[:,0],points[:,1],points[:,2], color=(1., 0., 0., 0.5))
i = 0
x = np.arange(-0.2,0.2,0.01)
y = np.arange(0.06,0.2,0.01)
x,y = np.meshgrid(x,y)
z = - (P[i,0]*x + P[i,1]*y + P[i,3])/P[i,2]
ax.plot_wireframe(x, y, z,  rstride=4, cstride=4)

pyplot.show()
