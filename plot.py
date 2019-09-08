from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import random

import numpy as np

points = np.float32([
            [0, 0, 0],
            [100, 0, 0],
            [0, 100, 0], 
            [100, 100, 0]])


points_x = []
points_y = []
points_z = []

for point in points:
	points_x.append(point[0])
	points_y.append(point[1])
	points_z.append(point[2])
	
# print(points_x)	
# print(points_y)	
# print(points_z)	

new_point = np.float32([[30,30,0]])
# points_x.append(new_point[0][0])
# points_y.append(new_point[0][1])
# points_z.append(new_point[0][2])



fig = pyplot.figure()
ax = Axes3D(fig)

# sequence_containing_x_vals = list(range(0, 100))
# sequence_containing_y_vals = list(range(0, 100))
# sequence_containing_z_vals = list(range(0, 100))

# random.shuffle(sequence_containing_x_vals)
# random.shuffle(sequence_containing_y_vals)
# random.shuffle(sequence_containing_z_vals)

# print(sequence_containing_x_vals)
# exit()

try:
	ax.scatter(points_x, points_y, points_z,c='blue')
	ax.scatter(new_point[0][0], new_point[0][1], new_point[0][2],c='red')
	ax.set_zlim(bottom=0)
	pyplot.show()
except Exception as e:
	raise e




