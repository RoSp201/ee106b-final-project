import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

hv = np.load("human_vels.npy")
rv = np.load("robot_vels.npy")

time = [i for i in range(len(hv))]
ax = Axes3D(plt.gcf())
ax.plot3D(time,hv,rv)
plt.show()