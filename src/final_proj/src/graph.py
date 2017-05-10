import matplotlib.pyplot as plt
import pandas
from mpl_toolkits.mplot3d import Axes3D

import numpy as np

hv = np.load("human_vels.npy")
rv = np.load("robot_vels.npy")

time = [i for i in range(len(hv))]
ax = Axes3D(plt.gcf())
ax.plot3D(hv,rv,time)
# ax.set_xlabel('X Position')
# ax.set_ylabel('Y Position')
# ax.set_zlabel('Theta Position')
plt.legend()
plt.show()