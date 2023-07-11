
import numpy as np
import matplotlib.pyplot as plt


data = np.loadtxt("/tmp/odom_tmp.csv", delimiter=',')
stamp = data[:,0]
rb = data[:, 1:4]
vb = data[:, 4:7]

fig, axs = plt.subplots(3, figsize=(10, 10))
axs[0].plot(stamp, rb[:,0], label="radar")
axs[0].plot(stamp, vb[:,0], label="vio")
axs[1].plot(stamp, rb[:,1], label="radar")
axs[1].plot(stamp, vb[:,1], label="vio")
axs[2].plot(stamp, rb[:,2], label="radar")
axs[2].plot(stamp, vb[:,2], label="vio")

plt.legend()
plt.savefig("/tmp/v_analysis.png", dpi=400, bbox_inches="tight")
plt.show()