
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R

data = pd.read_csv("/tmp/log_rio.csv")
data = data.to_numpy()
print(data.shape)
stamp = data[:, 0]
pos_rio = data[:, 1:4]
rot_rio = data[:, 4:]

data = pd.read_csv("/tmp/log_rio_mono.csv")
data = data.to_numpy()
print(data.shape)
stamp = data[:, 0]
pos_rio_mono = data[:, 1:4]
rot_rio_mono = data[:, 4:]


tf = R.from_euler('z', 180, degrees=True).as_matrix()
print(tf)

gt_data = pd.read_csv(
    "/home/rpl/Documents/RPL_radar/1692139890_nsh_atrium/odom_vio.csv")
gt_data = gt_data.to_numpy()
print(gt_data.shape)
stamp = gt_data[:, 0]
pos_vio = gt_data[:, 2:5]
rot_vio = gt_data[:, 5:9]

pos_vio = np.matmul(tf, np.swapaxes(pos_vio, 0, 1))
pos_vio = np.swapaxes(pos_vio, 0, 1)


fig, axs = plt.subplots(3, figsize=(10, 10))

axs[0].set_title("xy")
axs[0].set_xlabel("x")
axs[0].set_ylabel("y")
axs[0].plot(pos_rio[:, 0], pos_rio[:, 1], label="dual rio")
axs[0].plot(pos_rio_mono[:, 0], pos_rio_mono[:, 1], label="mono rio")
axs[0].plot(pos_vio[:, 0], pos_vio[:, 1], label="vio")

axs[1].set_title("xz")
axs[1].set_xlabel("x")
axs[1].set_ylabel("z")
axs[1].plot(pos_rio[:, 0], pos_rio[:, 2], label="dual rio")
axs[1].plot(pos_rio_mono[:, 0], pos_rio_mono[:, 2], label="mono rio")
axs[1].plot(pos_vio[:, 0], pos_vio[:, 2], label="vio")

axs[2].set_title("yz")
axs[2].set_xlabel("y")
axs[2].set_ylabel("z")
axs[2].plot(pos_rio[:, 1], pos_rio[:, 2], label="dual rio")
axs[2].plot(pos_rio_mono[:, 1], pos_rio_mono[:, 2], label="mono rio")
axs[2].plot(pos_vio[:, 1], pos_vio[:, 2], label="vio")

fig.tight_layout(pad=2.0)
plt.legend()
plt.savefig("/tmp/rio_paths.png", dpi=400, bbox_inches="tight")
plt.show()
