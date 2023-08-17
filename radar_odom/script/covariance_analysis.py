
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv("/tmp/cov_tmp.csv")
data = data.to_numpy()
print(data.shape)

stamp = data[:, 0]
v_cartesian = data[:, 1:4]
cov_cartesian = data[:, 4:7]
v_horizontal = data[:, 7:10]
cov_horizontal = data[:, 10:13]
v_vio = data[:, 13:16]

fig, axs = plt.subplots(3, figsize=(10, 10))

axs[0].plot(stamp, v_cartesian[:, 0], label="cartesian")
axs[0].plot(stamp, v_vio[:, 0], label="vio")
axs[0].plot(stamp, v_horizontal[:, 0], label="horizontal")

axs[1].plot(stamp, v_cartesian[:, 1], label="cartesian")
axs[1].plot(stamp, v_vio[:, 1], label="vio")
axs[1].plot(stamp, v_horizontal[:, 1], label="horizontal")

axs[2].plot(stamp, v_cartesian[:, 2], label="cartesian")
axs[2].plot(stamp, v_vio[:, 2], label="vio")
axs[2].plot(stamp, v_horizontal[:, 2], label="horizontal")


plt.legend()
plt.savefig("/tmp/hcs_v_analysis.png", dpi=400, bbox_inches="tight")
plt.show()


fig, axs = plt.subplots(3, figsize=(10, 10))
axs[0].plot(stamp, cov_cartesian[:, 0], label="cartesian")
axs[0].plot(stamp, cov_horizontal[:, 0], label="horizontal")

axs[1].plot(stamp, cov_cartesian[:, 1], label="cartesian")
axs[1].plot(stamp, cov_horizontal[:, 1], label="horizontal")

axs[2].plot(stamp, cov_cartesian[:, 2], label="cartesian")
axs[2].plot(stamp, cov_horizontal[:, 2], label="horizontal")


plt.legend()
plt.savefig("/tmp/hcs_cov_analysis.png", dpi=400, bbox_inches="tight")
plt.show()

