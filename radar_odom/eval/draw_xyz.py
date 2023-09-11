
import numpy as np
from scipy.spatial.transform import Rotation as R
from os.path import join as pjoin
import pandas as pd

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib import rc

fe = fm.FontEntry(
    fname=r'/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf',
    name="Times New Roman"
)
fm.fontManager.ttflist.insert(0, fe)
fontp = {'family':'Times New Roman', 'size': 12}
pdfp = {'fonttype':42}

# rc('text', usetex=True)
rc('font', **fontp)
rc('pdf', **pdfp)


root = "/home/rpl/Documents/RPL_radar"

sequence = ["1692139890_nsh_atrium",
            "1692923792_ghc_stair",
            "1692311541_ghc_ramp"]
names = ["NSH_atrium", "GHC_stair", "GHC_ramp"]
# sequence = ["1692139890_nsh_atrium",
#             "1692923792_ghc_stair",
#             "1692145970_frc",
#             "1692308112_frc_2nd",
#             "1692311541_ghc_ramp",
#             "1692312182_nsh_4th"]

r = R.from_euler('z', np.pi).as_matrix()

gt_odoms, odoms, h_odoms, v_odoms = [], [], [], []

for seq in sequence:
    odom_file = pjoin(root, seq, "result/razor/odom_rio.csv")
    df = pd.read_csv(odom_file).to_numpy()[:, 1:4]
    odoms.append(df)

    h_file = pjoin(root, seq, "result/razor/odom_r0_rio.csv")
    df = pd.read_csv(h_file).to_numpy()[:, 1:4]
    h_odoms.append(df)

    v_file = pjoin(root, seq, "result/razor/odom_r1_rio.csv")
    df = pd.read_csv(v_file).to_numpy()[:, 1:4]
    v_odoms.append(df)

    gt_file = pjoin(root, seq, "ground_truth/odom_vio.csv")
    gtdf = pd.read_csv(gt_file).to_numpy()[:, 2:5]
    gtdf = (r @ gtdf.T).T
    gt_odoms.append(gtdf)


fig = plt.figure(figsize=(10, 6))
seqn = len(sequence)
for i in range(seqn):
    ax = plt.subplot(2, seqn, i+1)
    ax.plot(gt_odoms[i][:, 0], gt_odoms[i][:, 1], label="vio")
    ax.plot(odoms[i][:, 0], odoms[i][:, 1], label="dual")
    ax.plot(h_odoms[i][:, 0], h_odoms[i][:, 1], label="horizontal")
    ax.plot(v_odoms[i][:, 0], v_odoms[i][:, 1], label="vertical")
    ax.set_title(names[i], size=11)
    # ax.set_xlabel("x (m)")
    if i == 0:
        ax.set_ylabel("y (m)")
        ax.legend(loc='lower left', bbox_to_anchor=(0.15, 0., 0.2, 0.2))
    ax.axis('equal')

    ax1 = plt.subplot(2, seqn, seqn + i+1)
    ax1.plot(gt_odoms[i][:, 0], gt_odoms[i][:, 2], label="vio")
    ax1.plot(odoms[i][:, 0], odoms[i][:, 2], label="dual")
    ax1.plot(h_odoms[i][:, 0], h_odoms[i][:, 2], label="horizontal")
    ax1.plot(v_odoms[i][:, 0], v_odoms[i][:, 2], label="vertical")
    ax1.set_xlabel("x (m)")
    if i == 0:
        ax1.set_ylabel("z (m)")
    ax1.axis('equal')

    # ax2 =  plt.subplot(3, seqn, 2*seqn + i+1)
    # ax2.plot(odoms[i][:, 1], odoms[i][:, 2], label="dual")
    # ax2.plot(gt_odoms[i][:, 1], gt_odoms[i][:, 2], label="vio")

fig.tight_layout()
plt.savefig('/tmp/vis_odom.pdf', format="pdf")
plt.show()
