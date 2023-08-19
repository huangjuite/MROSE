
import numpy as np
import pandas as pd
import argparse
import os
from pathlib import Path

from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--rotate_degree")
parser.add_argument("-g", "--gt_file")
args = parser.parse_args()

gt_file = Path(args.gt_file)
assert args.rotate_degree
assert gt_file.exists(), "ground truth file foler misssing"

gt = pd.read_csv(gt_file)

def to_se3(t, q):
    se3 = np.identity(4)
    se3[:3, -1] = t
    se3[:3, :3] = R.from_quat(q).as_matrix()
    return se3

def to_t_q(se3):
    d = np.zeros(7)
    d[:3] = se3[:3, -1]
    d[3:] = R.from_matrix(se3[:3, :3]).as_quat()
    return d

t = np.identity(4)
t[:3, :3] = R.from_euler('z', args.rotate_degree, degrees=True).as_matrix()

data = gt[[" tx", " ty", " tz", " qx", " qy", " qz", " qw"]].to_numpy()
transformed = []

for i in range(data.shape[0]):
    se3 = to_se3(data[i, :3], data[i, 3:])
    tq = to_t_q(np.matmul(t, se3))
    transformed.append(tq)

transformed = np.array(transformed)    

gt[[" tx", " ty", " tz", " qx", " qy", " qz", " qw"]] = transformed

gt.to_csv(gt_file.parent.__str__() + '/odom_vio_rotate.csv')

