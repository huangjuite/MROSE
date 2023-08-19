
import numpy as np
import pandas as pd
import argparse
import os
from pathlib import Path

from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser()
parser.add_argument("-r", "--rio_file")
parser.add_argument("-g", "--gt_file")
args = parser.parse_args()

rio_file = Path(args.rio_file)
gt_file = Path(args.gt_file)
assert rio_file.exists(), "rio file foler misssing"
assert gt_file.exists(), "ground truth file foler misssing"

rio = pd.read_csv(rio_file)
gt = pd.read_csv(gt_file)

rio_stamp = rio["stamp"].to_numpy()
gt_stamp = gt["stamp"].to_numpy()

idxs = []
for rt in rio_stamp:
    idx = np.abs(gt_stamp - rt).argmin()
    idxs.append(idx)


def to_se3(pos, quaternion):
    se3 = 

matched = gt.iloc[idxs][[" tx",
                         " ty", " tz", " qx", " qy", " qz", " qw"]]

matched.to_csv(rio_file.parent.__str__() +
               '/eval_gt.csv', header=False, index=False, sep=" ")

rio = rio[[" px"," py", " pz", " rx", " ry", " rz", " rw"]]
rio.to_csv(rio_file.parent.__str__() +
               '/eval_rio.csv', header=False, index=False, sep=" ")
