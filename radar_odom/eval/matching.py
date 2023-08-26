
import numpy as np
import pandas as pd
import argparse
import os
from pathlib import Path

from scipy.spatial.transform import Rotation as R

parser = argparse.ArgumentParser()
parser.add_argument("-r", "--rio_file")
parser.add_argument("-g", "--gt_file")
parser.add_argument("-or", "--rio_out", default="/tmp/eval_gt.csv")
parser.add_argument("-og", "--gt_out", default="/tmp/eval_rio.csv")
args = parser.parse_args()

rio_file = Path(args.rio_file)
gt_file = Path(args.gt_file)
assert rio_file.exists(), "rio file misssing"
assert gt_file.exists(), "ground truth file misssing"

rio = pd.read_csv(rio_file)
gt = pd.read_csv(gt_file)

rio_stamp = rio["stamp"].to_numpy()
gt_stamp = gt["stamp"].to_numpy()

idxs = []
for rt in rio_stamp:
    idx = np.abs(gt_stamp - rt).argmin()
    idxs.append(idx)


matched = gt.iloc[idxs][["stamp", " tx", " ty", " tz", " qx", " qy", " qz", " qw"]]
matched.to_csv(args.gt_out, header=False, index=False, sep=" ")

rio = rio[["stamp", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]]
rio.to_csv(args.rio_out, header=False, index=False, sep=" ")
