
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
assert rio_file.exists(), "rio file misssing"
assert gt_file.exists(), "ground truth file misssing"

rio = pd.read_csv(rio_file)
gt = pd.read_csv(gt_file, skiprows=1, header=None, delimiter=" ")
gt = gt[[0,1,2,3,4,5,6,7]]

gt.to_csv(rio_file.parent.__str__() +
               '/eval_gt.txt', header=False, index=False, sep=" ")

rio = rio[["stamp", " px"," py", " pz", " rx", " ry", " rz", " rw"]]
rio.to_csv(rio_file.parent.__str__() +
               '/eval_rio.txt', header=False, index=False, sep=" ")
