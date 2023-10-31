
import rospy
from radar_odom.srv import GetMapRequest, GetMapResponse, GetMap
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import numpy as np


fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('snr', 12, PointField.FLOAT32, 1),
    PointField('doppler', 12, PointField.FLOAT32, 1),
]


getMap = rospy.ServiceProxy('radar/get_map', GetMap)


req = GetMapRequest()
resp = getMap(req)

pc = []
for p in point_cloud2.read_points(resp.map, field_names=("x", "y", "z", "snr", "doppler")):
    pc.append(p)

pc = np.asarray(pc)[:, :3]

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pc)
o3d.io.write_point_cloud('/tmp/radar_map.pcd', pcd)

print(pc.shape)