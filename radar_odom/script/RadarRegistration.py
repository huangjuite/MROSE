#! /usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Quaternion, Point
from radar_odom.srv import Registration, RegistrationRequest, RegistrationResponse
from tf.transformations import quaternion_from_matrix


def get_fpfh(pcd, voxel_size):

    radius_normal = voxel_size * 4
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 10
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return pcd_fpfh


def global_registration(source_down, target_down,
                        source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 3
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        mutual_filter=False,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.
        TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.
            CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            1000000, 0.999999),
    )
    return result


class GRservice():
    def __init__(self) -> None:
        self.gr_service = rospy.Service(
            "global_registration", Registration, self.cb_gr)
        self.save = False
        self.idx = 0
        rospy.loginfo("global registration service")

    def cb_gr(self, req: RegistrationRequest):
        local_map = np.array(point_cloud2.read_points_list(req.local_map))
        sub_map = np.array(point_cloud2.read_points_list(req.sub_map))

        print("local_map:", local_map.shape, " sub_map:", sub_map.shape)

        sourcel_pcd, target_pcd = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
        sourcel_pcd.points = o3d.utility.Vector3dVector(local_map[:, :3])
        target_pcd.points = o3d.utility.Vector3dVector(sub_map[:, :3])

        if self.save:
            print("save %d" % self.idx)
            np.save("/home/rpl/rpl_radar/local%02d.npy" % self.idx, local_map)
            np.save("/home/rpl/rpl_radar/sub%02d.npy" % self.idx, sub_map)
            self.idx += 1

        voxel = 0.1
        source_pf = get_fpfh(sourcel_pcd, voxel)
        target_pf = get_fpfh(target_pcd, voxel)
        result = global_registration(
            sourcel_pcd, target_pcd, source_pf, target_pf, voxel)
        print(result)

        resp = RegistrationResponse()
        resp.fitness = result.fitness
        gr_tf = result.transformation
        quat = quaternion_from_matrix(gr_tf)
        resp.tf.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
        resp.tf.position = Point(gr_tf[0, 3], gr_tf[1, 3], gr_tf[2, 3])

        return resp


if __name__ == "__main__":
    rospy.init_node("global_registration")
    gr = GRservice()
    rospy.spin()
