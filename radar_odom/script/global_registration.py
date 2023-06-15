#! /usr/bin/env python3

import copy
import numpy as np
import open3d as o3d
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)


def draw_registration_result(source, target, result):
    aligned = copy.deepcopy(source)
    source.paint_uniform_color([1, 1, 1])
    aligned.paint_uniform_color([0, 1, 0])
    target.paint_uniform_color([1, 0, 0])
    aligned.transform(result.transformation)

    lines = np.asarray(result.correspondence_set)
    sp, tp = np.asarray(source.points), np.asarray(target.points)
    points = np.vstack((sp, tp))
    lines[:, 1] += sp.shape[0]

    colors = [[1, 1, 0] for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='global registration',
                      width=1440, height=810)
    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.15, 0.15, 0.15])
    vis.add_geometry(source)
    vis.add_geometry(target)
    # vis.add_geometry(aligned)
    vis.add_geometry(line_set)
    vis.run()
    vis.destroy_window()


def get_fpfh(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 4
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 10
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return pcd_down, pcd_fpfh


def global_registration(source_down, target_down,
                        source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 3
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down,
        target_down,
        source_fpfh,
        target_fpfh,
        mutual_filter=True,
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


for i in range(12):
    local_map = np.load("./testfile/local%02d.npy" % i)
    sub_map = np.load("./testfile/sub%02d.npy" % i)
    sourcel_pcd, target_pcd = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    sourcel_pcd.points = o3d.utility.Vector3dVector(local_map[:, :3])
    target_pcd.points = o3d.utility.Vector3dVector(sub_map[:, :3])

    voxel_size = 0.1
    source_down,  source_fpfh = get_fpfh(sourcel_pcd, voxel_size)
    target_down, target_fpfh = get_fpfh(target_pcd, voxel_size)

    result = global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size)
    print("===========================================")
    print(result)
    # print(np.asarray(result.correspondence_set))

    draw_registration_result(source_down, target_down, result)
