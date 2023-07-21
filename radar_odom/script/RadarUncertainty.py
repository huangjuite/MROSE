#! /usr/bin/env python3
import os

import numpy as np
import rospy
import torch

from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

from RadarModel import RadarTransformer
from radar_odom.srv import Prediction, PredictionRequest, PredictionResponse

class PRService():
    def __init__(self) -> None:
        self.predict_service = rospy.Service(
            "radar_uncertainty", Prediction, self.cb_pr)
        rospy.loginfo("uncertainty prediction service")

        self.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('speed', 16, PointField.FLOAT32, 1),
            PointField('weight', 20, PointField.FLOAT32, 1),
        ]

        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        print(self.device)

        self.model = RadarTransformer(
            feature_dim=5,
            embed_dim=32, 
            n_head=8,
            n_layer=3
        ).to(self.device)
        self.model.load_state_dict(torch.load(
            '/home/rpl/rpl_rio/src/rpl_rio/radar_odom/script/model.pth',
            map_location=self.device))
        self.model.eval()
        # self.model = torch.compile(self.model)

    def cb_pr(self, req: PredictionRequest):
        points = np.array(point_cloud2.read_points_list(req.input))
        points_copy = np.copy(points)
        points[:, -2] = np.log(points[:, -2])
        points[:, -2] = points[:, -2] / np.max(points[:, -2])

        sequence = torch.unsqueeze(torch.tensor(
            points, dtype=torch.float32, device=self.device), dim=1)
        pad_mask = torch.ones((1, points.shape[0])) < 0
        
        weights = self.model(sequence, pad_mask)
        weights = torch.squeeze(weights, dim=1).detach().cpu().numpy()
        points = np.hstack((points_copy, weights))

        resp = PredictionResponse()
        resp.output = point_cloud2.create_cloud(
            req.input.header, self.fields, points.tolist())
        
        return resp


if __name__ == "__main__":
    rospy.init_node("radar_prediction")
    pr = PRService()
    rospy.spin()
