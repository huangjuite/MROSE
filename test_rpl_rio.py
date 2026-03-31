"""test."""

import os
from pathlib import Path

import numpy as np

from rpl_rio import IMU, DIOManager, LogLevel, get_params


def test_rio_initialization():
    """Test basic initialization and data feeding of rpl_rio."""
    print("Testing rpl_rio initialization...")
    config_path = Path("cfg/config_hugin.yaml")
    if not config_path.exists():
        print(f"Error: Config file {config_path} not found.")
        return

    # Test get_params
    print(f"Loading params from {config_path}...")
    params = get_params(str(config_path))
    print(f"Log Level: {params.log_level}")

    # Test DIOManager initialization
    # Note: config_hugin.yaml specifies 'PointHugin' which might map to XYZSD or XYZD
    # based on how bindings are implemented. We will try XYZD first.
    print("Initializing DIOManager...")
    manager = DIOManager(params)
    print("DIOManager initialized successfully.")

    # Test feeding IMU data
    print("Feeding dummy IMU data...")
    acc = np.array([0.0, 0.0, 9.81])
    gyro = np.array([0.01, 0.02, 0.03])
    stamp = 100.0
    manager.feed_imu(acc, gyro, stamp)
    print("IMU data fed.")

    # Test feeding Radar data (points)
    print("Feeding dummy Radar points...")
    # Create dummy points: [x, y, z, doppler]
    num_points = 10
    points = np.random.randn(num_points, 4).astype(np.float32)
    # Ensure some structure
    points[:, 3] = -0.5 # doppler

    # frame_id should match config (radar_1)
    frame_id = "radar_1"
    keyframes = manager.feed_radar(points, stamp + 0.1, frame_id)
    print(f"Radar points fed. Produced {len(keyframes)} keyframes.")

    # Test feeding Radar velocity
    print("Feeding dummy Radar velocity...")
    velocity = np.array([1.0, 0.5, -0.1])
    velocity_cov = np.eye(3) * 0.01
    keyframes = manager.feed_radar_velocity(velocity, velocity_cov, stamp + 0.2, frame_id)
    print(f"Radar velocity fed. Produced {len(keyframes)} keyframes.")

    if keyframes:
        kf = keyframes[-1]
        print(f"Latest keyframe at {kf.stamp}: pos={kf.position}, vel={kf.velocity}")


    print("Test completed successfully!")


if __name__ == "__main__":
    test_rio_initialization()
