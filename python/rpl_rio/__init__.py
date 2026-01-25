"""rpl_rio - Radar-Inertial Odometry Python interface.

This module provides a Pythonic interface to the rpl_rio DIOManager
using nanobind bindings to the C++ implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

# Import the C++ bindings
from rpl_rio import _core as _rio

if TYPE_CHECKING:
    from numpy.typing import NDArray

# Re-export C++ types for convenience
IMU = _rio.IMU
Params = _rio.Params
RadarParams = _rio.RadarParams
LogLevel = _rio.LogLevel
get_params = _rio.get_params

# Type aliases for different point types
DIOManager_XYZD = _rio.DIOManager_XYZD
DIOManager_XYZSD = _rio.DIOManager_XYZSD

Radar_XYZD = _rio.Radar_XYZD
Radar_XYZSD = _rio.Radar_XYZSD

Keyframe_XYZD = _rio.Keyframe_XYZD
Keyframe_XYZSD = _rio.Keyframe_XYZSD

RadarVelocityEstimatorCeres_XYZD = _rio.RadarVelocityEstimatorCeres_XYZD
RadarVelocityEstimatorCeres_XYZSD = _rio.RadarVelocityEstimatorCeres_XYZSD

__all__ = [
    # Types
    "IMU",
    "Params",
    "RadarParams",
    "LogLevel",
    "KeyframeData",
    # Functions
    "get_params",
    # DIOManager variants
    "DIOManager",
    "DIOManager_XYZD",
    "DIOManager_XYZSD",
    # Radar variants
    "Radar_XYZD",
    "Radar_XYZSD",
    # Keyframe variants
    "Keyframe_XYZD",
    "Keyframe_XYZSD",
    # Velocity estimator variants
    "RadarVelocityEstimatorCeres_XYZD",
    "RadarVelocityEstimatorCeres_XYZSD",
]


@dataclass
class KeyframeData:
    """Python-friendly keyframe data structure."""

    stamp: float
    seq: int
    name: str
    position: NDArray[np.float64]  # (3,) xyz
    rotation: NDArray[np.float64]  # (4,) quaternion wxyz
    velocity: NDArray[np.float64]  # (3,) xyz
    pose_cov: NDArray[np.float64]  # (6, 6)
    velocity_cov: NDArray[np.float64]  # (3, 3)
    v_err: NDArray[np.float64]  # (3,)
    imu_acc: NDArray[np.float64]  # (3,)
    imu_gyro: NDArray[np.float64]  # (3,)
    imu_bias_acc: NDArray[np.float64]  # (3,)
    imu_bias_gyro: NDArray[np.float64]  # (3,)
    radar_velocity: NDArray[np.float64]  # (3,)
    radar_velocity_cov: NDArray[np.float64]  # (3, 3)
    point_cloud: NDArray[np.float32]  # (N, num_fields)

    @classmethod
    def from_keyframe(cls, kf) -> KeyframeData:
        """Create KeyframeData from a C++ Keyframe object."""
        return cls(
            stamp=kf.stamp,
            seq=kf.seq,
            name=kf.name,
            position=np.array(kf.position),
            rotation=np.array(kf.rotation),
            velocity=np.array(kf.velocity),
            pose_cov=np.array(kf.pose_cov),
            velocity_cov=np.array(kf.velocity_cov),
            v_err=np.array(kf.v_err),
            imu_acc=np.array(kf.imu_acc),
            imu_gyro=np.array(kf.imu_gyro),
            imu_bias_acc=np.array(kf.imu_bias_acc),
            imu_bias_gyro=np.array(kf.imu_bias_gyro),
            radar_velocity=np.array(kf.radar_velocity),
            radar_velocity_cov=np.array(kf.radar_velocity_cov),
            point_cloud=np.array(kf.point_cloud),
        )


class DIOManager:
    """Pythonic wrapper for DIOManager with automatic point type selection."""

    POINT_TYPES = {
        "PointXYZD": (
            DIOManager_XYZD,
            Radar_XYZD,
            RadarVelocityEstimatorCeres_XYZD,
        ),
        "PointXYZSD": (
            DIOManager_XYZSD,
            Radar_XYZSD,
            RadarVelocityEstimatorCeres_XYZSD,
        ),
    }

    def __init__(
        self,
        params: Params,
        point_type: str = "XYZD",
        doppler_sigma: float = 0.124,
        cauchy_scale: float = 1.0,
    ):
        """Initialize DIOManager.

        Args:
            params: DIO parameters (from get_params or manually constructed)
            point_type: Point cloud type ('PointXYZD', 'PointXYZSD')
            doppler_sigma: Doppler measurement noise standard deviation
            cauchy_scale: Cauchy loss scale for robust estimation
        """
        if point_type not in self.POINT_TYPES:
            raise ValueError(
                f"Unknown point type: {point_type}. "
                f"Available: {list(self.POINT_TYPES.keys())}"
            )

        manager_cls, self._radar_cls, vel_est_cls = self.POINT_TYPES[point_type]

        self._vel_estimator = vel_est_cls(
            doppler_sigma=doppler_sigma, cauchy_scale=cauchy_scale
        )
        self._manager = manager_cls(params, self._vel_estimator)
        self._params = params
        self._doppler_sigma = doppler_sigma
        self._cauchy_scale = cauchy_scale

    @classmethod
    def from_config(
        cls,
        config_path: str | Path,
        point_type: str | None = None,
        doppler_sigma: float = 0.124,
        cauchy_scale: float = 1.0,
    ) -> DIOManager:
        """Create DIOManager from a YAML config file.

        Args:
            config_path: Path to YAML config file
            point_type: Point cloud type (if None, inferred from config)
            doppler_sigma: Doppler measurement noise standard deviation
            cauchy_scale: Cauchy loss scale for robust estimation

        Returns:
            Configured DIOManager instance
        """
        import yaml

        params = get_params(str(config_path))

        if point_type is None:
            with open(config_path) as f:
                cfg = yaml.safe_load(f)
            point_type = cfg.get("point_type", "PointXYZD")

        return cls(
            params=params,
            point_type=point_type,
            doppler_sigma=doppler_sigma,
            cauchy_scale=cauchy_scale,
        )

    def feed_imu(
        self,
        acc: NDArray[np.float64],
        gyro: NDArray[np.float64],
        stamp: float,
    ) -> None:
        """Feed an IMU measurement.

        Args:
            acc: Accelerometer reading (3,) in m/s^2
            gyro: Gyroscope reading (3,) in rad/s
            stamp: Timestamp in seconds
        """
        imu = IMU(np.asarray(acc), np.asarray(gyro), stamp)
        self._manager.feed_imu(imu)

    def feed_radar(
        self,
        points: NDArray[np.float32],
        stamp: float,
        frame_id: str,
    ) -> list[KeyframeData]:
        """Feed a radar point cloud measurement.

        Args:
            points: Point cloud as (N, num_fields) float32 array
            stamp: Timestamp in seconds
            frame_id: Radar frame identifier (must match config)

        Returns:
            List of new keyframes produced by this measurement
        """
        if frame_id not in self._params.radars:
            raise ValueError(
                f"Unknown radar frame_id: {frame_id}. "
                f"Available: {list(self._params.radars.keys())}"
            )

        radar_params = self._params.radars[frame_id]
        radar = self._radar_cls(
            np.ascontiguousarray(points, dtype=np.float32),
            stamp,
            radar_params,
            frame_id,
        )
        self._manager.feed_radar(radar)
        return [
            KeyframeData.from_keyframe(kf) for kf in self._manager.get_latest_states()
        ]

    def feed_radar_velocity(
        self,
        velocity: NDArray[np.float64],
        velocity_cov: NDArray[np.float64],
        stamp: float,
        frame_id: str,
    ) -> list[KeyframeData]:
        """Feed a pre-computed radar velocity measurement.

        Args:
            velocity: 3D velocity vector (3,) in radar frame
            velocity_cov: Velocity covariance (3, 3)
            stamp: Timestamp in seconds
            frame_id: Radar frame identifier (must match config)

        Returns:
            List of new keyframes produced by this measurement
        """
        if frame_id not in self._params.radars:
            raise ValueError(
                f"Unknown radar frame_id: {frame_id}. "
                f"Available: {list(self._params.radars.keys())}"
            )

        radar_params = self._params.radars[frame_id]
        radar = self._radar_cls(
            np.asarray(velocity),
            np.asarray(velocity_cov),
            stamp,
            radar_params,
            frame_id,
        )
        self._manager.feed_radar(radar)
        return [
            KeyframeData.from_keyframe(kf) for kf in self._manager.get_latest_states()
        ]

    def get_latest_states(self) -> list[KeyframeData]:
        """Get the latest keyframe states.

        Returns:
            List of KeyframeData from the most recent optimization
        """
        return [
            KeyframeData.from_keyframe(kf) for kf in self._manager.get_latest_states()
        ]

    @property
    def params(self) -> Params:
        """Get the DIO parameters."""
        return self._params
