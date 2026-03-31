"""rpl_rio - Radar-Inertial Odometry Python interface.

This module provides a Pythonic interface to the rpl_rio DIOManager
using nanobind bindings to the C++ implementation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, List, Union

import numpy as np

# Import the C++ bindings
from rpl_rio import _core as _rio  # type: ignore

if TYPE_CHECKING:
    from numpy.typing import NDArray

# Re-export C++ types for convenience
IMU = _rio.IMU
Params = _rio.Params
RadarParams = _rio.RadarParams
LogLevel = _rio.LogLevel
get_params = _rio.get_params
Radar = _rio.Radar
Keyframe = _rio.Keyframe
RadarVelocityEstimatorCeres = _rio.RadarVelocityEstimatorCeres

__all__ = [
    # Types
    "IMU",
    "Params",
    "RadarParams",
    "LogLevel",
    "Radar",
    "Keyframe",
    "KeyframeData",
    # Functions
    "get_params",
    # Main classes
    "DIOManager",
    "RadarVelocityEstimatorCeres",
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
    imu_bias_cov: NDArray[np.float64]  # (6, 6)
    radar_velocity: NDArray[np.float64]  # (3,)
    radar_velocity_cov: NDArray[np.float64]  # (3, 3)
    point_cloud: NDArray[np.float32]  # (N, 4) [x, y, z, doppler]

    @classmethod
    def from_keyframe(cls, kf: _rio.Keyframe) -> KeyframeData:
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
            imu_bias_cov=np.array(kf.imu_bias_cov),
            radar_velocity=np.array(kf.radar_velocity),
            radar_velocity_cov=np.array(kf.radar_velocity_cov),
            point_cloud=np.array(kf.point_cloud),
        )


class DIOManager:
    """Pythonic wrapper for DIOManager."""

    def __init__(
        self,
        params: _rio.Params,
        doppler_sigma: float = 0.124,
        cauchy_scale: float = 1.0,
    ):
        """Initialize DIOManager.

        Args:
            params: DIO parameters (from get_params or manually constructed)
            velocity_estimator: Custom velocity estimator (default: Ceres-based)
            doppler_sigma: Doppler noise std (if using default estimator)
            cauchy_scale: Cauchy loss scale (if using default estimator)
        """
        velocity_estimator = RadarVelocityEstimatorCeres(
            doppler_sigma=doppler_sigma, cauchy_scale=cauchy_scale
        )

        self._manager = _rio.DIOManager(params, velocity_estimator)
        self._params = params
        self._vel_estimator = velocity_estimator

    @classmethod
    def from_config(
        cls,
        config_path: Union[str, Path],
        doppler_sigma: float = 0.124,
        cauchy_scale: float = 1.0,
    ) -> DIOManager:
        """Create DIOManager from a YAML config file.

        Args:
            config_path: Path to YAML config file
            doppler_sigma: Doppler noise std
            cauchy_scale: Cauchy loss scale

        Returns:
            Configured DIOManager instance
        """
        params = get_params(str(config_path))
        return cls(
            params=params,
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
    ) -> List[KeyframeData]:
        """Feed a radar point cloud measurement.

        Args:
            points: Point cloud as (N, 4) float32 array [x, y, z, doppler]
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
        radar = Radar(
            np.ascontiguousarray(points, dtype=np.float32),
            stamp,
            radar_params,
            frame_id,
        )
        self._manager.feed_radar(radar)
        return self.get_latest_states()

    def feed_radar_velocity(
        self,
        velocity: NDArray[np.float64],
        velocity_cov: NDArray[np.float64],
        stamp: float,
        frame_id: str,
    ) -> List[KeyframeData]:
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
        radar = Radar(
            np.asarray(velocity),
            np.asarray(velocity_cov),
            stamp,
            radar_params,
            frame_id,
        )
        self._manager.feed_radar(radar)
        return self.get_latest_states()

    def get_latest_states(self) -> List[KeyframeData]:
        """Get the latest keyframe states.

        Returns:
            List of KeyframeData from the most recent optimization
        """
        return [
            KeyframeData.from_keyframe(kf)
            for kf in self._manager.get_latest_states()
        ]

    @property
    def params(self) -> _rio.Params:
        """Get the DIO parameters."""
        return self._params
