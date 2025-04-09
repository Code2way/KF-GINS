"""
Python implementation of rotation utilities.
Translated from the original C++ implementation.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import warnings
from typing import Tuple

class Rotation:
    """Rotation utilities for handling various rotation representations"""

    @staticmethod
    def matrix2quaternion(matrix: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion (w, x, y, z)"""
        r = R.from_matrix(matrix)
        return r.as_quat()[[3, 0, 1, 2]]  # scipy uses (x,y,z,w), we use (w,x,y,z)

    @staticmethod
    def quaternion2matrix(quaternion: np.ndarray) -> np.ndarray:
        """Convert quaternion (w, x, y, z) to rotation matrix"""
        # Convert to scipy quaternion format (x,y,z,w)
        q_scipy = quaternion[[1, 2, 3, 0]]
        r = R.from_quat(q_scipy)
        return r.as_matrix()

    @staticmethod
    def matrix2euler(dcm: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to Euler angles (roll, pitch, yaw)
        Using ZYX rotation order for front-right-down IMU
        """
        euler = np.zeros(3)
        
        # Calculate pitch (Y-axis rotation)
        euler[1] = np.arctan2(-dcm[2, 0], 
                             np.sqrt(dcm[2, 1]**2 + dcm[2, 2]**2))

        # Check for gimbal lock
        if np.abs(dcm[2, 0]) > 0.999:
            warnings.warn("Singular Euler Angle! Setting roll angle to 0!")
            euler[0] = 0
            if dcm[2, 0] <= -0.999:
                euler[2] = np.arctan2((dcm[1, 2] - dcm[0, 1]), 
                                    (dcm[0, 2] + dcm[1, 1]))
            else:  # dcm[2, 0] >= 0.999
                euler[2] = np.pi + np.arctan2((dcm[1, 2] + dcm[0, 1]), 
                                            (dcm[0, 2] - dcm[1, 1]))
        else:
            euler[0] = np.arctan2(dcm[2, 1], dcm[2, 2])  # roll
            euler[2] = np.arctan2(dcm[1, 0], dcm[0, 0])  # yaw

        # Normalize heading to [0, 2π]
        if euler[2] < 0:
            euler[2] = 2 * np.pi + euler[2]

        return euler

    @staticmethod
    def quaternion2euler(quaternion: np.ndarray) -> np.ndarray:
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        return Rotation.matrix2euler(Rotation.quaternion2matrix(quaternion))

    @staticmethod
    def rotvec2quaternion(rotvec: np.ndarray) -> np.ndarray:
        """Convert rotation vector to quaternion"""
        angle = np.linalg.norm(rotvec)
        if angle < 1e-10:
            return np.array([1.0, 0.0, 0.0, 0.0])
        axis = rotvec / angle
        return np.array([np.cos(angle/2),
                        axis[0] * np.sin(angle/2),
                        axis[1] * np.sin(angle/2),
                        axis[2] * np.sin(angle/2)])

    @staticmethod
    def quaternion2vector(quaternion: np.ndarray) -> np.ndarray:
        """Convert quaternion to rotation vector"""
        # Convert to scipy quaternion format (x,y,z,w)
        q_scipy = quaternion[[1, 2, 3, 0]]
        r = R.from_quat(q_scipy)
        return r.as_rotvec()

    @staticmethod
    def euler2matrix(euler: np.ndarray) -> np.ndarray:
        """Convert Euler angles (roll, pitch, yaw) to rotation matrix
        Using ZYX rotation order
        """
        r = R.from_euler('ZYX', euler[::-1])  # Note: reverse order for scipy
        return r.as_matrix()

    @staticmethod
    def euler2quaternion(euler: np.ndarray) -> np.ndarray:
        """Convert Euler angles (roll, pitch, yaw) to quaternion"""
        r = R.from_euler('ZYX', euler[::-1])  # Note: reverse order for scipy
        quat = r.as_quat()
        return np.array([quat[3], quat[0], quat[1], quat[2]])  # to w,x,y,z order

    @staticmethod
    def skew_symmetric(vector: np.ndarray) -> np.ndarray:
        """Create skew-symmetric matrix from 3D vector"""
        return np.array([[0, -vector[2], vector[1]],
                        [vector[2], 0, -vector[0]],
                        [-vector[1], vector[0], 0]])

    @staticmethod
    def quaternion_left(q: np.ndarray) -> np.ndarray:
        """Create left quaternion multiplication matrix"""
        mat = np.zeros((4, 4))
        mat[0, 0] = q[0]
        mat[0, 1:] = -q[1:].T
        mat[1:, 0] = q[1:]
        mat[1:, 1:] = q[0] * np.eye(3) + Rotation.skew_symmetric(q[1:])
        return mat

    @staticmethod
    def quaternion_right(p: np.ndarray) -> np.ndarray:
        """Create right quaternion multiplication matrix"""
        mat = np.zeros((4, 4))
        mat[0, 0] = p[0]
        mat[0, 1:] = -p[1:].T
        mat[1:, 0] = p[1:]
        mat[1:, 1:] = p[0] * np.eye(3) - Rotation.skew_symmetric(p[1:])
        return mat 