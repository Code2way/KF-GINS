"""
Python implementation of KF-GINS basic types.
Translated from the original C++ implementation.
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class GNSS:
    """GNSS measurement data structure"""
    time: float
    blh: np.ndarray  # 3D vector: latitude, longitude, height
    std: np.ndarray  # 3D vector: standard deviations
    isvalid: bool = True

    def __post_init__(self):
        """Ensure numpy arrays are properly initialized"""
        if not isinstance(self.blh, np.ndarray):
            self.blh = np.array(self.blh, dtype=np.float64)
        if not isinstance(self.std, np.ndarray):
            self.std = np.array(self.std, dtype=np.float64)

@dataclass
class IMU:
    """IMU measurement data structure"""
    time: float
    dt: float
    dtheta: np.ndarray  # 3D vector: angular increments
    dvel: np.ndarray    # 3D vector: velocity increments
    odovel: float = 0.0

    def __post_init__(self):
        """Ensure numpy arrays are properly initialized"""
        if not isinstance(self.dtheta, np.ndarray):
            self.dtheta = np.array(self.dtheta, dtype=np.float64)
        if not isinstance(self.dvel, np.ndarray):
            self.dvel = np.array(self.dvel, dtype=np.float64)

@dataclass
class Pose:
    """3D pose representation with rotation matrix and translation"""
    R: np.ndarray  # 3x3 rotation matrix
    t: np.ndarray  # 3D translation vector

    def __post_init__(self):
        """Ensure numpy arrays are properly initialized"""
        if not isinstance(self.R, np.ndarray):
            self.R = np.array(self.R, dtype=np.float64)
        if not isinstance(self.t, np.ndarray):
            self.t = np.array(self.t, dtype=np.float64)

    @classmethod
    def identity(cls):
        """Create an identity pose"""
        return cls(R=np.eye(3), t=np.zeros(3)) 