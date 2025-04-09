"""
Python implementation of angle conversion utilities.
Translated from the original C++ implementation.
"""

import numpy as np
from typing import Union, TypeVar

# Type variable for numeric types
NumericType = TypeVar('NumericType', int, float, np.ndarray)

class Angle:
    """Angle conversion utilities"""
    
    # Constants for conversion
    D2R = np.pi / 180.0  # Degrees to radians
    R2D = 180.0 / np.pi  # Radians to degrees

    @staticmethod
    def rad2deg(rad: NumericType) -> NumericType:
        """Convert angle from radians to degrees
        
        Args:
            rad: Angle in radians (scalar or numpy array)
            
        Returns:
            Angle in degrees
        """
        return rad * Angle.R2D

    @staticmethod
    def deg2rad(deg: NumericType) -> NumericType:
        """Convert angle from degrees to radians
        
        Args:
            deg: Angle in degrees (scalar or numpy array)
            
        Returns:
            Angle in radians
        """
        return deg * Angle.D2R 