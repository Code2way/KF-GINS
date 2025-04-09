"""
Python implementation of GNSS file loader.
Translated from the original C++ implementation.
"""

import numpy as np
from typing import Optional
from .fileloader import FileLoader
from .filebase import FileType
from ..common.types import GNSS
from ..common.angle import Angle

class GnssFileLoader(FileLoader):
    """File loader for GNSS data"""

    def __init__(self, filename: str, columns: int = 7):
        """Initialize GNSS file loader
        
        Args:
            filename: GNSS data file
            columns: Number of columns in file
        """
        super().__init__(filename, columns, FileType.TEXT)
        
        # Initialize GNSS data
        self._gnss = GNSS(
            time=0.0,
            blh=np.zeros(3),
            std=np.zeros(3),
            isvalid=True
        )
        self._data = np.zeros(columns)

    def next(self) -> GNSS:
        """Read next GNSS measurement
        
        Returns:
            GNSS measurement
        """
        # Load next data
        self._data = self.load()
        
        # Parse GNSS data
        self._gnss.time = self._data[0]
        self._gnss.blh = self._data[1:4]
        
        # Handle different file formats
        # 7-column GNSS file: time, lat, lon, height, std_lat, std_lon, std_height
        # 13-column GNSS file: includes velocity
        if len(self._data) == 7:
            self._gnss.std = self._data[4:7]
        else:
            self._gnss.std = self._data[7:10]
        
        # Convert lat/lon from degrees to radians
        self._gnss.blh[0] = Angle.deg2rad(self._gnss.blh[0])
        self._gnss.blh[1] = Angle.deg2rad(self._gnss.blh[1])
        
        return self._gnss 