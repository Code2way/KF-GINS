"""
Python implementation of IMU file loader.
Translated from the original C++ implementation.
"""

import numpy as np
import os
from typing import Optional
from .fileloader import FileLoader
from .filebase import FileType
from ..common.types import IMU

class ImuFileLoader(FileLoader):
    """File loader for IMU data"""

    def __init__(self, filename: str, columns: int, rate: int = 200):
        """Initialize IMU file loader
        
        Args:
            filename: IMU data file
            columns: Number of columns in file
            rate: IMU sampling rate in Hz
        """
        super().__init__(filename, columns, FileType.TEXT)
        
        self._dt = 1.0 / rate
        
        # Initialize IMU data
        self._imu = IMU(
            time=0.0,
            dt=self._dt,
            dtheta=np.zeros(3),
            dvel=np.zeros(3),
            odovel=0.0
        )
        self._imu_pre = IMU(
            time=0.0,
            dt=self._dt,
            dtheta=np.zeros(3),
            dvel=np.zeros(3),
            odovel=0.0
        )
        self._data = np.zeros(columns)

    def next(self) -> IMU:
        """Read next IMU measurement
        
        Returns:
            IMU measurement
        """
        # Store previous IMU data
        self._imu_pre = IMU(
            time=self._imu.time,
            dt=self._imu.dt,
            dtheta=self._imu.dtheta.copy(),
            dvel=self._imu.dvel.copy(),
            odovel=self._imu.odovel
        )
        
        # Load next data
        self._data = self.load()
        
        # Parse IMU data
        self._imu.time = self._data[0]
        self._imu.dtheta = self._data[1:4]
        self._imu.dvel = self._data[4:7]
        
        # Calculate time interval
        dt = self._imu.time - self._imu_pre.time
        if dt < 0.1:
            self._imu.dt = dt
        else:
            self._imu.dt = self._dt
        
        # Handle odometer velocity if available
        if self._columns > 7:
            self._imu.odovel = self._data[7] * self._imu.dt
        
        return self._imu

    def starttime(self) -> float:
        """Get start time of IMU data
        
        Returns:
            Start time in seconds
        """
        # Save current position
        if self._file is not None:
            pos = self._file.tell()
        
        # Go to beginning
        if self._file is not None:
            self._file.seek(0)
        
        # Load first data
        starttime = self.load()[0]
        
        # Restore position
        if self._file is not None:
            self._file.seek(pos)
        
        return starttime

    def endtime(self) -> float:
        """Get end time of IMU data
        
        Returns:
            End time in seconds
        """
        if not self.is_open():
            return -1
        
        # Save current position
        pos = self._file.tell()  # type: ignore
        
        if self._filetype == FileType.TEXT:
            # Text file approach - go to end and read backwards to find last line
            self._file.seek(0, os.SEEK_END)  # type: ignore
            file_size = self._file.tell()  # type: ignore
            
            # Start from end and move backwards to find last line
            pos_end = file_size - 1
            line = ''
            while pos_end > 0:
                self._file.seek(pos_end)  # type: ignore
                char = self._file.read(1)  # type: ignore
                if char == '\n':
                    # Read the last line
                    line = self._file.readline()  # type: ignore
                    if line.strip():  # Make sure it's not an empty line
                        break
                pos_end -= 1
                
            if line:
                # Parse the last line
                parts = line.strip().split()
                endtime = float(parts[0])
            else:
                endtime = -1
                
        else:
            # Binary file approach - go directly to last record
            file_size = os.fstat(self._file.fileno()).st_size  # type: ignore
            record_size = 8 * self._columns  # Size of one record in bytes
            
            # Position at the start of the last record
            last_record_pos = file_size - record_size
            if last_record_pos >= 0:
                self._file.seek(last_record_pos)  # type: ignore
                endtime = self.load()[0]
            else:
                endtime = -1
        
        # Restore position
        self._file.seek(pos)  # type: ignore
        
        return endtime 