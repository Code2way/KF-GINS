"""
Python implementation of file saver.
Translated from the original C++ implementation.
"""

from typing import List, Optional, Union
import numpy as np
import struct
from .filebase import FileBase, FileType

class FileSaver(FileBase):
    """File saver for writing numeric data to text or binary files"""

    def __init__(self, filename: Optional[str] = None, columns: Optional[int] = None, 
                 filetype: FileType = FileType.TEXT):
        """Initialize FileSaver
        
        Args:
            filename: File to open (optional)
            columns: Number of columns in file (optional)
            filetype: File type (TEXT or BINARY)
        """
        super().__init__()
        
        if filename is not None and columns is not None:
            self.open(filename, columns, filetype)

    def open(self, filename: str, columns: int, filetype: FileType = FileType.TEXT) -> bool:
        """Open file for writing
        
        Args:
            filename: File to open
            columns: Number of columns in file
            filetype: File type (TEXT or BINARY)
            
        Returns:
            True if file was opened successfully, False otherwise
        """
        try:
            mode = 'w' if filetype == FileType.TEXT else 'wb'
            self._file = open(filename, mode)
            self._columns = columns
            self._filetype = filetype
            return True
        except:
            return False

    def dump(self, data: Union[List[float], np.ndarray]) -> None:
        """Write one row of data
        
        Args:
            data: Data to write (list or numpy array)
        """
        self._dump(data)

    def dumpn(self, data: Union[List[List[float]], List[np.ndarray]]) -> None:
        """Write multiple rows of data
        
        Args:
            data: List of data rows to write
        """
        for row in data:
            self._dump(row)

    def _dump(self, data: Union[List[float], np.ndarray]) -> None:
        """Internal method to write one row of data
        
        Args:
            data: Data to write (list or numpy array)
        """
        if not self.is_open():
            raise ValueError("File is not open")

        if isinstance(data, np.ndarray):
            data = data.tolist()

        if self._filetype == FileType.TEXT:
            # Format each number with 15 total width and 9 decimal places
            line = ' '.join(f'{x:15.9f}' for x in data)
            self._file.write(line + '\n')  # type: ignore
        else:
            # Write binary doubles
            binary_data = struct.pack(f'>{len(data)}d', *data)
            self._file.write(binary_data)  # type: ignore
            
        # Flush to ensure data is written
        self._file.flush()  # type: ignore 