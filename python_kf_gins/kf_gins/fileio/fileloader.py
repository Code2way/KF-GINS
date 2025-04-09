"""
Python implementation of file loader.
Translated from the original C++ implementation.
"""

from typing import List, Optional, Union
import numpy as np
import struct
from .filebase import FileBase, FileType

class FileLoader(FileBase):
    """File loader for reading numeric data from text or binary files"""

    def __init__(self, filename: Optional[str] = None, columns: Optional[int] = None, 
                 filetype: FileType = FileType.TEXT):
        """Initialize FileLoader
        
        Args:
            filename: File to open (optional)
            columns: Number of columns in file (optional)
            filetype: File type (TEXT or BINARY)
        """
        super().__init__()
        self._data: List[float] = []
        
        if filename is not None and columns is not None:
            self.open(filename, columns, filetype)

    def open(self, filename: str, columns: int, filetype: FileType = FileType.TEXT) -> bool:
        """Open file for reading
        
        Args:
            filename: File to open
            columns: Number of columns in file
            filetype: File type (TEXT or BINARY)
            
        Returns:
            True if file was opened successfully, False otherwise
        """
        try:
            mode = 'r' if filetype == FileType.TEXT else 'rb'
            self._file = open(filename, mode)
            self._columns = columns
            self._filetype = filetype
            return True
        except:
            return False

    def load(self) -> np.ndarray:
        """Load one row of data
        
        Returns:
            Array of values from one row
        """
        if self._load():
            return np.array(self._data)
        return np.array([])

    def loadn(self, epochs: int) -> List[np.ndarray]:
        """Load multiple rows of data
        
        Args:
            epochs: Number of rows to load
            
        Returns:
            List of arrays, each containing one row of data
        """
        datas = []
        for _ in range(epochs):
            if self._load():
                datas.append(np.array(self._data))
            else:
                break
        return datas

    def load_to(self, data: np.ndarray) -> bool:
        """Load one row of data into existing array
        
        Args:
            data: Array to store loaded data
            
        Returns:
            True if data was loaded, False if EOF or error
        """
        if self._load():
            data[:] = self._data
            return True
        return False

    def loadn_to(self, datas: List[np.ndarray], epochs: int) -> bool:
        """Load multiple rows of data into existing list
        
        Args:
            datas: List to store loaded data arrays
            epochs: Number of rows to load
            
        Returns:
            True if any data was loaded, False if EOF or error
        """
        datas.clear()
        for _ in range(epochs):
            if self._load():
                datas.append(np.array(self._data))
            else:
                break
        return len(datas) > 0

    def _load(self) -> bool:
        """Internal method to load one row of data
        
        Returns:
            True if data was loaded, False if EOF or error
        """
        if self.is_eof():
            return False

        self._data = []

        if self._filetype == FileType.TEXT:
            # Read one line
            line = self._file.readline()  # type: ignore
            if not line:
                return False

            # Split line and convert to float
            splits = [x for x in line.strip().split() if x]
            self._data = [float(x) for x in splits]

        else:  # BINARY
            # Read binary doubles
            try:
                data = self._file.read(8 * self._columns)  # type: ignore
                if not data or len(data) != 8 * self._columns:
                    return False
                self._data = list(struct.unpack(f'>{self._columns}d', data))
            except:
                return False

        return True 