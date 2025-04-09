"""
Python implementation of file I/O base class.
Translated from the original C++ implementation.
"""

from enum import IntEnum
from typing import Optional, BinaryIO, TextIO, Union
import os

class FileType(IntEnum):
    """File types supported by FileBase"""
    TEXT = 0
    BINARY = 1

class FileBase:
    """Base class for file I/O operations"""

    def __init__(self):
        """Initialize FileBase instance"""
        self._file: Optional[Union[TextIO, BinaryIO]] = None
        self._filetype: FileType = FileType.TEXT
        self._columns: int = 0

    def __del__(self):
        """Destructor to ensure file is closed"""
        self.close()

    def close(self) -> None:
        """Close the file if it's open"""
        if self.is_open():
            self._file.close()  # type: ignore
            self._file = None

    def is_open(self) -> bool:
        """Check if file is open
        
        Returns:
            True if file is open, False otherwise
        """
        return self._file is not None and not self._file.closed

    def is_eof(self) -> bool:
        """Check if end of file is reached
        
        Returns:
            True if at end of file, False otherwise
        
        Raises:
            ValueError: If file is not open
        """
        if not self.is_open():
            raise ValueError("File is not open")
        
        # For text files
        if self._filetype == FileType.TEXT:
            # Save current position
            pos = self._file.tell()  # type: ignore
            # Try to read one character
            char = self._file.read(1)  # type: ignore
            # Restore position
            self._file.seek(pos)  # type: ignore
            return not char
        
        # For binary files
        else:
            # Save current position
            pos = self._file.tell()  # type: ignore
            # Try to read one byte
            byte = self._file.read(1)  # type: ignore
            # Restore position
            self._file.seek(pos)  # type: ignore
            return not byte

    @property
    def file(self) -> Optional[Union[TextIO, BinaryIO]]:
        """Get file object
        
        Returns:
            File object if open, None otherwise
        """
        return self._file

    @property
    def columns(self) -> int:
        """Get number of columns in file
        
        Returns:
            Number of columns
        """
        return self._columns

    @property
    def filetype(self) -> FileType:
        """Get file type
        
        Returns:
            File type (TEXT or BINARY)
        """
        return self._filetype 