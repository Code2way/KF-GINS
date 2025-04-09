"""
Python implementation of logging utilities.
Translated from the original C++ implementation.
"""

import logging
import numpy as np
import sys
from typing import Optional
from pathlib import Path

class Logging:
    """Logging utilities for KF-GINS"""

    _logger: Optional[logging.Logger] = None
    
    @classmethod
    def initialization(cls, 
                      name: str = "KF-GINS",
                      log_to_stderr: bool = True,
                      log_to_file: bool = False,
                      log_file: Optional[str] = None) -> None:
        """Initialize logging system
        
        Args:
            name: Logger name
            log_to_stderr: Whether to log to stderr
            log_to_file: Whether to log to file
            log_file: Log file path (required if log_to_file is True)
        """
        # Create logger
        cls._logger = logging.getLogger(name)
        cls._logger.setLevel(logging.DEBUG)
        
        # Create formatters
        console_formatter = logging.Formatter(
            '%(levelname)s - %(message)s'
        )
        file_formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Add stderr handler if requested
        if log_to_stderr:
            console_handler = logging.StreamHandler(sys.stderr)
            console_handler.setFormatter(console_formatter)
            cls._logger.addHandler(console_handler)
        
        # Add file handler if requested
        if log_to_file:
            if log_file is None:
                raise ValueError("log_file must be specified when log_to_file is True")
            
            # Create log directory if it doesn't exist
            log_path = Path(log_file)
            log_path.parent.mkdir(parents=True, exist_ok=True)
            
            file_handler = logging.FileHandler(log_file)
            file_handler.setFormatter(file_formatter)
            cls._logger.addHandler(file_handler)

    @classmethod
    def info(cls, msg: str) -> None:
        """Log info message"""
        if cls._logger is None:
            cls.initialization()
        cls._logger.info(msg)

    @classmethod
    def warning(cls, msg: str) -> None:
        """Log warning message"""
        if cls._logger is None:
            cls.initialization()
        cls._logger.warning(msg)

    @classmethod
    def error(cls, msg: str) -> None:
        """Log error message"""
        if cls._logger is None:
            cls.initialization()
        cls._logger.error(msg)

    @classmethod
    def fatal(cls, msg: str) -> None:
        """Log fatal message and exit"""
        if cls._logger is None:
            cls.initialization()
        cls._logger.critical(msg)
        sys.exit(1)

    @classmethod
    def debug(cls, msg: str) -> None:
        """Log debug message"""
        if cls._logger is None:
            cls.initialization()
        cls._logger.debug(msg)

    @staticmethod
    def print_matrix(matrix: np.ndarray, prefix: str = "Matrix: ") -> None:
        """Print matrix with prefix
        
        Args:
            matrix: Matrix to print
            prefix: Prefix string
        """
        print(f"{prefix}{matrix.shape[0]}x{matrix.shape[1]}")
        if matrix.shape[1] == 1:  # Column vector
            print(matrix.T)
        else:
            print(matrix)

    @staticmethod
    def format_double(data: float, precision: int = 6) -> str:
        """Format double value with specified precision
        
        Args:
            data: Value to format
            precision: Number of decimal places
            
        Returns:
            Formatted string
        """
        return f"{data:.{precision}f}" 