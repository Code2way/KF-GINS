"""
Python implementation of KF-GINS types and structures.
Translated from the original C++ implementation.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional
from .common.angle import Angle

@dataclass
class Attitude:
    """Attitude representation with quaternion, rotation matrix and Euler angles"""
    qbn: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))  # w,x,y,z
    cbn: np.ndarray = field(default_factory=lambda: np.eye(3))
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))

@dataclass
class PVA:
    """Position, velocity and attitude"""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Latitude, longitude, height
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))  # North, east, down
    att: Attitude = field(default_factory=Attitude)

@dataclass
class ImuError:
    """IMU error parameters"""
    gyrbias: np.ndarray = field(default_factory=lambda: np.zeros(3))   # Gyroscope bias
    accbias: np.ndarray = field(default_factory=lambda: np.zeros(3))   # Accelerometer bias
    gyrscale: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Gyroscope scale factor
    accscale: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Accelerometer scale factor

@dataclass
class NavState:
    """Navigation state including position, velocity, attitude and IMU errors"""
    pos: np.ndarray = field(default_factory=lambda: np.zeros(3))       # Latitude, longitude, height
    vel: np.ndarray = field(default_factory=lambda: np.zeros(3))       # North, east, down
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))     # Roll, pitch, yaw
    imuerror: ImuError = field(default_factory=ImuError)

@dataclass
class ImuNoise:
    """IMU noise parameters"""
    gyr_arw: np.ndarray = field(default_factory=lambda: np.zeros(3))      # Angular random walk
    acc_vrw: np.ndarray = field(default_factory=lambda: np.zeros(3))      # Velocity random walk
    gyrbias_std: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Gyroscope bias std
    accbias_std: np.ndarray = field(default_factory=lambda: np.zeros(3))  # Accelerometer bias std
    gyrscale_std: np.ndarray = field(default_factory=lambda: np.zeros(3)) # Gyroscope scale factor std
    accscale_std: np.ndarray = field(default_factory=lambda: np.zeros(3)) # Accelerometer scale factor std
    corr_time: float = 3600.0  # Correlation time in seconds

@dataclass
class GINSOptions:
    """KF-GINS configuration options"""
    
    # Initial state and standard deviation
    initstate: NavState = field(default_factory=NavState)
    initstate_std: NavState = field(default_factory=NavState)
    
    # IMU noise parameters
    imunoise: ImuNoise = field(default_factory=ImuNoise)
    
    # Installation parameters
    antlever: np.ndarray = field(default_factory=lambda: np.zeros(3))
    
    def print_options(self) -> None:
        """Print configuration options"""
        print("---------------KF-GINS Options:---------------")
        
        # Print initial state
        print(" - Initial State: ")
        print(f"\t- initial position: {Angle.rad2deg(self.initstate.pos[0]):12.12f}  "
              f"{Angle.rad2deg(self.initstate.pos[1]):12.12f}  "
              f"{self.initstate.pos[2]:6.6f} [deg, deg, m]")
        print(f"\t- initial velocity: {self.initstate.vel[0]:.6f}  "
              f"{self.initstate.vel[1]:.6f}  "
              f"{self.initstate.vel[2]:.6f} [m/s]")
        print(f"\t- initial attitude: {Angle.rad2deg(self.initstate.euler[0]):.6f}  "
              f"{Angle.rad2deg(self.initstate.euler[1]):.6f}  "
              f"{Angle.rad2deg(self.initstate.euler[2]):.6f} [deg]")
        print(f"\t- initial gyrbias : {Angle.rad2deg(self.initstate.imuerror.gyrbias[0]) * 3600:.6f}  "
              f"{Angle.rad2deg(self.initstate.imuerror.gyrbias[1]) * 3600:.6f}  "
              f"{Angle.rad2deg(self.initstate.imuerror.gyrbias[2]) * 3600:.6f} [deg/h]")
        print(f"\t- initial accbias : {self.initstate.imuerror.accbias[0] * 1e5:.6f}  "
              f"{self.initstate.imuerror.accbias[1] * 1e5:.6f}  "
              f"{self.initstate.imuerror.accbias[2] * 1e5:.6f} [mGal]")
        print(f"\t- initial gyrscale: {self.initstate.imuerror.gyrscale[0] * 1e6:.6f}  "
              f"{self.initstate.imuerror.gyrscale[1] * 1e6:.6f}  "
              f"{self.initstate.imuerror.gyrscale[2] * 1e6:.6f} [ppm]")
        print(f"\t- initial accscale: {self.initstate.imuerror.accscale[0] * 1e6:.6f}  "
              f"{self.initstate.imuerror.accscale[1] * 1e6:.6f}  "
              f"{self.initstate.imuerror.accscale[2] * 1e6:.6f} [ppm]")
        
        # Print initial state std
        print(" - Initial State STD: ")
        print(f"\t- initial position std: {self.initstate_std.pos[0]:.6f}  "
              f"{self.initstate_std.pos[1]:.6f}  "
              f"{self.initstate_std.pos[2]:.6f} [m]")
        print(f"\t- initial velocity std: {self.initstate_std.vel[0]:.6f}  "
              f"{self.initstate_std.vel[1]:.6f}  "
              f"{self.initstate_std.vel[2]:.6f} [m/s]")
        print(f"\t- initial attitude std: {Angle.rad2deg(self.initstate_std.euler[0]):.6f}  "
              f"{Angle.rad2deg(self.initstate_std.euler[1]):.6f}  "
              f"{Angle.rad2deg(self.initstate_std.euler[2]):.6f} [deg]")
        print(f"\t- initial gyrbias std: {Angle.rad2deg(self.initstate_std.imuerror.gyrbias[0]) * 3600:.6f}  "
              f"{Angle.rad2deg(self.initstate_std.imuerror.gyrbias[1]) * 3600:.6f}  "
              f"{Angle.rad2deg(self.initstate_std.imuerror.gyrbias[2]) * 3600:.6f} [deg/h]")
        print(f"\t- initial accbias std: {self.initstate_std.imuerror.accbias[0] * 1e5:.6f}  "
              f"{self.initstate_std.imuerror.accbias[1] * 1e5:.6f}  "
              f"{self.initstate_std.imuerror.accbias[2] * 1e5:.6f} [mGal]")
        print(f"\t- initial gyrscale std: {self.initstate_std.imuerror.gyrscale[0] * 1e6:.6f}  "
              f"{self.initstate_std.imuerror.gyrscale[1] * 1e6:.6f}  "
              f"{self.initstate_std.imuerror.gyrscale[2] * 1e6:.6f} [ppm]")
        print(f"\t- initial accscale std: {self.initstate_std.imuerror.accscale[0] * 1e6:.6f}  "
              f"{self.initstate_std.imuerror.accscale[1] * 1e6:.6f}  "
              f"{self.initstate_std.imuerror.accscale[2] * 1e6:.6f} [ppm]")
        
        # Print IMU noise parameters
        print(" - IMU noise: ")
        print(f"\t- arw: {Angle.rad2deg(self.imunoise.gyr_arw[0]) * 60:.6f}  "
              f"{Angle.rad2deg(self.imunoise.gyr_arw[1]) * 60:.6f}  "
              f"{Angle.rad2deg(self.imunoise.gyr_arw[2]) * 60:.6f} [deg/sqrt(h)]")
        print(f"\t- vrw: {self.imunoise.acc_vrw[0] * 60:.6f}  "
              f"{self.imunoise.acc_vrw[1] * 60:.6f}  "
              f"{self.imunoise.acc_vrw[2] * 60:.6f} [m/s/sqrt(h)]")
        print(f"\t- gyrbias std: {Angle.rad2deg(self.imunoise.gyrbias_std[0]) * 3600:.6f}  "
              f"{Angle.rad2deg(self.imunoise.gyrbias_std[1]) * 3600:.6f}  "
              f"{Angle.rad2deg(self.imunoise.gyrbias_std[2]) * 3600:.6f} [deg/h]")
        print(f"\t- accbias std: {self.imunoise.accbias_std[0] * 1e5:.6f}  "
              f"{self.imunoise.accbias_std[1] * 1e5:.6f}  "
              f"{self.imunoise.accbias_std[2] * 1e5:.6f} [mGal]")
        print(f"\t- gyrscale std: {self.imunoise.gyrscale_std[0] * 1e6:.6f}  "
              f"{self.imunoise.gyrscale_std[1] * 1e6:.6f}  "
              f"{self.imunoise.gyrscale_std[2] * 1e6:.6f} [ppm]")
        print(f"\t- accscale std: {self.imunoise.accscale_std[0] * 1e6:.6f}  "
              f"{self.imunoise.accscale_std[1] * 1e6:.6f}  "
              f"{self.imunoise.accscale_std[2] * 1e6:.6f} [ppm]")
        print(f"\t- correlation time: {self.imunoise.corr_time / 3600.0:.6f} [h]")
        
        # Print GNSS antenna leverarm
        print(f" - Antenna leverarm: {self.antlever[0]:.6f}  "
              f"{self.antlever[1]:.6f}  "
              f"{self.antlever[2]:.6f} [m]\n") 