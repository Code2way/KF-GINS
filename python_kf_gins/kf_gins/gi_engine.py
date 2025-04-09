"""
Python implementation of KF-GINS GI Engine.
Translated from the original C++ implementation.
"""

import numpy as np
from typing import Optional, Tuple, List
from enum import IntEnum

from .common.types import IMU, GNSS
from .common.rotation import Rotation
from .kf_gins_types import GINSOptions, NavState, PVA, Attitude, ImuError
from .insmech import INSMech

class StateID(IntEnum):
    """State indices in state vector"""
    P_ID = 0    # Position states
    V_ID = 3    # Velocity states
    PHI_ID = 6  # Attitude error states
    BG_ID = 9   # Gyroscope bias states
    BA_ID = 12  # Accelerometer bias states
    SG_ID = 15  # Gyroscope scale factor states
    SA_ID = 18  # Accelerometer scale factor states

class NoiseID(IntEnum):
    """Noise indices in noise vector"""
    VRW_ID = 0   # Velocity random walk
    ARW_ID = 3   # Angular random walk
    BGSTD_ID = 6  # Gyroscope bias std
    BASTD_ID = 9  # Accelerometer bias std
    SGSTD_ID = 12 # Gyroscope scale factor std
    SASTD_ID = 15 # Accelerometer scale factor std

class GIEngine:
    """GNSS/INS integration engine using EKF"""
    
    # State vector size
    RANK = 21
    
    def __init__(self, options: GINSOptions):
        """Initialize GI Engine
        
        Args:
            options: KF-GINS configuration options
        """
        self._options = options
        
        # Initialize timestamp
        self._timestamp = 0.0
        
        # Update time alignment error threshold
        self._TIME_ALIGN_ERR = 0.001
        
        # Initialize IMU and GNSS data
        self._imupre = IMU(time=0.0, dt=0.0, dtheta=np.zeros(3), dvel=np.zeros(3), odovel=0.0)
        self._imucur = IMU(time=0.0, dt=0.0, dtheta=np.zeros(3), dvel=np.zeros(3), odovel=0.0)
        self._gnssdata = GNSS(time=0.0, blh=np.zeros(3), std=np.zeros(3), isvalid=False)
        
        # Initialize IMU states
        self._pvacur = PVA()
        self._pvapre = PVA()
        self._imuerror = ImuError()
        
        # Initialize EKF variables
        self._dx = np.zeros(self.RANK)  # State correction vector
        self._Phi = np.zeros((self.RANK, self.RANK))  # State transition matrix
        self._Qd = np.zeros((self.RANK, self.RANK))  # Process noise matrix
        self._Cov = np.zeros((self.RANK, self.RANK))  # State covariance matrix
        
        # Initialize system state and covariance
        self.initialize(options.initstate, options.initstate_std)
        
        # Print options
        options.print_options()
    
    def add_imu_data(self, imu: IMU, compensate: bool = False) -> None:
        """Add new IMU data with optional error compensation
        
        Args:
            imu: New IMU data
            compensate: Whether to compensate IMU errors
        """
        # Store previous IMU data
        self._imupre = self._imucur
        
        # Update current IMU data
        self._imucur = IMU(
            time=imu.time,
            dt=imu.dt,
            dtheta=imu.dtheta.copy(),
            dvel=imu.dvel.copy(),
            odovel=imu.odovel
        )
        
        # Compensate IMU errors if requested
        if compensate:
            self.imu_compensate(self._imucur)
    
    def add_gnss_data(self, gnss: GNSS) -> None:
        """Add new GNSS data
        
        Args:
            gnss: New GNSS data
        """
        # Store GNSS data
        self._gnssdata = GNSS(
            time=gnss.time,
            blh=gnss.blh.copy(),
            std=gnss.std.copy(),
            isvalid=True  # GNSS data is valid by default
        )
    
    @staticmethod
    def imu_interpolate(imu1: IMU, imu2: IMU, timestamp: float, midimu: IMU) -> None:
        """Interpolate incremental IMU data to given timestamp
        
        Args:
            imu1: Previous IMU data
            imu2: Current IMU data (will be modified)
            timestamp: Target timestamp for interpolation
            midimu: Output interpolated IMU data
        """
        # Check if interpolation is valid
        if imu1.time > timestamp or imu2.time < timestamp:
            return
        
        # Calculate interpolation factor
        lamda = (timestamp - imu1.time) / (imu2.time - imu1.time)
        
        # Interpolate IMU data
        midimu.time = timestamp
        midimu.dtheta = imu2.dtheta * lamda
        midimu.dvel = imu2.dvel * lamda
        midimu.dt = timestamp - imu1.time
        
        # Adjust current IMU data
        imu2.dtheta = imu2.dtheta - midimu.dtheta
        imu2.dvel = imu2.dvel - midimu.dvel
        imu2.dt = imu2.dt - midimu.dt
    
    def new_imu_process(self) -> None:
        """Process new IMU data with potential GNSS update"""
        # TODO: Implement IMU processing with GNSS update
        pass
    
    def timestamp(self) -> float:
        """Get current timestamp
        
        Returns:
            Current timestamp
        """
        return self._timestamp
    
    def get_nav_state(self) -> NavState:
        """Get current navigation state
        
        Returns:
            Current navigation state
        """
        navstate = NavState()
        navstate.pos = self._pvacur.pos.copy()
        navstate.vel = self._pvacur.vel.copy()
        navstate.euler = self._pvacur.att.euler.copy()
        navstate.imuerror = ImuError(
            gyrbias=self._imuerror.gyrbias.copy(),
            accbias=self._imuerror.accbias.copy(),
            gyrscale=self._imuerror.gyrscale.copy(),
            accscale=self._imuerror.accscale.copy()
        )
        return navstate
    
    def get_covariance(self) -> np.ndarray:
        """Get current state covariance
        
        Returns:
            Current state covariance matrix
        """
        return self._Cov.copy()
    
    def initialize(self, initstate: NavState, initstate_std: NavState) -> None:
        """Initialize system state and covariance
        
        Args:
            initstate: Initial navigation state
            initstate_std: Initial navigation state standard deviation
        """
        # Initialize navigation state
        self._pvacur.pos = initstate.pos.copy()
        self._pvacur.vel = initstate.vel.copy()
        
        # Initialize attitude
        self._pvacur.att.euler = initstate.euler.copy()
        self._pvacur.att.cbn = Rotation.euler2matrix(initstate.euler)
        self._pvacur.att.qbn = Rotation.euler2quaternion(initstate.euler)
        
        # Initialize IMU errors
        self._imuerror.gyrbias = initstate.imuerror.gyrbias.copy()
        self._imuerror.accbias = initstate.imuerror.accbias.copy()
        self._imuerror.gyrscale = initstate.imuerror.gyrscale.copy()
        self._imuerror.accscale = initstate.imuerror.accscale.copy()
        
        # Initialize state covariance matrix
        self._Cov = np.zeros((self.RANK, self.RANK))
        
        # Position states
        self._Cov[StateID.P_ID:StateID.P_ID+3, StateID.P_ID:StateID.P_ID+3] = np.diag(
            initstate_std.pos ** 2
        )
        
        # Velocity states
        self._Cov[StateID.V_ID:StateID.V_ID+3, StateID.V_ID:StateID.V_ID+3] = np.diag(
            initstate_std.vel ** 2
        )
        
        # Attitude states
        self._Cov[StateID.PHI_ID:StateID.PHI_ID+3, StateID.PHI_ID:StateID.PHI_ID+3] = np.diag(
            initstate_std.euler ** 2
        )
        
        # Gyroscope bias states
        self._Cov[StateID.BG_ID:StateID.BG_ID+3, StateID.BG_ID:StateID.BG_ID+3] = np.diag(
            initstate_std.imuerror.gyrbias ** 2
        )
        
        # Accelerometer bias states
        self._Cov[StateID.BA_ID:StateID.BA_ID+3, StateID.BA_ID:StateID.BA_ID+3] = np.diag(
            initstate_std.imuerror.accbias ** 2
        )
        
        # Gyroscope scale factor states
        self._Cov[StateID.SG_ID:StateID.SG_ID+3, StateID.SG_ID:StateID.SG_ID+3] = np.diag(
            initstate_std.imuerror.gyrscale ** 2
        )
        
        # Accelerometer scale factor states
        self._Cov[StateID.SA_ID:StateID.SA_ID+3, StateID.SA_ID:StateID.SA_ID+3] = np.diag(
            initstate_std.imuerror.accscale ** 2
        )
    
    def imu_compensate(self, imu: IMU) -> None:
        """Compensate IMU errors
        
        Args:
            imu: IMU data to be compensated
        """
        # Apply scale factor and bias corrections
        # Scale factor: 1 + scale_error
        imu.dtheta = imu.dtheta * (1.0 + self._imuerror.gyrscale) - self._imuerror.gyrbias * imu.dt
        imu.dvel = imu.dvel * (1.0 + self._imuerror.accscale) - self._imuerror.accbias * imu.dt
    
    def is_to_update(self, imutime1: float, imutime2: float, updatetime: float) -> int:
        """Determine when to perform state update
        
        Args:
            imutime1: Previous IMU time
            imutime2: Current IMU time
            updatetime: Time to update state
            
        Returns:
            0: No update needed
            1: Update previous state
            2: Update current state
            3: Interpolate IMU data to update time
        """
        # No need to update if GNSS time is later than current IMU time
        if updatetime > imutime2:
            return 0
        
        # Update previous state if GNSS time is close to previous IMU time
        if abs(updatetime - imutime1) < self._TIME_ALIGN_ERR:
            return 1
        
        # Update current state if GNSS time is close to current IMU time
        if abs(updatetime - imutime2) < self._TIME_ALIGN_ERR:
            return 2
        
        # Need to interpolate if GNSS time is between IMU times
        if imutime1 < updatetime < imutime2:
            return 3
        
        # Default: no update
        return 0
    
    def ins_propagation(self, imupre: IMU, imucur: IMU) -> None:
        """Perform INS state propagation and compute state transition matrix and noise matrix
        
        Args:
            imupre: Previous IMU data
            imucur: Current IMU data
        """
        # TODO: Implement INS propagation
        pass
    
    def gnss_update(self, gnssdata: GNSS) -> None:
        """Update state using GNSS position
        
        Args:
            gnssdata: GNSS data
        """
        # TODO: Implement GNSS update
        pass
    
    def ekf_predict(self, Phi: np.ndarray, Qd: np.ndarray) -> None:
        """Kalman Filter prediction step
        
        Args:
            Phi: State transition matrix
            Qd: Process noise matrix
        """
        # State covariance propagation
        self._Cov = Phi @ self._Cov @ Phi.T + Qd
        
        # Check for negative diagonal elements
        self.check_cov()
    
    def ekf_update(self, dz: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        """Kalman Filter update step
        
        Args:
            dz: Measurement innovation
            H: Measurement matrix
            R: Measurement noise matrix
        """
        # Calculate Kalman gain
        PHT = self._Cov @ H.T
        S = H @ PHT + R
        K = PHT @ np.linalg.inv(S)
        
        # Update state correction vector
        self._dx = K @ dz
        
        # Update state covariance
        I_KH = np.eye(self.RANK) - K @ H
        self._Cov = I_KH @ self._Cov @ I_KH.T + K @ R @ K.T  # Joseph form
        
        # Check for negative diagonal elements
        self.check_cov()
        
        # Apply state correction
        self.state_feedback()
    
    def state_feedback(self) -> None:
        """Apply state correction vector to current state"""
        # TODO: Implement state feedback
        pass
    
    def check_cov(self) -> None:
        """Check if covariance diagonal elements are all positive"""
        for i in range(self.RANK):
            if self._Cov[i, i] < 0:
                raise RuntimeError(f"Covariance is negative at {self._timestamp:.10f}!") 