"""
Python implementation of Earth model utilities.
Translated from the original C++ implementation.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple
from .types import Pose

# WGS84 ellipsoid model parameters
class WGS84:
    """WGS84 ellipsoid model constants"""
    WIE = 7.2921151467E-5      # Earth rotation rate (rad/s)
    F = 0.0033528106647474805  # Flattening
    RA = 6378137.0000000000    # Semi-major axis (m)
    RB = 6356752.3142451793    # Semi-minor axis (m)
    GM0 = 398600441800000.00   # Earth's gravitational constant (m³/s²)
    E1 = 0.0066943799901413156 # First eccentricity squared
    E2 = 0.0067394967422764341 # Second eccentricity squared

class Earth:
    """Earth model utilities for navigation calculations"""

    @staticmethod
    def gravity(blh: np.ndarray) -> float:
        """Calculate normal gravity
        
        Args:
            blh: Geodetic coordinates [latitude, longitude, height]
            
        Returns:
            Normal gravity value in m/s²
        """
        sin2 = np.sin(blh[0])**2
        return (9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
                blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 
                0.0000000000007211 * blh[2] * blh[2])

    @staticmethod
    def meridian_prime_vertical_radius(lat: float) -> np.ndarray:
        """Calculate meridian and prime vertical radius
        
        Args:
            lat: Latitude in radians
            
        Returns:
            Array of [meridian radius, prime vertical radius]
        """
        tmp = np.sin(lat)**2
        tmp = 1 - WGS84.E1 * tmp
        sqrttmp = np.sqrt(tmp)
        
        return np.array([
            WGS84.RA * (1 - WGS84.E1) / (sqrttmp * tmp),
            WGS84.RA / sqrttmp
        ])

    @staticmethod
    def RN(lat: float) -> float:
        """Calculate prime vertical radius
        
        Args:
            lat: Latitude in radians
            
        Returns:
            Prime vertical radius
        """
        sinlat = np.sin(lat)
        return WGS84.RA / np.sqrt(1.0 - WGS84.E1 * sinlat * sinlat)

    @staticmethod
    def cne(blh: np.ndarray) -> np.ndarray:
        """Calculate transformation matrix from navigation frame to ECEF frame
        
        Args:
            blh: Geodetic coordinates [latitude, longitude, height]
            
        Returns:
            3x3 transformation matrix
        """
        sinlat = np.sin(blh[0])
        sinlon = np.sin(blh[1])
        coslat = np.cos(blh[0])
        coslon = np.cos(blh[1])

        dcm = np.zeros((3, 3))
        dcm[0, 0] = -sinlat * coslon
        dcm[0, 1] = -sinlon
        dcm[0, 2] = -coslat * coslon

        dcm[1, 0] = -sinlat * sinlon
        dcm[1, 1] = coslon
        dcm[1, 2] = -coslat * sinlon

        dcm[2, 0] = coslat
        dcm[2, 1] = 0
        dcm[2, 2] = -sinlat

        return dcm

    @staticmethod
    def qne(blh: np.ndarray) -> np.ndarray:
        """Calculate quaternion from navigation frame to ECEF frame
        
        Args:
            blh: Geodetic coordinates [latitude, longitude, height]
            
        Returns:
            Quaternion [w, x, y, z]
        """
        coslon = np.cos(blh[1] * 0.5)
        sinlon = np.sin(blh[1] * 0.5)
        coslat = np.cos(-np.pi * 0.25 - blh[0] * 0.5)
        sinlat = np.sin(-np.pi * 0.25 - blh[0] * 0.5)

        return np.array([
            coslat * coslon,
            -sinlat * sinlon,
            sinlat * coslon,
            coslat * sinlon
        ])

    @staticmethod
    def blh_from_qne(qne: np.ndarray, height: float) -> np.ndarray:
        """Calculate geodetic coordinates from navigation to ECEF quaternion
        
        Args:
            qne: Quaternion [w, x, y, z]
            height: Height above ellipsoid
            
        Returns:
            Geodetic coordinates [latitude, longitude, height]
        """
        return np.array([
            -2 * np.arctan(qne[2] / qne[0]) - np.pi * 0.5,
            2 * np.arctan2(qne[3], qne[0]),
            height
        ])

    @staticmethod
    def blh2ecef(blh: np.ndarray) -> np.ndarray:
        """Convert geodetic coordinates to ECEF coordinates
        
        Args:
            blh: Geodetic coordinates [latitude, longitude, height]
            
        Returns:
            ECEF coordinates [x, y, z]
        """
        coslat = np.cos(blh[0])
        sinlat = np.sin(blh[0])
        coslon = np.cos(blh[1])
        sinlon = np.sin(blh[1])

        rn = Earth.RN(blh[0])
        rnh = rn + blh[2]

        return np.array([
            rnh * coslat * coslon,
            rnh * coslat * sinlon,
            (rnh - rn * WGS84.E1) * sinlat
        ])

    @staticmethod
    def ecef2blh(ecef: np.ndarray) -> np.ndarray:
        """Convert ECEF coordinates to geodetic coordinates
        
        Args:
            ecef: ECEF coordinates [x, y, z]
            
        Returns:
            Geodetic coordinates [latitude, longitude, height]
        """
        p = np.sqrt(ecef[0]**2 + ecef[1]**2)
        lat = np.arctan(ecef[2] / (p * (1.0 - WGS84.E1)))
        lon = 2.0 * np.arctan2(ecef[1], ecef[0] + p)

        h = 0
        while True:
            h2 = h
            rn = Earth.RN(lat)
            h = p / np.cos(lat) - rn
            lat = np.arctan(ecef[2] / (p * (1.0 - WGS84.E1 * rn / (rn + h))))
            if abs(h - h2) <= 1.0e-4:
                break

        return np.array([lat, lon, h])

    @staticmethod
    def DRi(blh: np.ndarray) -> np.ndarray:
        """Calculate matrix for converting navigation frame relative position to geodetic relative position
        
        Args:
            blh: Geodetic coordinates [latitude, longitude, height]
            
        Returns:
            3x3 transformation matrix
        """
        rmn = Earth.meridian_prime_vertical_radius(blh[0])
        dri = np.zeros((3, 3))
        dri[0, 0] = 1.0 / (rmn[0] + blh[2])
        dri[1, 1] = 1.0 / ((rmn[1] + blh[2]) * np.cos(blh[0]))
        dri[2, 2] = -1
        return dri

    @staticmethod
    def DR(blh: np.ndarray) -> np.ndarray:
        """Calculate matrix for converting geodetic relative position to navigation frame relative position
        
        Args:
            blh: Geodetic coordinates [latitude, longitude, height]
            
        Returns:
            3x3 transformation matrix
        """
        rmn = Earth.meridian_prime_vertical_radius(blh[0])
        dr = np.zeros((3, 3))
        dr[0, 0] = rmn[0] + blh[2]
        dr[1, 1] = (rmn[1] + blh[2]) * np.cos(blh[0])
        dr[2, 2] = -1
        return dr

    @staticmethod
    def local2global(origin: np.ndarray, local: np.ndarray) -> np.ndarray:
        """Convert local coordinates to geodetic coordinates
        
        Args:
            origin: Origin in geodetic coordinates [latitude, longitude, height]
            local: Local coordinates [x, y, z]
            
        Returns:
            Geodetic coordinates [latitude, longitude, height]
        """
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)
        ecef1 = ecef0 + cn0e @ local
        return Earth.ecef2blh(ecef1)

    @staticmethod
    def global2local(origin: np.ndarray, global_: np.ndarray) -> np.ndarray:
        """Convert geodetic coordinates to local coordinates
        
        Args:
            origin: Origin in geodetic coordinates [latitude, longitude, height]
            global_: Target in geodetic coordinates [latitude, longitude, height]
            
        Returns:
            Local coordinates [x, y, z]
        """
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)
        ecef1 = Earth.blh2ecef(global_)
        return cn0e.T @ (ecef1 - ecef0)

    @staticmethod
    def local2global_pose(origin: np.ndarray, local: Pose) -> Pose:
        """Convert local pose to global pose
        
        Args:
            origin: Origin in geodetic coordinates [latitude, longitude, height]
            local: Local pose
            
        Returns:
            Global pose
        """
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)
        ecef1 = ecef0 + cn0e @ local.t
        blh1 = Earth.ecef2blh(ecef1)
        cn1e = Earth.cne(blh1)
        
        return Pose(
            R=cn1e.T @ cn0e @ local.R,
            t=blh1
        )

    @staticmethod
    def global2local_pose(origin: np.ndarray, global_: Pose) -> Pose:
        """Convert global pose to local pose
        
        Args:
            origin: Origin in geodetic coordinates [latitude, longitude, height]
            global_: Global pose
            
        Returns:
            Local pose
        """
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)
        ecef1 = Earth.blh2ecef(global_.t)
        cn1e = Earth.cne(global_.t)
        
        return Pose(
            R=cn0e.T @ cn1e @ global_.R,
            t=cn0e.T @ (ecef1 - ecef0)
        )

    @staticmethod
    def iewe() -> np.ndarray:
        """Earth rotation rate in ECEF frame
        
        Returns:
            Angular velocity vector [wx, wy, wz]
        """
        return np.array([0, 0, WGS84.WIE])

    @staticmethod
    def iewn(lat: float) -> np.ndarray:
        """Earth rotation rate in navigation frame
        
        Args:
            lat: Latitude in radians
            
        Returns:
            Angular velocity vector [wx, wy, wz]
        """
        return np.array([
            WGS84.WIE * np.cos(lat),
            0,
            -WGS84.WIE * np.sin(lat)
        ]) 