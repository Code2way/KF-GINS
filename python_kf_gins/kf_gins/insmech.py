"""
Python implementation of INS Mechanization.
Translated from the original C++ implementation.
"""

import numpy as np
from .common.earth import Earth, WGS84
from .common.rotation import Rotation
from .common.types import IMU
from .kf_gins_types import PVA

class INSMech:
    """INS Mechanization algorithms"""

    @staticmethod
    def insmech(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU) -> None:
        """INS Mechanization, update velocity, position and attitude using IMU data
        
        Args:
            pvapre: Previous PVA state
            pvacur: Current PVA state (to be updated)
            imupre: Previous IMU data
            imucur: Current IMU data
        """
        # Perform velocity update, position update and attitude update in sequence (not reversible)
        INSMech.vel_update(pvapre, pvacur, imupre, imucur)
        INSMech.pos_update(pvapre, pvacur, imupre, imucur)
        INSMech.att_update(pvapre, pvacur, imupre, imucur)

    @staticmethod
    def vel_update(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU) -> None:
        """Velocity update
        
        Args:
            pvapre: Previous PVA state
            pvacur: Current PVA state (to be updated)
            imupre: Previous IMU data
            imucur: Current IMU data
        """
        # Identity matrix
        I33 = np.eye(3)
        
        # Calculate geographic parameters: Meridian and Prime Vertical radii,
        # earth rotation angular velocity in n-frame,
        # angular velocity of n-frame relative to e-frame in n-frame, and gravity
        rmrn = Earth.meridian_prime_vertical_radius(pvapre.pos[0])
        
        # Earth rotation rate in n-frame
        wie_n = np.array([
            WGS84.WIE * np.cos(pvapre.pos[0]),
            0,
            -WGS84.WIE * np.sin(pvapre.pos[0])
        ])
        
        # Angular rate of n-frame relative to e-frame, expressed in n-frame
        wen_n = np.array([
            pvapre.vel[1] / (rmrn[1] + pvapre.pos[2]),
            -pvapre.vel[0] / (rmrn[0] + pvapre.pos[2]),
            -pvapre.vel[1] * np.tan(pvapre.pos[0]) / (rmrn[1] + pvapre.pos[2])
        ])
        
        gravity = Earth.gravity(pvapre.pos)
        
        # Rotational and sculling motion compensation
        temp1 = np.cross(imucur.dtheta, imucur.dvel) / 2
        temp2 = np.cross(imupre.dtheta, imucur.dvel) / 12
        temp3 = np.cross(imupre.dvel, imucur.dtheta) / 12
        
        # Specific force increment in b-frame
        d_vfb = imucur.dvel + temp1 + temp2 + temp3
        
        # Specific force increment projected to n-frame
        temp1 = (wie_n + wen_n) * imucur.dt / 2
        cnn = I33 - Rotation.skew_symmetric(temp1)
        d_vfn = cnn @ pvapre.att.cbn @ d_vfb
        
        # Gravity and Coriolis increment
        gl = np.array([0, 0, gravity])
        d_vgn = (gl - np.cross(2 * wie_n + wen_n, pvapre.vel)) * imucur.dt
        
        # Velocity at k-1/2
        midvel = pvapre.vel + (d_vfn + d_vgn) / 2
        
        # Position extrapolation to k-1/2
        qnn = Rotation.rotvec2quaternion(temp1)
        temp2 = np.array([0, 0, -WGS84.WIE * imucur.dt / 2])
        qee = Rotation.rotvec2quaternion(temp2)
        qne = Earth.qne(pvapre.pos)
        qne = Rotation.quaternion_right(qee) @ Rotation.quaternion_right(qne) @ qnn
        
        midpos = np.zeros(3)
        midpos[2] = pvapre.pos[2] - midvel[2] * imucur.dt / 2
        midpos = Earth.blh_from_qne(qne, midpos[2])
        
        # Recompute rmrn, wie_n, and wen_n at k-1/2
        rmrn = Earth.meridian_prime_vertical_radius(midpos[0])
        wie_n = np.array([
            WGS84.WIE * np.cos(midpos[0]),
            0,
            -WGS84.WIE * np.sin(midpos[0])
        ])
        wen_n = np.array([
            midvel[1] / (rmrn[1] + midpos[2]),
            -midvel[0] / (rmrn[0] + midpos[2]),
            -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])
        ])
        
        # Recompute specific force increment in n-frame
        temp3 = (wie_n + wen_n) * imucur.dt / 2
        cnn = I33 - Rotation.skew_symmetric(temp3)
        d_vfn = cnn @ pvapre.att.cbn @ d_vfb
        
        # Recompute gravity and Coriolis increment
        gl = np.array([0, 0, Earth.gravity(midpos)])
        d_vgn = (gl - np.cross(2 * wie_n + wen_n, midvel)) * imucur.dt
        
        # Velocity update complete
        pvacur.vel = pvapre.vel + d_vfn + d_vgn

    @staticmethod
    def pos_update(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU) -> None:
        """Position update
        
        Args:
            pvapre: Previous PVA state
            pvacur: Current PVA state (to be updated)
            imupre: Previous IMU data
            imucur: Current IMU data
        """
        # Recompute velocity and position at k-1/2
        midvel = (pvacur.vel + pvapre.vel) / 2
        midpos = pvapre.pos + Earth.DRi(pvapre.pos) @ midvel * imucur.dt / 2
        
        # Recompute rmrn, wie_n, wen_n at k-1/2
        rmrn = Earth.meridian_prime_vertical_radius(midpos[0])
        
        wie_n = np.array([
            WGS84.WIE * np.cos(midpos[0]),
            0,
            -WGS84.WIE * np.sin(midpos[0])
        ])
        
        wen_n = np.array([
            midvel[1] / (rmrn[1] + midpos[2]),
            -midvel[0] / (rmrn[0] + midpos[2]),
            -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])
        ])
        
        # Recompute n-frame rotation vector (n(k) with respect to n(k-1)-frame)
        temp1 = (wie_n + wen_n) * imucur.dt
        qnn = Rotation.rotvec2quaternion(temp1)
        
        # e-frame rotation vector (e(k-1) with respect to e(k)-frame)
        temp2 = np.array([0, 0, -WGS84.WIE * imucur.dt])
        qee = Rotation.rotvec2quaternion(temp2)
        
        # Position update complete
        qne = Earth.qne(pvapre.pos)
        qne = Rotation.quaternion_right(qee) @ Rotation.quaternion_right(qne) @ qnn
        
        pvacur.pos = np.zeros(3)
        pvacur.pos[2] = pvapre.pos[2] - midvel[2] * imucur.dt
        pvacur.pos = Earth.blh_from_qne(qne, pvacur.pos[2])

    @staticmethod
    def att_update(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU) -> None:
        """Attitude update
        
        Args:
            pvapre: Previous PVA state
            pvacur: Current PVA state (to be updated)
            imupre: Previous IMU data
            imucur: Current IMU data
        """
        # Recompute velocity and position at k-1/2
        midvel = (pvapre.vel + pvacur.vel) / 2
        
        qne_pre = Earth.qne(pvapre.pos)
        qne_cur = Earth.qne(pvacur.pos)
        
        # Calculate transformation between qne_pre and qne_cur
        # First convert quaternions to matrices for inversion
        qne_pre_mat = Rotation.quaternion2matrix(qne_pre)
        qne_cur_mat = Rotation.quaternion2matrix(qne_cur)
        
        # Get rotation vector from inverse transformation
        temp1 = Rotation.quaternion2vector(
            Rotation.matrix2quaternion(qne_cur_mat.T @ qne_pre_mat)
        )
        
        # Calculate mid-point quaternion
        qne_mid = Rotation.quaternion_right(qne_pre) @ Rotation.quaternion_right(
            Rotation.rotvec2quaternion(-temp1 / 2)  # Note the negative sign for inverse
        )
        
        midpos = np.zeros(3)
        midpos[2] = (pvacur.pos[2] + pvapre.pos[2]) / 2
        midpos = Earth.blh_from_qne(qne_mid, midpos[2])
        
        # Recompute rmrn, wie_n, wen_n at k-1/2
        rmrn = Earth.meridian_prime_vertical_radius(midpos[0])
        
        wie_n = np.array([
            WGS84.WIE * np.cos(midpos[0]),
            0,
            -WGS84.WIE * np.sin(midpos[0])
        ])
        
        wen_n = np.array([
            midvel[1] / (rmrn[1] + midpos[2]),
            -midvel[0] / (rmrn[0] + midpos[2]),
            -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])
        ])
        
        # n-frame rotation vector (n(k-1) with respect to n(k)-frame)
        temp1 = -(wie_n + wen_n) * imucur.dt
        qnn = Rotation.rotvec2quaternion(temp1)
        
        # b-frame rotation vector (b(k) with respect to b(k-1)-frame)
        # compensate the second-order coning correction term
        temp1 = imucur.dtheta + np.cross(imupre.dtheta, imucur.dtheta) / 12
        qbb = Rotation.rotvec2quaternion(temp1)
        
        # Attitude update complete
        pvacur.att.qbn = Rotation.quaternion_right(qnn) @ pvapre.att.qbn @ qbb
        pvacur.att.cbn = Rotation.quaternion2matrix(pvacur.att.qbn)
        pvacur.att.euler = Rotation.matrix2euler(pvacur.att.cbn) 