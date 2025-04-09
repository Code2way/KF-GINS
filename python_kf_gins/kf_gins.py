#!/usr/bin/env python3
"""
KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System
Python implementation

Original C++ version:
Copyright (C) 2022 i2Nav Group, Wuhan University
Author: Liqiang Wang
Contact: wlq@whu.edu.cn

Python version translated by Claude.
"""

import argparse
import os
import sys
import time
import numpy as np
import yaml
from pathlib import Path

from kf_gins.common.angle import Angle
from kf_gins.common.types import IMU, GNSS
from kf_gins.fileio.imufileloader import ImuFileLoader
from kf_gins.fileio.gnssfileloader import GnssFileLoader
from kf_gins.fileio.filesaver import FileSaver
from kf_gins.kf_gins_types import GINSOptions, NavState, ImuError, ImuNoise
from kf_gins.gi_engine import GIEngine

def load_config(config_data, options):
    """Load configuration from YAML data
    
    Args:
        config_data: YAML configuration data
        options: GINSOptions object to populate
        
    Returns:
        True if successful, False otherwise
    """
    try:
        # Initial position, velocity, attitude
        vec1 = config_data["initpos"]
        vec2 = config_data["initvel"]
        vec3 = config_data["initatt"]
        
        # Convert to proper units and set in options
        for i in range(3):
            options.initstate.pos[i] = vec1[i] * Angle.D2R
            options.initstate.vel[i] = vec2[i]
            options.initstate.euler[i] = vec3[i] * Angle.D2R
            
        # Height doesn't need rad->deg conversion
        options.initstate.pos[2] = vec1[2]
        
        # IMU error initial values
        vec4 = config_data["initgyrbias"]
        vec5 = config_data["initaccbias"]
        vec6 = config_data["initgyrscale"]
        vec7 = config_data["initaccscale"]
        
        # Convert to proper units and set in options
        for i in range(3):
            options.initstate.imuerror.gyrbias[i] = vec4[i] * Angle.D2R / 3600.0
            options.initstate.imuerror.accbias[i] = vec5[i] / 1e5
            options.initstate.imuerror.gyrscale[i] = vec6[i] / 1e6
            options.initstate.imuerror.accscale[i] = vec7[i] / 1e6
            
        # Initial state standard deviation
        vec1 = config_data["initposstd"]
        vec2 = config_data["initvelstd"]
        vec3 = config_data["initattstd"]
        vec4 = config_data["gyrbiasstd"]
        vec5 = config_data["accbiasstd"]
        vec6 = config_data["gyrscalestd"]
        vec7 = config_data["accscalestd"]
        
        # Convert to proper units and set in options
        for i in range(3):
            options.initstate_std.pos[i] = vec1[i]
            options.initstate_std.vel[i] = vec2[i]
            options.initstate_std.euler[i] = vec3[i] * Angle.D2R
            options.initstate_std.imuerror.gyrbias[i] = vec4[i] * Angle.D2R / 3600.0
            options.initstate_std.imuerror.accbias[i] = vec5[i] / 1e5
            options.initstate_std.imuerror.gyrscale[i] = vec6[i] / 1e6
            options.initstate_std.imuerror.accscale[i] = vec7[i] / 1e6
        
        # IMU noise parameters
        vec1 = config_data["imunoise"]["arw"]
        vec2 = config_data["imunoise"]["vrw"]
        vec3 = config_data["imunoise"]["gbstd"]
        vec4 = config_data["imunoise"]["abstd"]
        vec5 = config_data["imunoise"]["gsstd"]
        vec6 = config_data["imunoise"]["asstd"]
        corr_time = config_data["imunoise"]["corrtime"]
        
        # Convert to proper units and set in options
        options.imunoise = ImuNoise()
        for i in range(3):
            options.imunoise.gyr_arw[i] = vec1[i] * Angle.D2R / 60.0
            options.imunoise.acc_vrw[i] = vec2[i] / 60.0
            options.imunoise.gyrbias_std[i] = vec3[i] * Angle.D2R / 3600.0
            options.imunoise.accbias_std[i] = vec4[i] / 1e5
            options.imunoise.gyrscale_std[i] = vec5[i] / 1e6
            options.imunoise.accscale_std[i] = vec6[i] / 1e6
        options.imunoise.corr_time = corr_time * 3600.0
        
        # GNSS lever arm
        antlever = config_data["antlever"]
        for i in range(3):
            options.antlever[i] = antlever[i]
            
        return True
    except Exception as e:
        print(f"Failed when loading configuration: {e}")
        return False

def write_nav_result(time, navstate, navfile, imuerrfile):
    """Write navigation results to output files
    
    Args:
        time: Current time
        navstate: Current navigation state
        navfile: FileSaver for navigation results
        imuerrfile: FileSaver for IMU error results
    """
    # Write navigation results
    nav_data = np.zeros(11)
    nav_data[0] = time // 604800  # GNSS week
    nav_data[1] = time % 604800   # Time of week
    nav_data[2:5] = [
        Angle.rad2deg(navstate.pos[0]),
        Angle.rad2deg(navstate.pos[1]),
        navstate.pos[2]
    ]
    nav_data[5:8] = navstate.vel
    nav_data[8:11] = Angle.rad2deg(navstate.euler)
    
    navfile.dump(nav_data)
    
    # Write IMU error results
    imuerr_data = np.zeros(13)
    imuerr_data[0] = time
    imuerr_data[1:4] = Angle.rad2deg(navstate.imuerror.gyrbias) * 3600.0  # deg/h
    imuerr_data[4:7] = navstate.imuerror.accbias * 1e5                   # mGal
    imuerr_data[7:10] = navstate.imuerror.gyrscale * 1e6                 # ppm
    imuerr_data[10:13] = navstate.imuerror.accscale * 1e6                # ppm
    
    imuerrfile.dump(imuerr_data)

def write_std(time, cov, stdfile):
    """Write standard deviations to output file
    
    Args:
        time: Current time
        cov: Covariance matrix
        stdfile: FileSaver for standard deviation results
    """
    std_data = np.zeros(22)
    std_data[0] = time
    
    # Extract standard deviations from covariance matrix
    for i in range(21):
        std_data[i+1] = np.sqrt(cov[i, i])
        
    # Convert angular errors from rad to deg
    for i in range(6, 9):
        std_data[i+1] = Angle.rad2deg(std_data[i+1])
        
    # Convert gyro bias errors from rad/s to deg/h
    for i in range(9, 12):
        std_data[i+1] = Angle.rad2deg(std_data[i+1]) * 3600.0
        
    # Convert accelerometer bias errors from m/s² to mGal
    for i in range(12, 15):
        std_data[i+1] = std_data[i+1] * 1e5
        
    # Convert scale factor errors to ppm
    for i in range(15, 21):
        std_data[i+1] = std_data[i+1] * 1e6
        
    stdfile.dump(std_data)

def main():
    """Main function for KF-GINS"""
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System")
    parser.add_argument("config", help="Path to configuration YAML file")
    args = parser.parse_args()
    
    print("\nKF-GINS: An EKF-Based GNSS/INS Integrated Navigation System\n")
    
    start_time = time.time()
    
    # Load configuration file
    try:
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Failed to read configuration file: {e}")
        print("Please check the path and format of the configuration file!")
        return -1
    
    # Read configuration parameters into GINSOptions
    options = GINSOptions()
    if not load_config(config, options):
        print("Error occurs in the configuration file!")
        return -1
    
    # Read file paths
    try:
        imupath = config["imupath"]
        gnsspath = config["gnsspath"]
        outputpath = config["outputpath"]
    except Exception as e:
        print(f"Failed when loading configuration: {e}")
        print("Please check the file path and output path!")
        return -1
    
    # Load IMU data configuration and processing time interval
    try:
        imudatalen = config["imudatalen"]
        imudatarate = config["imudatarate"]
        starttime = config["starttime"]
        endtime = config["endtime"]
    except Exception as e:
        print(f"Failed when loading configuration: {e}")
        print("Please check the data length, data rate, and the process time!")
        return -1
    
    # Create output directory if it doesn't exist
    os.makedirs(outputpath, exist_ok=True)
    
    # Load GNSS and IMU files
    gnssfile = GnssFileLoader(gnsspath)
    imufile = ImuFileLoader(imupath, imudatalen, imudatarate)
    
    # Construct GIEngine
    giengine = GIEngine(options)
    
    # Create output files
    # navfile: gnssweek(1) + time(1) + pos(3) + vel(3) + euler angle(3) = 11
    # imuerrfile: time(1) + gyrbias(3) + accbias(3) + gyrscale(3) + accscale(3) = 13
    # stdfile: time(1) + pva_std(9) + imubias_std(6) + imuscale_std(6) = 22
    nav_columns = 11
    imuerr_columns = 13
    std_columns = 22
    
    navfile = FileSaver(os.path.join(outputpath, "KF_GINS_Navresult.nav"), nav_columns)
    imuerrfile = FileSaver(os.path.join(outputpath, "KF_GINS_IMU_ERR.txt"), imuerr_columns)
    stdfile = FileSaver(os.path.join(outputpath, "KF_GINS_STD.txt"), std_columns)
    
    # Check if files are opened correctly
    if (not gnssfile.is_open() or not imufile.is_open() or
        not navfile.is_open() or not imuerrfile.is_open() or not stdfile.is_open()):
        print("Failed to open data file!")
        return -1
    
    # Check processing time
    if endtime < 0:
        endtime = imufile.endtime()
    
    if endtime > 604800 or starttime < imufile.starttime() or starttime > endtime:
        print("Process time ERROR!")
        return -1
    
    # Data alignment
    imu_cur = None
    
    # Skip IMU data until starttime
    while True:
        imu_cur = imufile.next()
        if imu_cur.time >= starttime:
            break
    
    # Skip GNSS data until after starttime
    gnss = None
    while True:
        gnss = gnssfile.next()
        if gnss.time > starttime:
            break
    
    # Add initial IMU and GNSS data to GIEngine
    giengine.add_imu_data(imu_cur, True)
    giengine.add_gnss_data(gnss)
    
    # For storing processing results
    timestamp = 0.0
    navstate = NavState()
    cov = np.zeros((21, 21))
    
    # For displaying processing progress
    percent = 0
    last_percent = 0
    interval = endtime - starttime
    
    # Main processing loop
    while True:
        # Load new GNSS data when current state time is newer than GNSS time
        if gnss.time < imu_cur.time and not gnssfile.is_eof():
            gnss = gnssfile.next()
            giengine.add_gnss_data(gnss)
        
        # Load new IMU data
        imu_cur = imufile.next()
        if imu_cur.time > endtime or imufile.is_eof():
            break
            
        giengine.add_imu_data(imu_cur)
        
        # Process new IMU data
        giengine.new_imu_process()
        
        # Get current timestamp, navigation state and covariance
        timestamp = giengine.timestamp()
        navstate = giengine.get_nav_state()
        cov = giengine.get_covariance()
        
        # Save processing results
        write_nav_result(timestamp, navstate, navfile, imuerrfile)
        write_std(timestamp, cov, stdfile)
        
        # Display processing progress
        percent = int((imu_cur.time - starttime) / interval * 100)
        if percent - last_percent >= 1:
            print(f" - Processing: {percent:3d}%", end="\r", flush=True)
            last_percent = percent
    
    # Close all files
    imufile.close()
    gnssfile.close()
    navfile.close()
    imuerrfile.close()
    stdfile.close()
    
    # Process finished
    end_time = time.time()
    print("\n\nKF-GINS Process Finish! ", end="")
    print(f"From {starttime} s to {endtime} s, total {interval} s!")
    print(f"Cost {end_time - start_time:.2f} s in total")
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 