#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Convert sensor CSV data to KF-GINS IMU format

KF-GINS IMU text file format is defined as:
| Columns | Data description         | Units |
| ------- | ------------------------ | ----- |
| 1       | GNSS seconds of week     | s     |
| 2~4     | X-Y-Z axes incremental angles    | rad   |
| 5~7     | X-Y-Z axes incremental velocity  | m/s   |
"""

import os
import sys
import numpy as np
import pandas as pd
import re
from pathlib import Path
from datetime import datetime

def fix_csv_header(csv_file, temp_file=None):
    """
    Fix header issues in the CSV file
    
    Args:
        csv_file: Input CSV file
        temp_file: Temporary file to write fixed CSV
    
    Returns:
        Path to the fixed CSV file
    """
    if temp_file is None:
        temp_file = f"{csv_file}.fixed.csv"
    
    try:
        with open(csv_file, 'r') as f_in, open(temp_file, 'w') as f_out:
            # Read and fix header
            header = f_in.readline().strip()
            
            # Fix common typos
            if 'Gryo.temp' in header:
                header = header.replace('Gryo.temp', 'Gyro.temp')
            
            # Sometimes columns get split over multiple lines
            if header.count(',') < 10:  # Too few columns, likely corrupted
                # Read next line and append
                next_line = f_in.readline().strip()
                header = header + next_line
            
            # Write fixed header
            f_out.write(header + '\n')
            
            # Copy the rest of the file
            for line in f_in:
                f_out.write(line)
        
        return temp_file
    except Exception as e:
        print(f"Error fixing CSV header: {e}")
        return csv_file

def detect_sampling_rate(df, timestamp_col='timestamp'):
    """
    Detect the sampling rate from the data
    
    Args:
        df: DataFrame with the data
        timestamp_col: Name of the timestamp column
    
    Returns:
        Estimated sampling rate in Hz
    """
    try:
        # Get unique timestamps sorted
        times = sorted(df[timestamp_col].unique())
        
        # Calculate time differences
        diffs = [times[i+1] - times[i] for i in range(len(times)-1)]
        
        # Get the most common time difference
        if len(diffs) > 0:
            # Convert to ms if needed
            avg_diff = np.median(diffs)
            if avg_diff > 1000:  # If time is in milliseconds
                avg_diff /= 1000  # Convert to seconds
            
            if avg_diff > 0:
                rate = 1.0 / avg_diff
                print(f"Detected sampling rate: {rate:.2f} Hz")
                return int(round(rate))
    except Exception as e:
        print(f"Error detecting sampling rate: {e}")
    
    # Default to 200 Hz if detection fails
    print("Using default sampling rate: 200 Hz")
    return 200

def csv_to_imu(csv_file, output_file, sampling_rate=None, start_time=None, scale_factor=1.0):
    """
    Convert sensor CSV data to KF-GINS IMU format
    
    Args:
        csv_file: Input CSV file with sensor data
        output_file: Output IMU text file
        sampling_rate: IMU sampling rate in Hz (auto-detect if None)
        start_time: Start time in GNSS seconds of week (optional)
        scale_factor: Scale factor for accelerometer data (default: 1.0)
    """
    print(f"Converting {csv_file} to {output_file}...")
    
    # Fix CSV header issues
    fixed_csv = fix_csv_header(csv_file)
    
    # Read the CSV file
    try:
        df = pd.read_csv(fixed_csv)
        print(f"Successfully read CSV file with {len(df)} rows and {len(df.columns)} columns.")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        try:
            # Try with explicit encoding
            df = pd.read_csv(fixed_csv, encoding='utf-8')
        except Exception as e2:
            print(f"Second attempt failed: {e2}")
            try:
                # Try with a more flexible approach
                print("Trying to read CSV with flexible parser...")
                df = pd.read_csv(fixed_csv, sep=None, engine='python')
            except Exception as e3:
                print(f"All attempts failed: {e3}")
                sys.exit(1)
    
    # Print column names for debugging
    print("Available columns:")
    for i, col in enumerate(df.columns):
        print(f"  [{i}] {col}")
    
    # Auto-detect sampling rate if not provided
    if sampling_rate is None:
        sampling_rate = detect_sampling_rate(df)
    
    # Calculate time interval (dt) between samples
    dt = 1.0 / sampling_rate
    print(f"Using sampling rate of {sampling_rate} Hz (dt = {dt:.6f} s)")
    
    # Map column names
    # Try to find the best match for each required column
    column_mapping = {
        'timestamp': 'timestamp',
        'gyro_x': 'Gyro.x',
        'gyro_y': 'Gyro.y',
        'gyro_z': 'Gyro.z',
        'acc_x': 'Gsns.x',
        'acc_y': 'Gsns.y',
        'acc_z': 'Gsns.z'
    }
    
    # Try to find the best match for each column
    for target, default in column_mapping.items():
        if default not in df.columns:
            # Try to find a similar column name
            matches = [col for col in df.columns if re.search(default.replace('.', '\.?').lower(), col.lower())]
            if matches:
                column_mapping[target] = matches[0]
                print(f"Mapping '{target}' to '{matches[0]}'")
            else:
                print(f"Warning: Could not find a match for '{target}' (default: '{default}')")
    
    # Create output DataFrame
    output_data = []
    
    # Determine GNSS seconds of week base
    if start_time is not None:
        gnss_sow_base = start_time
    else:
        # Use the timestamp directly as seconds of week
        gnss_sow_base = 0
    
    # For each timestamp in the input data
    timestamps = df[column_mapping['timestamp']].unique()
    timestamps = sorted(timestamps)
    
    print(f"Processing {len(timestamps)} unique timestamps...")
    
    for t in timestamps:
        # Get rows with the current timestamp
        rows = df[df[column_mapping['timestamp']] == t]
        
        # Calculate GNSS seconds of week - assuming timestamps are in milliseconds
        gnss_sow = gnss_sow_base + (t / 1000.0)
        
        try:
            # Calculate the average of gyroscope data for this timestamp
            avg_gyr_x = rows[column_mapping['gyro_x']].mean() / 262.14
            avg_gyr_y = rows[column_mapping['gyro_y']].mean() / 262.14
            avg_gyr_z = rows[column_mapping['gyro_z']].mean() / 262.14
            
            # Calculate the average of accelerometer data for this timestamp
            avg_acc_x = rows[column_mapping['acc_x']].mean() / 16384 * 9.81
            avg_acc_y = rows[column_mapping['acc_y']].mean() / 16384 * 9.81
            avg_acc_z = rows[column_mapping['acc_z']].mean() / 16384 * 9.81
            
            # Convert averaged angular rates to incremental angles (rad)
            dtheta_x = (avg_gyr_x * np.pi / 180.0) * dt
            dtheta_y = (avg_gyr_y * np.pi / 180.0) * dt
            dtheta_z = (avg_gyr_z * np.pi / 180.0) * dt
            
            # Convert averaged accelerations to incremental velocities (m/s)
            dvel_x = avg_acc_x * dt
            dvel_y = avg_acc_y * dt
            dvel_z = avg_acc_z * dt
            
            # Append to output data
            output_data.append([gnss_sow, dtheta_x, dtheta_y, dtheta_z, dvel_x, dvel_y, dvel_z])
            
        except Exception as e:
            print(f"Error processing rows for timestamp {t}: {e}")
            continue
    
    # Create output DataFrame and sort by time
    output_df = pd.DataFrame(output_data, columns=['time', 'dtheta_x', 'dtheta_y', 'dtheta_z', 'dvel_x', 'dvel_y', 'dvel_z'])
    output_df = output_df.sort_values(by='time')
    
    # Check for gaps in the data
    times = output_df['time'].values
    diffs = times[1:] - times[:-1]
    max_diff = np.max(diffs) if len(diffs) > 0 else 0
    if max_diff > 5 * dt:
        print(f"Warning: Found gaps in the data. Maximum time difference: {max_diff:.6f} s (expected: {dt:.6f} s)")
    
    # Save output data to text file
    np.savetxt(output_file, output_df.values, fmt='%.9f')
    
    print(f"Conversion complete. Output file: {output_file}")
    print(f"Converted {len(output_data)} IMU samples spanning {times[-1] - times[0]:.2f} seconds.")
    print(f"Averaged {len(df)} raw IMU readings into {len(output_data)} unique timestamps.")
    
    # Cleanup temporary file if created
    if fixed_csv != csv_file and os.path.exists(fixed_csv):
        try:
            os.remove(fixed_csv)
        except:
            pass

def main():
    # Define input parameters
    csv_file = "/mnt/d/GitHub/KF-GINS/bin/250331062313_000_Sensor.csv"
    output_file = "sensor_imu.txt"
    sampling_rate = 10  # Auto-detect
    start_time = None     # Auto-detect
    scale_factor = 1.0    # No scaling
    
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    csv_to_imu(csv_file, output_file, sampling_rate, start_time, scale_factor)

if __name__ == "__main__":
    main() 