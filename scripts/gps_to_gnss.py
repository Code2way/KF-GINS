#!/usr/bin/env python3
import re
from datetime import datetime, timedelta
import math

def parse_gga(gga_line):
    """Parse GGA sentence to get position data"""
    fields = gga_line.split(',')
    if len(fields) < 15 or fields[6] == '0':  # No fix
        return None
    
    try:
        time = fields[1]
        lat = float(fields[2]) if fields[2] else 0.0
        lat_dir = fields[3]
        lon = float(fields[4]) if fields[4] else 0.0
        lon_dir = fields[5]
        alt = float(fields[9]) if fields[9] else 0.0
        
        # Convert DDMM.MMMM to decimal degrees
        lat_deg = int(lat / 100)
        lat_min = lat - lat_deg * 100
        lat = lat_deg + lat_min / 60
        if lat_dir == 'S':
            lat = -lat
            
        lon_deg = int(lon / 100)
        lon_min = lon - lon_deg * 100
        lon = lon_deg + lon_min / 60
        if lon_dir == 'W':
            lon = -lon
            
        return time, lat, lon, alt
    except (ValueError, IndexError):
        return None

def parse_gst(gst_line):
    """Parse GST sentence to get standard deviation data"""
    fields = gst_line.split(',')
    if len(fields) < 8:
        return None
    
    try:
        # Fields 6,7,8 contain std_lat, std_lon, std_alt
        std_lat = float(fields[6]) if fields[6] else 0.0
        std_lon = float(fields[7]) if fields[7] else 0.0
        std_alt = float(fields[8].split('*')[0]) if fields[8] else 0.0
        return std_lat, std_lon, std_alt
    except (ValueError, IndexError):
        return None

def convert_gps_log(input_file, output_file):
    """Convert GPS NMEA log to GNSS format"""
    current_data = {
        'timestamp': None,
        'position': None,
        'std': None
    }
    
    with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
        # Write header
        f_out.write("# TimeStamp,latitude(deg),longitude(deg),height(m),std_n(m),std_e(m),std_d(m)\n")
        
        for line in f_in:
            line = line.strip()
            
            # Parse HEAD message for timestamp
            if line.startswith('#HEAD'):
                fields = line.split(',')
                if len(fields) >= 3:
                    current_data['timestamp'] = int(fields[2])
                    continue
            
            # Parse GGA for position
            elif line.startswith('$GNGGA'):
                pos_data = parse_gga(line)
                if pos_data:
                    current_data['position'] = pos_data
            
            # Parse GST for standard deviations
            elif line.startswith('$GNGST'):
                std_data = parse_gst(line)
                if std_data:
                    current_data['std'] = std_data
            
            # If we have all needed data, write to output
            if all(current_data.values()):
                # Use the timestamp from HEAD directly
                timestamp = current_data['timestamp']
                
                f_out.write(f"{timestamp},"
                           f"{current_data['position'][1]:.9f},"
                           f"{current_data['position'][2]:.9f},"
                           f"{current_data['position'][3]:.3f},"
                           f"{current_data['std'][0]:.3f},"
                           f"{current_data['std'][1]:.3f},"
                           f"{current_data['std'][2]:.3f}\n")
                
                # Reset current_data for next set
                current_data = {'timestamp': None, 'position': None, 'std': None}

if __name__ == "__main__":
    # 直接指定输入输出文件名
    input_file = "/mnt/d/GitHub/KF-GINS/bin/241008020814_000_Gnss.log"  # 输入GPS NMEA日志文件
    output_file = "gnss_output.txt"  # 输出GNSS格式文件
    
    print(f"Converting {input_file} to {output_file}...")
    convert_gps_log(input_file, output_file)
    print("Conversion completed!") 