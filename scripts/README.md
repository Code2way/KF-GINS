# Sensor Data Conversion Tools

This directory contains tools to convert various sensor data formats to formats compatible with KF-GINS.

## CSV to IMU Converter

The `csv_to_imu.py` script converts CSV sensor data to the IMU format required by KF-GINS.

### Usage

```bash
python csv_to_imu.py <csv_file> <output_file> [sampling_rate] [start_time] [scale_factor]
```

Parameters:
- `csv_file`: Input CSV file with sensor data
- `output_file`: Output IMU text file in KF-GINS format
- `sampling_rate`: Optional. IMU sampling rate in Hz. If "auto" or not provided, the script will try to detect it
- `start_time`: Optional. Start time in GNSS seconds of week. If "auto" or not provided, timestamps from CSV are used
- `scale_factor`: Optional. Scale factor for accelerometer data (default: 1.0)

### Example

```bash
# Convert with automatic parameter detection
python csv_to_imu.py bin/250331062313_000_Sensor.csv output/sensor_imu.txt auto auto

# Convert with specific parameters
python csv_to_imu.py bin/250331062313_000_Sensor.csv output/sensor_imu.txt 200 456250.0 0.001
```

### Output Format

The script produces a text file in the KF-GINS IMU format:

| Column | Description              | Unit |
|--------|--------------------------|------|
| 1      | GNSS seconds of week     | s    |
| 2-4    | X-Y-Z incremental angles | rad  |
| 5-7    | X-Y-Z incremental velocity | m/s |

### Converting Sensor CSV Data

The script handles common issues in CSV files:
- Typos in column names (e.g., "Gryo" instead of "Gyro")
- Column mapping to standard names
- Data validation and sorting
- Auto-detection of sampling rate

### Notes

- The script assumes the input CSV contains gyroscope data (angular rates in deg/s) and accelerometer data (accelerations in m/s²)
- Angular rates are converted to incremental angles (rad)
- Accelerations are converted to incremental velocities (m/s)
- Data is sorted by timestamp and checked for gaps 