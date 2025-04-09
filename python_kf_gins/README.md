# Python KF-GINS

Python implementation of KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System. This is a translation of the original C++ version from i2Nav Group, Wuhan University.

## Introduction

KF-GINS is a GNSS/INS integration system based on Extended Kalman Filter (EKF). It uses IMU and GNSS measurements to provide accurate position, velocity, and attitude estimation. This Python implementation replicates the functionality of the original C++ version, making it more accessible to researchers and developers who are more familiar with Python.

## Installation

### Requirements

- Python 3.8+
- NumPy
- SciPy
- PyYAML

### Install from Source

1. Clone this repository
2. Navigate to the repository folder
3. Install with pip:

```bash
pip install -e .
```

## Usage

### Configuration

KF-GINS requires a YAML configuration file to specify input/output paths, initial states, and other parameters. An example configuration file is provided in `example_config.yaml`.

### Running KF-GINS

You can run KF-GINS using the command line:

```bash
kf_gins config.yaml
```

Where `config.yaml` is your configuration file.

### Input Data Format

#### IMU Data

The IMU data file should contain at least 7 columns:
1. Time (seconds)
2-4. Angular increments around X, Y, Z axes (rad)
5-7. Velocity increments along X, Y, Z axes (m/s)

Optional 8th column:
8. Odometer velocity (m/s)

#### GNSS Data

The GNSS data file should contain at least 7 columns:
1. Time (seconds)
2-4. Latitude (deg), Longitude (deg), Height (m)
5-7. Standard deviations of Latitude, Longitude, Height (m)

### Output Files

KF-GINS generates three output files:
- `KF_GINS_Navresult.nav`: Navigation results (position, velocity, attitude)
- `KF_GINS_IMU_ERR.txt`: IMU error estimates (biases, scale factors)
- `KF_GINS_STD.txt`: Standard deviations of all states

## Note on Implementation Status

This Python implementation includes all the core components of KF-GINS but some of the detailed algorithm implementations in the GI Engine are still marked with TODO comments. The full implementation of these components requires additional translation work.

## License

This project is licensed under the GNU General Public License v3.0 - see the original C++ repository for details.

## References

Original C++ implementation: [KF-GINS](https://github.com/i2Nav-WHU/KF-GINS)

```
@article{wang2022kf-gins,
    title={KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System},
    author={Wang, Liqiang and Tang, Hailiang and Wang, Li and Yu, Xingsheng and Tao, Zhuoran and Gao, Yang and Liang, Kai and Guo, Xianghui},
    year={2022}
}
``` 