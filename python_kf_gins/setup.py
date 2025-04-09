#!/usr/bin/env python3

from setuptools import setup, find_packages

setup(
    name="kf_gins",
    version="0.1.0",
    description="An EKF-Based GNSS/INS Integrated Navigation System",
    author="Claude (translated from i2Nav Group, Wuhan University)",
    author_email="original: wlq@whu.edu.cn",
    url="https://github.com/i2Nav-WHU/KF-GINS",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "pyyaml>=6.0",
    ],
    entry_points={
        'console_scripts': [
            'kf_gins=kf_gins:main',
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Programming Language :: Python :: 3",
        "Topic :: Scientific/Engineering",
    ],
    python_requires=">=3.8",
) 