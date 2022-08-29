#!/usr/bin/env python

from setuptools import setup

setup(
    packages=['ros_system_fingerprint'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ros_system_fingerprint', ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/ros_system_fingerprint']),
    ],
    version='0.3.0',
    description='The ros_system_fingerprint package',
    license='BSD 2-clause',
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    entry_points={'console_scripts': ['imprint = ros_system_fingerprint.imprint:main']},
)
