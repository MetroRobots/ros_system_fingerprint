#!/usr/bin/env python

from setuptools import setup

setup(
    packages=['system_fingerprint'],
    package_dir={'': 'src'},
    data_files=[
        ('share/system_fingerprint', ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/system_fingerprint']),
    ],
    version='0.7.0',
    description='The system_fingerprint package',
    license='BSD 2-clause',
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    entry_points={'console_scripts': ['imprint = system_fingerprint.imprint:main']},
    tests_require=['pytest'],
)
