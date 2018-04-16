from setuptools import setup, find_packages
import sys, os

with open('requirements.txt') as f:
    required = f.read().splitlines()

version = '0.2'

setup(name='ILCC',
      version=version,
      description="Intensity-based_Lidar_Camera_Calibration",
      long_description="""\
An package for automatic 3D_LiDAR-Panoramic_camera extrinsic calibration based on corner detection from the sparse point cloud with reflectance intensity. """,
      classifiers=[], # 
      keywords='lidar panoramic camera calibration extrinsic visualization',
      author='wangwm',
      author_email='weimin@ucl.nuee.nagoya-u.ac.jp',
      url='https://github.com/mfxox/ILCC',
      license='BSD-2',
      packages=find_packages(exclude=['ez_setup', 'examples', 'tests']),
      include_package_data=True,
      zip_safe=False,
      install_requires=required,
      entry_points="""
      # -*- Entry points: -*-
      """,
      )
