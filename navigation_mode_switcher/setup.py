#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['navigation_mode_switcher_ros'],
   package_dir={'navigation_mode_switcher_ros': 'ros/src/navigation_mode_switcher_ros'}
)

setup(**d)
