# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/melodic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/melodic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/parallels/lecture/autonomous_exploration/devel_isolated/scan_tools;/home/parallels/lecture/autonomous_exploration/devel_isolated/scan_to_cloud_converter;/home/parallels/lecture/autonomous_exploration/devel_isolated/polar_scan_matcher;/home/parallels/lecture/autonomous_exploration/devel_isolated/ncd_parser;/home/parallels/lecture/autonomous_exploration/devel_isolated/mycar;/home/parallels/lecture/autonomous_exploration/devel_isolated/my_move_base;/home/parallels/lecture/autonomous_exploration/devel_isolated/my_frontier_exploration;/home/parallels/lecture/autonomous_exploration/devel_isolated/my_controller;/home/parallels/lecture/autonomous_exploration/devel_isolated/map_server;/home/parallels/lecture/autonomous_exploration/devel_isolated/laser_scan_splitter;/home/parallels/lecture/autonomous_exploration/devel_isolated/laser_scan_sparsifier;/home/parallels/lecture/autonomous_exploration/devel_isolated/laser_scan_matcher;/home/parallels/lecture/autonomous_exploration/devel_isolated/laser_ortho_projector;/home/parallels/lecture/autonomous_exploration/devel_isolated/generate_odom;/opt/ros/melodic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/parallels/github_ws/v0/src/my_frontier_exploration/cmake-build-debug/devel/env.sh')

output_filename = '/home/parallels/github_ws/v0/src/my_frontier_exploration/cmake-build-debug/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
