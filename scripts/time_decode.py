#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cProfile
import pstats
import os
from phobos import pose
from phobos import load

script_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = os.path.join(script_dir, os.path.pardir, 'projects')
flimnap_file = os.path.join(project_dir, 'flimnap', 'main.cc')

if __name__ == '__main__':
    _, dtype, _ = pose.parse_format(flimnap_file)
    filename = 'pose5.log'
    profile_exec = 'load.pose_logfile(filename, dtype)'

    cProfile.run(profile_exec, 'output_stats')
    p = pstats.Stats('output_stats')
    p.sort_stats('cumulative').print_stats(20)
