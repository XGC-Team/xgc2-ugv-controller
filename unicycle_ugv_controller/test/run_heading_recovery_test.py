#!/usr/bin/env python3
"""Compile/run the production heading-recovery kernel without ROS."""
import os
from pathlib import Path
import subprocess
import tempfile

package = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as directory:
    executable = Path(directory)/'heading_recovery_test'
    subprocess.run([os.environ.get('CXX', 'g++'), '-std=c++17', '-O2', '-DNDEBUG',
                    '-Wall', '-Wextra', '-Werror', '-I'+str(package/'include'),
                    str(package/'test/heading_recovery_test.cpp'), '-o', str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
