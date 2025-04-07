#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash
source ../../venvs/p3_11_2-venv/bin/activate
ros2 run servo servo
