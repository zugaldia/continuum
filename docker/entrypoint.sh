#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/kilted/setup.bash
source /app/workspace/install/setup.bash

# Launch desktop with bridge
exec ros2 launch continuum_desktop launch_desktop_with_bridge.py continuum_config:=../continuum.yaml
