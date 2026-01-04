#!/bin/bash
set -e

# Source ROS environment
source /opt/ros/kilted/setup.bash
source /app/workspace/install/setup.bash

# Launch desktop with bridge
exec ros2 launch continuum_desktop launch_docker.py continuum_config:=/app/continuum.yaml
