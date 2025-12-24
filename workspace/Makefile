.PHONY: deps clean build-all test lint format kill-zombies websocket foxglove launch-desktop launch-desktop-with-bridge launch-desktop-with-foxglove launch-desktop-with-fakes

deps:
	rosdep install -i --from-path src --rosdistro kilted -y

clean:
	rm -rf build install log

build-all:
	colcon build --symlink-install

test:
	colcon test

lint:
	ruff check src/
	mypy --ignore-missing-imports --explicit-package-bases --exclude '/build/' src/

format:
	ruff format src/

kill-zombies:
	ps aux | grep "continuum/workspace" | grep -v grep | awk '{print $$2}' | xargs -r kill -9 || true
	ps aux | grep "rosbridge_server/rosbridge_websocket" | grep -v grep | awk '{print $$2}' | xargs -r kill -9 || true

#
# Connections
#

websocket:
	ros2 launch rosbridge_server rosbridge_websocket_launch.xml

foxglove:
	ros2 launch foxglove_bridge foxglove_bridge_launch.xml

#
# Launch files
#

launch-desktop:
	ros2 launch continuum_desktop launch_desktop.py continuum_config:=../continuum.yaml

launch-desktop-with-bridge:
	ros2 launch continuum_desktop launch_desktop_with_bridge.py continuum_config:=../continuum.yaml

launch-desktop-with-foxglove:
	ros2 launch continuum_desktop launch_desktop_with_foxglove.py continuum_config:=../continuum.yaml

launch-desktop-with-fakes:
	ros2 launch continuum_desktop launch_desktop_with_fakes.py continuum_config:=../continuum.yaml
