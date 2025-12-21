# Setup

1. Install ROS 2

Currently, we only support ROS 2 and the system has been tested with the
[`kilted` release](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html).

2. Install `rosbridge_suite`

It's part of the ROS 2 distro (`sudo apt install ros-kilted-rosbridge-server`). It provides a JSON interface to ROS,
allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more.
It supports a variety of transport layers, including WebSockets and TCP.

3. Install the Foxglove bridge

Foxglove is a fantastic platform to record, upload, organize, and visualize multimodal log data such as time series,
text logs, video, 3D, maps, and more. It is most often used in hardware, robotics, and physical AI.
It has [native support for ROS 2](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2).

# Adding a new package

A package is the organizational unit for ROS 2 code, and they can be written in either C++ or Python.
The following commands need to take place inside the `workspace/src` folder.

## C++

We only have one C++ package which is required to create custom message interfaces. All packages depend on this one:

```bash
$ ros2 pkg create --build-type ament_cmake \
    --description "Continuum interfaces package" \
    --license MIT \
    continuum_interfaces
```

## Python

Everything else is Python-based because the third-party libraries we are integrating with are Python-native. E.g.:

```bash
$ ros2 pkg create --build-type ament_python \
    --description "Continuum core package" \
    --license MIT \
    continuum_core
```