FROM ros:kilted-ros-core AS base

# Install extra packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-dev-tools \
    ros-kilted-ament-cmake \
    ros-kilted-ament-cmake-python \
    ros-kilted-rosidl-default-generators \
    ros-kilted-rosbridge-server \
    ros-kilted-joy \
    espeak-ng && \
    rm -rf /var/lib/apt/lists/*

# Copy Python library code and metadata
COPY lib/pyproject.toml lib/README.md /app/lib/
COPY lib/src/ /app/lib/src/

# Install Python library
WORKDIR /app/lib
RUN pip install --no-cache-dir --break-system-packages --ignore-installed .

# Download spacy language model
RUN python3 -m spacy download en_core_web_sm --break-system-packages

# Copy ROS workspace source code
COPY workspace/src/ /app/workspace/src/

# Install ROS workspace dependencies
WORKDIR /app/workspace
RUN rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro kilted -y

# Build ROS workspace
RUN /bin/bash -c "source /opt/ros/kilted/setup.bash && colcon build"

# Copy configuration file
COPY continuum.yaml /app/

# Copy and setup entrypoint script
COPY --chmod=755 docker/entrypoint.sh /docker/entrypoint.sh

# Use entrypoint script to launch desktop
ENTRYPOINT ["/docker/entrypoint.sh"]
