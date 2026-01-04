FROM ros:kilted-ros-core AS base

# Install Python & ROS extra packages
# + Kokoro (espeak-ng)
# + Reachy: https://github.com/pollen-robotics/reachy_mini/blob/develop/docs/SDK/gstreamer-installation.md
RUN apt-get update && apt-get install -y \
    curl \
    espeak-ng \
    git git-lfs \
    gstreamer1.0-alsa \
    gstreamer1.0-nice \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-good \
    libcairo2-dev \
    libgirepository1.0-dev \
    libglib2.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libnice10 \
    libportaudio2 \
    libssl-dev \
    python3-gi \
    python3-gi-cairo \
    python3-pip \
    ros-dev-tools \
    ros-kilted-ament-cmake \
    ros-kilted-ament-cmake-python \
    ros-kilted-joy \
    ros-kilted-rosbridge-server \
    ros-kilted-rosidl-default-generators && \
    rm -rf /var/lib/apt/lists/*

# Install Rust using rustup (Ubuntu's bundled version is too old for cargo-c)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"

# Install GStreamer WebRTC plugin (Reachy)
RUN git lfs install && \
    cargo install cargo-c && \
    git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git /tmp/gst-plugins-rs && \
    cd /tmp/gst-plugins-rs && \
    git checkout 0.14.1 && \
    cargo cinstall -p gst-plugin-webrtc --prefix=/opt/gst-plugins-rs --release && \
    rm -rf /tmp/gst-plugins-rs ~/.cargo/registry ~/.cargo/git

# Set GStreamer plugin path
ENV GST_PLUGIN_PATH=/opt/gst-plugins-rs/lib/x86_64-linux-gnu:${GST_PLUGIN_PATH}

# Copy Python library code and metadata
COPY lib/pyproject.toml lib/README.md /app/lib/
COPY lib/src/ /app/lib/src/

# Install Python library
WORKDIR /app/lib
RUN pip install --no-cache-dir --break-system-packages --ignore-installed .

# Download spacy language model (Kokoro) + delete pip cache
RUN python3 -m spacy download en_core_web_sm --break-system-packages && \
    rm -rf /root/.cache/pip

# Copy ROS workspace source code
COPY workspace/src/ /app/workspace/src/

# Install ROS workspace dependencies
WORKDIR /app/workspace
RUN rosdep init && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro kilted -y

# Build ROS workspace
RUN /bin/bash -c "source /opt/ros/kilted/setup.bash && colcon build"

# Copy and setup entrypoint script
COPY --chmod=755 docker/entrypoint.sh /docker/entrypoint.sh

# Use entrypoint script to launch desktop
ENTRYPOINT ["/docker/entrypoint.sh"]
