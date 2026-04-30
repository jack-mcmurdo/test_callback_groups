FROM osrf/ros:humble-desktop-full

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source ~/ws/install/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /home/root

# Create workspace directory
RUN mkdir -p ~/ws/src

CMD ["/bin/bash"]