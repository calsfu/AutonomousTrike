# sudo docker run -it -v /home/sd-admin/AutonomousTrike/:/workspace/AutonomousTrike ros2-humble-11-22-24:latest

# Use a multi-architecture base image for ROS2 Humble
FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    LANG=C.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    build-essential \
    git \
    cmake \
    wget \
    curl \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 Humble Desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch (ARM/Jetson and x86_64 specific builds)
ARG ARCH
RUN if [ "$(uname -m)" = "aarch64" ]; then \
        # 4.162 ERROR: Could not find a version that satisfies the requirement torch==1.13.0+nv22.12 (from versions: 1.10.2, 1.11.0, 1.12.0, 1.12.1, 1.13.0, 1.13.1, 2.0.0, 2.0.1, 2.1.0, 2.1.1, 2.1.2, 2.2.0, 2.2.1, 2.2.2, 2.3.0, 2.3.1, 2.4.0, 2.4.1, 2.5.0, 2.5.1)
        pip3 install torch==2.5.1+nv22.12 torchvision==0.14.0+nv22.12 -f https://developer.download.nvidia.com/compute/redist/jp/v502/pytorch/; \
    else \
        pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu; \
    fi

# Install additional Python libraries
RUN pip3 install numpy opencv-python

# Set the ROS2 entry point
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set up workspace (optional)
WORKDIR /workspace
RUN mkdir -p src
CMD ["/bin/bash"]
