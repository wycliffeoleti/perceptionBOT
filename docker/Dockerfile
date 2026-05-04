# PerceptionBOT — Dockerfile
#
# Builds the workspace on top of the official osrf/ros:jazzy-desktop-full
# image with MoveIt 2, MTC, gz_ros2_control, and the GPU build of ONNX
# Runtime. Designed to be run with --gpus all and X11 forwarding for the
# Gazebo + RViz GUIs.
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ONNXRUNTIME_VERSION=1.20.1
ENV DEBIAN_FRONTEND=noninteractive

# System + ROS deps -----------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl ca-certificates git python3-pip python3-vcstool \
        build-essential cmake \
        ros-${ROS_DISTRO}-ros-gz \
        ros-${ROS_DISTRO}-gz-ros2-control \
        ros-${ROS_DISTRO}-moveit \
        ros-${ROS_DISTRO}-moveit-task-constructor-core \
        ros-${ROS_DISTRO}-moveit-task-constructor-capabilities \
        ros-${ROS_DISTRO}-moveit-task-constructor-visualization \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ONNX Runtime ----------------------------------------------------------
RUN curl -fL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}.tgz" \
      -o /tmp/ort.tgz && \
    mkdir -p /opt/onnxruntime && \
    tar -xzf /tmp/ort.tgz -C /tmp && \
    cp -r /tmp/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}/* /opt/onnxruntime/ && \
    rm -rf /tmp/ort.tgz /tmp/onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}
ENV LD_LIBRARY_PATH=/opt/onnxruntime/lib:${LD_LIBRARY_PATH}

# Workspace -------------------------------------------------------------
WORKDIR /workspace
COPY . /workspace
# Make sure the franka_description submodule is present even if the host
# clone forgot --recurse-submodules.
RUN if [ ! -f src/franka_description/package.xml ]; then \
      git -C /workspace submodule update --init --recursive ; \
    fi

# Build -----------------------------------------------------------------
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Entrypoint sources both ROS2 and the workspace overlay.
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
