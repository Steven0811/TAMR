# ROS 2 Humble images for the IMU, Orbbec depth camera, wheel drive and foxglove_bridge.
# Build context: the repo root (/home/steven/sensor). The lidar uses ydlidar/Dockerfile.

# ---------------------------------------------------------------- base
FROM ros:humble-ros-base AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-foxglove-bridge \
        ros-humble-tf2-ros \
        ros-humble-twist-mux \
        ros-humble-teleop-twist-keyboard \
        python3-serial \
        python3-numpy \
        python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
ENTRYPOINT ["/ros_entrypoint.sh"]

# ---------------------------------------------------------------- foxglove_bridge
FROM base AS foxglove
CMD ["ros2", "launch", "foxglove_bridge", "foxglove_bridge_launch.xml", "port:=8765"]

# ---------------------------------------------------------------- IMU
FROM base AS imu
COPY imu/imu_viewer.py imu/imu_ros2_node.py /app/
CMD ["python3", "/app/imu_ros2_node.py"]

# ---------------------------------------------------------------- RC teleop + DDSM115 wheels
FROM base AS drive
COPY remote_control/rc_teleop_node.py remote_control/ddsm115_node.py /app/
CMD ["python3", "/app/ddsm115_node.py"]

# ---------------------------------------------------------------- Orbbec camera
FROM base AS camera

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake python3-dev python3-pip \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir pybind11==2.11.0

# Build pyorbbecsdk (bundles OrbbecSDK 1.10.16 for linux_x64)
COPY depth_camera/third_party/pyorbbecsdk /opt/pyorbbecsdk
RUN cmake -S /opt/pyorbbecsdk -B /opt/pyorbbecsdk/build \
        -DBUILD_TESTING=OFF \
        -Dpybind11_DIR="$(python3 -m pybind11 --cmakedir)" \
    && cmake --build /opt/pyorbbecsdk/build -j"$(nproc)" \
    && cmake --install /opt/pyorbbecsdk/build \
    && rm -rf /opt/pyorbbecsdk/build

ENV PYTHONPATH=/opt/pyorbbecsdk/install/lib \
    LD_LIBRARY_PATH=/opt/pyorbbecsdk/install/lib

COPY depth_camera/ob_utils.py depth_camera/orbbec_ros2_node.py /app/
CMD ["python3", "/app/orbbec_ros2_node.py"]
