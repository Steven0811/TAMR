# ROS 2 Humble + YDLidar-SDK + ydlidar_ros2_driver, with RViz2
FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        cmake \
        pkg-config \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 1. Build and install YDLidar-SDK (into /usr/local)
COPY YDLidar-SDK /tmp/YDLidar-SDK
RUN cmake -S /tmp/YDLidar-SDK -B /tmp/YDLidar-SDK/build -DBUILD_EXAMPLES=OFF \
    && cmake --build /tmp/YDLidar-SDK/build -j"$(nproc)" \
    && cmake --install /tmp/YDLidar-SDK/build \
    && ldconfig \
    && rm -rf /tmp/YDLidar-SDK

# 2. Build ydlidar_ros2_driver
WORKDIR /ros2_ws
COPY ydlidar_ros2_driver src/ydlidar_ros2_driver
RUN source /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS and the workspace automatically on container start
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\nsource /ros2_ws/install/setup.bash\nexec "$@"\n' \
        > /entrypoint.sh \
    && chmod +x /entrypoint.sh \
    && echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ydlidar_ros2_driver", "ydlidar_launch_view.py"]
