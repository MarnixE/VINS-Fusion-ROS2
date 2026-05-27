FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ARG CERES_VERSION=2.1.0
ENV ROS_WS=/ros2_ws

RUN apt-get update && apt-get install -y \
    cmake \
    git \
    libatlas-base-dev \
    libboost-all-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libopencv-dev \
    libsuitesparse-dev \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-ament-index-cpp \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-launch \
    ros-${ROS_DISTRO}-launch-ros \
    ros-${ROS_DISTRO}-rcpputils \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch ${CERES_VERSION} --depth 1 https://ceres-solver.googlesource.com/ceres-solver /tmp/ceres-solver && \
    cmake -S /tmp/ceres-solver -B /tmp/ceres-solver/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF && \
    cmake --build /tmp/ceres-solver/build -j"$(nproc)" --target install && \
    rm -rf /tmp/ceres-solver

WORKDIR ${ROS_WS}
COPY . src/vins-fusion-ros2/

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build/ log/

RUN sed -i '/exec "$@"/i source "${ROS_WS}/install/setup.bash"' /ros_entrypoint.sh

CMD ["bash"]
