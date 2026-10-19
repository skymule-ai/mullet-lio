ARG BASE_IMAGE=ghcr.io/skymule-ai/mullet-base:v0.5.1
# livox_ros_driver2 (CustomMsg) comes from the driver image, so the message type and its Jazzy type
# hash match the driver exactly. Use a driver image built on the same BASE_IMAGE.
ARG LIVOX_IMAGE=ghcr.io/skymule-ai/mullet-drivers-livox:sha-29ab1e8
FROM ${LIVOX_IMAGE} AS livox

FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Pin FAST_LIO_REF to a full commit SHA for reproducible production images.
ARG FAST_LIO_REPOSITORY=https://github.com/hku-mars/FAST_LIO.git
ARG FAST_LIO_REF=a4743b095409588842a5b30ddfa27e29d2f99164

LABEL org.opencontainers.image.title="mullet ROS FAST-LIO2"
LABEL org.opencontainers.image.description="ROS 2 FAST-LIO2 runtime for the Mullet Jetson platform"

COPY --from=livox /opt/livox_ws/install /opt/livox_ws/install
COPY --from=livox /usr/local/lib/liblivox_lidar_sdk_shared.so /usr/local/lib/
RUN ldconfig

RUN apt-get update && apt-get install -y --no-install-recommends \
  libeigen3-dev libpcl-dev python3-dev \
  ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions \
  && rm -rf /var/lib/apt/lists/*

COPY images/fast-lio/patches /tmp/fast-lio-patches

RUN mkdir -p /opt/fast_lio_ws/src/ \
  && cd /opt/fast_lio_ws/src/ \
  && git clone "${FAST_LIO_REPOSITORY}" fast_lio \
  && cd fast_lio \
  && git checkout "${FAST_LIO_REF}" \
  && git submodule update --init --recursive \
  # Jazzy's rclcpp needs C++17; upstream forces C++14.
  && sed -i -E 's/c\+\+(14|0x)/c++17/g; s/(CMAKE_CXX_STANDARD) 14/\1 17/' CMakeLists.txt \
  && git apply /tmp/fast-lio-patches/*.patch \
  && rm -rf /tmp/fast-lio-patches

RUN source /usr/local/share/mullet-ros/setup.bash \
  && source /opt/livox_ws/install/setup.bash \
  && cd /opt/fast_lio_ws/ \
  && colcon build \
    --merge-install \
    --packages-up-to fast_lio \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && source install/setup.bash \
  && ros2 pkg prefix fast_lio \
  && rm -rf build log

COPY images/fast-lio/entrypoint.sh /usr/local/bin/mullet-fast-lio-entrypoint
COPY images/fast-lio/profile.sh /etc/profile.d/10-mullet-fast-lio.sh

RUN chmod 0755 /usr/local/bin/mullet-fast-lio-entrypoint && \
    chmod 0644 /etc/profile.d/10-mullet-fast-lio.sh

ENTRYPOINT ["/usr/local/bin/mullet-fast-lio-entrypoint"]
# Upstream publishes on absolute names (/Odometry, /path, ...): remap them under /uav0/fast_lio.
# mapping.launch.py is not used: it starts RViz by default.
CMD ["ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args", \
     "--params-file", "/config/mid360.yaml", \
     "-r", "/Odometry:=/uav0/fast_lio/odometry", \
     "-r", "/path:=/uav0/fast_lio/path", \
     "-r", "/cloud_registered:=/uav0/fast_lio/cloud_registered", \
     "-r", "/cloud_registered_body:=/uav0/fast_lio/cloud_registered_body", \
     "-r", "/cloud_effected:=/uav0/fast_lio/cloud_effected", \
     "-r", "/Laser_map:=/uav0/fast_lio/laser_map"]
