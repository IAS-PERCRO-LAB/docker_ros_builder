ARG ros_distro
FROM ros-${ros_distro}-base

ARG ros_distro
ARG username
ARG WS_DEPS=/home/${username}/ws_deps


# --------------------- FastRTPS and CycloneDDS ---------------------

USER root

RUN apt update && \
	DEBIAN_FRONTEND=noninteractive apt -yq --allow-downgrades install \
        ros-${ros_distro}-rmw-fastrtps-cpp \
        ros-${ros_distro}-rmw-cyclonedds-cpp && \
    DEBIAN_FRONTEND=noninteractive apt clean && \
	rm -rf /var/lib/apt/lists/*

USER ${username}

# ##################### FastRTPS and CycloneDDS #####################

RUN mkdir -p ${WS_DEPS}/src && \
    echo "source ${WS_DEPS}/devel/local_setup.bash" >> /home/${username}/.bashrc

# --------------------- Zenoh DDS ---------------------

# TODO: build in an intermediate Docker stage and just copy the `devel` directory
RUN . /opt/ros/${ros_distro}/setup.sh && \
    cd ${WS_DEPS}/src && \
    git clone -b release-0.10.1-rc https://github.com/eclipse-zenoh/zenoh-plugin-dds.git && \
    git clone -b release-0.10.1-rc2 https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds.git && \
    cd .. && \
    sudo apt update && \
    rosdep update --rosdistro=${ros_distro} && \
    rosdep install --from-paths src/zenoh-plugin-dds --ignore-src -r -y && \
    rosdep install --from-paths src/zenoh-plugin-ros2dds --ignore-src -r -y && \
    colcon build --packages-select zenoh-plugin-dds zenoh_bridge_ros2dds \
        --packages-skip-build-finished --event-handler console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    DEBIAN_FRONTEND=noninteractive sudo apt clean && \
    sudo rm -rf /var/lib/apt/lists/*

# ##################### Zenoh DDS #####################
