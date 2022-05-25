ARG ros_distro
FROM osrf/ros:${ros_distro}-desktop-full

ARG ros_distro
ARG real_username
ARG uid
ARG gid

RUN apt-get update && \
	DEBIAN_FRONTEND=noninteractive apt-get -yq --allow-downgrades install \
		git \
		vim tmux \
		sudo \
		dialog \
		less \
		x-window-system \
		mesa-utils && \
	DEBIAN_FRONTEND=noninteractive apt-get -yq upgrade && \
	DEBIAN_FRONTEND=noninteractive apt-get -yq --allow-downgrades install \
		python3-catkin-tools \
        python3-osrf-pycommon && \
    DEBIAN_FRONTEND=noninteractive apt clean && \
	rm -rf /var/lib/apt/lists/*

# An alternative, if catkin-tools does not work
# pip install -U "git+https://github.com/catkin/catkin_tools.git#egg=catkin_tools"

RUN mkdir -p /home/${real_username}/catkin_ws/src && \
    echo "${real_username}:x:${uid}:${gid}:Developer,,,:/home/${real_username}:/bin/bash" >> /etc/passwd && \
    echo "${real_username}:\$6\$EikgdMS033suRSOQ\$MlQ0Z8iVq6.9x81X0Ffhz5Me..beIO94MurFT3DC.tEtQkQCe.3tz1DAEJdm/vQPLFMVkd1pFlzVAeVys8PTS.:18946::::::" >> /etc/shadow && \
    echo "${real_username}:x:${uid}:" >> /etc/group && \
    echo "${real_username} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${real_username} && \
    chmod 0440 /etc/sudoers.d/${real_username} && \
    chown ${uid}:${gid} -R /home/${real_username} && \
    ssh-keygen -A && \
    rm /ros_entrypoint.sh

COPY bashrc /home/${real_username}/.bashrc
COPY bashrc /root/.bashrc
COPY bash_profile /home/${real_username}/.bash_profile
COPY bash_profile /root/.bash_profile
COPY ros_container_setup.sh /home/${real_username}/ros_container_setup.sh
RUN chown ${real_username}:${real_username} /home/${real_username}/.bash* /home/${real_username}/ros_container_setup.sh
USER ${real_username}
ENV HOME /home/${real_username}
RUN echo "source /opt/ros/${ros_distro}/setup.bash" >> /home/${real_username}/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> /home/${real_username}/.bashrc

RUN rosdep update

COPY rosbox_entrypoint.sh /rosbox_entrypoint.sh
ENTRYPOINT /rosbox_entrypoint.sh
