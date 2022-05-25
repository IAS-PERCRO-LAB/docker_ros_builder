#!/usr/bin/bash

LATTEDRONE_IP='10.30.5.228'
SOURCE_WS_PATH=~/sources/ros_workspaces/lattedrone_ws

LOCAL_DOCKER_PATH=~/sources/docker_images/ros_lattedrone
REMOTE_DOCKER_PATH='~/ros_docker_noetic/ros_ws'

green=$'\e[92m'
yellow=$'\e[33m'
red=$'\e[31m'
reset=$'\e[0m'

echo "==> Synching sources with docker container..."
synched_sources=`rsync -rlt --stats --human-readable --exclude=".*" --delete-delay ${SOURCE_WS_PATH}/src/. ${LOCAL_DOCKER_PATH}/ros_ws/src/. | fgrep 'Number of regular files transferred' | cut -d' ' -f6 | tr -d ,`

#if [[ $synched_sources -eq 0 ]]; then
#    echo -n "${yellow}"
#    echo " -> Everything seems already synched. Nothing more to do."
#    echo -n "${reset}"
#    exit 0
#fi
echo "${green} -> ${synched_sources} files synched.${reset}"

echo "==> Building workspace locally..."
#build_errors=`docker exec ros-lattedrone-container bash -c "cd ~/catkin_ws && catkin build" >/dev/null`
build_errors=`${LOCAL_DOCKER_PATH}/exec_no_it.sh "cd ~/catkin_ws && catkin build" >/dev/null`
if [ $? -ne 0 ]; then
	echo $build_errors
	echo "${red} -> Errors occurred during build. Not synching anything with LattePanda.${reset}"
	exit 1
fi
echo -n "${green}"
echo " -> Compiled succesfully.${reset}"

echo "==> Synching 'ros_ws' with LattePanda..."
uploaded_files=`rsync -rlt --stats --human-readable --exclude=".*" --delete-delay ${LOCAL_DOCKER_PATH}/ros_ws/. percro@${LATTEDRONE_IP}:${REMOTE_DOCKER_PATH}/. | fgrep 'Number of regular files transferred' | cut -d' ' -f6 | tr -d ,`
echo "${green} -> ${uploaded_files} files uploaded.${reset}"
echo "==> Done! Enjoy your flight."
