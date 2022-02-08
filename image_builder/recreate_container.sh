#!/bin/bash

set -e

current_dir=`pwd -P`
script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

usage() {
    echo
    echo "Usage: `basename $0` -d ros_distro -t target [-e] [-w] [-h]"
    echo "  d   ROS distribution to work with (lunar, kinetic, etc.)"
    echo "  t   target directory to deploy the basic setup."
    echo "  n   username in the guest system. Default is yours."
    echo "  e   create headless container (no x-server bindings)."
    echo "  w   mount an entire ROS workspace instead of just the src directory."
    echo "  s   allow ssh connections (port 22)."
    echo "  h   help and usage message."
    echo
}

# `getopts` ref: https://www.computerhope.com/unix/bash/getopts.htm
while getopts ":d:t:n:u:g:qewh" arg; do
    case $arg in
        d) ros_distro="${OPTARG}";;
        t) target="${OPTARG}";;
        n) guest_username="${OPTARG}";;
        e) xserver_bindings=true;;
        w) mount_ros_ws=true;;
        s) allow_ssh=true;;
        h)  echo "Recreate container for a given docker ROS image."; usage; exit 0;;
        \?) echo "Unknown option: -$OPTARG" >&2; usage; exit 1;;
        :)  echo "Missing option argument for -$OPTARG" >&2; usage; exit 1;;
    esac
done

# check mandatory arguments
if [[ -z $ros_distro || -z $target ]]; then
    echo 'Missing -d or -t' >&2; usage; exit 1
fi

# Make sure the target directory exists
if [[ ! -d "${target}" ]]; then
	echo "${target} directory not found!"
	exit 1
fi

# convert target to an absolute path
target=$( cd "${target}" ; pwd -P )

# set optional arguments
if [[ -z $guest_username ]]; then
    guest_username=$USER
fi
if ${xserver_bindings:-true}; then
    create_options="--env='DISPLAY' -v /tmp/.X11-unix:/tmp/.X11-unix:rw "
fi
if ${mount_ros_ws:-false}; then
    host_ros_dir="${target}/ros_ws"
    create_options="${create_options}-v ${host_ros_dir}:/home/${guest_username}/catkin_ws"
else
    host_ros_dir="${target}/src"
    create_options="${create_options}-v ${host_ros_dir}:/home/${guest_username}/catkin_ws/src"
fi
if ${allow_ssh:-false}; then
    create_options="${create_options} --cap-add sys_ptrace -p 127.0.0.1:2200:22"
else
    create_options="${create_options} --network host"
fi

image_tag=`basename "${target}" | sed 's/_/-/g'`

echo "removing old container (if present)..."
container_name="${image_tag}-container"
docker container stop ${container_name} > /dev/null 2>&1 || true
docker container rm ${container_name} > /dev/null 2>&1 || true

echo "creating a new container from the given image..."
cd "${target}"
cd "${target}"
docker create ${create_options} \
    -v /dev:/dev \
    --privileged \
    --user=${guest_username} \
    --name "${container_name}" \
    -it ${image_tag}

sudo rm -f "${target}/docker_id"
docker ps -aqf "name=${container_name}" > "${target}/docker_id"
chmod 444 "${target}/docker_id"


# That's it!
cd "${current_dir}"

echo
echo "Your dockerized ROS box is now ready in '${target}'."
echo "There you will find:"
echo "  docker_id   This file contains the ROS distribution used in your project."
echo "              Do not touch this file."
if ${mount_ros_ws:-false}; then
echo "  catkin_ws   Put your ROS project workspace in this directory."
echo "              It is automatically mounted in ~/catkin_ws inside the ROS box."
else
echo "  src         Put your ROS project sources in this directory."
echo "              It is automatically mounted in ~/catkin_ws/src inside the ROS box."
fi
echo "  go.sh       Run this script to start the container and/or open a shell in it."
echo
echo "Have fun!"
echo
