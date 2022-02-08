#!/bin/bash

set -e

current_dir=`pwd -P`
script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

usage() {
    echo
    echo "Usage: `basename $0` -d ros_distro -t target [-n username] [-u uid] [-g gid] [-q] [-e] [-w] [-h]"
    echo "  d   ROS distribution to work with (lunar, kinetic, etc.)"
    echo "  t   target directory to deploy the basic setup."
    echo "  n   username in the guest system. Default is yours."
    echo "  u   UID (User ID) in the guest system. Default is yours."
    echo "  g   GID (Group ID) in the guest system. Default is yours."
    echo "  q   build docker image quietly."
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
        u) uid="${OPTARG}";;
        g) gid="${OPTARG}";;
        q) quiet_build=true;;
        e) xserver_bindings=true;;
        w) mount_ros_ws=true;;
        s) allow_ssh=true;;
        h)  echo "Builds a docker image to run ROS and deploys a basic setup to work with it."; usage; exit 0;;
        \?) echo "Unknown option: -$OPTARG" >&2; usage; exit 1;;
        :)  echo "Missing option argument for -$OPTARG" >&2; usage; exit 1;;
    esac
done

# check mandatory arguments
if [[ -z $ros_distro || -z $target ]]; then
    echo 'Missing -d or -t' >&2; usage; exit 1
fi

# make sure the target directory exists
if [[ ! -d "${target}" ]]; then
    mkdir -p "${target}"
fi

# convert target to an absolute path
target=$( cd "${target}" ; pwd -P )

# set optional arguments
if [[ -z $guest_username ]]; then
    guest_username=$USER
fi
if [[ -z $uid ]]; then
    uid=`id -u`
fi
if [[ -z $gid ]]; then
    gid=`id -g`
fi
if ${quiet_build:-false}; then
    build_options="--quiet"
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

# copy target files
echo "Prepare the target environment..."
cp -R "${script_dir}/target/"* "${target}/"
if [[ ! -d "${host_ros_dir}" ]]; then
    mkdir "${host_ros_dir}"
fi

# build the docker image
echo "Build the docker image... (This can take some time)"
cd "${script_dir}/docker"
image_tag=`basename "${target}" | sed 's/_/-/g'`
docker build ${build_options} \
    --build-arg ros_distro="${ros_distro}" \
    --build-arg real_username="${guest_username}" \
    --build-arg uid="${uid}" \
    --build-arg gid="${gid}" \
    -t ${image_tag} \
    .

echo "remove old container (if present)..."
container_name="${image_tag}-container"
docker container stop ${container_name} > /dev/null 2>&1 || true
docker container rm ${container_name} > /dev/null 2>&1 || true

echo "create a new container from this image..."
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
echo "P.S.: default user password is 'asdqwe'."
echo
