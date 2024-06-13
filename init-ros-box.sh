#!/bin/bash

set -e

current_dir=`pwd -P`
script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

ROS1_DISTROS=(kinetic lunar melodic noetic)

usage() {
    echo
    echo "Usage: `basename $0` -d ros_distro -v version [-t target] [-n username] [-u uid] [-g gid] [-q] [-e] [-w] [-h]"
    echo
    echo "This script is two-fold:"
    echo "  1. It builds a ROS docker image of your desired distribution."
    echo "  2. It deploys a basic workspace setup with a persistent container to work with."
    echo
    echo "Build arguments:"
    echo "  d   ROS distribution to work with. Either ROS1 (kinetic, lunar, melodic, noetic) or ROS2 (humble)."
    echo "  v   Dockerfile version: base, or something more."
    echo "  u   UID (User ID) in the guest system. Default is yours (`id -u`)."
    echo "  g   GID (Group ID) in the guest system. Default is yours (`id -g`)."
    echo "  q   build docker image quietly."
    echo
    echo "Deploy arguments:"
    echo "  t   target directory to deploy the basic setup. If omitted, there will be no deploy."
    echo "  n   username in the guest system. Default is yours (`whoami`)."
    echo "  e   create headless container (no x-server bindings)."
    echo "  w   mount an entire ROS workspace instead of just the src directory."
    echo "  s   allow ssh connections (port 22)."
    echo
    echo "  h   help and usage message (that you're just reading)."
    echo
}

print_distros() {
    echo
    echo "ROS1 distributions: ${ROS1_DISTROS[@]}"
    echo "ROS2 distributions: humble"
    echo
    echo "Example: `basename $0` -d noetic -v full-gpu -t ~/rosbox"
    echo

}

target=""
build_options=""
# build_options+="--pull "

# `getopts` ref: https://www.computerhope.com/unix/bash/getopts.htm
while getopts ":d:v:t:n:u:g:qcewh" arg; do
    case $arg in
        d) ros_distro="${OPTARG}";;
        v) version="${OPTARG}";;
        u) uid="${OPTARG}";;
        g) gid="${OPTARG}";;
        q) quiet_build=true;;
        c) build_options+="--no-cache ";;

        t) target="${OPTARG}";;
        n) guest_username="${OPTARG}";;
        e) xserver_bindings=true;;
        w) mount_ros_ws=true;;
        s) allow_ssh=true;;

        h)  usage; exit 0;;
        \?) echo "Unknown option: -$OPTARG" >&2; usage; exit 1;;
        :)  echo "Missing option argument for -$OPTARG" >&2; usage; exit 1;;
    esac
done

# check mandatory arguments
if [[ -z $ros_distro ]]; then
    echo 'Missing desired distribution (-d)' >&2; usage; exit 1
fi
if [[ -z $version ]]; then
    echo 'Missing desired version (-v)' >&2; usage; exit 1
fi

if [[ ! " ${ROS1_DISTROS[@]} " =~ " ${ros_distro} " && ${ros_distro} != "humble" ]]; then
    echo "Invalid ROS distribution: ${ros_distro}" >&2; usage; print_distros; exit 1
fi

# Deploy only if target is set
deploy=$(test -n "$target" && echo true || echo false)

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
    build_options+="--quiet"
fi

if $deploy; then
    mkdir -p "$current_dir/$target"

    # convert target to an absolute path
    target=$( cd "${target}" ; pwd -P )

    create_options="--privileged "
    # limit memory usage to 80% of host's total memory
    create_options+="--memory $(( $(grep MemTotal /proc/meminfo | awk '{print $2}') * 8 / 10 /1024 ))M "
    create_options+="-v /dev:/dev "
    create_options+="--user=${guest_username} "
    create_options+="-e TERM=xterm-256color "
    create_options+="-v /etc/localtime:/etc/localtime:ro "
    create_options+="-v /var/run/dbus/system_bus_socket:/var/run/dbus/system_bus_socket "

    # rosout bugfix in Archlinux
    # Ref: https://answers.ros.org/question/336963/rosout-high-memory-usage/
    create_options+="--ulimit nofile=1024:524288 "

    if ${xserver_bindings:-true}; then
        create_options+="-e DISPLAY=$DISPLAY "
        create_options+="-e QT_X11_NO_MITSHM=1 "
        create_options+="-v /tmp/.X11-unix:/tmp/.X11-unix:rw "
    fi
    if ${mount_ros_ws:-false}; then
        host_ros_dir="${target}/ros_ws"
        create_options+="-v ${host_ros_dir}:/home/${guest_username}/catkin_ws "
    else
        host_ros_dir="${target}/src"
        create_options+="-v ${host_ros_dir}:/home/${guest_username}/catkin_ws/src "
    fi
    if ${allow_ssh:-false}; then
        create_options+="--cap-add sys_ptrace -p 127.0.0.1:2200:22 "
    else
        create_options+="--network host "
        create_options+="--ipc host "
        create_options+="--pid host "
    fi

    # Detect Nvidia GPU presence
    if [[ -n `lspci | grep -i nvidia` ]]; then
        create_options+="--gpus all "
        # create_options+="--runtime=nvidia "  # alternative of the above
        # TODO: get NVIDIA_VISIBLE_DEVICES
        create_options+="-e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} "
        create_options+="-e NVIDIA_DRIVER_CAPABILITIES=all "
        create_options+="-e __NV_PRIME_RENDER_OFFLOAD=1 "
        create_options+="-e __GLX_VENDOR_LIBRARY_NAME=nvidia "
        echo "Nvidia GPU detected. Enabling GPU support."
    fi

    # copy target files
    echo "Prepare the target environment..."
    cp -R "${script_dir}/target/"* "${target}/"
    mkdir -p "${host_ros_dir}"
fi

# build the docker image
image_tag="ros-${ros_distro}-${version}"
echo "Building docker image $image_tag (This can take some time)"

# If it's a ROS1 distro
if [[ " ${ROS1_DISTROS[@]} " =~ " ${ros_distro} " ]]; then
    cd "${script_dir}/dockerfile-ros1"

    dockerfile_name="Dockerfile.${version}"
    if [[ ! -f "${dockerfile_name}" ]]; then
        echo "It does not seem to exist a Docker file for version ${version}. Check it again, please!" >&2; print_distros; exit 1
    fi

    docker build ${build_options} \
        --file "${dockerfile_name}" \
        --build-arg ros_distro="${ros_distro}" \
        --build-arg username="${guest_username}" \
        --build-arg uid="${uid}" \
        --build-arg gid="${gid}" \
        -t ${image_tag} \
        .

else
    echo "not implemented"
    # TODO: Humble build
fi

echo
echo "Docker image built successfully! Tagged as ${image_tag}"

if $deploy; then
    container_name=$(basename ${target} | sed 's/[^a-zA-Z0-9]/-/g')
    echo "Deploying ${container_name}..."

    echo "remove old container (if present)..."
    docker container stop ${container_name} > /dev/null 2>&1 || true
    docker container rm ${container_name} > /dev/null 2>&1 || true

    echo "create a new container from this image..."
    cd "${target}"
    docker create ${create_options} \
        --name "${container_name}" \
        -it ${image_tag}

    sudo rm -f "${target}/docker_id"
    docker ps -aqf "name=${container_name}" > "${target}/docker_id"
    chmod 444 "${target}/docker_id"

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
fi
