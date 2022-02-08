#!/bin/bash

current_dir=`pwd -P`
script_dir="$( cd "$(dirname "$0")" ; pwd -P )"
container_id=`cat "${script_dir}/docker_id"`

if [ "${container_id}" == "" ]; then
	echo "Error: No docker id found in '${script_dir}/docker_id'"
	exit 1
fi

# Check if the container is running
if [ "`docker ps -qf "id=${container_id}"`" == "" ]; then
	echo "Starting previously stopped container..."
	docker start "${container_id}"
fi

# Joining the container
docker exec -ti ${container_id} bash
