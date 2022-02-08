#!/usr/bin/bash

script_dir="$( cd "$(dirname "$0")" ; pwd -P )"

cd $script_dir/../ros_ws/src
for d in `ls -d percro_*`; do
    if [[ -d $d ]]; then
        echo
        echo "----- $d -----"

        cd $d
        git push
        cd ..
    fi
done
