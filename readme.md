# ROS LatteDrone
Create ROS image with:
```bash
./image_builder/init-ros-box.sh -d noetic -t ./ros_lattedrone -w -e
```
Then, it can be started with `go.sh`.
Once inside the container:
```bash
# populate the workspace
wstool init src lattedrone.rosinstall

# then install dependencies and build
~/catkin_ws/ros_container_setup.sh
```

## LattePanda setup
Realsense camera (guest side):
```bash
lattedrone=user@ip
scp scripts/lattepanda_realsense_setup.sh $lattedrone:~

ssh $lattedrone
./lattepanda_realsense_setup.sh && rm lattepanda_realsense_setup.sh
```
