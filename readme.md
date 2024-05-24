# ROS LatteDrone
Create ROS image with:
```bash
./image_builder/init-ros-box.sh -d noetic -n percro -t ./ros_lattedrone -w -e

# then copy source files inside the shared directory (needed to install dependencies)
cp -r YOUR_SRC_PATH ./ros_lattedrone/ros_ws/src/.
```

Then, start the container with `go.sh`. Once inside:
```bash
~/init_catkin_ws.sh
```
Dependencies should be installed and the workspace should be compiled succesfully.
Now you can use `./scripts/upload_ws.sh` to build and uploading your entire workspace to LattePanda (just remember to set `SOURCE_WS_PATH` accordingly).

## LattePanda setup
Realsense camera (guest side, a.k.a. LattePanda but outside docker):
```bash
lattedrone=user@ip
scp scripts/lattepanda_realsense_setup.sh $lattedrone:~

ssh $lattedrone
./lattepanda_realsense_setup.sh && rm lattepanda_realsense_setup.sh
```
