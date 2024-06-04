# Docker ROS images builder

This repo has many different uses:
1. Build a ROS image for either ROS 1 or 2
2. Work with a persistent container with a mounted ROS workspace
3. Upload the entire workspace to an edge device

## Supported images

Currently, you can target against:
[x] any ROS1 distribution (e.g. Noetic)
    [x] base: no-GUI, base packages
    [x] full: GUI with RViz and Gazebo, based on the `base` image
    [x] full-gpu: GUI with RViz and Gazebo with GPU support, based on `full` image
    [x] full-cudagl: (deprecated!) GUI with RViz and Gazebo with GPU support, based on official `nvidia/cudagl` image
[ ] any ROS2 distribution (e.g. Foxy)

## How to use
Create a ROS (either 1 or 2) image with:
```bash
# Just build the image
# NOTE: you have to manually build any dependent image, e.g. if you want full-gpu:
./init-ros-box.sh -d noetic -n base
./init-ros-box.sh -d noetic -n full # depends on 'base'
./init-ros-box.sh -d noetic -n full-gpu # depends on 'full'

# Build the image and deploy a container, using `ros_noetic` directory as your workspace
./init-ros-box.sh -d noetic -n full-gpu -t ./ros_noetic
```

The deployed container can be started with `go.sh`. Once inside:
```bash
~/init_catkin_ws.sh # to install missing dependencies with rosdep and build your workspace
```

## Upload the entire workspace in some edge device

Take a look at `./scripts/upload_ws.sh` to build and upload the workspace you've deployed anywhere with rsync (just remember to set `SOURCE_WS_PATH` accordingly).

## ToDos
[ ] Add support for more ROS2 distributions
[ ] Add support for more edge devices (cross compile with buildx)
[ ] Resolve internal image depedencies automatically when building
