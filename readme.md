# Docker ROS images builder

This repo has many different uses:
1. Build a ROS image for either ROS 1 or 2
2. Work with a persistent container with a mounted ROS workspace
3. Upload the entire workspace to an edge device

## How to use
Create a ROS (either 1 or 2) image with:
```bash
# Just build the image
./init-ros-box.sh -d noetic -n full-gpu

# Build the image and deploy a container, using `ros_noetic` directory as your workspace
./init-ros-box.sh -d noetic -n full-gpu -t ./ros_noetic
```

The deployed container can be started with `go.sh`. Once inside:
```bash
~/init_catkin_ws.sh # to install missing dependencies with rosdep and build your workspace
```

## Upload the entire workspace in some edge device

Take a look at `./scripts/upload_ws.sh` to build and upload the workspace you've deployed anywhere with rsync (just remember to set `SOURCE_WS_PATH` accordingly).
