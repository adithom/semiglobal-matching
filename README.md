#  Semi-Global Matching for Depth Estimation

Semi-Global Matching (SGM) is an algorithm used in computer vision to calculate depth information from two images taken by cameras that slightly offset (stereo vision). This project implements a GPU accelerated version of the algorithm and creates a ROS wrapper for use with ROS enabled control systems.

### Package Structure
```
semiglobal-matching/
├─ CMakeLists.txt
├─ Dockerfile
├─ include/
│  └─ sgm_gpu/
│     ├─ configuration.h
│     ├─ costs.h
│     ├─ cost_aggregation.h
│     ├─ hamming_cost.h
│     ├─ left_right_consistency.h
│     ├─ median_filter.h
│     ├─ sgm_gpu.h
│     └─ util.h
├─ launch/
│  ├─ semiglobal_launch.py
│  └─ semiglobal_matching.launch
├─ package.xml
├─ README.md
├─ src/
│  ├─ costs.cu
│  ├─ hamming_cost.cu
│  ├─ left_right_consistency.cu
│  ├─ median_filter.cu
│  ├─ sgm_gpu.cu
│  ├─ sgm_gpu_node.cpp
│  └─ sgm_gpu_node_main.cpp
└─ test/
   ├─ calib.txt
   ├─ depth_image.py
   ├─ image_publisher.py
   ├─ rectified_left.png
   └─ rectified_right.png

```

### Usage Instructions

1. Depends on Linux, ROS Noetic and CUDA. Additionally:
```
source /opt/ros/noetic/setup.bash
echo "source /opt/noetic/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```
sudo apt-get update
sudo apt-get install -y ros-noetic-roscpp ros-noetic-sensor-msgs ros-noetic-stereo-msgs ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-image-view
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y libopencv-dev
```

2. Create a catkin workspace
```
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
3. Clone the repo
```
git clone https://github.com/adithom/semiglobal-matching.git
chmod +x test/image_publisher.py
chmod +x test/depth_image.py
cd ~/ros_ws
catkin build
source install/setup.bash
echo "source ~/ros_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
4. Run the launch file
```
roslaunch semiglobal-matching semiglobal_matching.launch
```

### Instructions with Docker

You need raw rectified images or access to a stereo camera
```
docker build -t sgm:latest .
docker run -it --rm --gpus all sgm:latest
```
##### Inside the container:

```
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Run the sgm_gpu_node
rosrun semiglobal_matching sgm_gpu_node

# In another terminal (or use tmux inside the container)
# Publish images
rosrun semiglobal_matching image_publisher.py

# In another terminal
rosrun semiglobal_matching depth_image.py
```
