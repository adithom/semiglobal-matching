##  Semi-Global Matching for Depth Estimation

Semi-Global Matching (SGM) is an algorithm used in computer vision to calculate depth information from two images taken by cameras that slightly offset (stereo vision). This project implements a GPU accelerated version of the algorithm and creates a ROS wrapper for use with ROS enabled control systems.

#### Package Structure
```
semiglobal-matching/
├─ CMakeLists.txt
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
│  └─ semiglobal_matching.launch
├─ package.xml
├─ README.md
└─ src/
   ├─ costs.cu
   ├─ hamming_cost.cu
   ├─ left_right_consistency.cu
   ├─ median_filter.cu
   ├─ sgm_gpu.cu
   ├─ sgm_gpu_node.cpp
   └─ sgm_gpu_node_main.cpp
```

#### Usage Instructions

1. Depends on ROS Foxy and CUDA. Additionally:
```
sudo apt-get update
sudo apt-get install -y ros-foxy-roscpp ros-foxy-sensor-msgs ros-foxy-stereo-msgs ros-foxy-image-transport ros-foxy-cv-bridge ros-foxy-image-view
sudo apt-get install -y build-essential cmake git
sudo apt-get install -y libopencv-dev
```

2. Create a catkin workspace
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```
3. Clone the repo
```
git clone https://github.com/adithom/semiglobal-matching.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
4. Run the launch file
```
roslaunch semiglobal-matching semiglobal_matching.launch left_image_topic:=/my_cam/left/image_rect right_image_topic:=/my_cam/right/image_rect 
```

##### Sample Run

You need raw rectified images 
```
still working on this lmao
```