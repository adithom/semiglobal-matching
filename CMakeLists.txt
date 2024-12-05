cmake_minimum_required(VERSION 3.0.2)
project(semiglobal_matching)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  stereo_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES semiglobal_matching
  CATKIN_DEPENDS roscpp image_transport cv_bridge stereo_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

cuda_add_library(sgm_gpu_lib
  src/costs.cu
  src/hamming_cost.cu
  src/left_right_consistency.cu
  src/median_filter.cu
  src/sgm_gpu.cu
)
target_link_libraries(sgm_gpu_lib ${OpenCV_LIBS} ${CUDA_LIBRARIES})

add_executable(sgm_gpu_node src/sgm_gpu_node.cpp src/sgm_gpu_node_main.cpp)
target_link_libraries(sgm_gpu_node sgm_gpu_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})
