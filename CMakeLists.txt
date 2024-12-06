cmake_minimum_required(VERSION 3.5)
project(semiglobal_matching)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# Compile GPU code
cuda_add_library(sgm_gpu_lib
  src/costs.cu
  src/hamming_cost.cu
  src/left_right_consistency.cu
  src/median_filter.cu
  src/sgm_gpu.cu
)
target_link_libraries(sgm_gpu_lib ${OpenCV_LIBS} ${CUDA_LIBRARIES})

add_executable(sgm_gpu_node src/sgm_gpu_node.cpp src/sgm_gpu_node_main.cpp)
target_link_libraries(sgm_gpu_node sgm_gpu_lib ${OpenCV_LIBS} ${CUDA_LIBRARIES})

# Link against ROS 2 packages
ament_target_dependencies(sgm_gpu_node rclcpp image_transport cv_bridge stereo_msgs sensor_msgs)

install(TARGETS sgm_gpu_lib sgm_gpu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(PROGRAMS
  scripts/image_publisher.py
  scripts/depth_image.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()