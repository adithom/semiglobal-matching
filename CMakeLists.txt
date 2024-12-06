cmake_minimum_required(VERSION 3.10)
project(semiglobal_matching LANGUAGES CXX CUDA)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  stereo_msgs
  sensor_msgs
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

# Add CUDA source files to a library
add_library(sgm_gpu_lib
  src/costs.cu
  src/hamming_cost.cu
  src/left_right_consistency.cu
  src/median_filter.cu
  src/sgm_gpu.cu
)

# Set CUDA-specific properties
set_target_properties(sgm_gpu_lib PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

# Link the CUDA library with OpenCV and Catkin libraries
target_link_libraries(sgm_gpu_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)

# Add the SGM GPU Node executable
add_executable(sgm_gpu_node
  src/sgm_gpu_node.cpp
  src/sgm_gpu_node_main.cpp
)

# Link the executable with the library and dependencies
target_link_libraries(sgm_gpu_node
  sgm_gpu_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)

# Declare a Catkin package
catkin_package(
  CATKIN_DEPENDS roscpp image_transport cv_bridge stereo_msgs sensor_msgs message_filters
  INCLUDE_DIRS include
  LIBRARIES sgm_gpu_lib
)

# Install the library and executable
install(TARGETS sgm_gpu_lib sgm_gpu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install the node executable
install(TARGETS sgm_gpu_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)