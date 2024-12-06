find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  stereo_msgs
  sensor_msgs
  # and any other ROS 1 packages
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

add_library(sgm_gpu_lib
  src/costs.cu
  src/hamming_cost.cu
  src/left_right_consistency.cu
  src/median_filter.cu
  src/sgm_gpu.cu
)
target_link_libraries(sgm_gpu_lib ${OpenCV_LIBS} ${CUDA_LIBRARIES})

add_executable(sgm_gpu_node src/sgm_gpu_node.cpp src/sgm_gpu_node_main.cpp)
target_link_libraries(sgm_gpu_node sgm_gpu_lib ${OpenCV_LIBS} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS sgm_gpu_node sgm_gpu_lib
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS
  scripts/image_publisher.py
  scripts/depth_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
