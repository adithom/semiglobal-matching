#include "sgm_gpu/sgm_gpu_node.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sgm_gpu_node");
  sgm_gpu::SgmGpuNode sgm_gpu;
  ros::spin();

  return 0;
}

