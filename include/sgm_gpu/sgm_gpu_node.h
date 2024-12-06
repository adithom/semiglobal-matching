#ifndef SGM_GPU_NODE_H
#define SGM_GPU_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <memory>

#include "sgm_gpu/sgm_gpu.h"

namespace sgm_gpu {

class SgmGpuNode {
public:
  SgmGpuNode();

private:
  void stereoCallback(
    const sensor_msgs::ImageConstPtr &left_image,
    const sensor_msgs::ImageConstPtr &right_image,
    const sensor_msgs::CameraInfoConstPtr &left_info,
    const sensor_msgs::CameraInfoConstPtr &right_info
  );

  // Node handles
  std::unique_ptr<ros::NodeHandle> node_handle_;
  std::unique_ptr<ros::NodeHandle> private_node_handle_;

  // Image transport and subscribers
  std::unique_ptr<image_transport::ImageTransport> image_transport_;
  message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;

  // Synchronization policy
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    sensor_msgs::CameraInfo
  > StereoSyncPolicy;

  std::unique_ptr<message_filters::Synchronizer<StereoSyncPolicy>> stereo_synchronizer_;

  // Publisher
  ros::Publisher disparity_pub_;

  // SGM algorithm instance
  std::unique_ptr<SgmGpu> sgm_;
};

} // namespace sgm_gpu

#endif // SGM_GPU_NODE_H
