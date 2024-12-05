#include "sgm_gpu_node.h"

namespace sgm_gpu {

SgmGpuNode::SgmGpuNode() {
  node_handle_.reset(new ros::NodeHandle());
  private_node_handle_.reset(new ros::NodeHandle("~"));

  image_transport_.reset(new image_transport::ImageTransport(*node_handle_));

  // Read SGM parameters
  int p1, p2;
  bool check_consistency;
  private_node_handle_->param("p1", p1, 10);  // Default P1 = 10
  private_node_handle_->param("p2", p2, 120); // Default P2 = 120
  private_node_handle_->param("check_consistency", check_consistency, true);

  // Initialize SGM algorithm with parameters
  sgm_.reset(new SgmGpu(p1, p2, check_consistency));

  // Read topic names
  std::string left_image_topic, right_image_topic, left_camera_info_topic, right_camera_info_topic, disparity_topic;
  private_node_handle_->param("left_image_topic", left_image_topic, std::string("/camera/left/image_rect"));
  private_node_handle_->param("right_image_topic", right_image_topic, std::string("/camera/right/image_rect"));
  private_node_handle_->param("left_camera_info_topic", left_camera_info_topic, std::string("/camera/left/camera_info"));
  private_node_handle_->param("right_camera_info_topic", right_camera_info_topic, std::string("/camera/right/camera_info"));
  private_node_handle_->param("disparity_topic", disparity_topic, std::string("/sgm_gpu/disparity"));

  // Set up publishers and subscribers
  disparity_pub_ = private_node_handle_->advertise<stereo_msgs::DisparityImage>(disparity_topic, 1);

  left_image_sub_.subscribe(*image_transport_, left_image_topic, 10);
  right_image_sub_.subscribe(*image_transport_, right_image_topic, 10);
  left_info_sub_.subscribe(*node_handle_, left_camera_info_topic, 10);
  right_info_sub_.subscribe(*node_handle_, right_camera_info_topic, 10);

  // Synchronize image and camera info topics
  stereo_synchronizer_.reset(
    new StereoSynchronizer(left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_, 10)
  );
  stereo_synchronizer_->registerCallback(&SgmGpuNode::stereoCallback, this);
}

void SgmGpuNode::stereoCallback(
  const sensor_msgs::ImageConstPtr &left_image,
  const sensor_msgs::ImageConstPtr &right_image,
  const sensor_msgs::CameraInfoConstPtr &left_info,
  const sensor_msgs::CameraInfoConstPtr &right_info
) {
  if (disparity_pub_.getNumSubscribers() == 0)
    return;

  stereo_msgs::DisparityImage disparity;
  sgm_->computeDisparity(*left_image, *right_image, *left_info, *right_info, disparity);

  disparity_pub_.publish(disparity);
}

} // namespace sgm_gpu
