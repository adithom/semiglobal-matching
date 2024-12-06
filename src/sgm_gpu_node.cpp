#include "sgm_gpu/sgm_gpu_node.h"

namespace sgm_gpu {

SgmGpuNode::SgmGpuNode() {
  // Initialize node handles
  node_handle_.reset(new ros::NodeHandle());
  private_node_handle_.reset(new ros::NodeHandle("~"));

  // Initialize image transport
  image_transport_.reset(new image_transport::ImageTransport(*node_handle_));

  // Read image dimensions
  int image_width, image_height;
  private_node_handle_->param("image_width", image_width, 1920);
  private_node_handle_->param("image_height", image_height, 1080);

  // Initialize the SGM algorithm
  sgm_.reset(new SgmGpu(*private_node_handle_, image_width, image_height));

  // Read topic names
  std::string left_image_topic, right_image_topic, left_camera_info_topic, right_camera_info_topic, disparity_topic;
  private_node_handle_->param("left_image_topic", left_image_topic, std::string("/camera/left/image_rect"));
  private_node_handle_->param("right_image_topic", right_image_topic, std::string("/camera/right/image_rect"));
  private_node_handle_->param("left_camera_info_topic", left_camera_info_topic, std::string("/camera/left/camera_info"));
  private_node_handle_->param("right_camera_info_topic", right_camera_info_topic, std::string("/camera/right/camera_info"));
  private_node_handle_->param("disparity_topic", disparity_topic, std::string("/sgm_gpu/disparity"));

  // Set up publishers
  disparity_pub_ = private_node_handle_->advertise<stereo_msgs::DisparityImage>(disparity_topic, 1);

  // Set up message_filters subscribers
  left_image_sub_.subscribe(*node_handle_, left_image_topic, 10);
  right_image_sub_.subscribe(*node_handle_, right_image_topic, 10);
  left_info_sub_.subscribe(*node_handle_, left_camera_info_topic, 10);
  right_info_sub_.subscribe(*node_handle_, right_camera_info_topic, 10);

  // Synchronize subscribers
  stereo_synchronizer_.reset(
    new message_filters::Synchronizer<StereoSyncPolicy>(StereoSyncPolicy(10), left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_)
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
