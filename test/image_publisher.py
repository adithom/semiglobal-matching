#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoImagePublisher:
    def __init__(self, left_image_path, right_image_path):
        # Initialize the ROS node
        rospy.init_node('stereo_image_publisher', anonymous=True)

        # Create publishers for left and right images
        self.left_pub = rospy.Publisher('/camera/left/image_rect', Image, queue_size=10)
        self.right_pub = rospy.Publisher('/camera/right/image_rect', Image, queue_size=10)

        # Load images using OpenCV
        self.bridge = CvBridge()
        self.left_image = cv2.imread(left_image_path, cv2.IMREAD_COLOR)
        self.right_image = cv2.imread(right_image_path, cv2.IMREAD_COLOR)

        # Publish images at 10 Hz
        self.rate = rospy.Rate(10)

    def publish_images(self):
        while not rospy.is_shutdown():
            # Convert OpenCV images to ROS Image messages
            left_msg = self.bridge.cv2_to_imgmsg(self.left_image, encoding='bgr8')
            right_msg = self.bridge.cv2_to_imgmsg(self.right_image, encoding='bgr8')

            # Publish the messages
            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        # Provide paths to the rectified images
        left_image_path = 'rectified_left.png'
        right_image_path = 'rectified_right.png'

        # Create and run the publisher
        node = StereoImagePublisher(left_image_path, right_image_path)
        node.publish_images()
    except rospy.ROSInterruptException:
        pass
