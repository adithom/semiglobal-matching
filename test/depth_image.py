#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DisparitySaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('disparity_saver', anonymous=True)

        # Create a subscriber to the disparity topic
        self.bridge = CvBridge()
        self.processed = False  # To process only one image
        self.subscriber = rospy.Subscriber('/sgm_gpu/disparity', Image, self.listener_callback)

    def listener_callback(self, msg):
        if not self.processed:
            try:
                # Convert ROS Image message to OpenCV image
                disparity_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

                # Normalize and colorize the disparity map
                normalized_disparity = cv2.normalize(disparity_image, None, 0, 255, cv2.NORM_MINMAX)
                normalized_disparity = normalized_disparity.astype(np.uint8)
                colored_disparity = cv2.applyColorMap(normalized_disparity, cv2.COLORMAP_JET)

                # Save the image as a PNG file
                cv2.imwrite('disparity_colored.png', colored_disparity)
                rospy.loginfo('Disparity map saved as disparity_colored.png')

                # Optionally, display the image
                # cv2.imshow("Disparity Map", colored_disparity)
                # cv2.waitKey(1)

                # Mark as processed and unregister the subscriber
                self.processed = True
                self.subscriber.unregister()
                rospy.loginfo('Unsubscribed after processing one image.')
            except Exception as e:
                rospy.logerr(f'Failed to process image: {e}')

if __name__ == '__main__':
    try:
        node = DisparitySaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
