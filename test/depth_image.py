import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DisparitySaver(Node):
    def __init__(self):
        super().__init__('disparity_saver')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/sgm_gpu/disparity',
            self.listener_callback,
            10
        )
        self.processed = False  # To track if we've already processed an image

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
                self.get_logger().info('Disparity map saved as disparity_colored.png')

                # Optionally, display the image
                #cv2.imshow("Disparity Map", colored_disparity)
                #cv2.waitKey(1)

                # Mark as processed and unsubscribe
                self.processed = True
                self.subscription.destroy()
                self.get_logger().info('Unsubscribed after processing one image.')
            except Exception as e:
                self.get_logger().error(f'Failed to process image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = DisparitySaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
