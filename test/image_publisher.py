import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class StereoImagePublisher(Node):
    def __init__(self, left_image_path, right_image_path):
        super().__init__('stereo_image_publisher')
        self.left_pub = self.create_publisher(Image, '/my_cam/left/image_rect', 10)
        self.right_pub = self.create_publisher(Image, '/my_cam/right/image_rect', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz
        self.bridge = CvBridge()
        self.left_image = cv2.imread(left_image_path, cv2.IMREAD_COLOR)
        self.right_image = cv2.imread(right_image_path, cv2.IMREAD_COLOR)

    def timer_callback(self):
        left_msg = self.bridge.cv2_to_imgmsg(self.left_image, encoding='bgr8')
        right_msg = self.bridge.cv2_to_imgmsg(self.right_image, encoding='bgr8')
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StereoImagePublisher('rectified_left.png', 'rectified_right.png')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()