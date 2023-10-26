import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageReaderNode(Node):
    def __init__(self):
        super().__init__('image_reader_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.image_rec_sub = self.create_subscription(
            Image, '/camera/image_rect', self.image_rect_callback, 10)
        self.image_processed_sub = self.create_subscription(
            Image, '/camera/image_processed', self.image_processed_callback, 10)
        self.image_binarized_sub = self.create_subscription(
            Image, '/camera/image_binarized', self.image_binarized_callback, 10)
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Image Viewer", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error('CV Bridge Error: %s' % str(e))

    def image_rect_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv2.imshow("Image Rectified Viewer", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error('CV Bridge Error: %s' % str(e))

    def image_processed_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv2.imshow("Image Processed Viewer", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error('CV Bridge Error: %s' % str(e))

    def image_binarized_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv2.imshow("Image Binarized Viewer", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            self.get_logger().error('CV Bridge Error: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    image_reader_node = ImageReaderNode()
    rclpy.spin(image_reader_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
