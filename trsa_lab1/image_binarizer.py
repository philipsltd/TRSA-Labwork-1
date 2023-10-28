#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2

class ImageBinarizationNode(Node):
    def __init__(self):
        super().__init__('image_binarization_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_binarized', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Apply Binarization
            _, binarized_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)  # Adjust the threshold value as needed

            # Convert the processed image to an Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(binarized_image, encoding="mono8")

            # Publish the binarized image
            self.image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            self.get_logger().error('CV Bridge Error: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    image_binarization_node = ImageBinarizationNode()
    rclpy.spin(image_binarization_node)
    image_binarization_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
