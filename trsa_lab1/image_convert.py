#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2

class ImageConvertNode(Node):
    def __init__(self):
        super().__init__('image_convert_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, '/camera/image_rect', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_processed', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Convert the image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur
            smoothed_image = cv2.GaussianBlur(gray_image, (21, 21), 0, 0)

            # Apply Canny edge detection
            edged_image = cv2.Canny(smoothed_image, 15, 20) # TODO - Adjust these parameters for bolder contours

            # Convert the processed image to an Image message
            processed_image_msg = self.bridge.cv2_to_imgmsg(edged_image, encoding="mono8")

            # Publish the processed image
            self.image_pub.publish(processed_image_msg)
        except CvBridgeError as e:
            self.get_logger().error('CV Bridge Error: %s' % str(e))



def main(args=None):
    rclpy.init(args=args)
    image_convert_node = ImageConvertNode()
    rclpy.spin(image_convert_node)
    image_convert_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
