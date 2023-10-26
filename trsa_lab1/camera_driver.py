#!/usr/bin/env python

import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main(args=None):
	rclpy.init(args=args)
	node = rclpy.create_node('camera_driver')
	image_pub = node.create_publisher(Image, '/camera/image_raw', 10)

	cap = cv2.VideoCapture("/home/trsa2024/ros2_lab1/src/trsa_lab1/video/test.mov")  # Use the default camera (change if necessary)
	bridge = CvBridge()

	while rclpy.ok():
		ret, frame = cap.read()
		if ret:
			try:
				image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
				image_pub.publish(image_msg)
			except Exception as e:
				node.get_logger().error(str(e))

	rclpy.shutdown()

if __name__ == '__main__':
	main()
