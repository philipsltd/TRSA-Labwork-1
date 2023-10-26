#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from image_geometry import PinholeCameraModel
import os
import yaml
from sensor_msgs.msg import CameraInfo
import ament_index_python

class ImageRectifierNode(Node):
    def __init__(self):
        super().__init__('image_rectifier_node')
        self.camera_model = PinholeCameraModel()
        calibration_path = os.path.join(ament_index_python.get_package_share_directory('trsa_lab1'), 'calibration','ost.yaml')
        self.load_camera_calibration(calibration_path)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        self.bridge = CvBridge()

    def load_camera_calibration(self, calibration_path):
        with open(calibration_path, 'r') as file:
            self.camera_info = CameraInfo()
            calibration_data = yaml.safe_load(file)
            self.camera_info.width = calibration_data['image_width']
            self.camera_info.height = calibration_data['image_height']
            self.camera_info.distortion_model = calibration_data['distortion_model']
            self.camera_info.d = calibration_data['distortion_coefficients']['data']
            self.camera_info.k = calibration_data['camera_matrix']['data']
            self.camera_info.r = calibration_data['rectification_matrix']['data']
            self.camera_info.p = calibration_data['projection_matrix']['data']

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rectified_image = cv_image.copy()  # Create a copy
            self.camera_model.fromCameraInfo(self.camera_info)
            self.camera_model.rectifyImage(cv_image, rectified_image)
            rectified_image_msg = self.bridge.cv2_to_imgmsg(rectified_image, encoding="bgr8")
            self.image_pub.publish(rectified_image_msg)
        except CvBridgeError as e:
            self.get_logger().error('CV Bridge Error: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    image_rectifier_node = ImageRectifierNode()
    rclpy.spin(image_rectifier_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
