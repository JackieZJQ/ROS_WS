import sys
sys.path.append(f'/home/jack/miniconda3/envs/py36/lib/python3.6/site-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2



class camLane(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("---车道线检测模块初始化---")
        self.shortImagesub = self.create_subscription(Image, "short_focal_image", self.shortImagecallback, 10)
        self.bridge = CvBridge()
        
    
    def shortImagecallback(self, image):

        img = self.bridge.imgmsg_to_cv2(image, 'bgr8')

        cv2.imshow("Test Window", img)

        return None

def main(args=None):
    
    rclpy.init(args=args)
    node = camLane("camLane")
    rclpy.spin(node)
    rclpy.shutdown()
