#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('process_image')
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.pub = self.create_publisher(Image, "/processed_image", 10)

    def image_callback(self, msg):
        # You could preprocess here (resize, filter, etc.)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

