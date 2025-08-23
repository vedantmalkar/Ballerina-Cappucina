#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist

class Movement_Node(Node):
    
    def __init__(self):
        super().__init__("move_bot")
        self.get_logger().info("Movement has been initialised")
        self.point_subscriber =  self.create_subscription(Point,"/detected_marker", self.movement_callback,10)
        self.movement_publisher = self.create_publisher(Twist,"/omni_bot/cmd_vel",1)
        
        self.timer = self.create_timer(0.2,self.timer_callback)
        self.marker_detected = False

    def movement_callback(self, msg: Point):
        self.get_logger().info(str(msg))
        pos_x = msg.x   # normalized [-1,1], left/right
        pos_y = msg.y   # normalized [-1,1], up/down
        marker_id = int(msg.z)  # which marker ID

        vel_msg = Twist()
        self.get_logger().info(f"Marker {marker_id} detected at x={pos_x:.2f}, y={pos_y:.2f}")

        # Rotate to center marker
        if -0.2 < pos_x < 0.2:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.3   # move forward
            self.get_logger().info("Moving towards marker")
        else:
            vel_msg.angular.z = -0.5 * pos_x   # proportional rotation
            self.get_logger().info("Rotating to align with marker")

        # Example stopping condition (if marker is low in image → close)
        if pos_y > 0.6:  
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.get_logger().info("Marker close - stopped")

        self.marker_detected = True
        self.movement_publisher.publish(vel_msg)

    def timer_callback(self):
        if not self.marker_detected:
            self.get_logger().info("No marker detected - scanning")
            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.4
            self.movement_publisher.publish(vel_msg)
        else:
            self.marker_detected = False

def main(args=None):
    rclpy.init(args=args)
    node = Movement_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

