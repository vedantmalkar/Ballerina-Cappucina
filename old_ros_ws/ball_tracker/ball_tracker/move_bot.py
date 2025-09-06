#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist

class Movement_Node(Node):
    
    def __init__(self):
        super().__init__("move_bot")
        self.get_logger().info("Movement has been initialised")
        self.point_subscriber =  self.create_subscription(Point,"/detected_ball", self.movement_callback,10)
        self.movement_publisher = self.create_publisher(Twist,"/omni_bot/cmd_vel",1)
        
        self.timer = self.create_timer(0.2,self.timer_callback)
        self.ball_detected = False

        
    def movement_callback(self, msg: Point):
        self.get_logger().info(str(msg))
        pos_x = msg.x
        pos_y = msg.y
        pos_z = msg.z

        vel_msg = Twist()
        self.get_logger().info("message recieved")
        if -0.4 < pos_x < 0.4:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.5
            self.get_logger().info("moving towards bot")
            
        else:
            vel_msg.angular.z = -0.1   
            self.get_logger().info("rotating")         

        if pos_z >= 0.065:
            vel_msg.linear.x = 0.0
            self.get_logger().info("ball close - stopped")

        self.ball_detected = True

        self.movement_publisher.publish(vel_msg)

    def timer_callback(self):
        if not self.ball_detected:
            self.get_logger().info("started rotation - no ball detected")

            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5
            self.movement_publisher.publish(vel_msg)
        else:
            self.ball_detected = False

def main(args=None):
    rclpy.init(args=args)

    node = Movement_Node()
    rclpy.spin(node)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()