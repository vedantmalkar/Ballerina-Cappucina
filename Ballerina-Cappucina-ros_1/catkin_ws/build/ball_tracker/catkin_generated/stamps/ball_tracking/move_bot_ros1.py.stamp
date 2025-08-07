#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Twist

class MovementNode:
    
    def __init__(self):
        rospy.init_node("move_bot")
        rospy.loginfo("Movement has been initialised")

        self.point_sub = rospy.Subscriber("/detected_ball", Point, self.movement_callback)
        self.cmd_pub = rospy.Publisher("/omni_bot/cmd_vel", Twist, queue_size=1)

        self.ball_detected = False

        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)

    def movement_callback(self, msg):
        rospy.loginfo(f"Received: {msg}")

        pos_x = msg.x
        pos_y = msg.y
        pos_z = msg.z

        vel_msg = Twist()

        if -0.3 < pos_x < 0.3:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.5
            rospy.loginfo("Moving towards ball")
        else:
            vel_msg.angular.z = -0.1
            rospy.loginfo("Rotating")

        if pos_z >= 0.065:
            vel_msg.linear.x = 0.0
            rospy.loginfo("Ball close - stopped")

        self.ball_detected = True
        self.cmd_pub.publish(vel_msg)

    def timer_callback(self, event):
        if not self.ball_detected:
            rospy.loginfo("Started rotation - no ball detected")

            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5
            self.cmd_pub.publish(vel_msg)
        else:
            self.ball_detected = False


if __name__ == '__main__':
    node = MovementNode()
    rospy.spin()