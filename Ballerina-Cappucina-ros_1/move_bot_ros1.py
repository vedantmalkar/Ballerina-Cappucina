#!/usr/bin/env python

import rospy
import serial
import time
from geometry_msgs.msg import Point, Twist

class MovementNode:
    
    def __init__(self):
        rospy.init_node("move_bot")
        rospy.loginfo("Movement has been initialised")
		
 	ser = serial.Serial(
        	port= "/dev/ttyTHS1",
        	baudrate= 115200,
        	timeout=1
    	)
	
	time.sleep(2)	
	rospy.loginfo("serial port has been connected")

        self.point_sub = rospy.Subscriber("/detected_ball", Point, self.movement_callback)
        self.cmd_pub = rospy.Publisher("/omni_bot/cmd_vel", Twist, queue_size=1)

        self.ball_detected = False

        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)

	

    def movement_callback(self, msg):
        rospy.loginfo("Received: %s", msg)

        pos_x = msg.x
        pos_y = msg.y
        pos_z = msg.z

        vel_msg = Twist()
	
	linear_x = 0.0
	angular_z = 0.0	

        if -0.3 < pos_x < 0.3:
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = 0.5
            rospy.loginfo("Moving towards ball")
	    linear_x = 0.5
	    angular_z = 0.0	

        else:
            vel_msg.angular.z = -0.1
            rospy.loginfo("Rotating")
	    linear_x = 0.0
	    angular_z = -0.1	

        if pos_z >= 0.065:
            vel_msg.linear.x = 0.0
            rospy.loginfo("Ball close - stopped")
	    linear_x = 0.0
	    angular_z = 0.0	

        self.ball_detected = True
        self.cmd_pub.publish(vel_msg)
	data_to_send = "%f %f" % (linear_x,angular_z)
	self.ser.write(data_to_send.encode("utf-8"))
	rospy.loginfo("data sent to esp: %s", data_to_send)

    def timer_callback(self, event):

	linear_x = 0.0
	angular_z = 0.0	

        if not self.ball_detected:
            rospy.loginfo("Started rotation - no ball detected")

            vel_msg = Twist()
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5
            self.cmd_pub.publish(vel_msg)

	    linear_x = 0.0
	    angular_z = -0.5

	    data_to_send = "%f %f\n" % (linear_x,angular_z)
	    self.ser.write(data_to_send.encode("utf-8"))
	    rospy.loginfo("data sent to esp: %s", data_to_send)	

        else:
            self.ball_detected = False


if __name__ == '__main__':
    node = MovementNode()
    rospy.spin()
