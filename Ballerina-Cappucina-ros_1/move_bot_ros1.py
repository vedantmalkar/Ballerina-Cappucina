#!/usr/bin/env python

import rospy
import serial
import time
import sys
from geometry_msgs.msg import Point

class MovementNode:

    def __init__(self):
        rospy.init_node("move_bot")
        rospy.loginfo("Movement has been initialised")

        # Initialize serial port inside the class and handle errors gracefully
        try:
            self.ser = serial.Serial(
                port="/dev/ttyTHS1",
                baudrate=115200,
                timeout=1
            )
            time.sleep(2)  # Wait for the serial port to be ready
            rospy.loginfo("Serial port connected on %s", self.ser.port)
        except serial.SerialException as e:
            rospy.logfatal("Failed to connect to serial port: %s", e)
            sys.exit(1)

        self.point_sub = rospy.Subscriber("/detected_ball", Point, self.movement_callback)
        # The ROS publisher is no longer needed, as we are using serial.
        # self.cmd_pub = rospy.Publisher("/omni_bot/cmd_vel", Twist, queue_size=1)

        self.ball_detected = False
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)
    
    def movement_callback(self, msg):
        rospy.loginfo("Received: %s", msg)
        self.ball_detected = True
        
        pos_x = msg.x
        pos_z = msg.z
        
        linear_x = 0.0
        angular_z = 0.0
        
        # Determine the movement command based on ball position
        if -0.3 < pos_x < 0.3:
            linear_x = 0.5
            angular_z = 0.0
            rospy.loginfo("Moving towards ball")
        else:
            linear_x = 0.0
            angular_z = -0.1
            rospy.loginfo("Rotating")

        # If ball is close, stop movement
        if pos_z >= 0.065:
            linear_x = 0.0
            angular_z = 0.0
            rospy.loginfo("Ball close - stopped")

        # Send the final calculated command via serial
        data_to_send = "%f %f\n" % (linear_x, angular_z)
        try:
            self.ser.write(data_to_send.encode("utf-8"))
            rospy.loginfo("Data sent to ESP: %s", data_to_send)
        except serial.SerialException as e:
            rospy.logerr("Serial communication error: %s", e)
    
    def timer_callback(self, event):
        if not self.ball_detected:
            rospy.loginfo("Started rotation - no ball detected")
            
            # Send a command to rotate when no ball is detected
            linear_x = 0.0
            angular_z = -0.5
            
            data_to_send = "%f %f\n" % (linear_x, angular_z)
            try:
                self.ser.write(data_to_send.encode("utf-8"))
                rospy.loginfo("Data sent to ESP: %s", data_to_send)
            except serial.SerialException as e:
                rospy.logerr("Serial communication error: %s", e)
        else:
            self.ball_detected = False

if __name__ == '__main__':
    node = MovementNode()
    try:
        rospy.spin()
    finally:
        rospy.loginfo("Shutting down node, closing serial port...")
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            rospy.loginfo("Serial port closed.")

