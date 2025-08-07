#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def main():
    rospy.init_node('webcam_publisher')
    pub = rospy.Publisher('/image_in', Image, queue_size=1)
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Webcam not accessible")
        return

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    main()
