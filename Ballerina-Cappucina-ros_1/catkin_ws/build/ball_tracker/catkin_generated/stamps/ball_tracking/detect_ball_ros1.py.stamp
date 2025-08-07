#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import src.process_image as proc  

class BallDetectorNode:
    def __init__(self):
        rospy.init_node('detect_ball', anonymous=True)
        rospy.loginfo('Ball Tracking Initialised')

        self.bridge = CvBridge()

        # Load parameters
        self.tuning_mode = rospy.get_param('~tuning_mode', False)

        self.tuning_params = {
            'x_min': rospy.get_param('~x_min', 0),
            'x_max': rospy.get_param('~x_max', 100),
            'y_min': rospy.get_param('~y_min', 0),
            'y_max': rospy.get_param('~y_max', 100),
            'h_min': rospy.get_param('~h_min', 0),
            'h_max': rospy.get_param('~h_max', 180),
            's_min': rospy.get_param('~s_min', 0),
            's_max': rospy.get_param('~s_max', 255),
            'v_min': rospy.get_param('~v_min', 0),
            'v_max': rospy.get_param('~v_max', 255),
            'sz_min': rospy.get_param('~sz_min', 0),
            'sz_max': rospy.get_param('~sz_max', 100)
        }

        if self.tuning_mode:
            proc.create_tuning_window(self.tuning_params)

        # Publishers
        self.image_out_pub = rospy.Publisher("/image_out", Image, queue_size=1)
        self.image_tuning_pub = rospy.Publisher("/image_tuning", Image, queue_size=1)
        self.ball_point_pub = rospy.Publisher("/detected_ball", Point, queue_size=1)

        # Subscriber
        rospy.Subscriber("/image_in", Image, self.image_callback)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        try:
            if self.tuning_mode:
                self.tuning_params = proc.get_tuning_params()

            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = msg.header
            self.image_out_pub.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = msg.header
            self.image_tuning_pub.publish(img_to_pub)

            point_out = Point()

            for i, kp in enumerate(keypoints_norm):
                x, y, s = kp.pt[0], kp.pt[1], kp.size
                rospy.loginfo(f"Pt {i}: ({x},{y},{s})")
                if s > point_out.z:
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if point_out.z > 0:
                self.ball_point_pub.publish(point_out)

        except CvBridgeError as e:
            rospy.logerr(e)

    def spin(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            proc.wait_on_gui()
            rate.sleep()


if __name__ == '__main__':
    node = BallDetectorNode()
    node.spin()