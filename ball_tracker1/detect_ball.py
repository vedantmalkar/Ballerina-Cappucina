#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('detect_marker')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.marker_pub = self.create_publisher(Point, "/detected_marker", 10)

        # ArUco dictionary and parameters (OpenCV ≤ 4.2 compatible)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        point_msg = Point()

        if ids is not None and len(ids) > 0:
            for i, corner in enumerate(corners):
                pts = corner[0].astype(int)
                cx = int(np.mean(pts[:, 0]))
                cy = int(np.mean(pts[:, 1]))
                marker_id = int(ids[i][0])

                # Normalize centroid to range [-1, 1]
                h, w = frame.shape[:2]
                norm_x = (cx - w/2) / (w/2)
                norm_y = (cy - h/2) / (h/2)

                point_msg.x = norm_x
                point_msg.y = norm_y
                point_msg.z = marker_id

                self.marker_pub.publish(point_msg)

                # Draw marker for debugging
                cv2.polylines(frame, [pts], True, (0,255,0), 2)
                cv2.putText(frame, f"ID:{marker_id}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                cv2.circle(frame, (cx, cy), 5, (255,0,0), -1)

                break  # just publish first detected marker

        # Debug window (optional)
        cv2.imshow("Marker Detection", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

