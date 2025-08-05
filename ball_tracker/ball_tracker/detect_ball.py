import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import ball_tracker.process_image as proc

class Ball_Detector_Node(Node): 

    def __init__(self):
        super().__init__('detect_ball')

        self.get_logger().info('Ball Tracking Initialised')
        self.image_subscriber = self.create_subscription(Image,"/image_in",self.image_callback,rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        self.image_out_publisher = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_publisher = self.create_publisher(Image, "/image_tuning", 1)
        self.ball_point_publisher  = self.create_publisher(Point,"/detected_ball",1)

        self.declare_parameter('tuning_mode', False)

        self.declare_parameter("x_min",0)
        self.declare_parameter("x_max",100)
        self.declare_parameter("y_min",0)
        self.declare_parameter("y_max",100)
        self.declare_parameter("h_min",0)
        self.declare_parameter("h_max",180)
        self.declare_parameter("s_min",0)
        self.declare_parameter("s_max",255)
        self.declare_parameter("v_min",0)
        self.declare_parameter("v_max",255)
        self.declare_parameter("sz_min",0)
        self.declare_parameter("sz_max",100)
        
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()

        if(self.tuning_mode):
            proc.create_tuning_window(self.tuning_params)

    def image_callback(self,msg): 
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            #uses the find_circles function to get tuning and out images and publishes them 
            if (self.tuning_mode):
                self.tuning_params = proc.get_tuning_params()

            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")   #converts opencv image to ros_msg
            img_to_pub.header = msg.header
            self.image_out_publisher.publish(img_to_pub)

            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = msg.header
            self.image_tuning_publisher.publish(img_to_pub)
	
            point_out = Point()

            for i, kp in enumerate(keypoints_norm):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size

                self.get_logger().info(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s

            if (point_out.z > 0):
                self.ball_point_publisher.publish(point_out) 
        except CvBridgeError as e:
            print(e)  

	
def main(args=None): 
    rclpy.init(args=args)

    detect_ball = Ball_Detector_Node()
    while rclpy.ok():       #checks if ros2 is running
        rclpy.spin_once(detect_ball)
        proc.wait_on_gui()      #waitKey(0)

    detect_ball.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
