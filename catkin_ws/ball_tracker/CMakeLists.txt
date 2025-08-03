cmake_minimum_required(VERSION 3.0.2)
project(ball_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/detect_ball_ros1.py
  scripts/move_bot_ros1.py
  scripts/webcam_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
