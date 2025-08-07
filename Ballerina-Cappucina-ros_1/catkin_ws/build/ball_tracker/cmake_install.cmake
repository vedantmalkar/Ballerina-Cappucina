# Install script for directory: /home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/src/ball_tracker

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/build/ball_tracker/catkin_generated/installspace/ball_tracking.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_tracking/cmake" TYPE FILE FILES
    "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/build/ball_tracker/catkin_generated/installspace/ball_trackingConfig.cmake"
    "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/build/ball_tracker/catkin_generated/installspace/ball_trackingConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_tracking" TYPE FILE FILES "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/src/ball_tracker/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ball_tracking" TYPE PROGRAM FILES "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/build/ball_tracker/catkin_generated/installspace/detect_ball_ros1.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ball_tracking" TYPE PROGRAM FILES "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/build/ball_tracker/catkin_generated/installspace/move_bot_ros1.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ball_tracking" TYPE PROGRAM FILES "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/build/ball_tracker/catkin_generated/installspace/webcam_publisher.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_tracking" TYPE DIRECTORY FILES "/home/balli/Desktop/Ballerina-Cappucina-ros_1/catkin_ws/src/ball_tracker/launch")
endif()

