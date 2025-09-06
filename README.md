# Ballerina-Cappucina

# SRA Eklavya 2025 Project - Ballerina-Cappucina

![Espressif](https://img.shields.io/badge/espressif-E7352C.svg?style=for-the-badge&logo=espressif&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

## Project Description:

Ballerina Cappucina, a graceful omnidirectional robot with a critical mission. This project brings her to life by designing and building a robot that can autonomously glide across a “dance floor,” identify scattered colorful “bombs” (balls), and scoop them up with precision.

A custom omnidirectional mobile base is created that allows for fluid movement, and a vision system using OpenCV for color detection. Integrating these systems we enable Ballerina Cappucina to autonomously seek, collect, and sort the objects, clearing the stage.




---
### Setup Instructions:
1. Clone this repository into your ROS 2 workspace folder:
```
git clone 
```
2. Build the workspace
```
cd Ballerina-Cappucina/
cd ros_ws/
colcon build
```
3. Source the workspace
```
source install/setup.bash
```
4. Launch the robot simulation
```
ros2 launch gz_rosa_control omni_bot.launch.py
```
5. To control your bot using keyboard, in a new terminal
```
source ~/ros_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/omni_bot/cmd_vel
```
6. Image processing node (select colour - blue,yellow,green)
```
ros2 launch ball_tracker detect_ball_launch.py color:=yellow
```
7. Start bot movement
```
ros2 run ball_tracker move_bot
```


<p align="center">
  <img src="media/simulation_gazebo.png" width="300" />
</p>

## Hardware:

To implement this project in real life, you’ll need the following hardware components:

- Jetson Nano (Main computing unit)

- 4 Omni-Wheels (For omnidirectional movement)

- 4 Rhino GB37 Servo Motors (For movement control)

- 4 Cytron DC Motor Drivers (For motor control)

- Webcam (For visual perception)

- ESP32 (For communication and control)

- MG995 Servo Motor (For ball trapping mechanism)

<p align="center">
  <img src="media/Ballerina_front_view.jpeg" width="300" />
</p>

[Ballerina Test Movement](media/Ballerina_test_movement.webm)
	
## Mentors:
- Prajwal Avhad
- Soham Kute
- Harsh

## Mentees:
- Lakshya Lalwani
- Vedant Malkar
- Bhakti Assar

# Contact Us

<p align="center">
  <div style="display: flex; justify-content: space-around; width: 100%; text-align: center;">
    <div style="width: 30%;">
      <h3>Vedant Malkar</h3>
      <p>
        Email: 
        <a href="https://mail.google.com/mail/?view=cm&fs=1&to=vmmalkar_b24@et.vjti.ac.in" target="_blank">
          <img src="https://upload.wikimedia.org/wikipedia/commons/5/51/Gmail_icon_%282013-2020%29.png" alt="Gmail" width="30" />
        </a>
      </p>
      <a href="https://github.com/vedantmalkar" target="_blank">
        <img src="https://upload.wikimedia.org/wikipedia/commons/9/91/Octicons-mark-github.svg" alt="GitHub" width="50" />
      </a>
    </div>
    <div style="width: 30%;">
      <h3>Lakshya Lalwani</h3>
      <p>
        Email: 
        <a href="https://mail.google.com/mail/?view=cm&fs=1&to=lakshya.lalwani@example.com" target="_blank">
          <img src="https://upload.wikimedia.org/wikipedia/commons/5/51/Gmail_icon_%282013-2020%29.png" alt="Gmail" width="30" />
        </a>
      </p>
      <a href="https://github.com/Lakshyaa1" target="_blank">
        <img src="https://upload.wikimedia.org/wikipedia/commons/9/91/Octicons-mark-github.svg" alt="GitHub" width="50" />
      </a>
    </div>
    <div style="width: 30%;">
      <h3>Bhakti Assar</h3>
      <p>
        Email: 
        <a href="https://mail.google.com/mail/?view=cm&fs=1&to=Bbassar_b24@et.vjti.ac.in" target="_blank">
          <img src="https://upload.wikimedia.org/wikipedia/commons/5/51/Gmail_icon_%282013-2020%29.png" alt="Gmail" width="30" />
        </a>
      </p>
      <a href="https://github.com/Bhakti-A" target="_blank">
        <img src="https://upload.wikimedia.org/wikipedia/commons/9/91/Octicons-mark-github.svg" alt="GitHub" width="50" />
      </a>
    </div>
  </div>
</p>





