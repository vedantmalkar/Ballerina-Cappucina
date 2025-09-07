# Ballerina-Cappucina

# SRA Eklavya 2025 Project - Ballerina-Cappucina

![Espressif](https://img.shields.io/badge/espressif-E7352C.svg?style=for-the-badge&logo=espressif&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

## Project Description:

Ballerina Cappucina, a graceful omnidirectional robot with a critical mission. This project brings her to life by designing and building a robot that can autonomously glide across a “dance floor,” identify scattered colorful “bombs” (balls) and scoop them up with precision.

A custom omnidirectional mobile base is created that allows for fluid movement, and a vision system using OpenCV for color detection. Integrating these systems we enable Ballerina Cappucina to autonomously seek, collect, and sort the objects, clearing the stage.


https://github.com/user-attachments/assets/bf337eea-feaa-4688-a723-102494bc5cdb


## Core Idea:
The robot begins by rotating in place, scanning the surroundings until it detects a ball of a specific color, which is chosen beforehand using OpenCV for color detection. Once it identifies the target ball, it moves toward it, aligning itself over the ball.

Underneath the robot, there’s a unique cavity mechanism, resembling a revolving door. As the robot moves over the ball, this mechanism traps the ball inside by closing a flapping door-like structure. After the ball is secured inside, the robot resumes its search, rotating again to find another ball, and the process repeats itself.

## [ros_ws](https://github.com/vedantmalkar/Ballerina-Cappucina/tree/main/ros_ws)
This folder contains the source files and configuration necessary to run the robot simulation in ROS. It includes the core scripts for controlling the robot’s movement, as well as pre-configured launch files. Additionally, you’ll find detailed instructions on how to set up and run the simulation in a ROS environment.
ROS files of Jetson are in jetson branch.

## [esp_code](https://github.com/vedantmalkar/Ballerina-Cappucina/tree/main/esp_code)
In this folder, you will find the source code for the ESP32 as well as detailed instructions on how to flash the code onto the device.

---
## Hardware:

To implement this project in real life, you’ll need the following hardware components:

- [Jetson Nano](https://www.electropi.in/nvidia-jetson-nano-developer-kit-b01) (Main computing unit)

- 4 [Omni-Wheels](https://robokits.co.in/robot-wheels/omni-wheels/premium-quality-brass-roller-bearing-omni-wheel-dual-row-100mm-dia) (For omnidirectional movement)

- 4 [Rhino GB37 Servo Motors](https://robokits.co.in/motors/rhino-gb37-12v-dc-geared-motor/dc-12v-encoder-servo-motors/rhino-gb37-12v-110rpm-6.5kgcm-dc-geared-encoder-servo-motor) (For movement control)

- 4 [Cytron DC Motor Drivers](https://robu.in/product/enhanced-13amp-dc-motor-driver-30a-peak-10-seconds/) (For motor control)

- Webcam (For visual perception)

- [ESP32](https://www.amazon.in/SquadPixel-ESP-32-Bluetooth-Development-Board/dp/B071XP56LM) (For communication and control)

- [MG995 Servo Motor](https://www.amazon.in/Robodo-Electronics-MG995-TowerPro-Servo/dp/B00MTH0RMI?source=ps-sl-shoppingads-lpcontext&psc=1&smid=AJ6SIZC8YQDZX) (For ball trapping mechanism)

<p align="center">
  <img src="media/Ballerina_front_view.jpeg" width="300" />
</p>

<img width="642" height="607" alt="topview" src="https://github.com/user-attachments/assets/4737a914-3214-4f21-a4ef-bb7ed9ccc2ce" />

[Ballerina Test Movement](media/Ballerina_test_movement.webm)
	
## Mentors:
- Prajwal Avhad
- Soham Kute
- Harsh Sagare

## Mentees:
- Lakshya Lalwani
- Vedant Malkar
- Bhakti Assar

## Contact us:

<p align="center">
  <div style="display: flex; justify-content: space-around; width: 100%; text-align: center;">
    <div style="width: 30%;">
      <h3>Vedant Malkar</h3>
      <p>
        <a href="https://mail.google.com/mail/?view=cm&fs=1&to=vmmalkar_b24@et.vjti.ac.in" target="_blank" style="text-decoration: none !important; display: inline-block;">
          <img src="media/gmail.png" alt="Gmail" width="30" style="border: 0;" />
        </a>
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
        <a href="https://github.com/vedantmalkar" target="_blank" style="text-decoration: none !important; display: inline-block;">
          <img src="media/github_icon.webp" alt="GitHub" width="30" style="border: 0;" />
        </a>
      </p>
    </div>
    <div style="width: 30%;">
      <h3>Lakshya Lalwani</h3>
      <p>
        <a href="https://mail.google.com/mail/?view=cm&fs=1&to=ldlalwani_b24@et.vjti.ac.in" target="_blank" style="text-decoration: none !important; display: inline-block;">
          <img src="media/gmail.png" alt="Gmail" width="30" style="border: 0;" />
        </a>
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
        <a href="https://github.com/Lakshyaa1" target="_blank" style="text-decoration: none !important; display: inline-block;">
          <img src="media/github_icon.webp" alt="GitHub" width="30" style="border: 0;" />
        </a>
      </p>
    </div>
    <div style="width: 30%;">
      <h3>Bhakti Assar</h3>
      <p>
        <a href="https://mail.google.com/mail/?view=cm&fs=1&to=Bbassar_b24@et.vjti.ac.in" target="_blank" style="text-decoration: none !important; display: inline-block;">
          <img src="media/gmail.png" alt="Gmail" width="30" style="border: 0;" />
        </a>
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
        <a href="https://github.com/Bhakti-A" target="_blank" style="text-decoration: none !important; display: inline-block;">
          <img src="media/github_icon.webp" alt="GitHub" width="30" style="border: 0;" />
        </a>
      </p>
    </div>
  </div>
</p>









