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

