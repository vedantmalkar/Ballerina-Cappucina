# JETSON NANO 

## Step 1: Connect JETSON
Connect the JETSON via HDMI and navigate to this folder:
```bash
cd path/to/this/catkin_ws
```

## Step 2: Build the Project
Run the following command to compile the firmware:
```bash
catkin_make
```

## Step 3: Source files
```bash
source devel/setup.bash
```
## Step 4: Flash the code
```bash
roslaunch ball_tracking follow_bal.launch
```
---

## Troubleshooting

- **Port not found error**    
  Run the following command:
  ```bash
  chmod 666 dev/ttyTHS1
  ```
  **Permission error**
  ```bash
  sudo chmod 666 dev/ttyTHS1
  ```

