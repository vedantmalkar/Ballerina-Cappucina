# ESP32 Firmware Flashing Guide

## Step 1: Connect ESP32
Connect the ESP32 via USB and navigate to this folder:
'''
cd path/to/this/Movement
'''

## Step 2: Build the Project
Run the following command to compile the firmware:
'''
idf.py build
'''

## Step 3: Flash the Firmware
Flash the compiled code onto the ESP32:
'''
idf.py flash
'''
or specify the port explicitly:
'''
idf.py -p <PORT> flash
'''

### Important Note
Do **not** connect the ESP32 to the perfboard while flashing the code.

---

## Troubleshooting

- **Port not found error**  
  Ensure the ESP32 is properly connected via USB.  
  Check available ports using:
  '''
  ls /dev/ttyUSB*
  '''
  Then run:
  '''bash
  idf.py -p /dev/ttyUSB0 flash
  '''

- **Permission denied error**  
  Run the commands with \`sudo\`. For example:  
  '''
  sudo idf.py flash
  '''

