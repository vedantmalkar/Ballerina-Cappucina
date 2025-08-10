import time
import serial

try:
    # Open serial port
    ser = serial.Serial(
        port="/dev/ttyUSB0",
        baudrate=115200,
        timeout=1
    )

    time.sleep(2)  # Allow serial connection to initialize

    try:
        while True:
            for pattern in range(1, 6):  # Loop through patterns 1 to 5
                cmd = "{}\n".format(pattern)  # Pattern number with newline
                ser.write(cmd.encode())       # Send as bytes
                print("Sent pattern:", pattern)

                # Wait long enough for the ESP32 pattern to finish
                time.sleep(6)  # 3s run + 1s stop + extra buffer

    except serial.SerialException:
        print("Port is not connected.")

    except KeyboardInterrupt:
        print("STOPPED by user.")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Communication closed.")

