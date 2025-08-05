import time
import serial

try:
    ser = serial.Serial(
        port= "/dev/ttyUSB0",
        baudrate= 115200,
        timeout=1
    )

    time.sleep(5)

    try:
        while(True):
            ser.write(b"0.4,0.3")

            print("The Letter 'A' has been sent!")
            time.sleep(1)

    except serial.SerialException:
        print("port is not connected")

    except KeyboardInterrupt:
        print("STOPPED")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("communication closed")
