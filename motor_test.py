import serial
import time

UART_PORT = "/dev/ttyTHS1"
BAUD_RATE = 115200

# Try to open serial port
try:
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
    print("Connected to {} at {} baud.".format(UART_PORT, BAUD_RATE))
except serial.SerialException as e:
    print("Error opening serial port {}: {}".format(UART_PORT, e))
    exit(1)

# Function to send pattern number
def send_pattern(pattern_num):
    if 1 <= pattern_num <= 5:
        cmd = "{}\n".format(pattern_num)
        ser.write(cmd.encode())
        print("Sent pattern: {}".format(pattern_num))
    else:
        print("Invalid pattern number. Use 1-5.")

# Main loop
if __name__ == "__main__":
    try:
        while True:
            # Cycle through patterns 1–5
            for p in range(1, 6):
                send_pattern(p)
                time.sleep(5)  # Wait between commands
    except KeyboardInterrupt:
        print("\nStopping sender.")
        ser.close()

