import serial
import time
UART_PORT = "/dev/ttyTHS1"
BAUD_RATE = 115200

try:
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {UART_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Error opening serial port {UART_PORT}: {e}")
    exit(1)

def send_pattern(pattern_num):
    if 1 <= pattern_num <= 5:
        cmd = f"{pattern_num}\n"
        ser.write(cmd.encode())
        print(f"Sent pattern: {pattern_num}")
    else:
        print("Invalid pattern number. Use 1-5.")

if __name__ == "__main__":
    try:
        while True:
            # Example: cycle through patterns 1–5
            for p in range(1, 6):
                send_pattern(p)
                time.sleep(5)  # Wait between commands
    except KeyboardInterrupt:
        print("\nStopping sender.")
        ser.close()

