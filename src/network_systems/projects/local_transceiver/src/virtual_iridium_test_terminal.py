import serial
import time


def read_all(ser, timeout=0.5):
    ser.timeout = timeout
    lines = []
    while True:
        data = ser.read(1024)
        if not data:
            break
        lines.append(data.decode("utf-8", errors="replace"))
    return "".join(lines)


def main():
    port = input("Enter the virtual serial port (e.g., /dev/pts/4): ").strip()
    baudrate = 19200

    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud.")
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    print("Type AT commands. Press Ctrl+C to exit.")
    try:
        while True:
            cmd = input("> ")
            if not cmd.endswith("\r"):
                cmd += "\r"
            ser.write(cmd.encode("utf-8"))
            time.sleep(
                0.1
            )  # Give the virtual iridium some time to respond, adjust this if you're finding that responses are staggered
            response = read_all(ser)
            if not response:
                response = ser.read(1024)
            print(response)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
