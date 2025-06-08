import serial
import time

COM_PORT = "/dev/ttyLP2"
BAUD_RATE = 9600

def enter_command_mode(ser, retries=3):
    for attempt in range(retries):
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(1)
        ser.write(b"+++")
        time.sleep(1)
        response = ser.read(ser.in_waiting).decode(errors="ignore").strip()
        if "OK" in response:
            print("Entered command mode.")
            return True
        else:
            print(f"Attempt {attempt+1} failed. Response: {response}")
    print("Failed to enter command mode after retries.")
    return False


def send_command(ser, command, wait_response=True):
    ser.write(command.encode() + b"\r")
    time.sleep(0.5)
    if wait_response:
        response = ser.read(ser.in_waiting).decode(errors="ignore").strip()
        print(f"Response: {response}")
        return response
    return ""

def exit_command_mode(ser):
    send_command(ser, "ATCN")
    print("Exited command mode.")

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=10)
    time.sleep(1)

    # Configure once if needed
    if enter_command_mode(ser):
        send_command(ser, "ATSH")
        send_command(ser, "ATSL")
        send_command(ser, "ATDH 13A200")
        send_command(ser, "ATDL 424F58F7")
        send_command(ser, "ATID")
        send_command(ser, "ATPL 2")
        exit_command_mode(ser)

    print("Now communicating in transparent mode")

    while True:
        # Read incoming messages
        if ser.in_waiting > 0:
            message = ser.read(ser.in_waiting).decode(errors="ignore")
            print(f"[Received]: {message.strip()}")

        # Continuously send message
        ser.write(b"Message from BFM\n Other Message\n")

        time.sleep(1)

except serial.SerialException as e:
    print(f"Serial error: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()


