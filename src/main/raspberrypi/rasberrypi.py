import serial
import time

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.5)

def wait_for_ready():
    print("Waiting for Arduino to calibrate...")        
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode("utf-8").strip()
                if line == "ready!":
                    print("Connection established! Robot is balanced.")
                    break
            except (UnicodeDecodeError, ValueError):
                continue
    # CLEAR THE INPUT BUFFER: Throw away any leftover data or boot-up noise sent from arduino
    ser.reset_input_buffer()

def send_configuration_parameters():
    print("Send configuration")
    # Expected format: "Kp,Ki,Kd,minPower,trim\n"
    # "50.0,200.0,4.0,10,1.0\n"
    Kp = 50.0
    Ki = 200.0
    Kd = 4.0
    minPower = 10
    trim = 1.0
    config_str = f"{Kp},{Ki},{Kd},{minPower},{trim}\n"
    ser.write(config_str.encode('utf-8'))
    while True:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode("utf-8").strip()
                if line != "ready!":
                    print(line)
                    break
                else:   #catch the remaining "ready!" messages from wait_for_ready phase
                    continue
            except (UnicodeDecodeError, ValueError):
                continue
    
    # CLEAR THE INPUT BUFFER: Throw away any leftover data or boot-up noise sent from arduino to start balancing process
    ser.reset_input_buffer()

def check_tick() -> bool:
    start_checking_tick_time = (int)(time.time())
    while True:
        if ser.in_waiting > 0:
            raw_block = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
            # Check if the very last character is a newline
            # If it's NOT, the last segment is a "partial" and we should ignore it
            is_latest_data_complete = raw_block.endswith('\n')
            lines = raw_block.strip().split('\n')
            lines = [l.strip() for l in lines]
            if not lines:
                continue
            # If the last line is complete, take [-1]. 
            # If not, and we have more data, take [-2].
            target_index = -1 if is_latest_data_complete else -2

            try:
                if abs(target_index) <= len(lines):
                    line = lines[target_index].strip()
                    angle = float(line)
                    print(f"Current angle: {angle}")
                    return True
            except ValueError:
                pass

        if (int)(time.time()) - start_checking_tick_time > 1:
            break
    return False

if __name__ == "__main__":
    ser.reset_input_buffer()

    while True:
        #Opening serial port from rasp causes arduino connected by USB cable to reset
        # --- Handshake Phase ---
        wait_for_ready()
        send_configuration_parameters()
        while True:
            try:
                # User enters -5 (lean back/move back) to 5 (lean forward/move forward)
                cmd = int(input("Enter tilt command (-5 to 5 degrees), minus is forward, plus is backward: "))
                cmd = max(-5, min(5, cmd)) # Safety clamp
                
                # Convert to a signed byte and send
                ser.write(cmd.to_bytes(1, byteorder='big', signed=True))
                print(f"Sent: {cmd} degrees")
            except ValueError:
                print("Please enter a valid number.")

            acknowledged = check_tick()
            if acknowledged == False:
                print("No acknowledged tick from arduino, try handshaking again.")
                break
                
                