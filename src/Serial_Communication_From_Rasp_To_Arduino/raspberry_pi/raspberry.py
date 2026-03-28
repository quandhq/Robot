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
            except:
                continue

def check_tick() -> bool:
    start_checking_tick_time = (int)(time.time())
    while True:
        if ser.in_waiting > 0:
            lines = ser.readlines()
            if lines:
                try:
                    line = lines[-1].decode("utf-8").strip()
                    angle = float(line)
                    print("Current angle: " + str(angle))
                    return True
                except (ValueError, UnicodeDecodeError, IndexError):
                    #Check the second latest data instead
                    if len(lines) >= 2:
                        try:
                            line = lines[-2].decode("utf-8").strip()
                            angle = float(line)
                            print("Current angle: " + str(angle))
                            return True
                        except:
                            pass
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
                
                