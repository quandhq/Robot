import serial
import time

if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    ser.reset_input_buffer()
    number = 0

    while True:
        #Opening serial port from rasp causes arduino connected by USB cable to reset
        # --- Handshake Phase ---
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

        while True:
            try:
                # User enters -5 (lean back/move back) to 5 (lean forward/move forward)
                cmd = int(input("Enter tilt command (-5 to 5 degrees): "))
                cmd = max(-5, min(5, cmd)) # Safety clamp
                
                # Convert to a signed byte and send
                ser.write(cmd.to_bytes(1, byteorder='big', signed=True))
                print(f"Sent: {cmd} degrees")
            except ValueError:
                print("Please enter a valid number.")

            start_checking_tick_time = (int)(time.time())
            acknowledged = False
            while True:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode("utf-8").strip()
                        print(line)
                        if line == "arduino_ack_tick":
                            acknowledged = True
                            break
                    except:
                        continue

                if (int)(time.time()) - start_checking_tick_time > 1:
                        print("No acknowledged tick from arduino, try handshaking again.")
                        acknowledged = False
                        break
            
            if acknowledged == False:
                break
                
                