import serial
import time

if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    ser.reset_input_buffer()
    number = 0

    while True:
        """Opening serial port from rasp causes arduino connected by USB cable to reset"""
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
            left_motor_speed = int(input("Enter left motor speed: "))
            right_motor_speed = int(input("Enter right motor speed: "))
            direction = int(input("Enter direction: "))
            if direction != 0 and direction != 1:
                direction = 0
            ser.write(bytearray([left_motor_speed, right_motor_speed, direction]))
            time.sleep(1)
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
                
                