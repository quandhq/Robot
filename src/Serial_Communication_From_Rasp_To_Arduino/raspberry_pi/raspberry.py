import serial
import time

if __name__ == "__main__":
    ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)
    ser.reset_input_buffer()
    number = 0

    """Opening serial port from rasp causes arduino connected by USB cable to reset"""
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8").rstrip()
            if line == "ready!":
                break

    while True:
        left_motor_speed = int(input("Enter left motor speed: "))
        right_motor_speed = int(input("Enter right motor speed: "))
        direction = int(input("Enter direction: "))
        if direction != 0 and direction != 1:
            direction = 0
        ser.write(bytearray([left_motor_speed, right_motor_speed, direction]))
        time.sleep(1)