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
        data_to_send = "Hello from rasp " + str(number) + "!\n"
        number += 1
        ser.write(data_to_send.encode("utf-8"))
        line = ser.readline().decode("utf-8").rstrip()
        print(line)
        time.sleep(1)