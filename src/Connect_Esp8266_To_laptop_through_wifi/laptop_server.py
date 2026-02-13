'''
netsh advfirewall set allprofiles state on
=> turn on or off firewall
'''
'''
const char* ssid = "RP_Hotspot";     // Replace with your Wi-Fi name
const char* password = "test1234"; // Replace with your Wi-Fi password
'''


import socket
import signal
import sys
import time



# Server settings
serverIP_wlan0 = "192.168.50.1" # Ip address of wifi hotspot on Raspberry Pi 4
HOST = serverIP_wlan0  # Listen on all available interfaces
PORT = 1369       # Port number

# Create a socket (IPv4, TCP)
serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serverSocket.bind((HOST, PORT))  # Bind to address & port
serverSocket.listen(1)  # Listen for incoming connections

print(f"Listening on {HOST}:{PORT}...")



# Handle Ctrl+C gracefully
def CleanupAndExit(sig, frame, clientSocket):
    print("\nShutting down server...")
    if clientSocket != None:
        clientSocket.close()
    serverSocket.close()
    sys.exit(0)
clientSocket = None
try:  
    while True:
        print("Waiting for connection!")
        # Accept connection from ESP8266
        clientSocket, addr = serverSocket.accept()
        print(f"Connected by {addr}")
        # **Set a timeout for receiving data**
        clientSocket.settimeout(5.0)  # Prevents blocking indefinitely
        while True:
            try:
                # Receive data from ESP8266
                data = clientSocket.recv(1024).decode()
                if data == "Disconnect from ESP!":
                    print("Client disconnected in 3s")
                    response = "ACK"
                    clientSocket.send(response.encode())
                    time.sleep(3)
                    clientSocket.close()
                    print("Client disconnected from ESP's request!")
                    break  # Stop
                print(f"ESP8266: {data}")

                # Send a response to ESP8266
                response = "Hello from Raspberry Pi!"
                clientSocket.send(response.encode())
            except socket.timeout:
                print("Timeout: Close client and reconnecting...!")
                clientSocket.close()
                print("Client disconnected from timeout!")
                break
except KeyboardInterrupt:
    CleanupAndExit(None, None, clientSocket)



# Cleanup
CleanupAndExit(None, None, clientSocket)