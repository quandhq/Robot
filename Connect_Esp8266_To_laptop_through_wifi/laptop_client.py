import socket
import time

SERVER_IP = "192.168.99.200"
SERVER_PORT = 1369
BUFFER_SIZE = 1024

# Create a TCP socket

while True:
    try:
        time.sleep(2)
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to server
        client_socket.connect((SERVER_IP, SERVER_PORT))
        print("Connected to server!")
        while True:
            time.sleep(1)
            # Send message to server
            name = "client1" # input("Enter your name: ")
            message = "Hello, " + name + "!"
            print(message)
            client_socket.sendall(message.encode())
            print("Message sent to server")
            client_socket.settimeout(5)
            try:
                # Receive response from server
                response = client_socket.recv(1024).decode()
                print("Server responded:", response)
            except socket.timeout:
                print("No response from server. Closing connection.")
                client_socket.close()
                print("Backing 3 seconds to reconnect server!!!")
                time.sleep(3)
                print("Reconnecting...")
                break

    except Exception as e:
        print("Error:", e)

# Close socket
client_socket.close()