/*
TODO:
Create a client from laptop and send command from there
Connection is not stable: Create a heartbeat mechanism
*/


#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <poll.h>
#include <chrono>
#include <unordered_map>
#include "include/json.hpp"
using json = nlohmann::json;

#define STATIC_SERVER_ETHERNET_IP_ADDRESS  "192.168.99.200"
#define STATIC_SERVER_LAN0_IP_ADDRESS "192.168.50.1"
#define STATIC_ANY_IP_ADDRESS "0.0.0.0"
#define PORT 1369
#define BUFFER_SIZE 1024
#define CURRENT_TIME std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()%1000000000
#define MAX_CLIENT_IN_QUEUE 3
#define MAX_THEAD_TO_HANDLE_CLIENT 1
#define TIMEOUT_SECONDS 3

std::queue<int> g_clientHandlerQueue{};
std::mutex g_m_g_clientHandlerQueue{};
std::condition_variable g_cv_g_clientHandlerQueue{};

bool IsJsonMsgParsed(char* buffer, std::unordered_map<std::string, std::string>& jsonMsg)
{
    // Parse the JSON message
    try {
        json receivedJson = json::parse(buffer);
        if (receivedJson.contains("from"))
        {
            std::string from = receivedJson["from"];
            std::cout << "from: " << from << "\n";
            jsonMsg["from"] = from;
        }
        else
        {
            throw("Missing [from]");
        }
        if (receivedJson.contains("to")) {
            std::string to = receivedJson["to"];
            std::cout << "to: " << to << "\n";
            jsonMsg["to"] = to;
        }
        else
        {
            throw("Missing [to]");
        }
        // Process the parsed JSON
        if (receivedJson.contains("cmd")) {
            std::string command = receivedJson["cmd"];
            std::cout << "Received command: " << command << "\n";
            jsonMsg["cmd"] = command;
        }
        else
        {
            throw("Missing [cmd]");
        }

    } catch (const std::exception& e) {
        std::cout << "Error parsing JSON: " << e.what() << "\n";
        return false;
    }
    return true;
}

bool IsMsgSentToController(int& clientHandler)
{
    char buffer[BUFFER_SIZE] = {0};
    fd_set readfds;
    struct timeval timeout;
    timeout.tv_sec = TIMEOUT_SECONDS;
    timeout.tv_usec = 0;

    FD_ZERO(&readfds);
    FD_SET(clientHandler, &readfds);

    int activity = select(clientHandler + 1, &readfds, NULL, NULL, &timeout);

    if (activity < 0) {
        std::cout << "Error in select function\n";
    } else if (activity == 0) {
        std::cout << CURRENT_TIME << ": " << "Timeout reached, no data received from client within " << TIMEOUT_SECONDS << " seconds\n";
    } else {
        if (FD_ISSET(clientHandler, &readfds)) {
            // Read data (non-blocking)
            int readStatus = read(clientHandler, buffer, BUFFER_SIZE);
            if (readStatus == -1) {
                std::cout << "Error: Cannot read data from client\n";
            } else if (readStatus == 0) {
                std::cout << "Client closed the connection\n";
            } else {
                // Check if the last character is '\n', indicating end of message
                if (buffer[readStatus - 1] == '\n') {
                    buffer[readStatus - 1] = '\0';
                    std::cout << "Received JSON message: " << buffer << "\n";
                    std::unordered_map<std::string, std::string> jsonMsg{};
                    if(IsJsonMsgParsed(buffer, jsonMsg) == true)
                    {
                        if(jsonMsg["cmd"] == "ACK") return true;
                    }
                } else {
                    std::cout << "Message does not end with newline or is incomplete.\n";
                }
            }
        }
    }
    return false;
}

std::string CreateMsgJsonToSend(std::string msg, std::string from, std::string to)
{
    // Create JSON message
    json j;
    j["cmd"] = msg;
    j["from"] = from;
    j["to"] = to;
    // Serialize the JSON to string
    std::string json_message = j.dump();
    json_message += '\n';
    return json_message;
}

bool IsConnectionFromController(char* buffer)
{
    std::unordered_map<std::string, std::string> jsonMsg{};
    if(IsJsonMsgParsed(buffer, jsonMsg) == false) return false;
    if(jsonMsg["from"] == "ESP" && jsonMsg["cmd"] == "HANDSHAKE") return true;
    return false;
}

bool IsControllerHandledSuccessfully(int& clientHandler, std::thread::id thread_id)
{
    std::cout << "IsControllerHandledSuccessfully\n";
    struct sockaddr_in clientAddress;
    socklen_t clientAddress_len = sizeof(clientAddress);
    getpeername(clientHandler, reinterpret_cast<struct sockaddr*>(&clientAddress), &clientAddress_len);
    char clientIpAdress[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &clientAddress.sin_addr, clientIpAdress, INET_ADDRSTRLEN) == NULL) 
    {
        std::cout << CURRENT_TIME << ": " << "Can not convert ip address of client\n";
        return false;
    }
    std::cout << CURRENT_TIME << ": " << "Thread - {" << thread_id << "} controlling client - " << std::string(clientIpAdress)<< "/" << clientAddress.sin_port << '\n';

    char buffer[BUFFER_SIZE] = {0};
    std::cout << CURRENT_TIME << ": " << "Thread - {" << thread_id << "}" << " waiting for command...\n";
    std::string in = "ON";
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if(in == "ON")
        {
            in = "OFF";
        }
        else
        {
            in = "ON";
        }
        std::string msg = CreateMsgJsonToSend(in, "SERVER", "ESP");
        int blockingMode = 0;
        if(send(clientHandler, msg.c_str(), msg.size(), blockingMode) == -1)
        {
            std::cout << CURRENT_TIME << ": " << "Can not send msg back to client!";
        }
        std::cout << CURRENT_TIME << ": " << "Thread - {" << thread_id << "}" << " sent command {" << msg << "} to controller\n";
        if(IsMsgSentToController(clientHandler) == true) continue;
        else
        {
            std::cout << CURRENT_TIME << ": " << "No ACK from CONTROLLER" << '\n';
            return false;
        }
    }
    return true;
}

void HandleClient(int& clientHandler, std::thread::id thread_id)
{
    struct sockaddr_in clientAddress;
    socklen_t clientAddress_len = sizeof(clientAddress);
    getpeername(clientHandler, reinterpret_cast<struct sockaddr*>(&clientAddress), &clientAddress_len);
    char clientIpAdress[INET_ADDRSTRLEN];
    if (inet_ntop(AF_INET, &clientAddress.sin_addr, clientIpAdress, INET_ADDRSTRLEN) == NULL) 
    {
        std::cout << CURRENT_TIME << ": " << "Can not convert ip address of client\n";
        return;
    }
    std::cout << CURRENT_TIME << ": " << "Thread - {" << thread_id << "} handle connection from client " << std::string(clientIpAdress)<< "/" << clientAddress.sin_port << '\n';

    char buffer[BUFFER_SIZE] = {0};
    std::cout << CURRENT_TIME << ": " << "Thread - {" << thread_id << "}" << " waiting for msg...\n";

    bool isConnectionClosed = false;
    int readStatus = read(clientHandler, buffer, BUFFER_SIZE);
    if(readStatus == -1)
    {
        std::cout << CURRENT_TIME << ": " << "Can not read client!" << '\n';
        isConnectionClosed = true;
    }
    else if(readStatus == 0)
    {
        std::cout << CURRENT_TIME << ": " << "Client close connection!!!\n";
        isConnectionClosed = true;
    }
    else
    {
        std::cout << CURRENT_TIME << ": " << "From Client " << std::string(clientIpAdress)<< "/" << clientAddress.sin_port << ": " << buffer << '\n';
        std::string msg = "ACK";
        int blockingMode = 0;
        if(send(clientHandler, msg.c_str(), msg.size(), blockingMode) == -1)
        {
            std::cout << CURRENT_TIME << ": " << "Can not send msg back to client!";
            isConnectionClosed = true;
        }
        std::cout << CURRENT_TIME << ": " << "Thread - {" << thread_id << "}" << " sent ACK\n";
        if(IsConnectionFromController(buffer) == true)
        {
            isConnectionClosed = ( 
                (IsControllerHandledSuccessfully(clientHandler, thread_id) == true) ? false:true
            );
        }
    }
    if(isConnectionClosed == true)
    {
        close(clientHandler);
        std::cout << CURRENT_TIME << ": " << "Thread - {" << thread_id << "}" << " CLOSE connection from client " << std::string(clientIpAdress)<< "/" << clientAddress.sin_port << '\n';
    }
    else
    {
        {
            std::unique_lock<std::mutex> ul_clientHandlerQueue{g_m_g_clientHandlerQueue};
            g_clientHandlerQueue.push(clientHandler);
        }
        g_cv_g_clientHandlerQueue.notify_one();
    }
}

void ThreadForHandlingClient()
{
    std::cout << CURRENT_TIME << ": " << "Start thread: " << std::this_thread::get_id() << '\n';
    while(true)
    {
        std::cout << CURRENT_TIME << ": " << "Thread: " << std::this_thread::get_id() << " waiting for connection of queue ..." << '\n';
        std::unique_lock<std::mutex> ul_clientHandlerQueue{g_m_g_clientHandlerQueue};
        g_cv_g_clientHandlerQueue.wait(ul_clientHandlerQueue, []()->bool{return g_clientHandlerQueue.size() != 0;});
        int newClientHandler = g_clientHandlerQueue.front();
        g_clientHandlerQueue.pop();
        ul_clientHandlerQueue.unlock();
        std::cout << CURRENT_TIME << ": " << "Start handle client " << newClientHandler << " in thread - " << std::this_thread::get_id() << '\n';
        HandleClient(newClientHandler, std::this_thread::get_id());
        std::cout << CURRENT_TIME << ": " << "Done client " << newClientHandler << '\n';
    }
}

int main()
{

    std::vector<std::thread> clientHandlerThreadPool{};
    for(int i=0; i<MAX_THEAD_TO_HANDLE_CLIENT; ++i)
    {
        clientHandlerThreadPool.push_back(std::thread(ThreadForHandlingClient));
    }
    std::string serverIpAddress = STATIC_ANY_IP_ADDRESS;
    struct sockaddr_in socketServerAddress;
    socketServerAddress.sin_family = AF_INET;
    if(inet_pton(AF_INET, serverIpAddress.c_str(), &socketServerAddress.sin_addr) <= 0)
    {
        std::cerr << "Can not set ip address to socketServerAddress!";
        exit(EXIT_FAILURE);
    }
    socketServerAddress.sin_port = htons(PORT);
    /*If PROTOCOL is zero, one is chosen automatically*/
    int automaticProtocol = 0;
    int serverSocket = socket(AF_INET, SOCK_STREAM, automaticProtocol);
    if(serverSocket == -1)
    {
        std::cerr << "Can not create serverSocket!";
        exit(EXIT_FAILURE);
    }

    struct linger linger_opt = {1, 0};  // Linger active, timeout 0
    /*port will close immediately when program exits*/
    setsockopt(serverSocket, SOL_SOCKET, SO_LINGER, &linger_opt, sizeof(linger_opt));

    if(bind(serverSocket, 
        reinterpret_cast<struct sockaddr*>(&socketServerAddress), 
        sizeof(socketServerAddress)) != 0)
    {
        std::cerr << "Can not bind socketServerAddress to serverSocket!" << '\n';
        exit(EXIT_FAILURE);
    }

    if(listen(serverSocket, 5) != 0)
    {
        std::cerr << "Can not listen on serverSocket!" << '\n';
        exit(EXIT_FAILURE);
    }
    std::cout << CURRENT_TIME << ": " << "Server listening on 192.168.99.200/" << PORT << '\n';

    while(true)
    {
        socklen_t socketServerAddress_len = sizeof(socketServerAddress); /*get client information*/
        int clientHandler = accept(serverSocket, 
            reinterpret_cast<struct sockaddr*>(&socketServerAddress),
            &socketServerAddress_len);
        if(clientHandler == -1)
        {
            std::cerr << "Can not accept connection from client!" << '\n';
            exit(EXIT_FAILURE);
        }
        {
            std::lock_guard<std::mutex> lg_clientHandlerQueue{g_m_g_clientHandlerQueue};
            if(g_clientHandlerQueue.size() < MAX_CLIENT_IN_QUEUE)
            {
                g_clientHandlerQueue.push(clientHandler);
                std::cout << CURRENT_TIME << ": " << "Push connection " << clientHandler << " to queue\n";
            }
            else
            {
                close(clientHandler);
                std::cout << CURRENT_TIME << ": " << "Reject connection " << clientHandler << '\n';
            }
        }
        g_cv_g_clientHandlerQueue.notify_one();
    }

    for(int i=0; i<MAX_THEAD_TO_HANDLE_CLIENT; ++i)
    {
        if(clientHandlerThreadPool[i].joinable())
        {
            clientHandlerThreadPool[i].join();
            std::cout << CURRENT_TIME << ": " << "Join thread - " << clientHandlerThreadPool[i].get_id() << '\n';
        }
    }
    
    close(serverSocket);   
}