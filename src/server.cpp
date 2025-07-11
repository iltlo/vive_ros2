#include "server.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <unistd.h> // For close()

// Server Configuration Constants
namespace ServerConfig {
    constexpr int DEFAULT_LISTEN_BACKLOG = 3;
    constexpr int SOCKET_REUSE_ENABLED = 1;
    constexpr int MESSAGE_SEND_DELAY_MS = 2;  // 500Hz transmission rate
}

// VR Data TCP Server - handles client connections and data transmission
Server::Server(int port, std::mutex &mutex, std::condition_variable &cv, VRControllerData &data) 
    : data_mutex(mutex), data_cv(cv), shared_data(data) {
    signal(SIGPIPE, SIG_IGN); // Ignore SIGPIPE signals

    if (!createSocket()) {
        exit(EXIT_FAILURE);
    }

    if (!configureSocket()) {
        exit(EXIT_FAILURE);
    }

    if (!bindSocket(port)) {
        exit(EXIT_FAILURE);
    }
}
Server::~Server() {
    if (server_fd != -1) {
        shutdown(server_fd, SHUT_RDWR);
        close(server_fd);
        server_fd = -1;
    }
}

bool Server::createSocket() {
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        return false;
    }
    return true;
}

bool Server::configureSocket() {
    int reuse = ServerConfig::SOCKET_REUSE_ENABLED;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &reuse, sizeof(opt))) {
        perror("setsockopt");
        return false;
    }
    return true;
}

bool Server::bindSocket(int port) {
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("bind failed");
        return false;
    }
    return true;
}

void Server::handleClientConnection(int clientSocket) {
    std::cout << "Connection established." << std::endl;
    while (true) {
        json data = prepareData();
        std::string message = data.dump();
        if (send(clientSocket, message.c_str(), message.length(), 0) == -1) {
            perror("send");
            close(clientSocket);
            break; // Exit the loop to wait for a new connection
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(ServerConfig::MESSAGE_SEND_DELAY_MS));  // 500Hz
    }
}

void Server::start() {
    if (listen(server_fd, ServerConfig::DEFAULT_LISTEN_BACKLOG) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << ntohs(address.sin_port) << std::endl;

    int addrlen = sizeof(address);
    while (true) {
        std::cout << "Waiting for new connection..." << std::endl;
        int new_socket;
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            perror("accept");
            continue; // Continue to accept next connection
        }

        handleClientConnection(new_socket);
    }
}

json Server::prepareData() {
    std::unique_lock<std::mutex> lock(data_mutex);
    data_cv.wait(lock); // Wait for new data

    json vrControllerMessage;
    vrControllerMessage["pose"] = {
        {"x", shared_data.pose_x}, 
        {"y", shared_data.pose_y}, 
        {"z", shared_data.pose_z}, 
        {"qx", shared_data.pose_qx}, 
        {"qy", shared_data.pose_qy}, 
        {"qz", shared_data.pose_qz}, 
        {"qw", shared_data.pose_qw}
    };
    vrControllerMessage["buttons"] = {
        {"menu", shared_data.menu_button}, 
        {"trigger", shared_data.trigger_button}, 
        {"trackpad_touch", shared_data.trackpad_touch}, 
        {"trackpad_button", shared_data.trackpad_button}, 
        {"grip", shared_data.grip_button}
    };
    vrControllerMessage["trackpad"] = {
        {"x", shared_data.trackpad_x}, 
        {"y", shared_data.trackpad_y}
    };
    vrControllerMessage["trigger"] = shared_data.trigger;
    vrControllerMessage["role"] = shared_data.role;
    vrControllerMessage["time"] = shared_data.time;
    return vrControllerMessage;
}

std::string Server::getCurrentTimeWithMilliseconds() {
    auto now = std::chrono::system_clock::now();
    auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
    auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(std::localtime(&nowAsTimeT), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << nowMs.count();
    return ss.str();
}