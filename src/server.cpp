#include "server.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <unistd.h> // For close()

Server::Server(int port, std::mutex &mutex, std::condition_variable &cv, VRControllerData &data) 
    : data_mutex(mutex), data_cv(cv), shared_data(data) {
    signal(SIGPIPE, SIG_IGN); // Ignore SIGPIPE signals

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
}
Server::~Server() {
    close(server_fd);
}

void Server::start() {
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server listening on port " << ntohs(address.sin_port) << std::endl;

    int addrlen = sizeof(address);
    while (true) {
        std::cout << "Waiting for new connection..." << std::endl;
        int new_socket;
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0) {
            perror("accept");
            continue; // Continue to accept next connection
        }

        std::cout << "Connection established." << std::endl;
        while (true) {
            json data = prepareData();
            std::string message = data.dump();
            if (send(new_socket, message.c_str(), message.length(), 0) == -1) {
                perror("send");
                close(new_socket);
                break; // Exit the inner loop to wait for a new connection
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    }
}

json Server::prepareData() {
    std::unique_lock<std::mutex> lock(data_mutex);
    data_cv.wait(lock); // Wait for new data

    json j;
    j["pose"] = {{"x", shared_data.pose_x}, {"y", shared_data.pose_y}, {"z", shared_data.pose_z}, {"qx", shared_data.pose_qx}, {"qy", shared_data.pose_qy}, {"qz", shared_data.pose_qz}, {"qw", shared_data.pose_qw}};
    j["buttons"] = {{"menu", shared_data.menu_button}, {"trigger", shared_data.trigger_button}, {"trackpad_touch", shared_data.trackpad_touch}, {"trackpad_button", shared_data.trackpad_button}, {"grip", shared_data.grip_button}};
    j["trackpad"] = {{"x", shared_data.trackpad_x}, {"y", shared_data.trackpad_y}};
    j["trigger"] = shared_data.trigger;
    j["role"] = shared_data.role;
    j["time"] = shared_data.time;
    return j;
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