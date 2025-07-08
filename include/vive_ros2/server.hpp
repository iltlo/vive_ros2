#ifndef SERVER_HPP
#define SERVER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>
#include <csignal>

#include "json.hpp"
#include "VRUtils.hpp"

using json = nlohmann::json;

class Server {
private:
    int server_fd;
    struct sockaddr_in address;
    int opt = 1;

    std::mutex &data_mutex;
    std::condition_variable &data_cv;
    VRControllerData &shared_data;

    json prepareData();

    static void signalHandler(int signum) {
        std::cout << "Interrupt signal (" << signum << ") received.\n";
        exit(signum);   // terminate program
    }
// protected:
//     virtual std::string prepareData() = 0; // Pure virtual function
public:
    Server(int port, std::mutex &mutex, std::condition_variable &cv, VRControllerData &data);
    ~Server();

    void start();
    static std::string getCurrentTimeWithMilliseconds();
    static void setupSignalHandlers() {
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
    }
};

#endif // SERVER_HPP
