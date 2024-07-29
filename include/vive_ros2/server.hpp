#ifndef SERVER_HPP
#define SERVER_HPP

#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <signal.h>

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
    JsonData &shared_data;

    json prepareData();

public:
    Server(int port, std::mutex &mutex, std::condition_variable &cv, JsonData &data);
    ~Server();

    void start();
    static std::string getCurrentTimeWithMilliseconds();
};

#endif // SERVER_HPP
