#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include "json.hpp" // Include nlohmann/json
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using json = nlohmann::json;

struct JsonData {
    double pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw;
    bool button1, button2;
    double trackpad_x, trackpad_y;
    double trigger;
    std::string time;
};

class Client : public rclcpp::Node {
private:
    int sock;
    struct sockaddr_in serv_addr;
    std::string address;
    int port;
    JsonData jsonData; // Use the struct for JSON data
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_publisher_;

    void connectToServer() {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation error");
            exit(EXIT_FAILURE);
        }

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection Failed");
            close(sock);
            sock = -1;
        }
    }

    void reconnect() {
        close(sock);
        sock = -1;
        while (sock < 0) {
            std::this_thread::sleep_for(std::chrono::seconds(1)); // Wait before attempting to reconnect
            connectToServer();
        }
    }

    void publishTransform() {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "vive_pose";
        transformStamped.transform.translation.x = -jsonData.pose_z;
        transformStamped.transform.translation.y = -jsonData.pose_x;
        transformStamped.transform.translation.z = jsonData.pose_y;
        transformStamped.transform.rotation.x = -jsonData.pose_qz;
        transformStamped.transform.rotation.y = -jsonData.pose_qx;
        transformStamped.transform.rotation.z = jsonData.pose_qy;
        transformStamped.transform.rotation.w = jsonData.pose_qw;

        // Publish to TF
        tf_broadcaster_->sendTransform(transformStamped);
        // Publish to the /vive_pose topic
        transform_publisher_->publish(transformStamped);

    }

public:
    Client(std::string addr, int p) : Node("client_node"), sock(-1), address(addr), port(p) {
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        if(inet_pton(AF_INET, address.c_str(), &serv_addr.sin_addr)<=0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported");
            exit(EXIT_FAILURE);
        }
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose", 10);
    }

    ~Client() {
        if (sock != -1) {
            close(sock);
        }
    }

    void start() {
        while (sock < 0) {
            RCLCPP_INFO(this->get_logger(), "Attempting to connect to server...");
            connectToServer();
        }

        char buffer[1024] = {0};
        while (rclcpp::ok()) {
            memset(buffer, 0, sizeof(buffer)); // Clear buffer
            int bytesReceived = read(sock, buffer, 1024);
            if (bytesReceived > 0) {
                std::string receivedData(buffer, bytesReceived);
                try {
                    json j = json::parse(receivedData);
                    // Store JSON data to the struct
                    jsonData.pose_x = j["pose"]["x"];
                    jsonData.pose_y = j["pose"]["y"];
                    jsonData.pose_z = j["pose"]["z"];
                    jsonData.pose_qx = j["pose"]["qx"];
                    jsonData.pose_qy = j["pose"]["qy"];
                    jsonData.pose_qz = j["pose"]["qz"];
                    jsonData.pose_qw = j["pose"]["qw"];
                    jsonData.button1 = j["buttons"]["button1"];
                    jsonData.button2 = j["buttons"]["button2"];
                    jsonData.trackpad_x = j["trackpad"]["x"];
                    jsonData.trackpad_y = j["trackpad"]["y"];
                    jsonData.trigger = j["trigger"];
                    jsonData.time = j["time"];
                    // Example of using stored data
                    RCLCPP_INFO(this->get_logger(), "Time: %s", jsonData.time.c_str());
                    RCLCPP_INFO(this->get_logger(), "Pose x: %f", jsonData.pose_x);
                    RCLCPP_INFO(this->get_logger(), "Pose y: %f", jsonData.pose_y);
                    RCLCPP_INFO(this->get_logger(), "Pose z: %f", jsonData.pose_z);

                    // Publish the transform
                    publishTransform();
                } catch (json::parse_error& e) {
                    RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
                    continue;
                }
            } else if (bytesReceived == 0) {
                RCLCPP_WARN(this->get_logger(), "Connection closed by server. Attempting to reconnect...");
                reconnect();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Read error.");
                break;
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto client = std::make_shared<Client>("127.0.0.1", 2048);
    client->start();
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
