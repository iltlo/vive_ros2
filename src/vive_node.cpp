#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include "json.hpp" // Include nlohmann/json
#include "VRUtils.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using json = nlohmann::json;

class Client : public rclcpp::Node {
private:
    int sock;
    struct sockaddr_in serv_addr;
    std::string address;
    int port;
    VRControllerData jsonData; // Use the struct for JSON data
    VRControllerData initialPose;
    bool gripButtonPressed = false;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr abs_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr rel_transform_publisher_;

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
        RCLCPP_INFO(this->get_logger(), "Reconnected to server.");
    }

    void publishTransform(const VRControllerData& pose, bool isRelative = false) {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        // transformStamped.child_frame_id = "vive_pose";
        transformStamped.child_frame_id = (isRelative) ? "vive_pose_rel" : "vive_pose_abs";

        transformStamped.transform.translation.x = -pose.pose_z;
        transformStamped.transform.translation.y = -pose.pose_x;
        transformStamped.transform.translation.z = pose.pose_y;
        transformStamped.transform.rotation.x = -pose.pose_qz;
        transformStamped.transform.rotation.y = -pose.pose_qx;
        transformStamped.transform.rotation.z = pose.pose_qy;
        transformStamped.transform.rotation.w = pose.pose_qw;

        // Publish to TF
        tf_broadcaster_->sendTransform(transformStamped);
        // Publish to the /vive_pose_abs or /vive_pose_rel topic
        if (isRelative) {
            rel_transform_publisher_->publish(transformStamped);
        } else {
            abs_transform_publisher_->publish(transformStamped);
        }
    }

    VRControllerData calculateRelativePose(const VRControllerData& initial, const VRControllerData& current) {
        VRControllerData relativePose;
        relativePose.pose_x = current.pose_x - initial.pose_x;
        relativePose.pose_y = current.pose_y - initial.pose_y;
        relativePose.pose_z = current.pose_z - initial.pose_z;
        // TODO: Calculate relative quaternion. Calculation below is just for demonstration purposes.
        relativePose.pose_qx = current.pose_qx - initial.pose_qx;
        relativePose.pose_qy = current.pose_qy - initial.pose_qy;
        relativePose.pose_qz = current.pose_qz - initial.pose_qz;
        relativePose.pose_qw = current.pose_qw - initial.pose_qw;
        return relativePose;
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
        abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_abs", 150);
        rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_rel", 150);
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
        RCLCPP_INFO(this->get_logger(), "Connected to server.");

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
                    jsonData.menu_button = j["buttons"]["menu"];
                    jsonData.trigger_button = j["buttons"]["trigger"];
                    jsonData.trackpad_touch = j["buttons"]["trackpad_touch"];
                    jsonData.trackpad_button = j["buttons"]["trackpad_button"];
                    jsonData.grip_button = j["buttons"]["grip"];
                    jsonData.trackpad_x = j["trackpad"]["x"];
                    jsonData.trackpad_y = j["trackpad"]["y"];
                    jsonData.trigger = j["trigger"];
                    jsonData.role = j["role"];
                    jsonData.time = j["time"];
                    // Example of using stored data
                    RCLCPP_INFO(this->get_logger(), "Time: %s", jsonData.time.c_str());
                    RCLCPP_INFO(this->get_logger(), "Pose x: %f", jsonData.pose_x);
                    RCLCPP_INFO(this->get_logger(), "Pose y: %f", jsonData.pose_y);
                    RCLCPP_INFO(this->get_logger(), "Pose z: %f", jsonData.pose_z);

                    RCLCPP_INFO(this->get_logger(), "Menu button: %s", jsonData.menu_button ? "true" : "false");
                    RCLCPP_INFO(this->get_logger(), "Trigger button: %s", jsonData.trigger_button ? "true" : "false");
                    RCLCPP_INFO(this->get_logger(), "Trackpad touch: %s", jsonData.trackpad_touch ? "true" : "false");
                    RCLCPP_INFO(this->get_logger(), "Trackpad button: %s", jsonData.trackpad_button ? "true" : "false");
                    RCLCPP_INFO(this->get_logger(), "Grip button: %s", jsonData.grip_button ? "true" : "false");
                    
                    RCLCPP_INFO(this->get_logger(), "Trackpad x: %f", jsonData.trackpad_x);
                    RCLCPP_INFO(this->get_logger(), "Trackpad y: %f", jsonData.trackpad_y);
                    RCLCPP_INFO(this->get_logger(), "Trigger: %f", jsonData.trigger);
                    RCLCPP_INFO(this->get_logger(), "Role: %d", jsonData.role);
                    printf("\n");

                    // Handle grip button state
                    if (jsonData.grip_button && !gripButtonPressed) {
                        // Grip button just pressed
                        initialPose = jsonData;
                        gripButtonPressed = true;
                    } else if (!jsonData.grip_button && gripButtonPressed) {
                        // Grip button just released
                        gripButtonPressed = false;
                    }

                    if (gripButtonPressed) {
                        // Calculate and publish relative transform
                        VRControllerData relativePose = calculateRelativePose(initialPose, jsonData);
                        publishTransform(relativePose, true);   // isRelative = true
                        publishTransform(jsonData);             // Publish absolute transform as well
                    } else {
                        // Publish the absolute transform
                        publishTransform(jsonData);
                    }

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
