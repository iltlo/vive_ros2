#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "json.hpp" // Include nlohmann/json
#include "VRUtils.hpp"
#include "EigenTransforms.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "vive_ros2/msg/vr_controller_data.hpp"

using json = nlohmann::json;

class Client : public rclcpp::Node {
private:
    int sock;
    struct sockaddr_in serv_addr;
    std::string address;
    int port;
    VRControllerData jsonData; // Use the struct for JSON data
    VRControllerData initialPoseLeft;
    VRControllerData initialPoseRight;
    bool triggerButtonPressedLeft = false;
    bool triggerButtonPressedRight = false;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr abs_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr rel_transform_publisher_;
    rclcpp::Publisher<vive_ros2::msg::VRControllerData>::SharedPtr controller_data_publisher_;
    
    // Role-specific publishers
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr left_abs_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr left_rel_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr right_abs_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr right_rel_transform_publisher_;

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
        // transformStamped.header.frame_id = (isRelative) ? "wx250s/ee_gripper_link" : "world";
        
        // Determine prefix based on controller role (0 = right, 1 = left)
        std::string prefix = (pose.role == 1) ? "left_vr" : "right_vr";
        std::string topic_name = ((isRelative) ? "vive_pose_rel" : "vive_pose_abs");
        
        // Set child_frame_id with the appropriate prefix
        transformStamped.child_frame_id = prefix + "/" + topic_name;

        transformStamped.transform.translation.x = -pose.pose_z;
        transformStamped.transform.translation.y = -pose.pose_x;
        transformStamped.transform.translation.z = pose.pose_y;
        transformStamped.transform.rotation.x = -pose.pose_qz;
        transformStamped.transform.rotation.y = -pose.pose_qx;
        transformStamped.transform.rotation.z = pose.pose_qy;
        transformStamped.transform.rotation.w = pose.pose_qw;

        // Publish to TF with role-based frame naming
        tf_broadcaster_->sendTransform(transformStamped);
        
        // Create a new message to publish to the role-specific topic
        auto topicMsg = transformStamped;
        
        // Publish to the appropriate role-specific topic
        // log out the pose.role
        RCLCPP_INFO(this->get_logger(), "Publishing transform for role: %d", pose.role);
        if (pose.role == 1) { // Left controller
            if (isRelative) {
                left_rel_transform_publisher_->publish(topicMsg);
            } else {
                left_abs_transform_publisher_->publish(topicMsg);
            }
        } else { // Right controller (role == 0)
            if (isRelative) {
                right_rel_transform_publisher_->publish(topicMsg);
            } else {
                right_abs_transform_publisher_->publish(topicMsg);
            }
        }
        
        // Also publish to generic topics for backward compatibility
        if (isRelative) {
            rel_transform_publisher_->publish(topicMsg);
        } else {
            abs_transform_publisher_->publish(topicMsg);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Published transform to %s with child_frame_id %s", 
                  (isRelative ? "vive_pose_rel" : "vive_pose_abs"), 
                  transformStamped.child_frame_id.c_str());
    }

    VRControllerData calculateRelativePose(const VRControllerData& initial, const VRControllerData& current) {
        // Use the new Eigen-based utility function
        return VRUtils::calculateRelativePose(initial, current);
    }

    VRControllerData filterPose(const VRControllerData& pose) {
        // Simple low-pass filter with separate state for each controller
        static VRControllerData prevPoseLeft;
        static VRControllerData prevPoseRight;
        static bool initialized[2] = {false, false};
        double alpha = 0.9999; // Smoothing factor
        
        // Get appropriate previous pose based on controller role (0=right, 1=left)
        VRControllerData& prevPose = (pose.role == 1) ? prevPoseLeft : prevPoseRight;
        
        // Initialize if this is the first data for this controller
        if (!initialized[pose.role]) {
            prevPose = pose;
            initialized[pose.role] = true;
            return pose; // Return unfiltered for first sample
        }

        // Use the new Eigen-based filtering utility
        VRControllerData filteredPose = VRUtils::filterPose(pose, prevPose, alpha);
        
        // Store this pose for next iteration
        prevPose = filteredPose;
        
        return filteredPose;
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
        
        // Create publishers with role-specific topic names
        abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_abs", 150);
        rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_rel", 150);
        controller_data_publisher_ = this->create_publisher<vive_ros2::msg::VRControllerData>("controller_data", 10);
        
        // Create role-specific publishers for better topic organization
        left_abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("left_vr/vive_pose_abs", 150);
        left_rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("left_vr/vive_pose_rel", 150);
        right_abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("right_vr/vive_pose_abs", 150);
        right_rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("right_vr/vive_pose_rel", 150);
        
        RCLCPP_INFO(this->get_logger(), "Publishers created with role-based topic names.");
    }

    ~Client() {
        if (sock != -1) {
            close(sock);
        }
    }

    void publishControllerData(const VRControllerData &data, const VRControllerData &relData) {
        vive_ros2::msg::VRControllerData msg;
        msg.grip_button = data.grip_button;
        msg.trigger_button = data.trigger_button;
        msg.trackpad_button = data.trackpad_button;
        msg.trackpad_touch = data.trackpad_touch;
        msg.menu_button = data.menu_button;
        msg.trackpad_x = data.trackpad_x;
        msg.trackpad_y = data.trackpad_y;
        msg.trigger = data.trigger;
        msg.role = data.role;
        msg.time = data.time;

        // Determine prefix based on controller role (0 = right, 1 = left)
        std::string prefix = (data.role == 1) ? "left_vr" : "right_vr";

        // Absolute pose data
        msg.abs_pose.header.stamp = this->get_clock()->now();
        msg.abs_pose.header.frame_id = "world";
        msg.abs_pose.child_frame_id = prefix + "/vive_pose_abs";
        msg.abs_pose.transform.translation.x = data.pose_x;
        msg.abs_pose.transform.translation.y = data.pose_y;
        msg.abs_pose.transform.translation.z = data.pose_z;
        msg.abs_pose.transform.rotation.x = data.pose_qx;
        msg.abs_pose.transform.rotation.y = data.pose_qy;
        msg.abs_pose.transform.rotation.z = data.pose_qz;
        msg.abs_pose.transform.rotation.w = data.pose_qw;

        // Relative pose data
        msg.rel_pose.header.stamp = this->get_clock()->now();
        msg.rel_pose.header.frame_id = "world";
        msg.rel_pose.child_frame_id = prefix + "/vive_pose_rel";
        msg.rel_pose.transform.translation.x = relData.pose_x;
        msg.rel_pose.transform.translation.y = relData.pose_y;
        msg.rel_pose.transform.translation.z = relData.pose_z;
        msg.rel_pose.transform.rotation.x = relData.pose_qx;
        msg.rel_pose.transform.rotation.y = relData.pose_qy;
        msg.rel_pose.transform.rotation.z = relData.pose_qz;
        msg.rel_pose.transform.rotation.w = relData.pose_qw;

        // Publish to the controller data topic with role-specific frame_ids
        controller_data_publisher_->publish(msg);
        
        // For debugging
        RCLCPP_DEBUG(this->get_logger(), "Published controller data for %s controller", prefix.c_str());
    }

    // void publishStaticTransform() {
    //     geometry_msgs::msg::TransformStamped transformStamped;
    //     transformStamped.header.stamp = this->now();
    //     transformStamped.header.frame_id = "vx300s/base_link";
    //     transformStamped.child_frame_id = "world";
    //     transformStamped.transform.translation.x = 0.0;
    //     transformStamped.transform.translation.y = 0.0;
    //     transformStamped.transform.translation.z = 0.0;
    //     transformStamped.transform.rotation.x = 0.0;
    //     transformStamped.transform.rotation.y = 0.0;
    //     transformStamped.transform.rotation.z = 0.0;
    //     transformStamped.transform.rotation.w = 1.0;

    //     tf_broadcaster_->sendTransform(transformStamped);
    // }

    // Helper method to get a topic name based on controller role
    std::string getRoleBasedTopicName(int role, const std::string& baseTopic) {
        std::string prefix = (role == 1) ? "left_vr" : "right_vr";
        return prefix + "/" + baseTopic;
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
                // Measure processing time for performance monitoring
                auto start_time = std::chrono::steady_clock::now();
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
                    // Log minimal information at INFO level
                    std::string controller_name = (jsonData.role == 1) ? "LEFT" : "RIGHT";
                    RCLCPP_INFO(this->get_logger(), "[%s] Received data at time: %s", 
                                controller_name.c_str(), jsonData.time.c_str());
                    
                    // Detailed logging only at DEBUG level - won't slow down processing in normal operation
                    RCLCPP_DEBUG(this->get_logger(), "[%s] Pose: x=%f, y=%f, z=%f", 
                                controller_name.c_str(), jsonData.pose_x, jsonData.pose_y, jsonData.pose_z);
                    RCLCPP_DEBUG(this->get_logger(), "[%s] Buttons: menu=%d, trigger=%d, trackpad=%d, grip=%d", 
                                controller_name.c_str(), 
                                jsonData.menu_button, jsonData.trigger_button, 
                                jsonData.trackpad_button, jsonData.grip_button);

                    // Filter the pose data
                    VRControllerData filteredPose = filterPose(jsonData);

                    // Get references to the correct variables based on controller role
                    VRControllerData& initialPose = (filteredPose.role == 1) ? initialPoseLeft : initialPoseRight;
                    bool& triggerButtonPressed = (filteredPose.role == 1) ? triggerButtonPressedLeft : triggerButtonPressedRight;

                    // Handle trigger button state
                    if (filteredPose.trigger_button && !triggerButtonPressed) {
                        // Trigger button just pressed
                        initialPose = filteredPose;
                        triggerButtonPressed = true;
                        RCLCPP_DEBUG(this->get_logger(), "[%s] Trigger pressed - setting initial pose", 
                                    (filteredPose.role == 1) ? "LEFT" : "RIGHT");
                    } else if (!filteredPose.trigger_button && triggerButtonPressed) {
                        triggerButtonPressed = false;
                        RCLCPP_DEBUG(this->get_logger(), "[%s] Trigger released", 
                                    (filteredPose.role == 1) ? "LEFT" : "RIGHT");
                    }

                    VRControllerData relativePose;
                    if (triggerButtonPressed) {
                        // Calculate and publish relative transform
                        relativePose = calculateRelativePose(initialPose, filteredPose);
                        publishTransform(relativePose, true);   // isRelative = true
                        publishTransform(filteredPose);         // Publish absolute transform as well
                    } else {
                        // Publish the absolute transform
                        publishTransform(filteredPose);
                    }

                    // If menu button is pressed, reset the initial pose
                    if (filteredPose.menu_button) {
                        initialPose = filteredPose;
                        RCLCPP_DEBUG(this->get_logger(), "[%s] Menu pressed - resetting initial pose", 
                                    (filteredPose.role == 1) ? "LEFT" : "RIGHT");
                    }

                    // Publish controller data with role-based naming
                    // publishStaticTransform();
                    publishControllerData(filteredPose, relativePose);
                    
                    // Calculate and log processing time
                    auto end_time = std::chrono::steady_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
                    
                    // Only log processing time periodically to avoid logging overhead
                    static int counter = 0;
                    if (++counter % 100 == 0) {
                        RCLCPP_INFO(this->get_logger(), "[%s] Processing time: %ld μs", 
                                 (filteredPose.role == 1) ? "LEFT" : "RIGHT", duration);
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
    auto client = std::make_shared<Client>("127.0.0.1", 12345);
    client->start();
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
