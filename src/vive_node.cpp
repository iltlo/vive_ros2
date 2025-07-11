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

// Constants for configuration
namespace VRConfig {
    constexpr int DEFAULT_QUEUE_SIZE = 150;
    constexpr int BUFFER_SIZE = 1024;
    constexpr int PERFORMANCE_LOG_INTERVAL = 100;
    constexpr int RECONNECTION_DELAY_SECONDS = 1;
    constexpr double POSE_SMOOTHING_FACTOR = 0.9999;
    constexpr int RIGHT_CONTROLLER_ROLE = 0;
    constexpr int LEFT_CONTROLLER_ROLE = 1;
    
    // Coordinate transformation constants (VR Y-up to ROS Z-up)
    constexpr double COORD_TRANSFORM_NEG = -1.0;
}

// Controller role enumeration for better type safety
enum class ControllerRole {
    Right = VRConfig::RIGHT_CONTROLLER_ROLE,
    Left = VRConfig::LEFT_CONTROLLER_ROLE
};

// Socket configuration parameter object
struct SocketConfig {
    std::string address;
    int port;
    
    SocketConfig(const std::string& addr, int p) : address(addr), port(p) {}
};

// VR Controller ROS2 Client - bridges VR input server to ROS2 transforms
class VRControllerClient : public rclcpp::Node {
private:
    int sock;
    struct sockaddr_in serv_addr;
    SocketConfig socketConfig;
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

    // Initialize socket address structure for TCP connection
    void setupSocket() {
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(socketConfig.port);
        if(inet_pton(AF_INET, socketConfig.address.c_str(), &serv_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported");
            exit(EXIT_FAILURE);
        }
    }

    void createPublishers() {
        // Create publishers with role-specific topic names
        abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_abs", VRConfig::DEFAULT_QUEUE_SIZE);
        rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_rel", VRConfig::DEFAULT_QUEUE_SIZE);
        controller_data_publisher_ = this->create_publisher<vive_ros2::msg::VRControllerData>("controller_data", 10);
        
        // Create role-specific publishers for better topic organization
        left_abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("left_vr/vive_pose_abs", VRConfig::DEFAULT_QUEUE_SIZE);
        left_rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("left_vr/vive_pose_rel", VRConfig::DEFAULT_QUEUE_SIZE);
        right_abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("right_vr/vive_pose_abs", VRConfig::DEFAULT_QUEUE_SIZE);
        right_rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("right_vr/vive_pose_rel", VRConfig::DEFAULT_QUEUE_SIZE);
        
        RCLCPP_INFO(this->get_logger(), "Publishers created with role-based topic names.");
    }

    // Create TCP socket for VR server communication
    void establishServerConnection() {
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
            // Exponential backoff could be implemented here for production use
            std::this_thread::sleep_for(std::chrono::seconds(VRConfig::RECONNECTION_DELAY_SECONDS));
            establishServerConnection();
        }
        RCLCPP_INFO(this->get_logger(), "Reconnected to server.");
    }

    // Transform from VR coordinate system (Y-up) to ROS (Z-up)
    void transformVRToROS(const VRControllerData& vrData, geometry_msgs::msg::TransformStamped& transform) {
        transform.transform.translation.x = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_z;
        transform.transform.translation.y = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_x;
        transform.transform.translation.z = vrData.pose_y;
        transform.transform.rotation.x = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_qz;
        transform.transform.rotation.y = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_qx;
        transform.transform.rotation.z = vrData.pose_qy;
        transform.transform.rotation.w = vrData.pose_qw;
    }

    std::string generateFrameId(ControllerRole role, bool isRelative) {
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";
        std::string suffix = isRelative ? "vive_pose_rel" : "vive_pose_abs";
        return prefix + "/" + suffix;
    }

    void publishTransform(const VRControllerData& pose, bool isRelative = false) {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        
        ControllerRole role = static_cast<ControllerRole>(pose.role);
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";
        std::string topic_name = (isRelative) ? "vive_pose_rel" : "vive_pose_abs";
        
        // Set child_frame_id with the appropriate prefix
        transformStamped.child_frame_id = generateFrameId(role, isRelative);

        // Transform from VR coordinate system (Y-up) to ROS (Z-up)
        transformVRToROS(pose, transformStamped);

        // Publish to TF with role-based frame naming
        tf_broadcaster_->sendTransform(transformStamped);
        
        // Create a new message to publish to the role-specific topic
        auto roleSpecificMsg = transformStamped;
        
        // Publish to the appropriate role-specific topic
        RCLCPP_INFO(this->get_logger(), "Publishing transform for role: %d", pose.role);
        if (role == ControllerRole::Left) {
            if (isRelative) {
                left_rel_transform_publisher_->publish(roleSpecificMsg);
            } else {
                left_abs_transform_publisher_->publish(roleSpecificMsg);
            }
        } else { // Right controller
            if (isRelative) {
                right_rel_transform_publisher_->publish(roleSpecificMsg);
            } else {
                right_abs_transform_publisher_->publish(roleSpecificMsg);
            }
        }
        
        // Also publish to generic topics for backward compatibility
        if (isRelative) {
            rel_transform_publisher_->publish(roleSpecificMsg);
        } else {
            abs_transform_publisher_->publish(roleSpecificMsg);
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
        // Low-pass filter with controller-specific state tracking
        static VRControllerData prevPoseLeft;
        static VRControllerData prevPoseRight;
        static bool initialized[2] = {false, false};
        
        // Get appropriate previous pose based on controller role
        VRControllerData& prevPose = (pose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? prevPoseLeft : prevPoseRight;
        
        // Initialize if this is the first data for this controller
        if (!initialized[pose.role]) {
            prevPose = pose;
            initialized[pose.role] = true;
            return pose; // Return unfiltered for first sample
        }

        // Use the new Eigen-based filtering utility
        VRControllerData filteredPose = VRUtils::filterPose(pose, prevPose, VRConfig::POSE_SMOOTHING_FACTOR);
        
        // Store this pose for next iteration
        prevPose = filteredPose;
        
        return filteredPose;
    }



public:
    VRControllerClient(std::string addr, int p) : Node("client_node"), sock(-1), socketConfig(addr, p) {
        setupSocket();
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        createPublishers();
    }

    ~VRControllerClient() {
        if (sock != -1) {
            close(sock);
        }
    }

    void assignCoordinates(const VRControllerData& data, geometry_msgs::msg::TransformStamped& transform) {
        transform.transform.translation.x = data.pose_x;
        transform.transform.translation.y = data.pose_y;
        transform.transform.translation.z = data.pose_z;
        transform.transform.rotation.x = data.pose_qx;
        transform.transform.rotation.y = data.pose_qy;
        transform.transform.rotation.z = data.pose_qz;
        transform.transform.rotation.w = data.pose_qw;
    }

    void populateControllerMessage(const VRControllerData& absoluteData, const VRControllerData& relativeData, 
                                 vive_ros2::msg::VRControllerData& msg) {
        msg.grip_button = absoluteData.grip_button;
        msg.trigger_button = absoluteData.trigger_button;
        msg.trackpad_button = absoluteData.trackpad_button;
        msg.trackpad_touch = absoluteData.trackpad_touch;
        msg.menu_button = absoluteData.menu_button;
        msg.trackpad_x = absoluteData.trackpad_x;
        msg.trackpad_y = absoluteData.trackpad_y;
        msg.trigger = absoluteData.trigger;
        msg.role = absoluteData.role;
        msg.time = absoluteData.time;

        // Determine prefix based on controller role
        ControllerRole role = static_cast<ControllerRole>(absoluteData.role);
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";

        // Absolute pose data
        msg.abs_pose.header.stamp = this->get_clock()->now();
        msg.abs_pose.header.frame_id = "world";
        msg.abs_pose.child_frame_id = prefix + "/vive_pose_abs";
        assignCoordinates(absoluteData, msg.abs_pose);

        // Relative pose data
        msg.rel_pose.header.stamp = this->get_clock()->now();
        msg.rel_pose.header.frame_id = "world";
        msg.rel_pose.child_frame_id = prefix + "/vive_pose_rel";
        assignCoordinates(relativeData, msg.rel_pose);
    }

    void publishControllerData(const VRControllerData& absoluteData, const VRControllerData& relativeData) {
        vive_ros2::msg::VRControllerData msg;
        populateControllerMessage(absoluteData, relativeData, msg);

        // Publish to the controller data topic with role-specific frame_ids
        controller_data_publisher_->publish(msg);
        
        // For debugging
        ControllerRole role = static_cast<ControllerRole>(absoluteData.role);
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";
        RCLCPP_DEBUG(this->get_logger(), "Published controller data for %s controller", prefix.c_str());
    }

    // Helper method to get a topic name based on controller role
    std::string getRoleBasedTopicName(int role, const std::string& baseTopic) {
        std::string prefix = (role == 1) ? "left_vr" : "right_vr";
        return prefix + "/" + baseTopic;
    }

    int readFromSocket(char* buffer) {
        memset(buffer, 0, VRConfig::BUFFER_SIZE);
        return read(sock, buffer, VRConfig::BUFFER_SIZE);
    }

    bool parseControllerData(const std::string& receivedData) {
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
            return true;
        } catch (json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            return false;
        }
    }

    void handleTriggerButton(const VRControllerData& filteredPose, VRControllerData& relativePose) {
        // Get references to the correct variables based on controller role
        VRControllerData& initialPose = (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? initialPoseLeft : initialPoseRight;
        bool& triggerButtonPressed = (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? triggerButtonPressedLeft : triggerButtonPressedRight;

        // Handle trigger button state
        if (filteredPose.trigger_button && !triggerButtonPressed) {
            // Trigger button just pressed
            initialPose = filteredPose;
            triggerButtonPressed = true;
            RCLCPP_DEBUG(this->get_logger(), "[%s] Trigger pressed - setting initial pose", 
                        (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT");
        } else if (!filteredPose.trigger_button && triggerButtonPressed) {
            triggerButtonPressed = false;
            RCLCPP_DEBUG(this->get_logger(), "[%s] Trigger released", 
                        (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT");
        }

        if (triggerButtonPressed) {
            // Calculate and publish relative transform
            relativePose = calculateRelativePose(initialPose, filteredPose);
            publishTransform(relativePose, true);   // isRelative = true
            publishTransform(filteredPose);         // Publish absolute transform as well
        } else {
            // Publish the absolute transform
            publishTransform(filteredPose);
        }
    }

    void handleMenuButton(const VRControllerData& filteredPose) {
        // If menu button is pressed, reset the initial pose
        if (filteredPose.menu_button) {
            VRControllerData& initialPose = (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? initialPoseLeft : initialPoseRight;
            initialPose = filteredPose;
            RCLCPP_DEBUG(this->get_logger(), "[%s] Menu pressed - resetting initial pose", 
                        (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT");
        }
    }

    void logPerformanceMetrics(const VRControllerData& filteredPose, 
                              std::chrono::steady_clock::time_point start_time) {
        // Calculate and log processing time
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        
        // Performance monitoring - logs every 100 messages to avoid overhead
        static int counter = 0;
        if (++counter % VRConfig::PERFORMANCE_LOG_INTERVAL == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] Processing time: %ld μs", 
                     (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT", duration);
        }
    }

    void start() {
        while (sock < 0) {
            RCLCPP_INFO(this->get_logger(), "Attempting to connect to server...");
            establishServerConnection();
        }
        RCLCPP_INFO(this->get_logger(), "Connected to server.");

        // Main message processing loop - handles VR data reception and ROS publishing
        char buffer[VRConfig::BUFFER_SIZE] = {0};
        while (rclcpp::ok()) {
            int receivedByteCount = readFromSocket(buffer);
            if (receivedByteCount > 0) {
                // Measure processing time for performance monitoring
                auto start_time = std::chrono::steady_clock::now();
                
                std::string receivedData(buffer, receivedByteCount);
                if (!parseControllerData(receivedData)) {
                    continue; // Skip this iteration if parsing failed
                }

                // Log minimal information at INFO level
                ControllerRole role = static_cast<ControllerRole>(jsonData.role);
                std::string controller_name = (role == ControllerRole::Left) ? "LEFT" : "RIGHT";
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
                VRControllerData relativePose;

                // Handle trigger button logic and transform publishing
                handleTriggerButton(filteredPose, relativePose);

                // Handle menu button logic
                handleMenuButton(filteredPose);

                // Publish controller data with role-based naming
                publishControllerData(filteredPose, relativePose);
                
                // Log performance metrics periodically
                logPerformanceMetrics(filteredPose, start_time);

            } else if (receivedByteCount == 0) {
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
    auto client = std::make_shared<VRControllerClient>("127.0.0.1", 12345);
    client->start();
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
