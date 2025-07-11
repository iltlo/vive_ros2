#include <string>
#include <openvr.h>
#include <chrono> // Include this for std::chrono
#include <thread>

#include "VRUtils.hpp"
#include "json.hpp" // Include nlohmann/json
#include "server.hpp"

class ViveInput {
public:
    ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data, double publish_freq = 50.0);
    ~ViveInput();
    void runVR();
    void setPublishFrequency(double freq);

private:
    vr::IVRSystem *pHMD = nullptr;
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

    std::mutex &data_mutex;
    std::condition_variable &data_cv;
    VRControllerData &shared_data;
    VRControllerData local_data;
    
    // Track both controllers separately (right = 0, left = 1)
    bool controller_detected[2] = {false, false};
    Eigen::Vector3d prev_position[2];  // Use Eigen for better vector operations
    std::chrono::steady_clock::time_point prev_time[2];
    bool first_run[2] = {true, true};
    
    // Store data for both controllers separately
    VRControllerData right_controller_data; // For right controller (role_index = 0)
    VRControllerData left_controller_data;  // For left controller (role_index = 1)
    bool right_controller_updated = false;
    bool left_controller_updated = false;
    
    // Configurable publishing frequency for each controller (in Hz)
    double CONTROLLER_PUBLISH_FREQUENCY = 50.0; // Default 50 Hz - adjust this as needed
    std::chrono::steady_clock::time_point last_publish_time[2]; // For each controller
    bool publish_time_initialized[2] = {false, false};

    bool initVR();
    bool shutdownVR();

    // const float velocity_threshold = 2.5f;
    const float distance_threshold = 0.1f;
};

ViveInput::ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data, double publish_freq) 
    : data_mutex(mutex), data_cv(cv), shared_data(data), CONTROLLER_PUBLISH_FREQUENCY(publish_freq) {
    if (!initVR()) {
        shutdownVR();
        throw std::runtime_error("Failed to initialize VR");
    }
}

ViveInput::~ViveInput() {
    shutdownVR();
}

void ViveInput::setPublishFrequency(double freq) {
    if (freq > 0.0 && freq <= 1000.0) { // Reasonable bounds
        CONTROLLER_PUBLISH_FREQUENCY = freq;
        logMessage(Info, "Controller publish frequency updated to: " + std::to_string(freq) + " Hz");
        
        // Reset timing for both controllers to apply new frequency immediately
        publish_time_initialized[0] = false;
        publish_time_initialized[1] = false;
    } else {
        logMessage(Warning, "Invalid frequency: " + std::to_string(freq) + " Hz. Must be between 0.1 and 1000.0 Hz");
    }
}

void ViveInput::runVR() {
    logMessage(Info, "Starting VR loop");
    logMessage(Info, "Controller publish frequency set to: " + std::to_string(CONTROLLER_PUBLISH_FREQUENCY) + " Hz");
    auto lastLogTime = std::chrono::steady_clock::now(); // Initialize the last log time
    
    // For trigger feedback
    static int previousStep[2] = {-1, -1}; // Initialize previous step for both controllers

    while (true) {
        // Reset controller detection status for this loop
        controller_detected[0] = false; // Right controller
        controller_detected[1] = false; // Left controller
        
        // Reset controller update flags for this iteration
        right_controller_updated = false;
        left_controller_updated = false;
        
        // Update the poses for all devices
        pHMD->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, trackedDevicePose, vr::k_unMaxTrackedDeviceCount);

        // First, scan for all controllers
        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            if (trackedDevicePose[i].bDeviceIsConnected && trackedDevicePose[i].bPoseIsValid
                && trackedDevicePose[i].eTrackingResult == vr::TrackingResult_Running_OK) {

                if (VRUtils::controllerIsConnected(pHMD, i)) {
                    // Get the controller's role (left or right)
                    vr::ETrackedControllerRole controllerRole = VRUtils::controllerRoleCheck(pHMD, i);
                    
                    // Process only if it's a valid controller (left or right hand)
                    if (controllerRole == vr::TrackedControllerRole_LeftHand || 
                        controllerRole == vr::TrackedControllerRole_RightHand) {
                        
                        // Map the controller role to our index (0=right, 1=left)
                        int role_index = (controllerRole == vr::TrackedControllerRole_LeftHand) ? 1 : 0;
                        
                        // Mark this controller as detected
                        controller_detected[role_index] = true;
                        
                        // Process this controller's data using Eigen transforms
                        // Get the pose of the device
                        vr::HmdMatrix34_t steamVRMatrix = trackedDevicePose[i].mDeviceToAbsoluteTracking;
                        
                        // Use new Eigen-based transform utilities
                        Eigen::Vector3d position = VRTransforms::getPositionFromVRMatrix(steamVRMatrix);
                        Eigen::Quaterniond quaternion = VRTransforms::getQuaternionFromVRMatrix(steamVRMatrix);
                        Eigen::Vector3d euler = VRTransforms::quaternionToEulerXYZ(quaternion);
                        
                        logMessage(Debug, "[CONTROLLER " + std::to_string(role_index) + "] [POSE CM]: " + 
                                std::to_string(position.x() * 100) + " " + 
                                std::to_string(position.y() * 100) + " " + 
                                std::to_string(position.z() * 100));
                        
                        // Reset local data for this controller
                        VRUtils::resetJsonData(local_data);
                        
                        // Store controller data using new Eigen-based methods
                        local_data.time = Server::getCurrentTimeWithMilliseconds();
                        local_data.role = role_index; // 0 = right, 1 = left
                        local_data.setPosition(position - Eigen::Vector3d(0, 0.6, 0)); // Apply Y offset
                        local_data.setQuaternion(quaternion);

                        // Process controller state and buttons
                        vr::VRControllerState_t controllerState;
                        pHMD->GetControllerState(i, &controllerState, sizeof(controllerState));
                        
                        if ((1LL << vr::k_EButton_ApplicationMenu) & controllerState.ulButtonPressed) {
                            logMessage(Debug, "Application Menu button pressed, resetting the pose");
                            first_run[role_index] = true; // Reset the first run flag for this controller
                            local_data.menu_button = true;
                            VRUtils::HapticFeedback(pHMD, i, 200);
                        }
                        
                        if ((1LL << vr::k_EButton_SteamVR_Trigger) & controllerState.ulButtonPressed) {
                            logMessage(Debug, "Trigger button pressed");
                            local_data.trigger_button = true;
                        }
                        
                        if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonPressed) {
                            logMessage(Debug, "Touchpad button pressed");
                            local_data.trackpad_button = true;
                            VRUtils::HapticFeedback(pHMD, i, 200);
                        }
                        
                        if ((1LL << vr::k_EButton_Grip) & controllerState.ulButtonPressed) {
                            logMessage(Debug, "Grip button pressed");
                            local_data.grip_button = true;
                        }
                        
                        if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonTouched) {
                            logMessage(Debug, "Touchpad button touched");
                            local_data.trackpad_x = controllerState.rAxis[0].x;
                            local_data.trackpad_y = controllerState.rAxis[0].y;
                            local_data.trackpad_touch = true;
                        }
                        
                        // Process trigger
                        const int numSteps = 6;
                        const float stepSize = 1.0f / numSteps;
                        float triggerValue = controllerState.rAxis[1].x;
                        int currentStep = static_cast<int>(triggerValue / stepSize);
                        logMessage(Debug, "Trigger: " + std::to_string(triggerValue) + "\n");
                        local_data.trigger = triggerValue;
                        
                        if (currentStep != previousStep[role_index]) {
                            int vibrationDuration = static_cast<int>(triggerValue * 3000);
                            VRUtils::HapticFeedback(pHMD, i, vibrationDuration);
                            previousStep[role_index] = currentStep;
                        }

                        // Check if the input data is reasonable using Eigen
                        auto current_time = std::chrono::steady_clock::now();
                        if (!first_run[role_index]) {
                            std::chrono::duration<float> time_diff = current_time - prev_time[role_index];
                            float delta_time = time_diff.count();
                            
                            // Calculate position change using Eigen
                            Eigen::Vector3d position_change = position - prev_position[role_index];
                            double delta_distance = position_change.norm();
                            double velocity = delta_distance / delta_time;

                            logMessage(Debug, "[CONTROLLER " + std::to_string(role_index) + "] Velocity: " + 
                                      std::to_string(velocity) + " units/s");
                            logMessage(Debug, "[CONTROLLER " + std::to_string(role_index) + "] Delta pos: " + 
                                      std::to_string(delta_distance) + " units");
                            
                            // Check if delta distance is reasonable using Eigen-based validation
                            if (!VRTransforms::isPositionChangeReasonable(position, prev_position[role_index], 0.05)) {
                                logMessage(Warning, "[CONTROLLER " + std::to_string(role_index) + 
                                          "] Unreasonable delta_distance detected: " + 
                                          std::to_string(delta_distance) + " units. Skipping this data.");
                                VRUtils::HapticFeedback(pHMD, i, 20);
                                continue; // Skip this data
                            } else {
                                logMessage(Debug, "[CONTROLLER " + std::to_string(role_index) + "] Will publish this data");
                            }
                        } else {
                            first_run[role_index] = false; // Set the flag to false after the first run
                        }

                        // Update previous record
                        prev_position[role_index] = position;
                        prev_time[role_index] = current_time;

                        // Store this controller's data - we'll send both controllers at the end of the loop
                        // Create a copy of the current controller data
                        if (role_index == 0) { // Right controller
                            right_controller_data = local_data;
                            right_controller_updated = true;
                        } else { // Left controller
                            left_controller_data = local_data;
                            left_controller_updated = true;
                        }
                    }
                }
            }
        }

        // Process and send controller data at controlled frequency
        // This ensures both controllers are sent at the same configurable rate
        auto currentTime = std::chrono::steady_clock::now();
        bool anyControllerDetected = controller_detected[0] || controller_detected[1];
        
        // Calculate time interval for desired frequency
        double interval_ms = 1000.0 / CONTROLLER_PUBLISH_FREQUENCY;
        auto interval_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double, std::milli>(interval_ms));
        
        // Check and send data for each controller at controlled frequency
        for (int role = 0; role < 2; role++) {
            bool controller_has_data = (role == 0) ? right_controller_updated : left_controller_updated;
            
            if (controller_has_data) {
                // Initialize publish time if not done yet
                if (!publish_time_initialized[role]) {
                    last_publish_time[role] = currentTime;
                    publish_time_initialized[role] = true;
                }
                
                // Check if enough time has passed for this controller
                auto time_since_last_publish = currentTime - last_publish_time[role];
                if (time_since_last_publish >= interval_duration) {
                    // Send data for this controller
                    {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        if (role == 0) {
                            shared_data = right_controller_data;
                        } else {
                            shared_data = left_controller_data;
                        }
                        shared_data.time = Server::getCurrentTimeWithMilliseconds();
                        shared_data.role = role;
                        data_cv.notify_one();
                    }
                    
                    // Update last publish time for this controller
                    last_publish_time[role] = currentTime;
                    
                    // Log publishing activity
                    std::string controller_name = (role == 0) ? "RIGHT" : "LEFT";
                    logMessage(Debug, "[" + controller_name + "] Data sent to server at " + 
                              std::to_string(CONTROLLER_PUBLISH_FREQUENCY) + " Hz");
                    
                    // Small delay to prevent mutex contention between controllers
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
            }
        }
        
        // Reset controller update flags for the next iteration
        right_controller_updated = false;
        left_controller_updated = false;
        
        // Handle controller detection status
        if (!anyControllerDetected) {
            if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastLogTime).count() >= 1) {
                logMessage(Info, "No controllers detected, currentTime: " + 
                        std::to_string(std::chrono::duration_cast<std::chrono::seconds>(currentTime.time_since_epoch()).count()));
                
                // Reset first_run flags for both controllers
                first_run[0] = true;
                first_run[1] = true;
                
                lastLogTime = currentTime; // Update the last log time
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20Hz when no controllers
        } else {
            // Run at higher frequency to ensure responsive data collection
            // but publishing is controlled by CONTROLLER_PUBLISH_FREQUENCY
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // ~200Hz data collection
            lastLogTime = currentTime;
            
            // Log which controllers were detected
            if (controller_detected[0] && controller_detected[1]) {
                logMessage(Debug, "Both controllers detected");
            } else if (controller_detected[0]) {
                logMessage(Debug, "Right controller detected");
            } else if (controller_detected[1]) {
                logMessage(Debug, "Left controller detected");
            }
        }
    }
}

bool ViveInput::initVR() {
    // Initialize VR runtime
    eError = vr::VRInitError_None;
    pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);
    if (eError != vr::VRInitError_None) {
        pHMD = NULL;
        std::string error_msg = vr::VR_GetVRInitErrorAsEnglishDescription(eError);
        logMessage(Error, "Unable to init VR runtime: " + error_msg);
        return false;
    } else {
        logMessage(Info, "VR runtime initialized");
    }
    return true;
}

bool ViveInput::shutdownVR() {
    // Shutdown VR runtime
    if (pHMD) {
        logMessage(Info, "Shutting down VR runtime");
        vr::VR_Shutdown();
    }
    return true;
}

int main(int argc, char **argv) {
    Server::setupSignalHandlers();

    std::mutex data_mutex;
    std::condition_variable data_cv;
    VRControllerData shared_data;
    
    // Configure publishing frequency (default 50Hz, can be adjusted)
    double publish_frequency = 50.0; // Hz
    
    // Check for command line argument to set frequency
    if (argc > 1) {
        try {
            publish_frequency = std::stod(argv[1]);
            if (publish_frequency <= 0.0 || publish_frequency > 1000.0) {
                std::cerr << "Invalid frequency. Using default 50Hz." << std::endl;
                publish_frequency = 50.0;
            }
        } catch (const std::exception& e) {
            std::cerr << "Invalid frequency argument. Using default 50Hz." << std::endl;
            publish_frequency = 50.0;
        }
    }

    Server server(12345, data_mutex, data_cv, shared_data);
    std::thread serverThread(&Server::start, &server);
    serverThread.detach(); // Detach the server thread

    ViveInput vive_input(data_mutex, data_cv, shared_data, publish_frequency);
    vive_input.runVR();

    return 0;
}
