#include <string>
#include <openvr.h>
#include <chrono> // Include this for std::chrono
#include <thread>

#include "VRUtils.hpp"
#include "json.hpp" // Include nlohmann/json
#include "server.hpp"

// VR Input Configuration Constants
namespace VRInputConfig {
    constexpr double DEFAULT_PUBLISH_FREQUENCY = 50.0;
    constexpr double MIN_PUBLISH_FREQUENCY = 0.1;
    constexpr double MAX_PUBLISH_FREQUENCY = 1000.0;
    constexpr int TRIGGER_FEEDBACK_STEPS = 6;
    constexpr int HAPTIC_FEEDBACK_DURATION = 200;
    constexpr int TRIGGER_HAPTIC_MULTIPLIER = 3000;
    constexpr int WARNING_HAPTIC_DURATION = 20;
    constexpr float POSITION_THRESHOLD = 0.05f;
    constexpr float DISTANCE_THRESHOLD = 0.1f;
    constexpr int NO_CONTROLLER_SLEEP_MS = 50;
    constexpr int DATA_COLLECTION_SLEEP_MS = 5;
    constexpr int CONTROLLER_DELAY_MS = 1;
    constexpr int LOG_INTERVAL_SECONDS = 1;
    constexpr double Y_OFFSET = 0.6;
    
    // Controller indices
    constexpr int RIGHT_CONTROLLER_INDEX = 0;
    constexpr int LEFT_CONTROLLER_INDEX = 1;
    constexpr int MAX_CONTROLLERS = 2;
}

// VR Controller Input Handler - manages VR device detection and data processing
// VR Controller Input Handler - manages VR device detection and data processing
class ViveInput {
public:
    ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data, double publish_freq = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY);
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
    bool controller_detected[VRInputConfig::MAX_CONTROLLERS] = {false, false};
    Eigen::Vector3d prev_position[VRInputConfig::MAX_CONTROLLERS];  // Use Eigen for better vector operations
    std::chrono::steady_clock::time_point prev_time[VRInputConfig::MAX_CONTROLLERS];
    bool first_run[VRInputConfig::MAX_CONTROLLERS] = {true, true};
    
    // Store data for both controllers separately
    VRControllerData right_controller_data; // For right controller (role_index = 0)
    VRControllerData left_controller_data;  // For left controller (role_index = 1)
    bool right_controller_updated = false;
    bool left_controller_updated = false;
    
    // Configurable publishing frequency for each controller (in Hz)
    double controllerPublishFrequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
    std::chrono::steady_clock::time_point last_publish_time[VRInputConfig::MAX_CONTROLLERS]; // For each controller
    bool publish_time_initialized[VRInputConfig::MAX_CONTROLLERS] = {false, false};

    bool initVR();
    bool shutdownVR();
    void processControllerData(uint32_t deviceIndex, int roleIndex);
    void processControllerButtons(uint32_t deviceIndex, vr::VRControllerState_t& controllerState, int roleIndex);
    void processTriggerFeedback(float triggerValue, uint32_t deviceIndex, int roleIndex);
    bool validatePositionChange(const Eigen::Vector3d& currentPosition, int roleIndex);
    void publishControllerDataAtFrequency();
    void handleControllerDetection(bool anyControllerDetected, std::chrono::steady_clock::time_point currentTime, std::chrono::steady_clock::time_point& lastLogTime);
};

ViveInput::ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data, double publish_freq) 
    : data_mutex(mutex), data_cv(cv), shared_data(data), controllerPublishFrequency(publish_freq) {
    if (!initVR()) {
        shutdownVR();
        throw std::runtime_error("Failed to initialize VR");
    }
}

ViveInput::~ViveInput() {
    shutdownVR();
}

void ViveInput::setPublishFrequency(double freq) {
    if (freq > VRInputConfig::MIN_PUBLISH_FREQUENCY && freq <= VRInputConfig::MAX_PUBLISH_FREQUENCY) {
        controllerPublishFrequency = freq;
        logMessage(Info, "Controller publish frequency updated to: " + std::to_string(freq) + " Hz");
        
        // Reset timing for both controllers to apply new frequency immediately
        publish_time_initialized[VRInputConfig::RIGHT_CONTROLLER_INDEX] = false;
        publish_time_initialized[VRInputConfig::LEFT_CONTROLLER_INDEX] = false;
    } else {
        logMessage(Warning, "Invalid frequency: " + std::to_string(freq) + " Hz. Must be between " + 
                   std::to_string(VRInputConfig::MIN_PUBLISH_FREQUENCY) + " and " + 
                   std::to_string(VRInputConfig::MAX_PUBLISH_FREQUENCY) + " Hz");
    }
}

void ViveInput::runVR() {
    logMessage(Info, "Starting VR loop");
    logMessage(Info, "Controller publish frequency set to: " + std::to_string(controllerPublishFrequency) + " Hz");
    auto lastLogTime = std::chrono::steady_clock::now(); // Initialize the last log time
    
    // For trigger feedback
    static int previousStep[VRInputConfig::MAX_CONTROLLERS] = {-1, -1}; // Initialize previous step for both controllers

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
                        
                        logMessage(Debug, "[CONTROLLER " + std::to_string(role_index) + "] [POSE CM]: " + 
                                std::to_string(position.x() * 100) + " " + 
                                std::to_string(position.y() * 100) + " " + 
                                std::to_string(position.z() * 100));
                        
                        // Reset local data for this controller
                        VRUtils::resetJsonData(local_data);
                        
                        // Store controller data using new Eigen-based methods
                        local_data.time = Server::getCurrentTimeWithMilliseconds();
                        local_data.role = role_index; // 0 = right, 1 = left
                        local_data.setPosition(position - Eigen::Vector3d(0, VRInputConfig::Y_OFFSET, 0)); // Apply Y offset
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
                                VRUtils::HapticFeedback(pHMD, i, VRInputConfig::WARNING_HAPTIC_DURATION);
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
        double interval_ms = 1000.0 / controllerPublishFrequency;
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
                              std::to_string(controllerPublishFrequency) + " Hz");
                    
                    // Small delay to prevent mutex contention between controllers
                    std::this_thread::sleep_for(std::chrono::milliseconds(VRInputConfig::CONTROLLER_DELAY_MS));
                }
            }
        }
        
        // Reset controller update flags for the next iteration
        right_controller_updated = false;
        left_controller_updated = false;
        
        // Handle controller detection status
        if (!anyControllerDetected) {
            if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastLogTime).count() >= VRInputConfig::LOG_INTERVAL_SECONDS) {
                logMessage(Info, "No controllers detected, currentTime: " + 
                        std::to_string(std::chrono::duration_cast<std::chrono::seconds>(currentTime.time_since_epoch()).count()));
                
                // Reset first_run flags for both controllers
                first_run[VRInputConfig::RIGHT_CONTROLLER_INDEX] = true;
                first_run[VRInputConfig::LEFT_CONTROLLER_INDEX] = true;
                
                lastLogTime = currentTime; // Update the last log time
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(VRInputConfig::NO_CONTROLLER_SLEEP_MS)); // ~20Hz when no controllers
        } else {
            // Run at higher frequency to ensure responsive data collection
            // but publishing is controlled by controllerPublishFrequency
            std::this_thread::sleep_for(std::chrono::milliseconds(VRInputConfig::DATA_COLLECTION_SLEEP_MS)); // ~200Hz data collection
            lastLogTime = currentTime;
            
            // Log which controllers were detected
            if (controller_detected[VRInputConfig::RIGHT_CONTROLLER_INDEX] && controller_detected[VRInputConfig::LEFT_CONTROLLER_INDEX]) {
                logMessage(Debug, "Both controllers detected");
            } else if (controller_detected[VRInputConfig::RIGHT_CONTROLLER_INDEX]) {
                logMessage(Debug, "Right controller detected");
            } else if (controller_detected[VRInputConfig::LEFT_CONTROLLER_INDEX]) {
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

void ViveInput::processControllerButtons(uint32_t deviceIndex, vr::VRControllerState_t& controllerState, int roleIndex) {
    if ((1LL << vr::k_EButton_ApplicationMenu) & controllerState.ulButtonPressed) {
        logMessage(Debug, "Application Menu button pressed, resetting the pose");
        first_run[roleIndex] = true; // Reset the first run flag for this controller
        local_data.menu_button = true;
        VRUtils::HapticFeedback(pHMD, deviceIndex, VRInputConfig::HAPTIC_FEEDBACK_DURATION);
    }
    
    if ((1LL << vr::k_EButton_SteamVR_Trigger) & controllerState.ulButtonPressed) {
        logMessage(Debug, "Trigger button pressed");
        local_data.trigger_button = true;
    }
    
    if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonPressed) {
        logMessage(Debug, "Touchpad button pressed");
        local_data.trackpad_button = true;
        VRUtils::HapticFeedback(pHMD, deviceIndex, VRInputConfig::HAPTIC_FEEDBACK_DURATION);
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
}

void ViveInput::processTriggerFeedback(float triggerValue, uint32_t deviceIndex, int roleIndex) {
    static int previousStep[VRInputConfig::MAX_CONTROLLERS] = {-1, -1};
    
    const float stepSize = 1.0f / VRInputConfig::TRIGGER_FEEDBACK_STEPS;
    int currentStep = static_cast<int>(triggerValue / stepSize);
    logMessage(Debug, "Trigger: " + std::to_string(triggerValue) + "\n");
    local_data.trigger = triggerValue;
    
    if (currentStep != previousStep[roleIndex]) {
        int vibrationDuration = static_cast<int>(triggerValue * VRInputConfig::TRIGGER_HAPTIC_MULTIPLIER);
        VRUtils::HapticFeedback(pHMD, deviceIndex, vibrationDuration);
        previousStep[roleIndex] = currentStep;
    }
}

bool ViveInput::validatePositionChange(const Eigen::Vector3d& currentPosition, int roleIndex) {
    auto current_time = std::chrono::steady_clock::now();
    if (!first_run[roleIndex]) {
        std::chrono::duration<float> time_diff = current_time - prev_time[roleIndex];
        float delta_time = time_diff.count();
        
        // Calculate position change using Eigen
        Eigen::Vector3d position_change = currentPosition - prev_position[roleIndex];
        double delta_distance = position_change.norm();
        double velocity = delta_distance / delta_time;

        logMessage(Debug, "[CONTROLLER " + std::to_string(roleIndex) + "] Velocity: " + 
                  std::to_string(velocity) + " units/s");
        logMessage(Debug, "[CONTROLLER " + std::to_string(roleIndex) + "] Delta pos: " + 
                  std::to_string(delta_distance) + " units");
        
        // Check if delta distance is reasonable using Eigen-based validation
        if (!VRTransforms::isPositionChangeReasonable(currentPosition, prev_position[roleIndex], VRInputConfig::POSITION_THRESHOLD)) {
            logMessage(Warning, "[CONTROLLER " + std::to_string(roleIndex) + 
                      "] Unreasonable delta_distance detected: " + 
                      std::to_string(delta_distance) + " units. Skipping this data.");
            return false;
        } else {
            logMessage(Debug, "[CONTROLLER " + std::to_string(roleIndex) + "] Will publish this data");
        }
    } else {
        first_run[roleIndex] = false; // Set the flag to false after the first run
    }

    // Update previous record
    prev_position[roleIndex] = currentPosition;
    prev_time[roleIndex] = current_time;
    return true;
}

int main(int argc, char **argv) {
    Server::setupSignalHandlers();

    std::mutex data_mutex;
    std::condition_variable data_cv;
    VRControllerData shared_data;
    
    // Configure publishing frequency (default 50Hz, can be adjusted)
    double publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
    
    // Check for command line argument to set frequency
    if (argc > 1) {
        try {
            publish_frequency = std::stod(argv[1]);
            if (publish_frequency <= VRInputConfig::MIN_PUBLISH_FREQUENCY || publish_frequency > VRInputConfig::MAX_PUBLISH_FREQUENCY) {
                std::cerr << "Invalid frequency. Using default " << VRInputConfig::DEFAULT_PUBLISH_FREQUENCY << "Hz." << std::endl;
                publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
            }
        } catch (const std::exception& e) {
            std::cerr << "Invalid frequency argument. Using default " << VRInputConfig::DEFAULT_PUBLISH_FREQUENCY << "Hz." << std::endl;
            publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
        }
    }

    Server server(12345, data_mutex, data_cv, shared_data);
    std::thread serverThread(&Server::start, &server);
    serverThread.detach(); // Detach the server thread

    ViveInput vive_input(data_mutex, data_cv, shared_data, publish_frequency);
    vive_input.runVR();

    return 0;
}
