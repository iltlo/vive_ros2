#ifndef VRUTILS_HPP
#define VRUTILS_HPP

#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <cmath>
#include <openvr.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "json.hpp"
#include "EigenTransforms.hpp"

/**
 * @brief Data structure for VR controller state
 */
struct VRControllerData {
    // Pose data (using double for precision)
    double pose_x, pose_y, pose_z;          // Position
    double pose_qx, pose_qy, pose_qz, pose_qw; // Quaternion orientation
    
    // Button states
    bool menu_button = false;
    bool trigger_button = false;
    bool trackpad_touch = false;
    bool trackpad_button = false;
    bool grip_button = false;
    
    // Analog inputs
    double trackpad_x = 0.0, trackpad_y = 0.0;
    double trigger = 0.0;
    
    // Controller identification
    int role = 1;  // 0 for right, 1 for left
    std::string time;
    
    /**
     * @brief Get position as Eigen vector
     */
    Eigen::Vector3d getPosition() const {
        return Eigen::Vector3d(pose_x, pose_y, pose_z);
    }
    
    /**
     * @brief Get orientation as Eigen quaternion
     */
    Eigen::Quaterniond getQuaternion() const {
        return Eigen::Quaterniond(pose_qw, pose_qx, pose_qy, pose_qz);
    }
    
    /**
     * @brief Set position from Eigen vector
     */
    void setPosition(const Eigen::Vector3d& pos) {
        pose_x = pos.x();
        pose_y = pos.y();
        pose_z = pos.z();
    }
    
    /**
     * @brief Set orientation from Eigen quaternion
     */
    void setQuaternion(const Eigen::Quaterniond& q) {
        pose_qw = q.w();
        pose_qx = q.x();
        pose_qy = q.y();
        pose_qz = q.z();
    }
    
    /**
     * @brief Reset all data to default values
     */
    void reset() {
        pose_x = pose_y = pose_z = 0.0;
        pose_qx = pose_qy = pose_qz = 0.0;
        pose_qw = 1.0;  // Identity quaternion
        menu_button = trigger_button = trackpad_touch = trackpad_button = grip_button = false;
        trackpad_x = trackpad_y = trigger = 0.0;
        role = 1;
        time = "";
    }
};

/**
 * @brief Log level enumeration
 */
enum LogLevel {
    Info,
    Debug,
    Warning,
    Error
};

/**
 * @brief Thread-safe logging function with timestamps
 */
inline void logMessage(LogLevel level, const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    
    switch (level) {
        case Info:
            std::cout << "[" << oss.str() << "][INFO] " << message << std::endl;
            break;
        case Debug:
            std::cout << "[" << oss.str() << "][DEBUG] " << message << std::endl;
            break;
        case Warning:
            std::cerr << "[" << oss.str() << "][WARNING] " << message << std::endl;
            break;
        case Error:
            std::cerr << "[" << oss.str() << "][ERROR] " << message << std::endl;
            break;
    }
}



/**
 * @brief Utility class for VR device management and data handling
 */
class VRUtils {
public:
    /**
     * @brief Get current timestamp with milliseconds
     */
    static std::string getCurrentTimeWithMilliseconds() {
        auto now = std::chrono::system_clock::now();
        auto nowAsTimeT = std::chrono::system_clock::to_time_t(now);
        auto nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        
        std::stringstream ss;
        ss << std::put_time(std::localtime(&nowAsTimeT), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << nowMs.count();
        return ss.str();
    }

    /**
     * @brief Reset controller data to default values
     */
    static void resetJsonData(VRControllerData& data) {
        data.reset();
    }

    /**
     * @brief Check if a device is connected
     */
    static bool deviceIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex) {
        if (!pHMD) return false;
        return pHMD->IsTrackedDeviceConnected(unDeviceIndex);
    }

    /**
     * @brief Check if a device is a controller
     */
    static bool controllerIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex) {
        if (!pHMD) return false;
        return pHMD->GetTrackedDeviceClass(unDeviceIndex) == vr::TrackedDeviceClass_Controller;
    }

    /**
     * @brief Log all connected devices
     */
    static void deviceConnectionCheck(vr::IVRSystem* pHMD) {
        if (!pHMD) return;
        
        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            if (deviceIsConnected(pHMD, i)) {
                vr::ETrackedDeviceClass trackedDeviceClass = pHMD->GetTrackedDeviceClass(i);
                logMessage(Debug, "[CONNECTED DEVICE " + std::to_string(i) + 
                          "]: class " + std::to_string(trackedDeviceClass));
            }
        }
    }

    /**
     * @brief Get controller role and log it
     */
    static vr::ETrackedControllerRole controllerRoleCheck(vr::IVRSystem* pHMD, 
                                                          vr::TrackedDeviceIndex_t i) {
        vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
        
        if (!pHMD || !controllerIsConnected(pHMD, i)) {
            return controllerRole;
        }
        
        controllerRole = pHMD->GetControllerRoleForTrackedDeviceIndex(i);
        
        if (controllerRole != vr::TrackedControllerRole_Invalid) {
            std::string roleStr;
            switch (controllerRole) {
                case vr::TrackedControllerRole_LeftHand:
                    roleStr = "Left";
                    break;
                case vr::TrackedControllerRole_RightHand:
                    roleStr = "Right";
                    break;
                default:
                    roleStr = "Unknown(" + std::to_string(controllerRole) + ")";
                    break;
            }
            logMessage(Debug, "[CONNECTED CONTROLLER " + std::to_string(i) + 
                      "]: role " + roleStr);
        }
        
        return controllerRole;
    }

    /**
     * @brief Check all controller connections
     */
    static void controllerConnectionCheck(vr::IVRSystem* pHMD) {
        if (!pHMD) return;
        
        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            controllerRoleCheck(pHMD, i);
        }
    }

    /**
     * @brief Trigger haptic feedback on controller
     */
    static void HapticFeedback(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unControllerDeviceIndex, 
                              unsigned short durationMicroSec) {
        if (pHMD) {
            pHMD->TriggerHapticPulse(unControllerDeviceIndex, 0, durationMicroSec);
        }
    }

    /**
     * @brief Extract pose from OpenVR matrix using Eigen transforms
     */
    static void extractPoseFromMatrix(const vr::HmdMatrix34_t& vrMatrix, VRControllerData& data) {
        // Use Eigen transforms for more robust calculations
        Eigen::Vector3d position = VRTransforms::getPositionFromVRMatrix(vrMatrix);
        Eigen::Quaterniond quaternion = VRTransforms::getQuaternionFromVRMatrix(vrMatrix);
        
        // Store in controller data
        data.setPosition(position);
        data.setQuaternion(quaternion);
    }

    /**
     * @brief Calculate relative pose between two controller states using Eigen
     */
    static VRControllerData calculateRelativePose(const VRControllerData& initial, 
                                                  const VRControllerData& current) {
        VRControllerData relativePose;
        relativePose.role = current.role;  // Maintain controller role
        
        // Get poses as Eigen objects
        Eigen::Vector3d initialPos = initial.getPosition();
        Eigen::Quaterniond initialQuat = initial.getQuaternion();
        Eigen::Vector3d currentPos = current.getPosition();
        Eigen::Quaterniond currentQuat = current.getQuaternion();
        
        // Calculate relative position and rotation
        Eigen::Vector3d relativePos = currentPos - initialPos;
        Eigen::Quaterniond relativeQuat = initialQuat.inverse() * currentQuat;
        
        // Rotate relative position by inverse of initial quaternion
        Eigen::Vector3d rotatedRelativePos = initialQuat.inverse() * relativePos;
        
        // Store results
        relativePose.setPosition(rotatedRelativePos);
        relativePose.setQuaternion(relativeQuat);
        
        // Copy button states
        relativePose.menu_button = current.menu_button;
        relativePose.trigger_button = current.trigger_button;
        relativePose.trackpad_touch = current.trackpad_touch;
        relativePose.trackpad_button = current.trackpad_button;
        relativePose.grip_button = current.grip_button;
        relativePose.trackpad_x = current.trackpad_x;
        relativePose.trackpad_y = current.trackpad_y;
        relativePose.trigger = current.trigger;
        relativePose.time = current.time;
        
        return relativePose;
    }

    /**
     * @brief Apply low-pass filter to controller data using Eigen
     */
    static VRControllerData filterPose(const VRControllerData& current, 
                                       const VRControllerData& previous, 
                                       double alpha = 0.8) {
        VRControllerData filtered = current;  // Copy all data first
        
        // Filter position
        Eigen::Vector3d filteredPos = VRTransforms::filterPosition(
            current.getPosition(), previous.getPosition(), alpha);
        
        // Filter quaternion using SLERP
        Eigen::Quaterniond filteredQuat = VRTransforms::filterQuaternion(
            current.getQuaternion(), previous.getQuaternion(), alpha);
        
        // Store filtered values
        filtered.setPosition(filteredPos);
        filtered.setQuaternion(filteredQuat);
        
        return filtered;
    }
};

#endif // VRUTILS_HPP
