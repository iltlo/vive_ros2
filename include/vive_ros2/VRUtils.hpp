#ifndef VRUTILS_HPP
#define VRUTILS_HPP

#include <string>
#include <iostream>
#include <cmath> // for std::sqrt, std::fmax, std::atan2, std::asin, std::abs, M_PI
#include <openvr.h>
#include "json.hpp"

struct VRControllerData {
    double pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw;
    bool menu_button, trigger_button, trackpad_touch, trackpad_button, grip_button;
    double trackpad_x, trackpad_y;
    double trigger;
    int role;  // 1 for left, 2 for right
    std::string time;
};

enum LogLevel {
    Info,
    Debug,
    Error
};

// Logging function
inline void logMessage(LogLevel level, const std::string& message) {
    switch (level) {
        case Info:
            std::cout << "[INFO] " << message << std::endl;
            break;
        case Debug:
            std::cout << "[DEBUG] " << message << std::endl;
            break;
        case Error:
            std::cerr << "[ERROR] " << message << std::endl;
            break;
    }
}

// Define a structure for Euler angles
struct EulerAngle {
    float x; // Roll
    float y; // Pitch
    float z; // Yaw
};


class VRUtils {
public:
    static void resetJsonData(VRControllerData& data) {
        data.pose_x = 0.0;
        data.pose_y = 0.0;
        data.pose_z = 0.0;
        data.pose_qx = 0.0;
        data.pose_qy = 0.0;
        data.pose_qz = 0.0;
        data.pose_qw = 0.0;
        data.menu_button = false;
        data.trigger_button = false;
        data.trackpad_touch = false;
        data.trackpad_button = false;
        data.grip_button = false;
        data.trackpad_x = 0.0;
        data.trackpad_y = 0.0;
        data.trigger = 0.0;
        data.role = 1;
        data.time = "";
    }

    static bool deviceIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex) {
        return pHMD->IsTrackedDeviceConnected(unDeviceIndex);
    }

    static bool controllerIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex) {
        return pHMD->GetTrackedDeviceClass(unDeviceIndex) == vr::TrackedDeviceClass_Controller;
    }

    static void deviceConnectionCheck(vr::IVRSystem* pHMD) {
        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            if (deviceIsConnected(pHMD, i)) {
                vr::ETrackedDeviceClass trackedDeviceClass = pHMD->GetTrackedDeviceClass(i);
                logMessage(Debug, "[CONNECTED DEVICE " + std::to_string(i) + "]: class " + std::to_string(trackedDeviceClass));
            }
        }
    }

    static vr::ETrackedControllerRole controllerRoleCheck(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t i) {
        vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;
        if (controllerIsConnected(pHMD, i)) {
            controllerRole = pHMD->GetControllerRoleForTrackedDeviceIndex(i);
            if (controllerRole != vr::TrackedControllerRole_Invalid) {
                if (controllerRole == vr::TrackedControllerRole_LeftHand) {
                    logMessage(Debug, "[CONNECTED CONTROLLER " + std::to_string(i) + "]: role Left");
                } else if (controllerRole == vr::TrackedControllerRole_RightHand) {
                    logMessage(Debug, "[CONNECTED CONTROLLER " + std::to_string(i) + "]: role Right");
                } else {
                    logMessage(Debug, "[CONNECTED CONTROLLER " + std::to_string(i) + "]: role " + std::to_string(controllerRole));
                }
            }
        }
        return controllerRole;
    }

    static void controllerConnectionCheck(vr::IVRSystem* pHMD) {
        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            controllerRoleCheck(pHMD, i);
        }
    }

    static void HapticFeedback(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unControllerDeviceIndex, unsigned short durationMicroSec) {
        pHMD->TriggerHapticPulse(unControllerDeviceIndex, 0, durationMicroSec);
    }
};

class VRTransformUtils {
public:
    static vr::HmdQuaternion_t GetQuaternion(const vr::HmdMatrix34_t& m) {
        vr::HmdQuaternion_t q;

        q.w = sqrt(std::fmax(0, 1 + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2;
        q.x = sqrt(std::fmax(0, 1 + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2;
        q.y = sqrt(std::fmax(0, 1 - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2;
        q.z = sqrt(std::fmax(0, 1 - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2;
        q.x = copysign(q.x, m.m[2][1] - m.m[1][2]);
        q.y = copysign(q.y, m.m[0][2] - m.m[2][0]);
        q.z = copysign(q.z, m.m[1][0] - m.m[0][1]);

        return q;
    }

    static vr::HmdVector3_t GetPosition(const vr::HmdMatrix34_t& m) {
        vr::HmdVector3_t vector;

        vector.v[0] = m.m[0][3];
        vector.v[1] = m.m[1][3];
        vector.v[2] = m.m[2][3];

        return vector;
    }

    static EulerAngle QuaternionToEulerXYZ(const vr::HmdQuaternion_t& q) {
        EulerAngle angles;

        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        angles.x = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1)
            angles.y = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.y = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        angles.z = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }
};

#endif // VRUTILS_HPP
