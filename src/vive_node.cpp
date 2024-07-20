#include <SDL.h>
#include <GL/glew.h>
#include <SDL_opengl.h>
#include <GL/glu.h>
#include <stdio.h>
#include <string>
#include <cstdlib>

#include <openvr.h>

#include "shared/lodepng.h"
#include "shared/Matrices.h"
#include "shared/pathtools.h"

#include <shared/compat.h>
#include <unistd.h>		// for sleep
#include <cmath>		// for M_PI

#include <boost/function.hpp>

#include <cstdio>

// ros2 include
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2/LinearMath/Quaternion.h"

enum LogLevel {
    Info,
    Debug,
    Error
};
// Logging function
void logMessage(LogLevel level, const std::string& message) {
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
    float x; // Pitch
    float y; // Yaw
    float z; // Roll
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

class VIVENode : public rclcpp::Node {
public:
    VIVENode();
    ~VIVENode();
    void runVR();

private:
    vr::IVRSystem *pHMD = nullptr;
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    bool initVR();
    bool shutdownVR();
    bool deviceIsConnected(vr::TrackedDeviceIndex_t unDeviceIndex);
    bool controllerIsConnected(vr::TrackedDeviceIndex_t unDeviceIndex);
    void deviceConnectionCheck();
    void controllerConnectionCheck();
    // void pubTFPose(int deviceIndex, const vr::HmdVector3_t& position, const vr::HmdQuaternion_t& quaternion);
};

VIVENode::VIVENode() : Node("vive_node") {
    // Constructor implementation...
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    if (!initVR()) {
        shutdownVR();
        throw std::runtime_error("Failed to initialize VR");
    }
}
VIVENode::~VIVENode() {
    shutdownVR();
}

void VIVENode::runVR() {
  logMessage(Info, "Starting VR loop");
  while (rclcpp::ok()) {
    deviceConnectionCheck();
    // controllerConnectionCheck();

    // update the poses
    pHMD->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, trackedDevicePose, vr::k_unMaxTrackedDeviceCount);
    // or vr::TrackingUniverseRawAndUncalibrated

    for (uint32_t i = 0; i <  vr::k_unMaxTrackedDeviceCount; i++) {
      if (trackedDevicePose[i].bDeviceIsConnected && trackedDevicePose[i].bPoseIsValid
        && trackedDevicePose[i].eTrackingResult == vr::TrackingResult_Running_OK) {
        // Get the pose of the device
        vr::HmdMatrix34_t steamVRMatrix = trackedDevicePose[i].mDeviceToAbsoluteTracking;
        vr::HmdVector3_t position = VRTransformUtils::GetPosition(steamVRMatrix);
        vr::HmdQuaternion_t quaternion = VRTransformUtils::GetQuaternion(steamVRMatrix);
        EulerAngle euler = VRTransformUtils::QuaternionToEulerXYZ(quaternion);
        logMessage(Debug, "[POSE CM]: " + std::to_string(position.v[0] * 100) + " " + std::to_string(position.v[1] * 100) + " " + std::to_string(position.v[2] * 100));
        logMessage(Debug, "[EULER DEG]: " + std::to_string(euler.x * (180.0 / M_PI)) + " " + std::to_string(euler.y * (180.0 / M_PI)) + " " + std::to_string(euler.z * (180.0 / M_PI)) + "\n");

        // TODO: create a function to broadcast the pose to ROS2 using TF
        // pubTFPose(i, position, quaternion);

      }
    }

    sleep(0.05);
  }
}

bool VIVENode::initVR() {
    // Initialize VR runtime
    eError = vr::VRInitError_None;
    pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);
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
bool VIVENode::shutdownVR() {
    // Shutdown VR runtime
    if (pHMD) {
        logMessage(Info, "Shutting down VR runtime");
        vr::VR_Shutdown();
    }
    return true;
}

bool VIVENode::deviceIsConnected(vr::TrackedDeviceIndex_t unDeviceIndex) {
	return vr::VRSystem()->IsTrackedDeviceConnected(unDeviceIndex);
}
bool VIVENode::controllerIsConnected(vr::TrackedDeviceIndex_t unDeviceIndex) {
	return vr::VRSystem()->GetTrackedDeviceClass(unDeviceIndex) == vr::TrackedDeviceClass_Controller;
}
void VIVENode::deviceConnectionCheck() {
  for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
    if (deviceIsConnected(i)) {
      vr::ETrackedDeviceClass trackedDeviceClass = pHMD->GetTrackedDeviceClass(i);
      logMessage(Debug, "[CONNECTED DEVICE " + std::to_string(i) + "]: class " + std::to_string(trackedDeviceClass));
    }
  }
}
void VIVENode::controllerConnectionCheck() {
  for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
    if (controllerIsConnected(i)) {
			vr::ETrackedControllerRole controllerRole = pHMD->GetControllerRoleForTrackedDeviceIndex(i);
      // Controller role 0: invalid, 1: left hand, 2: right hand
      if (controllerRole != vr::TrackedControllerRole_Invalid) {
        if (controllerRole == vr::TrackedControllerRole_LeftHand) {
          logMessage(Debug, "[CONNECTED CONTROLLER " + std::to_string(i) + "]: role Left");
        } else if (controllerRole == vr::TrackedControllerRole_RightHand) {
          logMessage(Debug, "[CONNECTED CONTROLLER " + std::to_string(i) + "]: role Right");
        } else {
          logMessage(Debug, "[CONNECTED CONTROLLER " + std::to_string(i) + "]: role " + std::to_string(controllerRole));
        }
        printf("\n");
      }
    }
  }
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto vive_node = std::make_shared<VIVENode>();
    vive_node->runVR();
    rclcpp::shutdown();
    return 0;
}
