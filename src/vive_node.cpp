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
#include "rclcpp/rclcpp.hpp"

// Define log levels
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

// Function to extract quaternion from the matrix
vr::HmdQuaternion_t GetQuaternion(vr::HmdMatrix34_t m) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + m.m[0][0] + m.m[1][1] + m.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + m.m[0][0] - m.m[1][1] - m.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - m.m[0][0] + m.m[1][1] - m.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - m.m[0][0] - m.m[1][1] + m.m[2][2])) / 2;
    q.x = copysign(q.x, m.m[2][1] - m.m[1][2]);
    q.y = copysign(q.y, m.m[0][2] - m.m[2][0]);
    q.z = copysign(q.z, m.m[1][0] - m.m[0][1]);

    return q;
}
// Function to extract position from the matrix
vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t m) {
    vr::HmdVector3_t vector;

    vector.v[0] = m.m[0][3];
    vector.v[1] = m.m[1][3];
    vector.v[2] = m.m[2][3];

    return vector;
}
// Define a structure for Euler angles
struct EulerAngle {
    float x; // Pitch
    float y; // Yaw
    float z; // Roll
};
// Function to convert quaternion to Euler angles (XYZ)
EulerAngle QuaternionToEulerXYZ(const vr::HmdQuaternion_t& q) {
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

bool isConnected(vr::TrackedDeviceIndex_t unDeviceIndex)
{
	return vr::VRSystem()->IsTrackedDeviceConnected(unDeviceIndex);
}
bool controllerIsConnected(vr::TrackedDeviceIndex_t unDeviceIndex)
{
	return vr::VRSystem()->GetTrackedDeviceClass(unDeviceIndex) == vr::TrackedDeviceClass_Controller;
}
// check if device is connected
void deviceConnectionCheck( vr::IVRSystem *m_pHMD ){
	for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
	{
		if (isConnected(i))
		{
			vr::ETrackedDeviceClass trackedDeviceClass = m_pHMD->GetTrackedDeviceClass(i);
      logMessage(Debug, "[CONNECTED DEVICE " + std::to_string(i) + "]: class " + std::to_string(trackedDeviceClass));
      // Device class 0: no device, 1: HMD, 2: controller, 3: generic tracker, 4: lighthouse base station
			
      // m_rDevClassChar[i] = (char)trackedDeviceClass;
		}
	}
  printf("\n");
}
// check if controller is connected
void controllerConnectionCheck( vr::IVRSystem *m_pHMD ){
	for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
		if (controllerIsConnected(i)) {
			vr::ETrackedControllerRole controllerRole = m_pHMD->GetControllerRoleForTrackedDeviceIndex(i);
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

vr::IVRSystem *pHMD;
vr::EVRInitError eError = vr::VRInitError_None;
vr::InputPoseActionData_t poseData;
vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

bool initVR() {
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
bool shutdownVR() {
  if (pHMD) {
    logMessage(Info, "Shutting down VR runtime");
    vr::VR_Shutdown();
  }
  return true;
}

// the runVR function should be a ROS2 loop that publishes the pose of the HMD and controllers
void runVR() {
  while (rclcpp::ok()) {
    double tf_matrix[3][4];

    // deviceConnectionCheck(pHMD);
    // controllerConnectionCheck(pHMD);

    // update the poses
    pHMD->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, trackedDevicePose, vr::k_unMaxTrackedDeviceCount);
    // or vr::TrackingUniverseRawAndUncalibrated

    for (int i = 0; i <  vr::k_unMaxTrackedDeviceCount; i++) {
      if (trackedDevicePose[i].bDeviceIsConnected && trackedDevicePose[i].bPoseIsValid
        && trackedDevicePose[i].eTrackingResult == vr::TrackingResult_Running_OK) {
        // Get the pose of the device
        vr::HmdMatrix34_t steamVRMatrix = trackedDevicePose[i].mDeviceToAbsoluteTracking;
        vr::HmdVector3_t position = GetPosition(steamVRMatrix);
        vr::HmdQuaternion_t quaternion = GetQuaternion(steamVRMatrix);
        EulerAngle euler = QuaternionToEulerXYZ(quaternion);
        logMessage(Debug, "[POSE CM]: " + std::to_string(position.v[0] * 100) + " " + std::to_string(position.v[1] * 100) + " " + std::to_string(position.v[2] * 100));
        logMessage(Debug, "[EULER DEG]: " + std::to_string(euler.x * (180.0 / M_PI)) + " " + std::to_string(euler.y * (180.0 / M_PI)) + " " + std::to_string(euler.z * (180.0 / M_PI)) + "\n");

        // TODO: create a function to broadcast the pose to ROS2 using TF

      }
    }

    sleep(0.05);
  }
}


int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  if ( !initVR() ) {
    // Failed to initialize VR runtime
    shutdownVR();
  }

  // Main loop
  runVR();

}
