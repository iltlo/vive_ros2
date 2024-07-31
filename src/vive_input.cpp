#include <string>
#include <openvr.h>
#include <chrono> // Include this for std::chrono
#include <thread>

#include "VRUtils.hpp"
#include "json.hpp" // Include nlohmann/json
#include "server.hpp"


class ViveInput {
public:
    ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data);
    ~ViveInput();
    void runVR();

private:
    vr::IVRSystem *pHMD = nullptr;
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

    std::mutex &data_mutex;
    std::condition_variable &data_cv;
    VRControllerData &shared_data;
    VRControllerData local_data;

    bool initVR();
    bool shutdownVR();
};

ViveInput::ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data) 
    : data_mutex(mutex), data_cv(cv), shared_data(data) {
    if (!initVR()) {
        shutdownVR();
        throw std::runtime_error("Failed to initialize VR");
    }
}
ViveInput::~ViveInput() {
    shutdownVR();
}

void ViveInput::runVR() {
  logMessage(Info, "Starting VR loop");
  auto lastLogTime = std::chrono::steady_clock::now(); // Initialize the last log time

  while (true) {
    bool controllerDetected = false;
    VRUtils::resetJsonData(local_data);

    // VRUtils::deviceConnectionCheck(pHMD);
    // VRUtils::controllerConnectionCheck(pHMD);

    // update the poses
    pHMD->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, trackedDevicePose, vr::k_unMaxTrackedDeviceCount);

    for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
      if (trackedDevicePose[i].bDeviceIsConnected && trackedDevicePose[i].bPoseIsValid
          && trackedDevicePose[i].eTrackingResult == vr::TrackingResult_Running_OK) {

        if (VRUtils::controllerIsConnected(pHMD, i)) {
          controllerDetected = true; // Mark that a controller was detected
          // controllerRoleCheck(i);
          // VRUtils::controllerRoleCheck(pHMD, i);

          // Get the pose of the device
          vr::HmdMatrix34_t steamVRMatrix = trackedDevicePose[i].mDeviceToAbsoluteTracking;
          vr::HmdVector3_t position = VRTransformUtils::GetPosition(steamVRMatrix);
          vr::HmdQuaternion_t quaternion = VRTransformUtils::GetQuaternion(steamVRMatrix);
          EulerAngle euler = VRTransformUtils::QuaternionToEulerXYZ(quaternion);
          logMessage(Debug, "[POSE CM]: " + std::to_string(position.v[0] * 100) + " " + std::to_string(position.v[1] * 100) + " " + std::to_string(position.v[2] * 100));
          logMessage(Debug, "[EULER DEG]: " + std::to_string(euler.x * (180.0 / M_PI)) + " " + std::to_string(euler.y * (180.0 / M_PI)) + " " + std::to_string(euler.z * (180.0 / M_PI)) + "\n");

          local_data.time = Server::getCurrentTimeWithMilliseconds();
          local_data.role = VRUtils::controllerRoleCheck(pHMD, i);
          local_data.pose_x = position.v[0];
          local_data.pose_y = position.v[1] - 0.6;
          local_data.pose_z = position.v[2];
          local_data.pose_qx = quaternion.x;
          local_data.pose_qy = quaternion.y;
          local_data.pose_qz = quaternion.z;
          local_data.pose_qw = quaternion.w;

          vr::VRControllerState_t controllerState;
          pHMD->GetControllerState(i, &controllerState, sizeof(controllerState));
          if ((1LL << vr::k_EButton_ApplicationMenu) & controllerState.ulButtonPressed){
            logMessage(Debug, "Application Menu button pressed");
            local_data.menu_button = true;
            VRUtils::HapticFeedback(pHMD, i, 200);
          }
          if ((1LL << vr::k_EButton_SteamVR_Trigger) & controllerState.ulButtonPressed) {
            logMessage(Debug, "Trigger button pressed");
            local_data.trigger_button = true;
            // VRUtils::HapticFeedback(pHMD, i, 500);
          }
          if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonPressed) {
            logMessage(Debug, "Touchpad button pressed");
            local_data.trackpad_button = true;
            VRUtils::HapticFeedback(pHMD, i, 200);
          }
          if ((1LL << vr::k_EButton_Grip) & controllerState.ulButtonPressed){
            logMessage(Debug, "Grip button pressed");
            local_data.grip_button = true;
            // VRUtils::HapticFeedback(pHMD, i, 10);
          }
          if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonTouched) {
            logMessage(Debug, "Touchpad button touched");
            local_data.trackpad_touch = true;
          }

          logMessage(Debug, "Trackpad: " + std::to_string(controllerState.rAxis[0].x) + " " + std::to_string(controllerState.rAxis[0].y));
          local_data.trackpad_x = controllerState.rAxis[0].x;
          local_data.trackpad_y = controllerState.rAxis[0].y;

          const int numSteps = 6;
          const float stepSize = 1.0f / numSteps;
          static int previousStep = -1; // Initialize previous step to an invalid value
          // Get the current trigger value
          float triggerValue = controllerState.rAxis[1].x;
          int currentStep = static_cast<int>(triggerValue / stepSize);
          logMessage(Debug, "Trigger: " + std::to_string(triggerValue));
          local_data.trigger = triggerValue;
          // Check if the trigger value has crossed a new step
          if (currentStep != previousStep) {
              int vibrationDuration = static_cast<int>(triggerValue * 2000); // Scale to a max of 2000 ms
              VRUtils::HapticFeedback(pHMD, i, vibrationDuration);
              previousStep = currentStep;
          }

          // Update shared data
          {
            std::lock_guard<std::mutex> lock(data_mutex);
            shared_data = local_data;
            shared_data.time = Server::getCurrentTimeWithMilliseconds();
          }
          data_cv.notify_one(); // Notify the server thread
        }
      }
    }

    auto currentTime = std::chrono::steady_clock::now();
    if (!controllerDetected) {
      if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastLogTime).count() >= 1) {
        // logMessage(Info, "[time] no controller detected");
        logMessage(Info, "no controller detected, currentTime: " + std::to_string(std::chrono::duration_cast<std::chrono::seconds>(currentTime.time_since_epoch()).count()));
        lastLogTime = currentTime; // Update the last log time
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20Hz
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20)); // ~50Hz
      lastLogTime = currentTime;
    }
  }
}

bool ViveInput::initVR() {
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
bool ViveInput::shutdownVR() {
    // Shutdown VR runtime
    if (pHMD) {
        logMessage(Info, "Shutting down VR runtime");
        vr::VR_Shutdown();
    }
    return true;
}

int main(int argc, char **argv) {
    std::mutex data_mutex;
    std::condition_variable data_cv;
    VRControllerData shared_data;

    Server server(2048, data_mutex, data_cv, shared_data);
    std::thread serverThread(&Server::start, &server);
    serverThread.detach(); // Detach the server thread

    ViveInput vive_input(data_mutex, data_cv, shared_data);
    vive_input.runVR();

    return 0;
}
