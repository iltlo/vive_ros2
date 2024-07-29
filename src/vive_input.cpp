#include <string>
#include <openvr.h>
#include <chrono> // Include this for std::chrono
#include <thread>

#include "VRUtils.hpp"
#include "json.hpp" // Include nlohmann/json
#include "server.hpp"


class ViveInput {
public:
    ViveInput(std::mutex &mutex, std::condition_variable &cv, JsonData &data);
    ~ViveInput();
    void runVR();

private:
    vr::IVRSystem *pHMD = nullptr;
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

    std::mutex &data_mutex;
    std::condition_variable &data_cv;
    JsonData &shared_data;

    bool initVR();
    bool shutdownVR();
};

ViveInput::ViveInput(std::mutex &mutex, std::condition_variable &cv, JsonData &data) 
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
          VRUtils::controllerRoleCheck(pHMD, i);

          // Get the pose of the device
          vr::HmdMatrix34_t steamVRMatrix = trackedDevicePose[i].mDeviceToAbsoluteTracking;
          vr::HmdVector3_t position = VRTransformUtils::GetPosition(steamVRMatrix);
          vr::HmdQuaternion_t quaternion = VRTransformUtils::GetQuaternion(steamVRMatrix);
          EulerAngle euler = VRTransformUtils::QuaternionToEulerXYZ(quaternion);
          logMessage(Debug, "[POSE CM]: " + std::to_string(position.v[0] * 100) + " " + std::to_string(position.v[1] * 100) + " " + std::to_string(position.v[2] * 100));
          logMessage(Debug, "[EULER DEG]: " + std::to_string(euler.x * (180.0 / M_PI)) + " " + std::to_string(euler.y * (180.0 / M_PI)) + " " + std::to_string(euler.z * (180.0 / M_PI)) + "\n");

          // Update shared data
          {
            std::lock_guard<std::mutex> lock(data_mutex);
            shared_data.pose_x = position.v[0];
            shared_data.pose_y = position.v[1];
            shared_data.pose_z = position.v[2];
            shared_data.pose_qx = quaternion.x;
            shared_data.pose_qy = quaternion.y;
            shared_data.pose_qz = quaternion.z;
            shared_data.pose_qw = quaternion.w;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Sleep to reduce CPU usage
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Sleep to maintain loop timing
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
    JsonData shared_data;

    Server server(2048, data_mutex, data_cv, shared_data);
    std::thread serverThread(&Server::start, &server);
    serverThread.detach(); // Detach the server thread

    ViveInput vive_input(data_mutex, data_cv, shared_data);
    vive_input.runVR();

    return 0;
}
