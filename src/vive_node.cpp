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
  while (true) {
    // deviceConnectionCheck(pHMD);
    controllerConnectionCheck(pHMD);
    sleep(1);
  }
}


int main(int argc, char ** argv) {
  if ( !initVR() ) {
    // Failed to initialize VR runtime
    shutdownVR();
  }

  // Main loop
  runVR();

}
