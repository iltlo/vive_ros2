cmake_minimum_required(VERSION 3.5)
project(vive_ros2)

if(NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -Wpedantic")
endif()
 
# -----------------------------------------------------------------------------
## SYSTEM PROPERTIES ##
if(NOT PLATFORM)
  if(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(PLATFORM 64)
  else()
    set(PLATFORM 32)
  endif()
endif()
message(STATUS "Compilation set for ${PLATFORM}bits architectures.")

# set(SDL_REQUIRED_LIBRARIES SDL2)
find_package(SDL2 REQUIRED)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  add_definitions(-DLINUX -DPOSIX)
  set(ARCH_TARGET linux64)

  if(${PLATFORM} MATCHES 32)
    message(WARNING "OpenVR x86 binaries not provided on GNU/Linux.")
  endif()
endif()

 # -----------------------------------------------------------------------------
 ## PATHS ##
#  set(OPENVR "${CMAKE_CURRENT_SOURCE_DIR}/../../openvr")
 set(OPENVR ${PROJECT_SOURCE_DIR}/../../libraries/openvr)
 add_definitions(-DOPENVR_PATH="${OPENVR}")
 set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
 set(SHARED_SRC_DIR ${OPENVR}/samples/shared)

 if(CMAKE_HOST_UNIX)
   find_file(OPENVRPATHS openvrpaths.vrpath PATHS $ENV{HOME}/.config/openvr "$ENV{HOME}/Library/Application Support/OpenVR/.openvr")
   if(${OPENVRPATHS} MATCHES OPENVRPATHS-NOTFOUND)
     message(FATAL_ERROR "${OPENVRPATHS} Please install SteamVR SDK to continue..")
   endif()
 endif()
 # -----------------------------------------------------------------------------
 ## COMPILER DETECTION ##
 if(   (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
   # Better to use the prebuilt GNU preprocessor define __GNUC__,
   # kept for legacy reason with the sample code.
   add_definitions(-DGNUC)
 
   set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++17 -include ${SHARED_SRC_DIR}/compat.h")
   set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
   set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
 
   # Handles x86 compilation support on x64 arch.
   if(${PLATFORM} MATCHES 32)
     set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} -m32")
     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")
   endif()
 else()
   message(FATAL_ERROR "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'")
 endif()
# -----------------------------------------------------------------------------
## LIBRARIES ##
## OpenGL / GLU
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
 
## GLEW 1.11
find_package(GLEW 1.11 REQUIRED)
 
## SDL 2
foreach(lib ${SDL_REQUIRED_LIBRARIES})
  find_library(${lib}_LIBRARY
    NAMES
      ${lib}
    PATHS
      ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
    PATH_SUFFIXES
      linux${PLATFORM}
      # ${WINDOWS_PATH_SUFFIXES}
  )
  list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
endforeach()
set(SDL2_INCLUDE_DIR ${THIRDPARTY_DIR}/sdl2-2.0.3/include)
 
## Vulkan
if(NOT (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
  find_library(VULKAN_LIBRARY
      NAMES
        vulkan-1
        vulkan
      PATHS
        ${THIRDPARTY_DIR}/vulkan-1.0.49.0/lib
      PATH_SUFFIXES
        linux${PLATFORM}
        ${WINDOWS_PATH_SUFFIXES}
      NO_DEFAULT_PATH
  )
  set(VULKAN_INCLUDE_DIR ${THIRDPARTY_DIR}/vulkan-1.0.49.0/include)
endif()

## find openvr
## OpenVR API path
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES
    linux64
  # NO_DEFAULT_PATH
  # NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers ${OPENVR}/samples)
# -----------------------------------------------------------------------------
## SHARED SOURCES ##
file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)
 
 
if(NOT (${CMAKE_SYSTEM_NAME} MATCHES "Darwin"))
  include_directories(${VULKAN_INCLUDE_DIR})
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT PLATFORM)
  if(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(PLATFORM 64)
  else()
    set(PLATFORM 32)
  endif()
endif()
message(STATUS "Compilation set for ${PLATFORM}bits architectures.")
 
set(SDL_REQUIRED_LIBRARIES SDL2)
 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages for building the project
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(std_srvs REQUIRED)

# Include directories for the project
include_directories(
  .
  ${OPENGL_INCLUDE_DIR}
  ${GLEW_INCLUDE_DIR}
  ${SDL2_INCLUDE_DIR}
  ${QT_INCLUDE_DIRS}
  ${OPENVR_INCLUDE_DIR}
  ${VULKAN_INCLUDE_DIR}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

# Define the executable target and its source files
add_executable(vive_example
  ${SHARED_SRC_FILES}
  src/hellovr_opengl_main.cpp
)

# Link the executable with necessary libraries
target_link_libraries(vive_example
  ${OPENGL_LIBRARIES}
  ${GLEW_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
  ${rclcpp_LIBRARIES}
  ${rmw_implementation_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)

# Install the executable and launch directory
install(TARGETS vive_example DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Specify dependencies for the ament build system
ament_target_dependencies(vive_example rclcpp std_msgs geometry_msgs sensor_msgs)

# Finalize the ament package
ament_package()