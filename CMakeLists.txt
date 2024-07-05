
cmake_minimum_required(VERSION 3.5)
project(vivetest)
 
if(NOT WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
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
set(SDL_REQUIRED_LIBRARIES SDL2)
 set(SDL_REQUIRED_LIBRARIES ${SDL_REQUIRED_LIBRARIES} SDL2main)
 add_definitions(-D_WIN32)
 set(ARCH_TARGET win${PLATFORM})
 # Binaries path for thirdparties are not generics so we try to guess their suffixes.
 set(WINDOWS_PATH_SUFFIXES win${PLATFORM} Win${PLATFORM} x${PLATFORM})
 if(${PLATFORM} MATCHES 64)
   message(WARNING "SDL x64 runtime binaries not provided on Windows.")
 endif()
 # -----------------------------------------------------------------------------
 ## PATHS ##
 set(THIRDPARTY_DIR C:/openvr-master/samples/thirdparty)
 set(SHARED_SRC_DIR C:/openvr-master/samples/shared)
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
 
   set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
   set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
   set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
 
   # Handles x86 compilation support on x64 arch.
   if(${PLATFORM} MATCHES 32)
     set(CMAKE_CXX_FLAGS        "${CMAKE_CXX_FLAGS} -m32")
     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -m32")
   endif()
 elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
   set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} /W2 /DEBUG")
   set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MP /INCREMENTAL:NO")
 else()
   message(FATAL_ERROR "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'")
 endif()
# -----------------------------------------------------------------------------
## LIBRARIES ##
## OpenGL / GLU
find_package(OpenGL REQUIRED)
 
## GLEW 1.11
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # On GNU/Linux, glew can be found on the package manager.
  find_package(GLEW 1.11 REQUIRED)
else()
  # Otherwise, use the shipped binaries.
  find_library(GLEW_LIBRARIES
    NAMES
      GLEW
      glew32
    PATHS
      ${THIRDPARTY_DIR}/glew/glew-1.11.0/lib/Release
    PATH_SUFFIXES
      osx32
      ${WINDOWS_PATH_SUFFIXES}
  )
  set(GLEW_INCLUDE_DIR ${THIRDPARTY_DIR}/glew/glew-1.11.0/include)
endif()
 
## SDL 2
## osx32 and linux32 cmake can spot each others' .so and .a files, so keep them apart
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  foreach(lib ${SDL_REQUIRED_LIBRARIES})
    find_library(${lib}_LIBRARY
      NAMES
        ${lib}
      PATHS
        ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
      PATH_SUFFIXES
        osx32
    )
    list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
  endforeach()
else()
  foreach(lib ${SDL_REQUIRED_LIBRARIES})
    find_library(${lib}_LIBRARY
      NAMES
        ${lib}
      PATHS
        ${THIRDPARTY_DIR}/sdl2-2.0.3/bin
      PATH_SUFFIXES
        linux${PLATFORM}
        ${WINDOWS_PATH_SUFFIXES}
    )
    list(APPEND SDL2_LIBRARIES ${${lib}_LIBRARY})
  endforeach()
endif()
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
## OpenVR API path
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    C:/openvr-master/bin
    C:/openvr-master/lib
  PATH_SUFFIXES
    osx32
    linux64
    ${WINDOWS_PATH_SUFFIXES}
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR C:/openvr-master/headers)
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
  set(CMAKE_CXX_STANDARD 14)
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
 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
 
include_directories(
        .
        ${OPENGL_INCLUDE_DIR}
        ${GLEW_INCLUDE_DIR}
        ${SDL2_INCLUDE_DIR}
        ${QT_INCLUDE_DIRS}
        ${OPENVR_INCLUDE_DIR}
        ${geometry_msgs_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
)
add_executable(vive
        ${SHARED_SRC_FILES}
        main.cpp
        )
target_link_libraries(vive
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
ament_target_dependencies(vive rclcpp std_msgs geometry_msgs sensor_msgs)
 
install(TARGETS
  vive
  DESTINATION lib/${PROJECT_NAME}
)
 
ament_package()