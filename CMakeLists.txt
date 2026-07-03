# Copyright (c) 2026 김승연
# MIT License - See LICENSE file for details

cmake_minimum_required(VERSION 3.16)
project(ecowarden_lidar_module
  VERSION   2.0.0
  LANGUAGES CXX
)

add_compile_options(-D__SANE_USERSPACE_TYPES__)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RPLIDAR_SDK_DIR ${CMAKE_SOURCE_DIR}/thirdparty/rplidar_sdk)

set(RPLIDAR_SDK_SOURCES
  ${RPLIDAR_SDK_DIR}/sdk/src/rplidar_driver.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/sl_async_transceiver.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/sl_crc.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/sl_lidar_driver.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/sl_lidarprotocol_codec.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/sl_serial_channel.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/sl_tcp_channel.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/sl_udp_channel.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/hal/thread.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/dataunpacker/dataunpacker.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/dataunpacker/unpacker/handler_capsules.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/dataunpacker/unpacker/handler_hqnode.cpp
  ${RPLIDAR_SDK_DIR}/sdk/src/dataunpacker/unpacker/handler_normalnode.cpp
)

if(APPLE)
    file(GLOB RPLIDAR_ARCH_SOURCES ${RPLIDAR_SDK_DIR}/sdk/src/arch/macOS/*.cpp)
    add_compile_definitions(_MACOS)
elseif(UNIX)
    file(GLOB RPLIDAR_ARCH_SOURCES ${RPLIDAR_SDK_DIR}/sdk/src/arch/linux/*.cpp)
    add_compile_definitions(_LINUX)
endif()

list(APPEND RPLIDAR_SDK_SOURCES ${RPLIDAR_ARCH_SOURCES})

set(RPLIDAR_SDK_INCLUDE_DIRS
  ${RPLIDAR_SDK_DIR}/sdk/include
  ${RPLIDAR_SDK_DIR}/sdk/src
)

if(EXISTS ${RPLIDAR_SDK_DIR}/sdk/src)
    add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SOURCES})
    target_include_directories(rplidar_sdk PUBLIC ${RPLIDAR_SDK_INCLUDE_DIRS})
    target_link_libraries(rplidar_sdk PUBLIC pthread)
endif()

find_package(CURL REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV QUIET)
if(OpenCV_FOUND)
  add_definitions(-DUSE_OPENCV)
endif()

# ── libgpiod: RPi5 GPIO 지원 (sysfs 대체) ──────────────────────────
option(USE_GPIOD "Use libgpiod for GPIO access (required for RPi5)" ON)
if(USE_GPIOD)
  find_library(GPIOD_LIB gpiod)
  find_path(GPIOD_INCLUDE gpiod.h)
  if(GPIOD_LIB AND GPIOD_INCLUDE)
    add_definitions(-DUSE_GPIOD)
    message(STATUS "libgpiod found: ${GPIOD_LIB}")
  else()
    message(STATUS "libgpiod not found — PIR will use sysfs fallback (install: apt install libgpiod-dev)")
    set(USE_GPIOD OFF)
  endif()
endif()

# ── nlohmann/json: apt 패키지 우선, 없으면 FetchContent 폴백 ────────
find_package(nlohmann_json 3.11 QUIET)
if(NOT nlohmann_json_FOUND)
  message(STATUS "nlohmann_json not found via apt, fetching from GitHub...")
  include(FetchContent)
  FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG        v3.11.3
    GIT_SHALLOW    TRUE
  )
  FetchContent_MakeAvailable(nlohmann_json)
else()
  message(STATUS "nlohmann_json found via system package")
endif()

set(APP_SOURCES
  src/rplidar_s2.cpp
  src/scan_processor.cpp
  src/background_filter.cpp
  src/cluster_tracker.cpp
  src/event_notifier.cpp
  src/udp_sender.cpp
  src/json_packet.cpp
  src/pir_sensor.cpp
  src/camera_module.cpp
  src/servo_controller.cpp
  src/main.cpp
)

add_executable(rplidar_app ${APP_SOURCES})
target_include_directories(rplidar_app PRIVATE ${CMAKE_SOURCE_DIR}/include ${RPLIDAR_SDK_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS})
target_link_libraries(rplidar_app PRIVATE rplidar_sdk CURL::libcurl nlohmann_json::nlohmann_json Threads::Threads)

if(OpenCV_FOUND)
  target_include_directories(rplidar_app PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(rplidar_app PRIVATE ${OpenCV_LIBS})
endif()

if(USE_GPIOD)
  target_link_libraries(rplidar_app PRIVATE ${GPIOD_LIB})
endif()

add_executable(check_pir test/test_pir.cpp src/pir_sensor.cpp)
target_include_directories(check_pir PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(check_pir PRIVATE pthread)
if(USE_GPIOD)
  target_link_libraries(check_pir PRIVATE ${GPIOD_LIB})
endif()

add_executable(check_lidar test/test_lidar.cpp src/rplidar_s2.cpp)
target_include_directories(check_lidar PRIVATE ${CMAKE_SOURCE_DIR}/include ${RPLIDAR_SDK_INCLUDE_DIRS})
target_link_libraries(check_lidar PRIVATE rplidar_sdk Threads::Threads)

add_executable(check_camera test/test_camera.cpp src/camera_module.cpp)
target_include_directories(check_camera PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(check_camera PRIVATE Threads::Threads)
if(OpenCV_FOUND)
  target_include_directories(check_camera PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(check_camera PRIVATE ${OpenCV_LIBS})
endif()

add_executable(servo_sweep_test
  test/servo_sweep_test.cpp
  src/servo_controller.cpp
  src/camera_module.cpp
)
target_include_directories(servo_sweep_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(servo_sweep_test PRIVATE Threads::Threads)
if(OpenCV_FOUND)
  target_include_directories(servo_sweep_test PRIVATE ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(servo_sweep_test PRIVATE ${OpenCV_LIBS})
endif()

# 카메라/LiDAR 의존 0 — sysfs PWM만 직접 두드리는 최소 서보 테스트
add_executable(servo_only_test test/servo_only_test.cpp)
target_include_directories(servo_only_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(servo_only_test PRIVATE Threads::Threads)

add_executable(rplidar_sim
  test/sim_main.cpp
  src/scan_processor.cpp
  src/background_filter.cpp
  src/cluster_tracker.cpp
)
target_include_directories(rplidar_sim PRIVATE ${CMAKE_SOURCE_DIR}/include)
