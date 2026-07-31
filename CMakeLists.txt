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
  # dnn 모듈이 있어야 얼굴 마스킹 DNN 백엔드 / 객체 분류 AI 를 쓸 수 있다.
  # 없으면 Haar cascade + LiDAR 휴리스틱으로 자동 폴백한다.
  if("dnn" IN_LIST OpenCV_LIBS OR TARGET opencv_dnn
     OR "opencv_dnn" IN_LIST OpenCV_LIBS)
    add_definitions(-DHAVE_OPENCV_DNN)
    message(STATUS "OpenCV dnn 모듈 사용 가능 — 얼굴 마스킹 DNN / 객체 분류 AI 활성")
  else()
    message(STATUS "OpenCV dnn 모듈 없음 — Haar cascade + LiDAR 휴리스틱으로 동작")
  endif()
else()
  message(STATUS "OpenCV 없음 — 카메라·마스킹 경로는 스텁으로 컴파일됩니다")
endif()

# ── OpenSSL: 증거 원본 암호화 보관(프라이버시 3계층) ────────────────
#   없으면 EvidenceVault 가 fail-closed 로 동작한다 — 평문 보관은 하지 않는다.
find_package(OpenSSL QUIET)
if(OpenSSL_FOUND OR OPENSSL_FOUND)
  add_definitions(-DUSE_OPENSSL)
  message(STATUS "OpenSSL found: ${OPENSSL_VERSION} — 증거 원본 AES-256-GCM 암호화 활성")
else()
  message(WARNING "OpenSSL 없음 — 증거 원본 암호화 보관 비활성 (fail-closed: 원본을 저장하지 않음). 설치: apt install libssl-dev")
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

# 프라이버시·AI 모듈 — 카메라를 쓰는 모든 타깃이 함께 링크해야 한다
# (camera_module.cpp 가 FaceMasker / EvidenceVault 를 참조하므로).
set(PRIVACY_SOURCES
  src/face_masking.cpp
  src/evidence_vault.cpp
)

# ── 선택 의존성 일괄 적용 헬퍼 ──────────────────────────────────────
#
#   USE_OPENCV / USE_OPENSSL 는 add_definitions() 로 **전역** 정의되는데
#   include 경로와 라이브러리는 타깃별로 붙여야 한다. 이 비대칭 때문에
#   "정의는 켜졌는데 헤더 경로가 없어서" 컴파일이 깨지는 사고가 있었다
#   (v56: rplidar_sim 에 OpenCV 경로 누락 → object_classifier.cpp 빌드 실패.
#    OpenCV 가 없는 개발 머신에서는 스텁 경로로 빠져 재현되지 않았다).
#
#   아래 함수로 일원화해, 카메라·프라이버시 소스를 쓰는 타깃은 반드시
#   같은 방식으로 의존성을 받도록 한다.
function(ecowarden_apply_optional_deps tgt)
  if(OpenCV_FOUND)
    target_include_directories(${tgt} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${tgt} PRIVATE ${OpenCV_LIBS})
  endif()
  if(OpenSSL_FOUND OR OPENSSL_FOUND)
    target_link_libraries(${tgt} PRIVATE OpenSSL::Crypto)
  endif()
endfunction()

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
  src/object_classifier.cpp
  ${PRIVACY_SOURCES}
  src/main.cpp
)

add_executable(rplidar_app ${APP_SOURCES})
target_include_directories(rplidar_app PRIVATE ${CMAKE_SOURCE_DIR}/include ${RPLIDAR_SDK_INCLUDE_DIRS} ${CURL_INCLUDE_DIRS})
target_link_libraries(rplidar_app PRIVATE rplidar_sdk CURL::libcurl nlohmann_json::nlohmann_json Threads::Threads)

ecowarden_apply_optional_deps(rplidar_app)

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

add_executable(check_camera test/test_camera.cpp src/camera_module.cpp ${PRIVACY_SOURCES})
target_include_directories(check_camera PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(check_camera PRIVATE Threads::Threads)
ecowarden_apply_optional_deps(check_camera)

add_executable(servo_sweep_test
  test/servo_sweep_test.cpp
  src/servo_controller.cpp
  src/camera_module.cpp
  ${PRIVACY_SOURCES}
)
target_include_directories(servo_sweep_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(servo_sweep_test PRIVATE Threads::Threads)
ecowarden_apply_optional_deps(servo_sweep_test)

# 카메라/LiDAR 의존 0 — sysfs PWM만 직접 두드리는 최소 서보 테스트
add_executable(servo_only_test test/servo_only_test.cpp)
target_include_directories(servo_only_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(servo_only_test PRIVATE Threads::Threads)

add_executable(rplidar_sim
  test/sim_main.cpp
  src/scan_processor.cpp
  src/background_filter.cpp
  src/cluster_tracker.cpp
  src/object_classifier.cpp
  src/evidence_vault.cpp
  src/face_masking.cpp
)
target_include_directories(rplidar_sim PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(rplidar_sim PRIVATE Threads::Threads)
# object_classifier.cpp / face_masking.cpp 가 USE_OPENCV 분기를 타므로
# OpenCV 가 있는 환경(Pi)에서는 include 경로가 반드시 필요하다.
ecowarden_apply_optional_deps(rplidar_sim)
