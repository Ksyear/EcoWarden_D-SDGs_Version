#include <iostream>
#include <sys/stat.h>
#include "camera_module.h"

int main() {
    std::cout << "[TEST] Camera Capture Test Start (Target: DFROBOT FIT0730)\n";

    // 1. 카메라 모듈 초기화
    ecowarden::CameraModule camera(0); // /dev/video0 사용
    camera.SetCaptureDir("captures_test");

    // 2. 사진 캡처 시도
    std::cout << "[INFO] Attempting to capture a frame...\n";
    std::string path = camera.CaptureToFile();

    // 3. 결과 확인
    if (!path.empty()) {
        struct stat buffer;
        if (stat(path.c_str(), &buffer) == 0) {
            std::cout << "[SUCCESS] Image saved and verified at: " << path << "\n";
            std::cout << "[INFO] File Size: " << buffer.st_size << " bytes\n";
        } else {
            std::cerr << "[ERROR] Captured file exists in theory, but cannot be found on disk.\n";
        }
    } else {
        std::cerr << "[FAILED] Camera failed to capture. Is OpenCV installed and /dev/video0 accessible?\n";
    }

    return 0;
}
