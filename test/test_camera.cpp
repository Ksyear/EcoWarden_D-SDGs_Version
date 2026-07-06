#include <chrono>
#include <iostream>
#include <sys/stat.h>
#include <thread>
#include "camera_module.h"

int main() {
    std::cout << "[TEST] Camera Capture Test Start (Target: DFROBOT FIT0730)\n";

    // 1. 카메라 모듈 초기화
    ecowarden::CameraModule camera(0); // /dev/video0 사용
    camera.SetCaptureDir("captures_test");

    // 2. 캡처 스레드 시작 — Start() 없이는 프레임이 버퍼에 들어오지 않는다.
    if (!camera.Start()) {
        std::cerr << "[FAILED] Camera Start failed.\n"
                  << "  - /dev/video0 존재 확인: ls -l /dev/video*\n"
                  << "  - video 그룹 권한 확인: groups (setup.sh 후 재로그인 필요)\n"
                  << "  - OpenCV 포함 빌드 확인: 빌드 로그의 USE_OPENCV\n";
        return 1;
    }

    // 3. 첫 프레임 대기 후 캡처 (USB 카메라 오픈에 수백 ms 걸릴 수 있음, 최대 5초)
    std::cout << "[INFO] Waiting for first frame (max 5s)...\n";
    std::string path;
    for (int i = 0; i < 50 && path.empty(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        path = camera.CaptureToFile();
    }
    camera.Stop();

    // 4. 결과 확인
    if (!path.empty()) {
        struct stat buffer;
        if (stat(path.c_str(), &buffer) == 0) {
            std::cout << "[SUCCESS] Image saved and verified at: " << path << "\n";
            std::cout << "[INFO] File Size: " << buffer.st_size << " bytes\n";
            return 0;
        }
        std::cerr << "[ERROR] Captured file exists in theory, but cannot be found on disk.\n";
        return 1;
    }

    std::cerr << "[FAILED] Camera opened but no frame arrived in 5s. "
              << "v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=1 로 장치를 직접 확인하세요.\n";
    return 1;
}
