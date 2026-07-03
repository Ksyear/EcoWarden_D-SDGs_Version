#include <iostream>
#include <vector>
#include <unistd.h>
#include "rplidar_s2.h"

int main(int argc, char* argv[]) {
    const char* port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    std::cout << "[TEST] LiDAR USB Data Collection Test Start (" << port << ")\n";

    ecowarden::RplidarS2 lidar;
    ecowarden::Config cfg;
    cfg.serial_port = port;
    cfg.baudrate = 1000000;

    if (lidar.Start(cfg) != ecowarden::Error::None) {
        std::cerr << "[ERROR] Failed to start LiDAR. Check USB connection and permissions.\n";
        return 1;
    }

    std::cout << "[INFO] Waiting for scan data (5 seconds)...\n";
    int frames_received = 0;
    for (int i = 0; i < 50; ++i) { // 5초 동안 테스트 (10Hz 기준)
        ecowarden::ScanFrame frame;
        if (lidar.GetScanFrame(frame) == ecowarden::Error::None && !frame.empty()) {
            frames_received++;
            if (frames_received % 10 == 0) {
                std::cout << "  - Received Frame #" << frames_received 
                          << " (Points: " << frame.size() << ")\n";
            }
        }
        usleep(100000);
    }

    lidar.Stop();
    if (frames_received > 0) {
        std::cout << "[SUCCESS] LiDAR is sending data via USB! Total frames: " << frames_received << "\n";
    } else {
        std::cerr << "[FAILED] No data received from LiDAR.\n";
    }

    return 0;
}
