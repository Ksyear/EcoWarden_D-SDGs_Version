#include "camera_module.h"
#include "production_params.h"
#include "servo_controller.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>

int main() {
    ecowarden::ServoParams sp = ecowarden::DefaultServoParams();
    sp.suspect_dir = "captures/sweep";
    sp.dump_dir = "captures/sweep_dumps";
    sp.settle_ms = 300;
    sp.max_move_ms = 1500;

    ecowarden::CameraModule camera(0);
    camera.SetCaptureDir("captures/sweep");
    const bool camera_ready = camera.Start();
    if (!camera_ready) {
        std::fprintf(stderr, "[WARN] Camera unavailable; sweep will test servo only\n");
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    ecowarden::ServoController servo(sp, camera);
    if (!servo.Init() || !servo.IsReady()) {
        std::fprintf(stderr,
                     "[ERROR] Servo backend unavailable. This test did not move the servo.\n"
                     "        backend=%s serial=%s pwm=/sys/class/pwm/%s/pwm%u\n"
                     "        Arduino: upload arduino/servo_zone_controller.ino, connect USB, check /dev/ttyACM0 and dialout.\n",
                     sp.backend.c_str(), sp.serial_device.c_str(),
                     sp.pwm_chip.c_str(), sp.pwm_channel);
        camera.Stop();
        return 2;
    }

    std::error_code ec;
    std::filesystem::create_directories("captures/sweep", ec);

    std::atomic<bool> sweep_done{false};
    std::atomic<int> sweep_result{0};

    std::thread sweep_thread([&] {
        for (int z = 0; z < 5; ++z) {
            std::printf("[SWEEP] zone %d (%.1f deg)\n", z, sp.zone_centers_deg[z]);
            if (!servo.MoveToZoneBlocking(z)) {
                std::fprintf(stderr, "[ERROR] Servo move failed at zone %d\n", z);
                sweep_result = 3;
                sweep_done = true;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(sp.settle_ms));
            if (camera_ready) {
                const std::string path =
                    "captures/sweep/zone_" + std::to_string(z) + ".jpg";
                const std::string saved = camera.CaptureToFilePath(path);
                if (saved.empty()) {
                    std::fprintf(stderr, "[WARN] Camera capture failed at zone %d\n", z);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        servo.MoveToZoneBlocking(2);
        sweep_done = true;
    });

    while (!sweep_done) {
        if (camera_ready && !camera.ShowPreviewFrame("EcoWarden Servo Sweep", 1)) {
            std::fprintf(stderr, "[WARN] Camera preview stopped\n");
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    if (sweep_thread.joinable()) sweep_thread.join();
    servo.Shutdown();
    camera.Stop();
    return sweep_result;
}
