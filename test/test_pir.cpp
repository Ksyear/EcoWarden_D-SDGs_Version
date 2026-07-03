#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include "pir_sensor.h"

static volatile bool g_running = true;

static void SignalHandler(int) { g_running = false; }

int main() {
    std::cout << "========================================\n";
    std::cout << " PIR Sensor Test\n";
    std::cout << " Target: Panasonic PaPIRs EKMC160111\n";
    std::cout << "========================================\n\n";

    // ── EKMC160111 사양 확인 ──────────────────────────────────────
    // - 동작 전압 : 3.0 ~ 6.0V (권장 5V)
    // - 출력 방식 : 디지털 (감지 시 HIGH, 미감지 시 LOW)
    // - 감지 거리 : 최대 5m
    // - 출력 안정화: 전원 인가 후 약 30초 필요
    // - 배선: VDD → 5V, GND → GND, OUT → BCM GPIO 17
    //
    // PirSensorConfig 기본값 확인:
    //   gpio_pin=17, active_high=true → EKMC160111과 일치

    ecowarden::PirSensorConfig config;
    config.gpio_pin        = 17;    // BCM GPIO 17 (물리 핀 11)
    config.active_high     = true;  // EKMC160111: 감지 시 HIGH 출력
    config.debounce_frames = 3;     // 0.3초 디바운스 (10Hz 기준)
    config.holdoff_frames  = 50;    // 5초 홀드오프

    std::cout << "[CONFIG] GPIO Pin      : BCM " << config.gpio_pin << "\n";
    std::cout << "[CONFIG] Active High   : " << (config.active_high ? "YES" : "NO") << "\n";
    std::cout << "[CONFIG] Debounce      : " << config.debounce_frames << " frames\n";
    std::cout << "[CONFIG] Holdoff       : " << config.holdoff_frames << " frames\n\n";

    // ── 초기화 ───────────────────────────────────────────────────
    ecowarden::PirSensor pir(config);

    std::cout << "[TEST] Initializing PIR sensor...\n";
    if (!pir.Init()) {
        std::cerr << "[FAILED] PIR Init failed.\n";
        std::cerr << "  - GPIO " << config.gpio_pin << " 접근 권한을 확인하세요 (sudo 또는 gpio 그룹)\n";
        std::cerr << "  - 배선을 확인하세요: VDD→5V, GND→GND, OUT→GPIO"
                  << config.gpio_pin << "\n";
        return 1;
    }
    std::cout << "[OK] PIR sensor initialized successfully.\n";
    std::cout << "[OK] IsReady() = " << (pir.IsReady() ? "true" : "false") << "\n\n";

    // ── 안정화 대기 (EKMC160111: ~30초) ──────────────────────────
    std::cout << "[INFO] EKMC160111 requires ~30s stabilization after power-on.\n";
    std::cout << "[INFO] If just powered on, wait for stabilization before testing.\n\n";

    // ── 실시간 모니터링 (10Hz, Ctrl+C로 종료) ────────────────────
    std::signal(SIGINT, SignalHandler);
    std::cout << "[TEST] Starting 10Hz polling loop (Ctrl+C to stop)...\n";
    std::cout << "----------------------------------------\n";

    uint32_t frame = 0;
    uint32_t motion_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (g_running) {
        pir.Read();
        frame++;

        bool motion = pir.IsMotionDetected();
        if (motion) motion_count++;

        // 상태 변경 시 로그 출력
        if (pir.HasStateChanged()) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            auto sec = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() / 1000.0;
            std::cout << "[" << sec << "s] Frame " << frame
                      << " | Motion: " << (motion ? "ON" : "OFF") << "\n";
        }

        // 매 50프레임(5초)마다 상태 요약
        if (frame % 50 == 0) {
            std::cout << "  ... frame " << frame
                      << " | current: " << (motion ? "ON" : "OFF")
                      << " | motion frames: " << motion_count << "/" << frame << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }

    // ── 결과 요약 ────────────────────────────────────────────────
    auto total_elapsed = std::chrono::steady_clock::now() - start_time;
    auto total_sec = std::chrono::duration_cast<std::chrono::milliseconds>(total_elapsed).count() / 1000.0;

    std::cout << "\n========================================\n";
    std::cout << " Test Summary\n";
    std::cout << "========================================\n";
    std::cout << "  Duration     : " << total_sec << "s\n";
    std::cout << "  Total Frames : " << frame << "\n";
    std::cout << "  Motion Frames: " << motion_count << "\n";
    std::cout << "  Motion Ratio : "
              << (frame > 0 ? (100.0 * motion_count / frame) : 0)
              << "%\n";
    std::cout << "  Final State  : " << (pir.IsMotionDetected() ? "ON" : "OFF") << "\n";
    std::cout << "========================================\n";

    return 0;
}
