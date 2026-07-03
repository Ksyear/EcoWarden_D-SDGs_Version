// EcoWarden 서보 전용 테스트 — 카메라/LiDAR 의존 없음
//
// 사용법:
//   sudo ./build/servo_only_test                       # 자동 chip 탐색
//   sudo ./build/servo_only_test pwmchip2              # chip 직접 지정
//   sudo ./build/servo_only_test pwmchip2 0            # chip + channel
//   sudo ./build/servo_only_test pwmchip2 0 12 21 0 0 90 180 90
//                                                     # 마지막 인자들은 sweep 각도 시퀀스
//
// 동작: sysfs로 PWM export → period=20ms → 1.5ms 중앙 정지 →
//       0/45/90/135/180도 순차 이동 → 90도 복귀 → unexport.
//       각 단계 콘솔에 펄스폭(µs)을 찍어주므로 오실로스코프 없이도
//       값이 의도대로 들어가는지 확인할 수 있다.

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint32_t kPeriodNs    = 20'000'000;   // 20ms (50Hz, hobby servo 표준)
constexpr uint32_t kMinPulseUs  = 544;          // SG90 0도
constexpr uint32_t kMaxPulseUs  = 2400;         // SG90 180도
constexpr uint32_t kSettleMs    = 600;          // 각 각도 유지 시간

bool WriteSysfs(const std::filesystem::path& path, const std::string& value) {
    const int fd = ::open(path.c_str(), O_WRONLY);
    if (fd < 0) {
        std::fprintf(stderr, "  [ERR] open %s: %s\n",
                     path.c_str(), std::strerror(errno));
        return false;
    }
    const ssize_t n = ::write(fd, value.data(), value.size());
    const int err = errno;
    ::close(fd);
    if (n != static_cast<ssize_t>(value.size())) {
        std::fprintf(stderr, "  [ERR] write %s <- %s: %s\n",
                     path.c_str(), value.c_str(), std::strerror(err));
        return false;
    }
    return true;
}

uint32_t AngleToDutyNs(double angle_deg) {
    angle_deg = std::clamp(angle_deg, 0.0, 180.0);
    const double pulse_us =
        kMinPulseUs + (kMaxPulseUs - kMinPulseUs) * (angle_deg / 180.0);
    return static_cast<uint32_t>(pulse_us * 1000.0 + 0.5);
}

std::string AutoDetectChip() {
    const std::filesystem::path root("/sys/class/pwm");
    if (!std::filesystem::exists(root)) return {};
    std::vector<std::string> chips;
    for (const auto& e : std::filesystem::directory_iterator(root)) {
        const std::string name = e.path().filename().string();
        if (name.rfind("pwmchip", 0) == 0) chips.push_back(name);
    }
    std::sort(chips.begin(), chips.end());
    // Pi5 RP1 외부 PWM은 보통 가장 큰 번호. Pi4까지는 pwmchip0 단일.
    if (!chips.empty()) return chips.back();
    return {};
}

void ListChips() {
    const std::filesystem::path root("/sys/class/pwm");
    std::fprintf(stderr, "  사용 가능한 chip 목록:\n");
    if (!std::filesystem::exists(root)) {
        std::fprintf(stderr, "    /sys/class/pwm 자체가 없음\n");
        return;
    }
    for (const auto& e : std::filesystem::directory_iterator(root)) {
        const std::string name = e.path().filename().string();
        if (name.rfind("pwmchip", 0) != 0) continue;
        std::string npwm = "?";
        FILE* f = std::fopen((e.path() / "npwm").c_str(), "r");
        if (f) {
            char buf[16] = {0};
            if (std::fread(buf, 1, sizeof(buf) - 1, f) > 0) {
                npwm = buf;
                if (!npwm.empty() && npwm.back() == '\n') npwm.pop_back();
            }
            std::fclose(f);
        }
        std::fprintf(stderr, "    %s (npwm=%s)\n", name.c_str(), npwm.c_str());
    }
}

} // namespace

int main(int argc, char** argv) {
    // Pi5(RP1) 기본 라우팅: dtoverlay=pwm-pi5 (옵션 없이)
    //   → pwmchip2 (npwm=2) 의 pwm0 = GPIO18 (물리 pin 12)
    std::string chip = (argc > 1) ? argv[1] : "pwmchip2";
    uint32_t channel = (argc > 2) ? static_cast<uint32_t>(std::atoi(argv[2])) : 0;

    std::vector<double> sweep;
    if (argc > 3) {
        for (int i = 3; i < argc; ++i) sweep.push_back(std::atof(argv[i]));
    } else {
        sweep = {90.0, 0.0, 45.0, 90.0, 135.0, 180.0, 90.0};
    }

    if (chip.empty()) {
        chip = AutoDetectChip();
        if (chip.empty()) {
            std::fprintf(stderr,
                "[ERROR] /sys/class/pwm 에 pwmchip이 없음.\n"
                "        - dtoverlay=pwm-pi5 (Pi5) 또는 dtoverlay=pwm-2chan (Pi4) 추가 후 재부팅 필요.\n");
            return 2;
        }
        std::printf("[INFO] chip 자동 선택: %s\n", chip.c_str());
    }

    const std::filesystem::path root = std::filesystem::path("/sys/class/pwm") / chip;
    const std::filesystem::path pwm  = root / ("pwm" + std::to_string(channel));

    std::printf("=== EcoWarden Servo-Only Test ===\n");
    std::printf("  chip       : %s\n", root.c_str());
    std::printf("  channel    : %u (path=%s)\n", channel, pwm.c_str());
    std::printf("  period_ns  : %u (50Hz)\n", kPeriodNs);
    std::printf("  pulse_range: %u..%u us (0..180 deg)\n",
                kMinPulseUs, kMaxPulseUs);

    if (!std::filesystem::exists(root)) {
        std::fprintf(stderr,
            "[ERROR] %s 가 존재하지 않음.\n", root.c_str());
        ListChips();
        return 2;
    }

    bool exported_here = false;
    if (!std::filesystem::exists(pwm)) {
        std::printf("[1/5] export channel %u...\n", channel);
        if (!WriteSysfs(root / "export", std::to_string(channel))) {
            std::fprintf(stderr,
                "[ERROR] export 실패. sudo로 실행했는지, %s/npwm 가 채널 번호보다 큰지 확인.\n",
                root.c_str());
            ListChips();
            return 3;
        }
        exported_here = true;
        for (int i = 0; i < 50 && !std::filesystem::exists(pwm); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!std::filesystem::exists(pwm)) {
            std::fprintf(stderr,
                "[ERROR] export 직후에도 %s 가 안 생김.\n", pwm.c_str());
            return 3;
        }
    } else {
        std::printf("[1/5] channel %u 이미 export 됨 (재사용).\n", channel);
    }

    std::printf("[2/5] enable=0 (재설정 안전화)...\n");
    WriteSysfs(pwm / "enable", "0");

    std::printf("[3/5] period=%u ns...\n", kPeriodNs);
    if (!WriteSysfs(pwm / "period", std::to_string(kPeriodNs))) {
        std::fprintf(stderr,
            "[ERROR] period 쓰기 실패. duty_cycle이 이전 period보다 크면 EINVAL이 날 수 있음.\n"
            "        대처: sudo sh -c 'echo 0 > %s/duty_cycle' 후 재실행.\n",
            pwm.c_str());
        return 4;
    }

    const uint32_t mid_duty = AngleToDutyNs(90.0);
    std::printf("[4/5] duty_cycle=%u ns (90deg / 1500us) → enable=1\n", mid_duty);
    if (!WriteSysfs(pwm / "duty_cycle", std::to_string(mid_duty))) return 4;
    if (!WriteSysfs(pwm / "enable", "1")) return 4;
    std::this_thread::sleep_for(std::chrono::milliseconds(kSettleMs));

    std::printf("[5/5] sweep 시작 (서보가 움직여야 정상)\n");
    for (double angle : sweep) {
        const uint32_t duty = AngleToDutyNs(angle);
        const double pulse_us = duty / 1000.0;
        std::printf("  -> %6.1f deg | duty=%u ns | pulse=%.1f us\n",
                    angle, duty, pulse_us);
        if (!WriteSysfs(pwm / "duty_cycle", std::to_string(duty))) {
            std::fprintf(stderr, "[WARN] duty_cycle 쓰기 실패 (각도 %.1f).\n", angle);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kSettleMs));
    }

    std::printf("[done] 90deg 복귀 후 disable.\n");
    WriteSysfs(pwm / "duty_cycle", std::to_string(AngleToDutyNs(90.0)));
    std::this_thread::sleep_for(std::chrono::milliseconds(kSettleMs));
    WriteSysfs(pwm / "enable", "0");
    if (exported_here) {
        WriteSysfs(root / "unexport", std::to_string(channel));
    }

    std::printf("\n서보가 안 움직였다면 점검 순서:\n"
                "  1) ls /sys/class/pwm/   →  pwmchip2 가 보여야 함 (Pi5 RP1)\n"
                "  2) cat /sys/class/pwm/pwmchip2/npwm  → 2 (채널 0,1만 존재)\n"
                "  3) /boot/firmware/config.txt:\n"
                "       dtoverlay=pwm-pi5    한 줄만 있을 것 (옵션 X)\n"
                "       dtparam=audio=on     이 줄은 주석처리(#) 또는 삭제할 것\n"
                "       (Pi5에는 아날로그 오디오 잭이 없는데도 GPIO18/19 PWM 을 선점함)\n"
                "  4) 신호선(주황/노랑)은 BCM GPIO18 = 물리 pin 12 에 연결\n"
                "  5) SG90 빨강=5V, 갈색/검정=GND, Pi GND와 공통 GND 필수\n"
                "  6) GPIO19 (pin35) 로 시도: sudo ./build/servo_only_test pwmchip2 1\n");
    return 0;
}
