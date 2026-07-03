#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "camera_module.h"
#include "cluster_tracker.h"

namespace ecowarden {

struct ServoBundle {
    std::string suspect_path;
    std::string confirm_path;
    std::string meta_path;
    std::string person_dir;
};

struct ServoParams {
    bool        enable          = true;
    bool        mirror          = false;
    std::string backend         = "arduino";    // arduino | pwm
    std::string serial_device   = "/dev/ttyACM0";
    uint32_t    serial_baud     = 115200;
    uint32_t    serial_boot_ms  = 2000;         // Arduino resets when serial opens
    // Pi5(RP1) 기본 라우팅:
    //   dtoverlay=pwm-pi5 (옵션 없이) 적용 시 pwmchip2 가 새로 생기고 npwm=2.
    //   sysfs 채널 번호는 0부터 재번호되므로:
    //     pwmchip2/pwm0 = GPIO18 (물리 pin 12)
    //     pwmchip2/pwm1 = GPIO19 (물리 pin 35)
    //   GPIO12/13 은 RP1 alt 매핑이 커널 버전마다 달라 신호가 안 잡히는
    //   사례가 잦아 본 코드는 RP1 기본 alt-pin 인 GPIO18 사용.
    //   ※ /boot/firmware/config.txt 에서 dtparam=audio=on 은 비활성화 필요
    //     (Pi5는 아날로그 오디오 잭이 없는데도 PWM 채널을 선점할 수 있음).
    std::string pwm_chip        = "pwmchip2";
    uint32_t    pwm_channel     = 0;       // pwmchip2/pwm0 = GPIO18 = 물리 pin 12
    uint32_t    period_ns       = 20000000;
    uint32_t    min_pulse_us    = 544;          // SG90/Arduino Servo default low end
    uint32_t    max_pulse_us    = 2400;         // SG90/Arduino Servo default high end
    double      zone_centers_deg[5] = {0.0, 45.0, 90.0, 135.0, 180.0};
    // zone 경계에서 측정 각도 노이즈(±2~3°)로 서보가 두 zone을 오가는
    // oscillation을 막는다. 현재 zone 중심에서 (22.5 + 이 값)° 를 벗어나야
    // zone 이 바뀐다. 0 이면 비활성.
    double      zone_hysteresis_deg = 10.0;
    // 전각도 즉시 이동 시 SG90 관성 오버슛이 생긴다. 한 번에 보내는
    // 각도 변화를 slew_step_deg 로 제한하고 스텝 사이 slew_step_ms 만큼
    // 기다린다. slew_step_deg<=0 이면 비활성(기존 단발 이동).
    double      slew_step_deg   = 15.0;
    uint32_t    slew_step_ms    = 20;
    double      ms_per_deg      = 2.5;
    uint32_t    settle_ms       = 80;
    uint32_t    max_move_ms     = 700;
    std::string suspect_dir     = "captures/suspects";
    std::string dump_dir        = "captures/dumps";
    std::string person_dir      = "captures/person_zones";
    uint32_t    person_capture_interval_ms = 2000;
    uint32_t    person_session_timeout_ms = 3000;
};

class ServoController {
public:
    ServoController(const ServoParams& params, CameraModule& camera);
    ~ServoController();

    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;

    bool Init();
    void Shutdown();

    void OnSuspect(const DumpingSuspectEvent& evt);
    void OnPersonDetected(uint32_t track_id, double x_mm, double y_mm);
    void CleanupPersonSessions();
    std::optional<ServoBundle> PromoteSuspectToDump(uint32_t object_track_id,
                                                    uint32_t person_track_id = 0);

    bool MoveToZoneBlocking(int zone_index);
    bool IsReady() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

ServoParams DefaultServoParams();

} // namespace ecowarden
