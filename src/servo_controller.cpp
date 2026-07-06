#include "servo_controller.h"

#include "servo_zone.h"

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <unordered_map>

namespace ecowarden {

namespace {

speed_t BaudToConstant(uint32_t baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        default: return B115200;
    }
}

bool WriteFile(const std::filesystem::path& path, const std::string& value) {
    const int fd = ::open(path.c_str(), O_WRONLY);
    if (fd < 0) {
        std::fprintf(stderr, "[WARN] PWM write open failed: %s (%s)\n",
                     path.c_str(), std::strerror(errno));
        return false;
    }
    const ssize_t written = ::write(fd, value.data(), value.size());
    if (written != static_cast<ssize_t>(value.size())) {
        std::fprintf(stderr, "[WARN] PWM write failed: %s (%s)\n",
                     path.c_str(), std::strerror(errno));
    }
    ::close(fd);
    return written == static_cast<ssize_t>(value.size());
}

std::string TimestampUtc() {
    const auto now = std::chrono::system_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count() % 1000;
    const auto t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_buf {};
    gmtime_r(&t, &tm_buf);

    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y%m%dT%H%M%S")
       << "_" << std::setw(3) << std::setfill('0') << ms;
    return ss.str();
}

std::string JoinName(const std::filesystem::path& dir,
                     const std::string& name) {
    return (dir / name).string();
}

uint64_t NowMs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

} // namespace

struct ServoController::Impl {
    enum class PendingKind {
        None,
        Suspect,
        Person,
    };

    struct PendingJob {
        PendingKind kind = PendingKind::None;
        DumpingSuspectEvent evt {};
        uint32_t track_id = 0;
        double x_mm = 0.0;
        double y_mm = 0.0;
    };

    struct CacheEntry {
        std::string path;
        std::string timestamp;
        int zone = 2;
        double x_mm = 0.0;
        double y_mm = 0.0;
    };

    struct PersonSession {
        std::filesystem::path dir;
        uint64_t last_seen_ms = 0;
        uint64_t last_capture_ms = 0;
        int last_zone = -1;
        uint32_t photo_count = 0;
    };

    explicit Impl(const ServoParams& p, CameraModule& cam)
        : params(p), camera(cam) {}

    ServoParams params;
    CameraModule& camera;

    mutable std::mutex mutex;
    std::condition_variable cv;
    std::thread worker;
    bool running = false;
    bool stop = false;
    PendingJob pending;
    std::unordered_map<uint32_t, CacheEntry> cache;
    std::unordered_map<uint32_t, PersonSession> person_sessions;

    bool pwm_ready = false;
    bool pwm_exported = false;
    bool serial_ready = false;
    int serial_fd = -1;
    double current_angle = 90.0;
    int current_zone = 2;
    std::filesystem::path pwm_root;
    std::filesystem::path pwm_path;

    bool Init() {
        std::error_code ec;
        std::filesystem::create_directories(params.suspect_dir, ec);
        std::filesystem::create_directories(params.dump_dir, ec);
        std::filesystem::create_directories(params.person_dir, ec);

        if (params.enable) {
            if (params.backend == "arduino") {
                serial_ready = InitSerial();
                if (!serial_ready) {
                    std::fprintf(stderr,
                                 "[WARN] Arduino servo init failed; using camera-only fallback\n");
                }
            } else {
                pwm_ready = InitPwm();
                if (!pwm_ready) {
                    std::fprintf(stderr,
                                 "[WARN] Servo PWM init failed; using camera-only fallback\n");
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = false;
            running = true;
        }
        worker = std::thread(&Impl::WorkerLoop, this);
        return IsReady() || !params.enable;
    }

    void Shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!running) return;
            stop = true;
        }
        cv.notify_all();
        if (worker.joinable()) worker.join();

        if (IsReady()) {
            MoveToAngle(90.0);
        }
        if (pwm_ready) {
            WriteFile(pwm_path / "enable", "0");
            if (pwm_exported) {
                WriteFile(pwm_root / "unexport",
                          std::to_string(params.pwm_channel));
            }
        }
        if (serial_fd >= 0) {
            ::close(serial_fd);
            serial_fd = -1;
        }

        running = false;
        pwm_ready = false;
        serial_ready = false;
    }

    bool IsReady() const {
        return pwm_ready || serial_ready;
    }

    bool InitSerial() {
        serial_fd = ::open(params.serial_device.c_str(),
                           O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd < 0) {
            std::fprintf(stderr, "[WARN] Arduino serial open failed: %s (%s)\n",
                         params.serial_device.c_str(), std::strerror(errno));
            return false;
        }

        termios tty {};
        if (tcgetattr(serial_fd, &tty) != 0) {
            std::fprintf(stderr, "[WARN] Arduino serial tcgetattr failed: %s\n",
                         std::strerror(errno));
            ::close(serial_fd);
            serial_fd = -1;
            return false;
        }

        cfmakeraw(&tty);
        const speed_t speed = BaudToConstant(params.serial_baud);
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
            std::fprintf(stderr, "[WARN] Arduino serial tcsetattr failed: %s\n",
                         std::strerror(errno));
            ::close(serial_fd);
            serial_fd = -1;
            return false;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(params.serial_boot_ms));
        tcflush(serial_fd, TCIOFLUSH);
        current_angle = 90.0;
        return SendArduinoAngle(90.0);
    }

    bool InitPwm() {
        pwm_root = std::filesystem::path("/sys/class/pwm") / params.pwm_chip;
        pwm_path = pwm_root / ("pwm" + std::to_string(params.pwm_channel));

        if (!std::filesystem::exists(pwm_path)) {
            if (!WriteFile(pwm_root / "export", std::to_string(params.pwm_channel))) {
                return false;
            }
            pwm_exported = true;
            for (int i = 0; i < 20 && !std::filesystem::exists(pwm_path); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        if (!std::filesystem::exists(pwm_path)) return false;

        WriteFile(pwm_path / "enable", "0");
        if (!WriteFile(pwm_path / "period", std::to_string(params.period_ns))) {
            return false;
        }
        if (!WriteAngleDuty(90.0)) return false;
        if (!WriteFile(pwm_path / "enable", "1")) return false;
        current_angle = 90.0;
        return true;
    }

    bool WriteAngleDuty(double angle_deg) {
        angle_deg = std::clamp(angle_deg, 0.0, 180.0);
        const double pulse_us =
            params.min_pulse_us +
            (params.max_pulse_us - params.min_pulse_us) * (angle_deg / 180.0);
        const auto duty_ns = static_cast<uint32_t>(std::round(pulse_us * 1000.0));
        return WriteFile(pwm_path / "duty_cycle", std::to_string(duty_ns));
    }

    bool WriteAngleToBackend(double angle_deg) {
        if (serial_ready) return SendArduinoAngle(angle_deg);
        if (pwm_ready) return WriteAngleDuty(angle_deg);
        return true; // 서보 미장착(camera-only fallback)도 성공으로 취급
    }

    bool MoveToAngle(double angle_deg) {
        angle_deg = std::clamp(angle_deg, 0.0, 180.0);

        // Slew rate: 한 번에 slew_step_deg 이상 움직이지 않도록 분할 전송해
        // 전각도 이동 시 SG90 관성 오버슛을 막는다.
        if (params.slew_step_deg > 0.0) {
            while (std::abs(angle_deg - current_angle) > 0.5) {
                const double diff = angle_deg - current_angle;
                const double step = std::clamp(diff, -params.slew_step_deg,
                                               params.slew_step_deg);
                const double next = current_angle + step;
                if (!WriteAngleToBackend(next)) return false;
                current_angle = next;
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(params.slew_step_ms));
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(params.settle_ms));
            return true;
        }

        const double delta = std::abs(angle_deg - current_angle);
        const auto move_ms = static_cast<uint32_t>(std::min<double>(
            params.max_move_ms,
            params.settle_ms + params.ms_per_deg * delta));

        if (!WriteAngleToBackend(angle_deg)) return false;
        current_angle = angle_deg;
        std::this_thread::sleep_for(std::chrono::milliseconds(move_ms));
        return true;
    }

    bool SendArduinoAngle(double angle_deg) {
        if (serial_fd < 0) return false;

        const int angle = static_cast<int>(std::round(
            std::clamp(angle_deg, 0.0, 180.0)));
        const std::string cmd = "A " + std::to_string(angle) + "\n";
        tcflush(serial_fd, TCIFLUSH);
        const ssize_t written = ::write(serial_fd, cmd.data(), cmd.size());
        if (written != static_cast<ssize_t>(cmd.size())) {
            std::fprintf(stderr, "[WARN] Arduino serial write failed: %s\n",
                         std::strerror(errno));
            return false;
        }

        std::string line;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(1500);
        while (std::chrono::steady_clock::now() < deadline) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(serial_fd, &rfds);
            timeval tv {};
            tv.tv_sec = 0;
            tv.tv_usec = 100000;
            const int ready = select(serial_fd + 1, &rfds, nullptr, nullptr, &tv);
            if (ready < 0) {
                std::fprintf(stderr, "[WARN] Arduino serial select failed: %s\n",
                             std::strerror(errno));
                return false;
            }
            if (ready == 0) continue;

            char ch = '\0';
            const ssize_t n = ::read(serial_fd, &ch, 1);
            if (n <= 0) continue;
            if (ch == '\n' || ch == '\r') {
                if (line.rfind("OK", 0) == 0) return true;
                if (!line.empty()) {
                    std::fprintf(stderr, "[WARN] Arduino replied: %s\n",
                                 line.c_str());
                    line.clear();
                }
                continue;
            }
            if (line.size() < 80) line.push_back(ch);
        }

        std::fprintf(stderr, "[WARN] Arduino serial timeout waiting for OK\n");
        return false;
    }

    bool MoveToZoneBlocking(int zone_index) {
        zone_index = std::clamp(zone_index, 0, 4);
        if (!MoveToAngle(params.zone_centers_deg[zone_index])) return false;
        current_zone = zone_index;
        return true;
    }

    // Zone hysteresis: 측정 각도가 현재 zone 중심에서
    // (zone 반폭 22.5° + zone_hysteresis_deg)를 벗어날 때만 zone 을 바꾼다.
    // LiDAR 각도 노이즈가 45° 경계를 넘나들며 서보가 진동하는 것을 막는다.
    int ComputeTargetZone(double x_mm, double y_mm) {
        const int raw_zone = SuspectToZone(x_mm, y_mm, params.mirror);
        if (raw_zone == current_zone || params.zone_hysteresis_deg <= 0.0) {
            return raw_zone;
        }
        const double angle = SuspectAngleDeg(x_mm, y_mm, params.mirror);
        const double cur_center = params.zone_centers_deg[current_zone];
        if (std::abs(angle - cur_center) <=
            22.5 + params.zone_hysteresis_deg) {
            return current_zone; // 경계 구간 — 현재 zone 유지
        }
        return raw_zone;
    }

    void WorkerLoop() {
        while (true) {
            PendingJob local;
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [&] {
                    return stop || pending.kind != PendingKind::None;
                });
                if (stop) break;
                local = pending;
                pending = PendingJob{};
            }
            if (local.kind == PendingKind::Suspect) {
                ProcessSuspect(local.evt);
            } else if (local.kind == PendingKind::Person) {
                ProcessPerson(local.track_id, local.x_mm, local.y_mm);
            }
        }
    }

    void ProcessPerson(uint32_t track_id, double x_mm, double y_mm) {
        const int zone = ComputeTargetZone(x_mm, y_mm);
        const uint64_t now = NowMs();
        std::filesystem::path session_dir;

        {
            std::lock_guard<std::mutex> lock(mutex);
            auto& session = person_sessions[track_id];
            if (session.dir.empty()) {
                session.dir = std::filesystem::path(params.person_dir) /
                    ("person_" + std::to_string(track_id));
            }
            session.last_seen_ms = now;

            const bool zone_changed = session.last_zone != zone;
            const bool interval_elapsed =
                now >= session.last_capture_ms + params.person_capture_interval_ms;
            if (session.photo_count > 0 && !zone_changed && !interval_elapsed) {
                return;
            }
            session_dir = session.dir;
        }

        MoveToZoneBlocking(zone);
        std::this_thread::sleep_for(std::chrono::milliseconds(params.settle_ms));

        std::error_code ec;
        std::filesystem::create_directories(session_dir, ec);
        if (ec) return;

        const std::string ts = TimestampUtc();
        const std::string filename =
            ts + "_zone_" + std::to_string(zone) + ".jpg";
        const std::string path = (session_dir / filename).string();
        const std::string annotation =
            "ID:" + std::to_string(track_id) +
            " ZONE:" + std::to_string(zone) + " " + ts + "Z";
        // 머리 위 ID 라벨: 서보는 존 중심각으로 조준하므로, 사람의 실제
        // 각도와 존 중심각의 차가 화면 중심 대비 수평 offset 이 된다.
        const double person_deg = SuspectAngleDeg(x_mm, y_mm, params.mirror);
        const double head_offset_deg =
            person_deg - params.zone_centers_deg[zone];
        const double head_distance_mm = std::hypot(x_mm, y_mm);
        const std::string saved = camera.CaptureToFilePath(
            path, annotation, "ID:" + std::to_string(track_id),
            head_offset_deg, head_distance_mm);
        if (!saved.empty()) {
            std::lock_guard<std::mutex> lock(mutex);
            auto it = person_sessions.find(track_id);
            if (it != person_sessions.end()) {
                it->second.last_capture_ms = now;
                it->second.last_zone = zone;
                it->second.photo_count++;
            }
        }
    }

    void ProcessSuspect(const DumpingSuspectEvent& evt) {
        const int zone = ComputeTargetZone(evt.object_x_mm, evt.object_y_mm);
        MoveToZoneBlocking(zone);
        std::this_thread::sleep_for(std::chrono::milliseconds(params.settle_ms));

        const std::string ts = TimestampUtc();
        const std::string filename =
            ts + "_" + std::to_string(evt.object_track_id) + ".jpg";
        const std::string path = JoinName(params.suspect_dir, filename);
        const std::string annotation =
            "SUSPECT OBJ:" + std::to_string(evt.object_track_id) +
            " PERSON:" + std::to_string(evt.person_track_id) +
            " ZONE:" + std::to_string(zone) + " " + ts + "Z";
        const std::string saved = camera.CaptureToFilePath(path, annotation);
        if (saved.empty()) return;

        std::lock_guard<std::mutex> lock(mutex);
        cache[evt.object_track_id] = CacheEntry{
            saved, ts, zone, evt.object_x_mm, evt.object_y_mm
        };
    }

    void CleanupPersonSessions() {
        const uint64_t now = NowMs();
        std::error_code ec;
        std::lock_guard<std::mutex> lock(mutex);
        for (auto it = person_sessions.begin(); it != person_sessions.end();) {
            const auto& session = it->second;
            if (session.last_seen_ms > 0 &&
                now > session.last_seen_ms + params.person_session_timeout_ms) {
                if (!session.dir.empty()) {
                    std::filesystem::remove_all(session.dir, ec);
                    ec.clear();
                }
                it = person_sessions.erase(it);
            } else {
                ++it;
            }
        }
    }

    std::string PromotePersonSession(uint32_t person_track_id,
                                     const std::filesystem::path& dump_dir) {
        if (person_track_id == 0) return "";
        std::lock_guard<std::mutex> lock(mutex);
        auto it = person_sessions.find(person_track_id);
        if (it == person_sessions.end()) return "";
        if (it->second.dir.empty() || !std::filesystem::exists(it->second.dir)) {
            person_sessions.erase(it);
            return "";
        }

        const std::filesystem::path dst =
            dump_dir / ("person_" + std::to_string(person_track_id));
        std::error_code ec;
        std::filesystem::rename(it->second.dir, dst, ec);
        if (ec) {
            ec.clear();
            std::filesystem::create_directories(dst, ec);
            if (!ec) {
                std::filesystem::copy(it->second.dir, dst,
                    std::filesystem::copy_options::recursive |
                    std::filesystem::copy_options::overwrite_existing, ec);
                if (!ec) std::filesystem::remove_all(it->second.dir);
            }
        }
        person_sessions.erase(it);
        return std::filesystem::exists(dst) ? dst.string() : "";
    }

    std::optional<ServoBundle> Promote(uint32_t object_track_id,
                                       uint32_t person_track_id) {
        CacheEntry entry;
        bool has_entry = false;
        {
            std::lock_guard<std::mutex> lock(mutex);
            auto it = cache.find(object_track_id);
            if (it != cache.end()) {
                entry = it->second;
                cache.erase(it);
                has_entry = true;
            }
        }

        const std::string ts = TimestampUtc();
        const std::filesystem::path dump_dir =
            std::filesystem::path(params.dump_dir) /
            (ts + "_" + std::to_string(object_track_id));
        std::error_code ec;
        std::filesystem::create_directories(dump_dir, ec);
        if (ec) return std::nullopt;

        const std::filesystem::path suspect_path = dump_dir / "suspect.jpg";
        const std::filesystem::path confirm_path = dump_dir / "confirm.jpg";
        const std::filesystem::path meta_path = dump_dir / "meta.json";

        std::string final_suspect_path;
        if (has_entry) {
            final_suspect_path = suspect_path.string();
            std::filesystem::rename(entry.path, suspect_path, ec);
            if (ec) {
                ec.clear();
                std::filesystem::copy_file(entry.path, suspect_path,
                                           std::filesystem::copy_options::overwrite_existing,
                                           ec);
                if (!ec) std::filesystem::remove(entry.path);
                else final_suspect_path = entry.path;
            }
        }

        const std::string confirm_annotation =
            "DUMP OBJ:" + std::to_string(object_track_id) +
            (person_track_id ? " PERSON:" + std::to_string(person_track_id) : "") +
            " " + ts + "Z";
        camera.CaptureToFilePath(confirm_path.string(), confirm_annotation);
        const std::string person_dir =
            PromotePersonSession(person_track_id, dump_dir);
        WriteMeta(meta_path, object_track_id, entry);

        return ServoBundle{
            final_suspect_path,
            confirm_path.string(),
            meta_path.string(),
            person_dir
        };
    }

    void WriteMeta(const std::filesystem::path& path,
                   uint32_t object_track_id,
                   const CacheEntry& entry) const {
        std::ofstream out(path);
        if (!out) return;
        out << "{\n"
            << "  \"object_track_id\": " << object_track_id << ",\n"
            << "  \"suspect_timestamp\": \"" << entry.timestamp << "\",\n"
            << "  \"zone\": " << entry.zone << ",\n"
            << "  \"zone_center_deg\": " << params.zone_centers_deg[entry.zone] << ",\n"
            << "  \"object_x_mm\": " << entry.x_mm << ",\n"
            << "  \"object_y_mm\": " << entry.y_mm << "\n"
            << "}\n";
    }
};

ServoController::ServoController(const ServoParams& params, CameraModule& camera)
    : impl_(std::make_unique<Impl>(params, camera)) {}

ServoController::~ServoController() {
    Shutdown();
}

bool ServoController::Init() {
    return impl_->Init();
}

void ServoController::Shutdown() {
    if (impl_) impl_->Shutdown();
}

void ServoController::OnSuspect(const DumpingSuspectEvent& evt) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->running) return;
    if (impl_->pending.kind == Impl::PendingKind::Suspect) return;
    impl_->pending.kind = Impl::PendingKind::Suspect;
    impl_->pending.evt = evt;
    impl_->cv.notify_one();
}

void ServoController::OnPersonDetected(uint32_t track_id,
                                       double x_mm,
                                       double y_mm) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->running) return;
    if (impl_->pending.kind == Impl::PendingKind::Suspect) return;
    impl_->pending.kind = Impl::PendingKind::Person;
    impl_->pending.track_id = track_id;
    impl_->pending.x_mm = x_mm;
    impl_->pending.y_mm = y_mm;
    impl_->cv.notify_one();
}

std::optional<ServoBundle>
ServoController::PromoteSuspectToDump(uint32_t object_track_id,
                                      uint32_t person_track_id) {
    return impl_->Promote(object_track_id, person_track_id);
}

void ServoController::CleanupPersonSessions() {
    impl_->CleanupPersonSessions();
}

bool ServoController::MoveToZoneBlocking(int zone_index) {
    return impl_->MoveToZoneBlocking(zone_index);
}

bool ServoController::IsReady() const {
    return impl_->IsReady();
}

} // namespace ecowarden
