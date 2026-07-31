/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 증거 원본 암호화 보관소 구현 (AES-256-GCM)
 */

#include "evidence_vault.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

#ifdef USE_OPENSSL
#include <openssl/evp.h>
#include <openssl/rand.h>
#endif

namespace ecowarden {

namespace {

constexpr char     kMagic[4]     = {'E', 'W', 'V', '1'};
constexpr uint8_t  kFormatVer    = 1;
constexpr size_t   kHeaderAad    = 20;   // AAD 로 묶는 헤더 길이
constexpr size_t   kNonceLen     = 12;
constexpr size_t   kTagLen       = 16;
constexpr size_t   kPayloadStart = kHeaderAad + kNonceLen + kTagLen;  // 48
constexpr size_t   kKeyLen       = 32;   // AES-256

std::mutex g_log_mutex;

void PutU32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}
uint32_t GetU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}
void PutU64(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; ++i) p[i] = static_cast<uint8_t>(v >> (i * 8));
}
uint64_t GetU64(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(p[i]) << (i * 8);
    return v;
}

int64_t NowUnix() {
    return std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::string IsoUtc(int64_t unix_sec) {
    const std::time_t t = static_cast<std::time_t>(unix_sec);
    std::tm tm_utc{};
#if defined(_WIN32)
    gmtime_s(&tm_utc, &t);
#else
    gmtime_r(&t, &tm_utc);
#endif
    std::ostringstream os;
    os << std::put_time(&tm_utc, "%Y-%m-%dT%H:%M:%SZ");
    return os.str();
}

// 로그 인젝션 방지 — 개행/제어문자 제거
std::string SanitizeField(const std::string& s, size_t max_len = 200) {
    std::string out;
    out.reserve(std::min(s.size(), max_len));
    for (char c : s) {
        if (out.size() >= max_len) break;
        out.push_back((static_cast<unsigned char>(c) < 0x20 || c == 0x7f)
                          ? ' '
                          : c);
    }
    if (out.empty()) out = "-";
    return out;
}

} // namespace

// ── hex 유틸 ─────────────────────────────────────────────────────────
bool HexToBytes(const std::string& hex, std::vector<uint8_t>* out) {
    if (out == nullptr) return false;
    if (hex.size() % 2 != 0 || hex.empty()) return false;
    out->clear();
    out->reserve(hex.size() / 2);
    auto nib = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (size_t i = 0; i < hex.size(); i += 2) {
        const int hi = nib(hex[i]);
        const int lo = nib(hex[i + 1]);
        if (hi < 0 || lo < 0) { out->clear(); return false; }
        out->push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return true;
}

std::string BytesToHex(const uint8_t* data, size_t size) {
    static const char* kHex = "0123456789abcdef";
    std::string s;
    s.reserve(size * 2);
    for (size_t i = 0; i < size; ++i) {
        s.push_back(kHex[data[i] >> 4]);
        s.push_back(kHex[data[i] & 0x0f]);
    }
    return s;
}

// ── 생성자 ───────────────────────────────────────────────────────────
EvidenceVault::EvidenceVault(const EvidenceVaultParams& params)
    : params_(params) {
    if (!params_.enable) {
        unavailable_reason_ = "원본 보관이 비활성 (ECOWARDEN_EVIDENCE_VAULT=0)";
        return;
    }
#ifndef USE_OPENSSL
    unavailable_reason_ =
        "OpenSSL 없이 빌드됨 — 평문 보관을 피하기 위해 원본을 저장하지 않습니다";
    std::cerr << "[VAULT] " << unavailable_reason_ << "\n";
    return;
#else
    if (!LoadKey()) {
        std::cerr << "[VAULT] " << unavailable_reason_ << "\n";
        return;
    }
    std::error_code ec;
    std::filesystem::create_directories(params_.vault_dir, ec);
    if (ec) {
        unavailable_reason_ =
            "vault 디렉토리 생성 실패: " + params_.vault_dir + " (" +
            ec.message() + ")";
        std::cerr << "[VAULT] " << unavailable_reason_ << "\n";
        return;
    }
    available_ = true;
    std::cout << "[VAULT] 원본 암호화 보관 활성 — dir=" << params_.vault_dir
              << " 보존 " << params_.retain_days << "일 (AES-256-GCM)\n";
#endif
}

EvidenceVault::~EvidenceVault() {
    // 키를 메모리에서 지운다 (best-effort)
    if (!key_.empty()) {
        std::fill(key_.begin(), key_.end(), 0);
    }
}

// ── 키 로드 ──────────────────────────────────────────────────────────
bool EvidenceVault::LoadKey() {
    std::string hex = params_.key_hex;

    if (hex.empty() && !params_.key_file.empty()) {
        std::ifstream in(params_.key_file);
        if (in) {
            std::string line;
            while (std::getline(in, line)) {
                if (!line.empty() && line.back() == '\r') line.pop_back();
                // "ECOWARDEN_EVIDENCE_KEY=<hex>" 또는 hex 단독
                const std::string kPrefix = "ECOWARDEN_EVIDENCE_KEY=";
                if (line.rfind(kPrefix, 0) == 0) {
                    hex = line.substr(kPrefix.size());
                    break;
                }
                if (line.size() == kKeyLen * 2 &&
                    line.find_first_not_of("0123456789abcdefABCDEF") ==
                        std::string::npos) {
                    hex = line;
                    break;
                }
            }
        }
    }

    if (hex.empty()) {
        unavailable_reason_ =
            "암호화 키 없음 — ECOWARDEN_EVIDENCE_KEY(hex 64자) 또는 " +
            params_.key_file + " 에 설정하세요. "
            "생성: openssl rand -hex 32";
        return false;
    }
    if (!HexToBytes(hex, &key_) || key_.size() != kKeyLen) {
        std::fill(key_.begin(), key_.end(), 0);
        key_.clear();
        unavailable_reason_ =
            "암호화 키 형식 오류 — 정확히 64자 hex(32바이트)여야 합니다";
        return false;
    }
    return true;
}

// ── 저장 ─────────────────────────────────────────────────────────────
VaultStoreResult EvidenceVault::StoreOriginal(const std::string& name,
                                              const uint8_t* data,
                                              size_t size) {
    VaultStoreResult r;
    r.plain_bytes = size;

    if (!available_) {
        r.error = unavailable_reason_.empty() ? "vault 사용 불가"
                                              : unavailable_reason_;
        return r;
    }
    if (data == nullptr || size == 0) {
        r.error = "빈 데이터";
        return r;
    }

#ifdef USE_OPENSSL
    // ── 헤더 구성 ────────────────────────────────────────────────────
    std::vector<uint8_t> out(kPayloadStart + size + 16);
    std::memcpy(out.data(), kMagic, 4);
    out[4] = kFormatVer;
    out[5] = out[6] = out[7] = 0;
    PutU64(out.data() + 8, static_cast<uint64_t>(NowUnix()));
    PutU32(out.data() + 16, params_.retain_days);

    uint8_t* nonce = out.data() + kHeaderAad;
    uint8_t* tag   = out.data() + kHeaderAad + kNonceLen;
    if (RAND_bytes(nonce, static_cast<int>(kNonceLen)) != 1) {
        r.error = "난수 생성 실패";
        return r;
    }

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (ctx == nullptr) {
        r.error = "EVP 컨텍스트 생성 실패";
        return r;
    }

    bool ok = true;
    int len = 0;
    int cipher_len = 0;

    ok = ok && EVP_EncryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr,
                                  nullptr) == 1;
    ok = ok && EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN,
                                   static_cast<int>(kNonceLen), nullptr) == 1;
    ok = ok && EVP_EncryptInit_ex(ctx, nullptr, nullptr, key_.data(),
                                  nonce) == 1;
    // 헤더를 AAD 로 묶어 메타데이터 위·변조도 검출한다.
    ok = ok && EVP_EncryptUpdate(ctx, nullptr, &len, out.data(),
                                 static_cast<int>(kHeaderAad)) == 1;
    ok = ok && EVP_EncryptUpdate(ctx, out.data() + kPayloadStart, &len, data,
                                 static_cast<int>(size)) == 1;
    if (ok) cipher_len = len;
    ok = ok && EVP_EncryptFinal_ex(ctx, out.data() + kPayloadStart + cipher_len,
                                   &len) == 1;
    if (ok) cipher_len += len;
    ok = ok && EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_GET_TAG,
                                   static_cast<int>(kTagLen), tag) == 1;
    EVP_CIPHER_CTX_free(ctx);

    if (!ok) {
        r.error = "AES-256-GCM 암호화 실패";
        return r;
    }

    out.resize(kPayloadStart + cipher_len);

    std::error_code ec;
    std::filesystem::create_directories(params_.vault_dir, ec);
    const std::string path =
        (std::filesystem::path(params_.vault_dir) / (name + ".ewv")).string();

    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) {
        r.error = "파일 생성 실패: " + path;
        return r;
    }
    f.write(reinterpret_cast<const char*>(out.data()),
            static_cast<std::streamsize>(out.size()));
    f.close();
    if (!f) {
        r.error = "파일 쓰기 실패: " + path;
        return r;
    }

    // 소유자만 읽을 수 있게
    std::filesystem::permissions(
        path,
        std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
        std::filesystem::perm_options::replace, ec);

    r.ok = true;
    r.path = path;
    r.cipher_bytes = out.size();
    return r;
#else
    (void)name;
    r.error = "OpenSSL 없이 빌드됨";
    return r;
#endif
}

// ── 열람 (복호화 + 로그) ─────────────────────────────────────────────
bool EvidenceVault::OpenOriginal(const std::string& path,
                                 std::vector<uint8_t>* out,
                                 const std::string& reason,
                                 const std::string& actor) {
    if (out == nullptr) return false;
    out->clear();

    const std::string log_base =
        IsoUtc(NowUnix()) + "\tpath=" + SanitizeField(path) +
        "\tactor=" + SanitizeField(actor) + "\treason=" + SanitizeField(reason);

    if (!available_) {
        AppendAccessLog(log_base + "\tresult=DENIED_UNAVAILABLE");
        return false;
    }

#ifdef USE_OPENSSL
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        AppendAccessLog(log_base + "\tresult=FAIL_OPEN");
        return false;
    }
    std::vector<uint8_t> buf((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
    f.close();

    if (buf.size() < kPayloadStart ||
        std::memcmp(buf.data(), kMagic, 4) != 0 || buf[4] != kFormatVer) {
        AppendAccessLog(log_base + "\tresult=FAIL_FORMAT");
        return false;
    }

    const uint8_t* nonce  = buf.data() + kHeaderAad;
    const uint8_t* tag    = buf.data() + kHeaderAad + kNonceLen;
    const uint8_t* cipher = buf.data() + kPayloadStart;
    const int cipher_len  = static_cast<int>(buf.size() - kPayloadStart);

    out->resize(static_cast<size_t>(cipher_len) + 16);

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (ctx == nullptr) {
        AppendAccessLog(log_base + "\tresult=FAIL_CTX");
        out->clear();
        return false;
    }

    bool ok = true;
    int len = 0;
    int plain_len = 0;

    ok = ok && EVP_DecryptInit_ex(ctx, EVP_aes_256_gcm(), nullptr, nullptr,
                                  nullptr) == 1;
    ok = ok && EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_IVLEN,
                                   static_cast<int>(kNonceLen), nullptr) == 1;
    ok = ok && EVP_DecryptInit_ex(ctx, nullptr, nullptr, key_.data(),
                                  nonce) == 1;
    ok = ok && EVP_DecryptUpdate(ctx, nullptr, &len, buf.data(),
                                 static_cast<int>(kHeaderAad)) == 1;
    ok = ok && EVP_DecryptUpdate(ctx, out->data(), &len, cipher,
                                 cipher_len) == 1;
    if (ok) plain_len = len;
    ok = ok && EVP_CIPHER_CTX_ctrl(ctx, EVP_CTRL_GCM_SET_TAG,
                                   static_cast<int>(kTagLen),
                                   const_cast<uint8_t*>(tag)) == 1;
    // 태그 검증은 여기서 일어난다 — 실패하면 위·변조된 파일.
    const bool tag_ok =
        ok && EVP_DecryptFinal_ex(ctx, out->data() + plain_len, &len) == 1;
    if (tag_ok) plain_len += len;
    EVP_CIPHER_CTX_free(ctx);

    if (!tag_ok) {
        out->clear();
        AppendAccessLog(log_base + "\tresult=FAIL_TAG_TAMPERED");
        return false;
    }

    out->resize(static_cast<size_t>(plain_len));
    AppendAccessLog(log_base + "\tresult=OK\tbytes=" +
                    std::to_string(plain_len));
    return true;
#else
    AppendAccessLog(log_base + "\tresult=DENIED_NO_OPENSSL");
    return false;
#endif
}

// ── 만료 판정 ────────────────────────────────────────────────────────
bool EvidenceVault::IsExpired(const std::string& path, int64_t now_unix) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    uint8_t hdr[kHeaderAad];
    f.read(reinterpret_cast<char*>(hdr), static_cast<std::streamsize>(kHeaderAad));
    if (f.gcount() != static_cast<std::streamsize>(kHeaderAad)) return false;
    if (std::memcmp(hdr, kMagic, 4) != 0) return false;

    const int64_t created = static_cast<int64_t>(GetU64(hdr + 8));
    const uint32_t days   = GetU32(hdr + 16);
    if (days == 0) return false;   // 무기한

    const int64_t expire_at = created + static_cast<int64_t>(days) * 86400;
    return now_unix >= expire_at;
}

// ── 보존기한 자동 파기 ───────────────────────────────────────────────
VaultSweepResult EvidenceVault::SweepExpired() {
    VaultSweepResult r;
    std::error_code ec;
    if (!std::filesystem::exists(params_.vault_dir, ec)) return r;

    const int64_t now = NowUnix();
    for (const auto& entry :
         std::filesystem::directory_iterator(params_.vault_dir, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        if (entry.path().extension() != ".ewv") continue;

        ++r.scanned;
        const std::string p = entry.path().string();
        if (!IsExpired(p, now)) continue;

        std::error_code rm_ec;
        if (std::filesystem::remove(p, rm_ec)) {
            ++r.deleted;
            AppendAccessLog(IsoUtc(now) + "\tpath=" + SanitizeField(p) +
                            "\tactor=system\treason=retention_expired"
                            "\tresult=DELETED");
        } else {
            ++r.failed;
        }
    }
    if (r.deleted > 0) {
        std::cout << "[VAULT] 보존기한 만료 " << r.deleted << "건 파기 (검사 "
                  << r.scanned << "건)\n";
    }
    return r;
}

// ── 열람 로그 ────────────────────────────────────────────────────────
void EvidenceVault::AppendAccessLog(const std::string& line) {
    if (params_.access_log.empty()) return;
    std::lock_guard<std::mutex> lock(g_log_mutex);

    std::error_code ec;
    const std::filesystem::path p(params_.access_log);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path(), ec);
    }
    std::ofstream f(params_.access_log, std::ios::app);
    if (!f) return;
    f << line << "\n";
}

} // namespace ecowarden
