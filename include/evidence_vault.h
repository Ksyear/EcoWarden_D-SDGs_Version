/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - 증거 원본 암호화 보관소 (3계층 프라이버시 3계층)
 */

/**
 * @file  evidence_vault.h
 * @brief 마스킹 전 원본을 암호화 보관하고 보존기한이 지나면 자동 파기한다.
 *
 * 프라이버시 3계층 중 **3계층(잠금본)**을 담당한다.
 *
 *   1계층 평상시  : 카메라 미작동, 비식별 좌표만
 *   2계층 공개본  : 얼굴 마스킹본 저장·전송           (face_masking.h)
 *   3계층 원본    : 암호화 + 보존기한 자동 파기 + 열람 로그  ← 이 모듈
 *
 * ── 설계 원칙 ────────────────────────────────────────────────────────
 *
 * 1. **fail-closed**: 키가 없거나 OpenSSL 이 빠진 빌드면 원본을 *아예
 *    저장하지 않는다*. 평문으로 남기느니 안 남기는 게 낫다. 이 경우
 *    `Available()` 이 false 이고 `StoreOriginal()` 은 실패를 돌려준다.
 *
 * 2. **인증 암호화**: AES-256-GCM. 헤더(생성시각·보존기한)를 AAD 로
 *    묶어 메타데이터 위·변조도 검출된다. 목표 16(증거 무결성) 근거.
 *
 * 3. **열람 로그**: 복호화할 때마다 append-only 로그에 시각·사유·요청자를
 *    남긴다. "법적 절차로만 열람" 주장을 검증 가능하게 만드는 장치.
 *
 * 4. **보존기한 자동 파기**: `SweepExpired()` 가 기한 지난 파일을 지운다.
 *    파일 헤더에 보존기한이 들어 있으므로 파일만 봐도 판정 가능하다.
 *
 * ── 파일 포맷 (little-endian) ────────────────────────────────────────
 *
 *   offset  size  내용
 *   0       4     매직 "EWV1"
 *   4       1     포맷 버전 (=1)
 *   5       3     예약 (0)
 *   8       8     생성 시각 (unix epoch seconds)
 *   16      4     보존 일수 (0 = 무기한)
 *   20      12    GCM nonce
 *   32      16    GCM tag
 *   48      ...   ciphertext
 *
 *   AAD = offset 0~19 (헤더 20바이트)
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace ecowarden {

// ── 파라미터 ─────────────────────────────────────────────────────────
struct EvidenceVaultParams {
    // 원본 보관 자체를 켤지. 끄면 마스킹본만 남는다 (가장 보수적).
    bool enable = false;

    // 원본 보관 디렉토리
    std::string vault_dir = "captures/vault";

    // 열람 로그 경로 (append-only)
    std::string access_log = "captures/vault/access.log";

    // 보존 일수. 0 이면 무기한(권장하지 않음).
    uint32_t retain_days = 30;

    // 32바이트 키의 hex 표현(64자). 비어 있으면 key_file 에서 읽는다.
    std::string key_hex;

    // 키 파일 경로. `ECOWARDEN_EVIDENCE_KEY=<hex>` 형식 한 줄, 또는
    // hex 문자열만 있는 파일. 기본은 시크릿 파일과 같은 위치.
    std::string key_file = "/etc/ecowarden/secrets.conf";
};

// ── 보관 결과 ────────────────────────────────────────────────────────
struct VaultStoreResult {
    bool        ok = false;
    std::string path;         // 저장된 암호문 경로
    std::string error;
    size_t      plain_bytes  = 0;
    size_t      cipher_bytes = 0;
};

// ── 파기 결과 ────────────────────────────────────────────────────────
struct VaultSweepResult {
    uint32_t scanned = 0;
    uint32_t deleted = 0;
    uint32_t failed  = 0;
};

/**
 * @brief 증거 원본 암호화 보관소.
 *
 *   상태가 거의 없어서(키만 보유) 스레드 안전하다. 열람 로그 append 는
 *   내부에서 직렬화한다.
 */
class EvidenceVault {
public:
    explicit EvidenceVault(const EvidenceVaultParams& params = {});
    ~EvidenceVault();

    EvidenceVault(const EvidenceVault&) = delete;
    EvidenceVault& operator=(const EvidenceVault&) = delete;

    /**
     * @brief 암호화 저장이 가능한 상태인가.
     *   false 면 `StoreOriginal()` 은 항상 실패한다 — 평문 폴백은 없다.
     */
    bool Available() const { return available_; }

    // 사용 불가 사유 (진단·로그용)
    const std::string& unavailable_reason() const { return unavailable_reason_; }

    /**
     * @brief 원본 바이트를 암호화해 vault 에 저장한다.
     * @param name  파일명 베이스 (확장자 없이). `<vault_dir>/<name>.ewv`
     * @param data  평문 (JPEG 원본 등)
     */
    VaultStoreResult StoreOriginal(const std::string& name,
                                   const uint8_t* data, size_t size);

    VaultStoreResult StoreOriginal(const std::string& name,
                                   const std::vector<uint8_t>& data) {
        return StoreOriginal(name, data.data(), data.size());
    }

    /**
     * @brief vault 파일을 복호화한다. **열람 로그를 남긴다.**
     * @param path   `.ewv` 경로
     * @param out    복호화된 평문
     * @param reason 열람 사유 (예: "수사기관 협조 요청 2026-08-01")
     * @param actor  열람 주체 (예: "admin@daejeon.go.kr")
     * @return 성공 여부. 태그 검증 실패(위·변조) 시 false.
     */
    bool OpenOriginal(const std::string& path, std::vector<uint8_t>* out,
                      const std::string& reason, const std::string& actor);

    /**
     * @brief 보존기한이 지난 파일을 파기한다.
     *   운영 루프에서 주기적으로(하루 1회 등) 호출한다.
     */
    VaultSweepResult SweepExpired();

    /**
     * @brief 파일 헤더만 읽어 만료 여부를 판정한다 (복호화 없음).
     */
    static bool IsExpired(const std::string& path, int64_t now_unix);

    const EvidenceVaultParams& params() const { return params_; }

private:
    bool LoadKey();
    void AppendAccessLog(const std::string& line);

    EvidenceVaultParams params_;
    std::vector<uint8_t> key_;          // 32 bytes
    bool        available_ = false;
    std::string unavailable_reason_;
};

// ── 유틸 (테스트에서도 사용) ─────────────────────────────────────────
// hex 문자열 → 바이트. 실패 시 false.
bool HexToBytes(const std::string& hex, std::vector<uint8_t>* out);
// 바이트 → hex 소문자.
std::string BytesToHex(const uint8_t* data, size_t size);

} // namespace ecowarden
