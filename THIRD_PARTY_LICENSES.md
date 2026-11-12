# Third-Party Licenses

이 프로젝트에서 사용하는 서드파티 라이브러리의 라이선스 정보입니다.

| 라이브러리 | 라이선스 | 포함 방식 | 출처 |
|---|---|---|---|
| Slamtec rplidar_sdk | BSD 2-Clause | 저장소 동봉 (`thirdparty/rplidar_sdk`) | https://github.com/Slamtec/rplidar_sdk |
| nlohmann/json | MIT | 시스템 패키지 / 빌드 시 내려받기 | https://github.com/nlohmann/json |
| libcurl | curl license (MIT/X 계열) | 시스템 패키지 | https://curl.se/docs/copyright.html |
| OpenCV (선택) | Apache 2.0 | 시스템 패키지 | https://opencv.org/license/ |
| OpenSSL (선택) | Apache 2.0 | 시스템 패키지 | https://www.openssl.org/source/license.html |
| libgpiod (선택) | LGPL-2.1-or-later | 시스템 패키지 (동적 링크) | https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git |

## Slamtec rplidar_sdk

- **라이선스**: BSD 2-Clause License (전문: `thirdparty/rplidar_sdk/LICENSE`)
- **저작권자**: RoboPeak Team (2009–2014), Shanghai Slamtec Co., Ltd. (2014–2018)
- **출처**: https://github.com/Slamtec/rplidar_sdk
- **용도**: RPLiDAR S2 센서 시리얼 통신 및 스캔 데이터 수집

## nlohmann/json

- **라이선스**: MIT License
- **저작권자**: Niels Lohmann
- **출처**: https://github.com/nlohmann/json
- **용도**: 클러스터 및 이벤트 데이터의 JSON 직렬화

## libcurl

- **라이선스**: curl license (MIT/X 계열)
- **저작권자**: Daniel Stenberg and contributors
- **출처**: https://curl.se/docs/copyright.html
- **용도**: FastAPI 서버로의 HTTP POST 이벤트 전송 / 증거 영상 업로드

## OpenCV (선택 의존성)

- **라이선스**: Apache License 2.0 (OpenCV 4.5.0 이상)
- **출처**: https://opencv.org/license/
- **용도**: 카메라 캡처, 증거 사진 오버레이, 얼굴 마스킹(Haar cascade / DNN), 블랙박스 영상
- 없으면 카메라·마스킹 경로가 스텁으로 컴파일된다.

## OpenSSL (선택 의존성)

- **라이선스**: Apache License 2.0 (OpenSSL 3.0 이상)
- **출처**: https://www.openssl.org/source/license.html
- **용도**: 증거 원본 AES-256-GCM 암호화 보관
- 없으면 원본을 저장하지 않는다 (fail-closed).

## libgpiod (선택 의존성)

- **라이선스**: LGPL-2.1-or-later
- **출처**: https://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
- **용도**: Raspberry Pi 5 PIR 센서 GPIO 입력
- 없으면 sysfs GPIO 로 폴백한다.
