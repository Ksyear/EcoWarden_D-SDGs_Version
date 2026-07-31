/*
 * Copyright (c) 2026 김승연
 *
 * This software is released under the MIT License.
 * See LICENSE file in the project root for details.
 *
 * Project: EcoWarden — LiDAR 기반 사생활 보호형 불법 투기 감지 시스템
 * Module : EMBEDDED - RplidarS2 LiDAR 센서 인터페이스 및 객체 탐지
 */

/**
 * @file  kalman_filter.h
 * @brief 2D 등속 선형 칼만 필터 (header-only)
 * @date  2026
 *
 * 상태 벡터: [x, y, vx, vy] (위치 + 속도)
 * 관측 벡터: [x, y] (위치만 관측)
 * 모션 모델: 등속 선형 (Constant Velocity)
 *
 * 행렬 연산을 외부 라이브러리 없이 4x4 고정 크기로 구현.
 */

#pragma once

#include <cmath>
#include <cstring>

namespace ecowarden {

// -- 4x4 행렬 (스택 할당, 고정 크기) --
struct Mat4 {
    double m[4][4] = {};

    Mat4() { std::memset(m, 0, sizeof(m)); }

    static Mat4 Identity() {
        Mat4 I;
        I.m[0][0] = I.m[1][1] = I.m[2][2] = I.m[3][3] = 1.0;
        return I;
    }

    Mat4 operator+(const Mat4& b) const {
        Mat4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.m[i][j] = m[i][j] + b.m[i][j];
        return r;
    }

    Mat4 operator-(const Mat4& b) const {
        Mat4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.m[i][j] = m[i][j] - b.m[i][j];
        return r;
    }

    Mat4 operator*(const Mat4& b) const {
        Mat4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k)
                    r.m[i][j] += m[i][k] * b.m[k][j];
        return r;
    }
};

// -- 4x1 벡터 --
struct Vec4 {
    double v[4] = {};

    Vec4() { std::memset(v, 0, sizeof(v)); }
    Vec4(double a, double b, double c, double d) {
        v[0] = a; v[1] = b; v[2] = c; v[3] = d;
    }
};

inline Vec4 MatVecMul(const Mat4& A, const Vec4& x) {
    Vec4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            r.v[i] += A.m[i][j] * x.v[j];
    return r;
}

// -- 2x2 행렬 역행렬 (칼만 게인 계산용) --
struct Mat2 {
    double m[2][2] = {};
};

// Invert2x2: 성공 시 true, 특이 행렬(det ≈ 0) 시 false 반환.
// 호출자는 false 시 Update를 스킵해야 한다 (Predict 상태 유지가 수학적으로 올바름).
inline bool Invert2x2(const Mat2& a, Mat2& out) {
    double det = a.m[0][0] * a.m[1][1] - a.m[0][1] * a.m[1][0];
    if (std::fabs(det) < 1e-12) {
        return false;
    }
    double inv_det = 1.0 / det;
    out.m[0][0] =  a.m[1][1] * inv_det;
    out.m[0][1] = -a.m[0][1] * inv_det;
    out.m[1][0] = -a.m[1][0] * inv_det;
    out.m[1][1] =  a.m[0][0] * inv_det;
    return true;
}

// -- 2D 등속 칼만 필터 --
class KalmanFilter2D {
public:
    KalmanFilter2D() = default;

    /**
     * @brief 초기 상태 설정
     * @param x0  초기 X 위치 (mm)
     * @param y0  초기 Y 위치 (mm)
     * @param process_noise   프로세스 노이즈 (클수록 모델 불확실)
     * @param measure_noise   관측 노이즈 (클수록 관측 불확실)
     */
    void Init(double x0, double y0,
              double process_noise = 50.0,
              double measure_noise = 100.0)
    {
        // 상태: [x, y, vx, vy]
        state_ = Vec4(x0, y0, 0.0, 0.0);

        // 상태 전이 행렬 F (dt=1 frame)
        F_ = Mat4::Identity();
        F_.m[0][2] = 1.0;  // x += vx * dt
        F_.m[1][3] = 1.0;  // y += vy * dt

        // 오차 공분산 P (초기: 큰 불확실성)
        P_ = Mat4::Identity();
        P_.m[0][0] = 1000.0;
        P_.m[1][1] = 1000.0;
        P_.m[2][2] = 1000.0;
        P_.m[3][3] = 1000.0;

        // 프로세스 노이즈 Q (dt=1 기준 초기값, Predict(dt)에서 스케일링)
        q_base_ = process_noise;
        Q_ = Mat4();
        Q_.m[0][0] = q_base_ * 0.25;  Q_.m[0][2] = q_base_ * 0.5;
        Q_.m[1][1] = q_base_ * 0.25;  Q_.m[1][3] = q_base_ * 0.5;
        Q_.m[2][0] = q_base_ * 0.5;   Q_.m[2][2] = q_base_;
        Q_.m[3][1] = q_base_ * 0.5;   Q_.m[3][3] = q_base_;

        // 관측 노이즈 R
        R_[0][0] = measure_noise;
        R_[0][1] = 0.0;
        R_[1][0] = 0.0;
        R_[1][1] = measure_noise;

        initialized_ = true;
    }

    /**
     * @brief 예측 단계: 다음 프레임 위치를 예측
     * @param dt  프레임 간격 (frame 단위, 기본 1.0). 실측 간격 전달 시 지터 내성 향상.
     */
    void Predict(double dt = 1.0) {
        if (!initialized_) return;

        // dt 에 맞게 F 행렬 갱신 (등속 모델: x += vx*dt)
        F_.m[0][2] = dt;
        F_.m[1][3] = dt;

        // Q 도 dt 에 비례하여 스케일링 (연속 시간 모델 이산화 근사)
        Mat4 Q_scaled = Q_;
        double dt2 = dt * dt;
        Q_scaled.m[0][0] = q_base_ * 0.25 * dt2 * dt2;
        Q_scaled.m[0][2] = q_base_ * 0.5  * dt2 * dt;
        Q_scaled.m[1][1] = q_base_ * 0.25 * dt2 * dt2;
        Q_scaled.m[1][3] = q_base_ * 0.5  * dt2 * dt;
        Q_scaled.m[2][0] = q_base_ * 0.5  * dt2 * dt;
        Q_scaled.m[2][2] = q_base_ * dt2;
        Q_scaled.m[3][1] = q_base_ * 0.5  * dt2 * dt;
        Q_scaled.m[3][3] = q_base_ * dt2;

        // x' = F * x
        state_ = MatVecMul(F_, state_);

        // P' = F * P * F^T + Q
        Mat4 FP = F_ * P_;
        Mat4 Ft = Transpose(F_);
        P_ = FP * Ft + Q_scaled;
    }

    /**
     * @brief 갱신 단계: 실측값으로 상태를 보정
     * @param mx  관측 X (mm)
     * @param my  관측 Y (mm)
     */
    void Update(double mx, double my) {
        if (!initialized_) return;

        // 관측 잔차: z - H * x  (H = [[1,0,0,0],[0,1,0,0]])
        double y0 = mx - state_.v[0];
        double y1 = my - state_.v[1];

        // S = H * P * H^T + R  (2x2)
        Mat2 S;
        S.m[0][0] = P_.m[0][0] + R_[0][0];
        S.m[0][1] = P_.m[0][1] + R_[0][1];
        S.m[1][0] = P_.m[1][0] + R_[1][0];
        S.m[1][1] = P_.m[1][1] + R_[1][1];

        // S^-1 — 특이 행렬이면 Update 스킵 (Predict 상태 유지)
        Mat2 Sinv;
        if (!Invert2x2(S, Sinv)) return;

        // K = P * H^T * S^-1  (4x2)
        // K[i][j] = P[i][0]*Sinv[0][j] + P[i][1]*Sinv[1][j]
        double K[4][2];
        for (int i = 0; i < 4; ++i) {
            K[i][0] = P_.m[i][0] * Sinv.m[0][0] + P_.m[i][1] * Sinv.m[1][0];
            K[i][1] = P_.m[i][0] * Sinv.m[0][1] + P_.m[i][1] * Sinv.m[1][1];
        }

        // x = x + K * y
        for (int i = 0; i < 4; ++i) {
            state_.v[i] += K[i][0] * y0 + K[i][1] * y1;
        }

        // P = (I - K*H) * P * (I - K*H)^T + K * R * K^T  (Joseph form, 수치 안정)
        Mat4 KH = Mat4();
        for (int i = 0; i < 4; ++i) {
            KH.m[i][0] = K[i][0];
            KH.m[i][1] = K[i][1];
        }
        Mat4 I_KH = Mat4::Identity() - KH;
        Mat4 I_KH_T = Transpose(I_KH);
        Mat4 P_joseph = I_KH * P_ * I_KH_T;

        // + K * R * K^T 항 추가
        // KRKt[i][j] = sum_a sum_b K[i][a] * R[a][b] * K[j][b]
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double val = 0.0;
                for (int a = 0; a < 2; ++a)
                    for (int b = 0; b < 2; ++b)
                        val += K[i][a] * R_[a][b] * K[j][b];
                P_joseph.m[i][j] += val;
            }
        }
        P_ = P_joseph;
    }

    // -- 상태 접근자 --
    double GetX()  const { return state_.v[0]; }
    double GetY()  const { return state_.v[1]; }
    double GetVX() const { return state_.v[2]; }
    double GetVY() const { return state_.v[3]; }

    // 위치(x,y) 불확실성 합. AssociateGreedy 2nd pass 적응형 게이트에서 사용.
    double PositionUncertaintySq() const { return P_.m[0][0] + P_.m[1][1]; }

    bool IsInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
    Vec4 state_;
    Mat4 F_;         // 상태 전이 행렬
    Mat4 P_;         // 오차 공분산
    Mat4 Q_;         // 프로세스 노이즈 (dt=1 기준)
    double q_base_ = 50.0; // 프로세스 노이즈 기저값 (Predict에서 dt 스케일링)
    double R_[2][2] = {};  // 관측 노이즈 (2x2)

    static Mat4 Transpose(const Mat4& a) {
        Mat4 r;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                r.m[i][j] = a.m[j][i];
        return r;
    }
};

} // namespace ecowarden
