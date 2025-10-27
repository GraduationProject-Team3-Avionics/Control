# Step 2 — 데이터 수집(실험 설계 & 로깅)

## 목적
각 축(Roll/Pitch/Yaw/Altitude)의 **단일 입력–단일 출력(SISO)** 실험 데이터를 안전하게 수집해 식별에 사용.

## 입력 설계
- **PRBS** 또는 **멀티-사인/chirp** 권장 (작은 진폭, hover 근처 소신호)
- 채널별로 **하나씩**: 다른 채널은 제로명령 유지  
  - Roll: `vx_cmd` 또는 `θ_cmd`에 소신호 → 응답 \(v_x, φ, p\)
  - Pitch: `vy_cmd` 또는 `φ_cmd`에 소신호 → 응답 \(v_y, θ, q\)
  - Yaw: `r_cmd`에 소신호 → 응답 \(r, ψ\)
  - Alt: `vz_cmd`에 소신호 → 응답 \(v_z, z\)

## 로깅 토픽(예시)
- 입력: `/fmu/in/trajectory_setpoint` (명령), 또는 자체 퍼블리시한 cmd
- 상태: `/uav/odom` (x,y,z,orientation), IMU가 있다면 각속도 토픽도 병행
- 추가: TF(`map↔odom` yaw 보정), 상태 추정 품질 신호(있다면)

## 실행 절차(예시)
1. 시뮬/실기 환경 구동 → hover 진입
2. `record_<axis>.launch.py` 실행 → 해당 축 cmd 신호 주입 + rosbag record
3. 20–40 s 가량 데이터 수집 (여러 주파수/진폭 포함)
4. 안전 착륙 및 disarm

## 파일 네이밍
```
bags/
  roll_YYYYMMDD_HHMM.bag
  pitch_YYYYMMDD_HHMM.bag
  yaw_YYYYMMDD_HHMM.bag
  alt_YYYYMMDD_HHMM.bag
```

## 체크리스트
- 진폭: 명령 최대치의 30–50% 이내
- hover 유지, 위치 드리프트 과대 시 즉시 중지
- PRBS 주기, 멀티-사인 주파수 목록 기록
