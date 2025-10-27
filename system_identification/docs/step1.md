# Step 1 — 프로젝트 셋업 & 범위 정의

## 목적
쿼드콥터 **System Identification 패키지**를 체계적으로 진행하기 위한 기본 뼈대와 범위를 확정한다.  
(상위 제어는 *velocity/yaw-rate offboard* 인터페이스 기준, 향후 *acceleration 입력형*으로 확장 가능)

## 산출물
- 패키지 디렉터리 구조
- 상태/입력/출력 정의 문서
- 사용 토픽, 좌표계, 샘플링 주기(Ts) 명세
- 실험 안전 규칙 및 체크리스트

## 권장 디렉터리 구조
```
system_identification/
├─ config/
│  ├─ topics.yaml            # 기록할 토픽, QoS, frame 이름
│  ├─ ident_params.yaml      # 필터/리샘플/미분/창길이 등
│  └─ model_export.yaml      # A,B,C,D 저장 형식/정밀도
├─ launch/
│  ├─ record_roll.launch.py
│  ├─ record_pitch.launch.py
│  ├─ record_yaw.launch.py
│  └─ record_alt.launch.py
├─ scripts/
│  ├─ preprocess.py          # rosbag → csv, 필터, 동기화, 미분
│  ├─ fit_roll.py            # 축별 식별 스크립트
│  ├─ fit_pitch.py
│  ├─ fit_yaw.py
│  ├─ fit_alt.py
│  ├─ validate_models.py     # 시뮬/잔차/교차상관
│  └─ discretize_export.py   # 연속→이산, YAML/JSON 내보내기
├─ notebooks/
│  ├─ ident_roll.ipynb
│  ├─ ident_pitch.ipynb
│  ├─ ident_yaw.ipynb
│  └─ ident_alt.ipynb
├─ models/
│  ├─ roll_ss.yml
│  ├─ pitch_ss.yml
│  ├─ yaw_ss.yml
│  └─ alt_ss.yml
└─ README.md
```

## 상태/입력/출력(초기안)
### 1. 상태 벡터 (State Vector)
$$
x = [x, y, z, v_x, v_y, v_z, \psi]^\top
$$
* $x, y, z$: 위치 (Position)
* $v_x, v_y, v_z$: 속도 (Velocity)
* $\psi$: 요 각 (Yaw angle)

### 2. 입력 (Input - 현행 인터페이스)
$$
u = [v_x^{cmd}, v_y^{cmd}, v_z^{cmd}, r^{cmd}]^\top
$$
* $v_x^{cmd}, v_y^{cmd}, v_z^{cmd}$: 목표 속도 (Commanded velocity)
* $r^{cmd}$: 목표 요 각속도 (Commanded yaw rate)

### 3. 출력 (Output)
$$
y = [x, y, z, \psi]^\top
$$

### 4. 속도 추종 1차 근사 (Velocity Tracking 1st-order Approx.)

시스템의 동역학을 1차 시스템으로 근사할 때, 다음과 같이 표현할 수 있습니다.

* **선형 속도 (Linear Velocity):**
    $$
    \dot{v} = -\frac{1}{T_v}(v - v^{cmd})
    $$
    (여기서 $T_v$는 속도 응답의 시정수입니다.)

* **각속도 (Angular Velocity):**

    * **옵션 1 (Rate Command Model):** 요 각속도($r^{cmd}$)를 직접 추종하는 경우
    $$
    \dot{\psi} = r^{cmd}
    $$

    * **옵션 2 (Angle Command Model):** (만약 인터페이스가 $\psi^{cmd}$를 받는 경우) 목표 각도를 추종하는 1차 시스템
    $$
    \dot{\psi} = -\frac{1}{T_\psi}(\psi - \psi^{cmd})
    $$
    (여기서 $T_\psi$는 자세 응답의 Time Constant 입니다.)

> 향후 가속도 입력형(더블 인티그레이터)로도 병행 예정.

## 좌표계/프레임
- 기록 기준: **ENU(odom/map)**  
- PX4 명령: **NED**이므로 변환 로직은 기존 offboard 노드와 동일 사용

## 샘플링 주기
- 제어/기록: 50 Hz 권장 (20–100 Hz 범위)

## 안전 체크리스트
- 배터리/프로펠러/프롭가드 확인
- 비행 금지구역 확인, 인원/물체 3m 이상 이격
- 초반 0.5–1.0 s **워밍업** 후 ARM/OFFBOARD 전환
- 최대 속도/요율 제한 설정 (xy: 1 m/s, z: 0.8 m/s, yaw: 0.8 rad/s 기본)
