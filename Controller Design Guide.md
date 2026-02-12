# Controller Design Guide

이 문서는 쿼드콥터 MPC 제어 시스템의 **최종 형태**를 정리한 문서임.
Position MPC + Yaw PID 분리 제어 구조를 기반으로, Level 3 (Attitude Setpoint) 인터페이스를 사용함.

> **docs** 폴더와 **system_model** 폴더를 참고할 것.

---

## 1. 전체 제어 구조

![Control System Structure](system_model/images/full_control_system_structure.png)


---

## 2. Position 제어 (MPC)

### 2.1 상태 벡터 & 입력 벡터

```math
\mathbf{x}_{pos} = \begin{bmatrix} p_x \\ p_y \\ p_z \\ v_x \\ v_y \\ v_z \end{bmatrix} \in \mathbb{R}^6, \quad
\mathbf{u}_{pos} = \begin{bmatrix} \phi_d \\ \theta_d \\ \Delta T \end{bmatrix} \in \mathbb{R}^3
```

| 기호 | 의미 | 단위 | 프레임 |
|------|------|------|--------|
| $p_x, p_y, p_z$ | 위치 | m | World (ENU) |
| $v_x, v_y, v_z$ | 속도 | m/s | World (ENU) |
| $\phi_d$ | 목표 Roll 각도 | rad | Body |
| $\theta_d$ | 목표 Pitch 각도 | rad | Body |
| $\Delta T$ | 추력 변화량 ($T - mg$) | N | Body z축 |

### 2.2 동역학 모델

**비선형 병진 운동 방정식:**

```math
\ddot{p}_x = \frac{T}{m}(\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi)
```
```math
\ddot{p}_y = \frac{T}{m}(\sin\psi \sin\theta \cos\phi - \cos\psi \sin\phi)
```
```math
\ddot{p}_z = \frac{T}{m}\cos\theta \cos\phi - g
```

**선형화 (Hover 근방, 작은 각도 가정, $\psi = 0$):**

$\sin\alpha \approx \alpha$, $\cos\alpha \approx 1$ 적용:

```math
\ddot{p}_x \approx g \cdot \theta_d, \quad
\ddot{p}_y \approx -g \cdot \phi_d, \quad
\ddot{p}_z \approx \frac{\Delta T}{m}
```

> **핵심 가정**: PX4 내부 자세 제어기가 $\phi_d, \theta_d$를 빠르게 추종 → 입력이 즉시 가속도에 반영된다고 가정

### 2.3 상태공간 모델

**연속 시간:**

```math
\dot{\mathbf{x}} = A_c \mathbf{x} + B_c \mathbf{u}
```

```math
A_c = \begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}, \quad
B_c = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & g & 0 \\
-g & 0 & 0 \\
0 & 0 & \frac{1}{m}
\end{bmatrix}
```

**이산화 (ZOH, $dt$ = 0.05 s / 20 Hz):**

```math
\mathbf{x}[k+1] = A_d \mathbf{x}[k] + B_d \mathbf{u}[k]
```

$A_d, B_d$는 MATLAB `c2d(sys_c, dt, 'zoh')`로 계산.

### 2.4 MPC 최적화 문제

**비용 함수:**

```math
J = \sum_{k=1}^{N} \left[ (\mathbf{x}_k - \mathbf{x}_{ref})^T Q (\mathbf{x}_k - \mathbf{x}_{ref}) + \mathbf{u}_k^T R \, \mathbf{u}_k \right]
```

**QP 형태로 변환:**

예측 모델 $X = \Phi \mathbf{x}_0 + \Gamma U$ 를 대입하면:

```math
\min_U \; \frac{1}{2} U^T H U + f^T U
```

```math
H = 2(\Gamma^T \bar{Q} \Gamma + \bar{R}), \quad
f = 2\Gamma^T \bar{Q}(\Phi \mathbf{x}_0 - X_{ref})
```

**MPC 파라미터:**

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| $N$ | 20 | Prediction horizon (20 × 0.05 = 1.0 s) |
| $Q$ | diag(50, 50, 100, 5, 5, 5) | 상태 가중치 (위치 > 속도, 고도 강조) |
| $R$ | diag(0.5, 0.5, 0.05) | 입력 가중치 (적극적 제어) |
| Solver | `quadprog` | MATLAB QP solver |

**제약 조건:**

| 제약 | 범위 | 비고 |
|------|------|------|
| $\phi_d$ | ±30° | Roll 자세 제한 |
| $\theta_d$ | ±30° | Pitch 자세 제한 |
| $\Delta T$ | ±19.62 N (±$mg$) | 추력 변화 제한 |
| $v_x, v_y, v_z$ | ±3.0 m/s | 속도 Hard Constraint |

---

## 3. Yaw 제어 (PID)

### 3.1 상태 벡터 & 입력 벡터

```math
\mathbf{x}_{yaw} = \begin{bmatrix} \psi \\ r \end{bmatrix} \in \mathbb{R}^2, \quad
u_{yaw} = \tau_z \in \mathbb{R}
```

| 기호 | 의미 | 단위 |
|------|------|------|
| $\psi$ | Yaw 각도 | rad |
| $r$ | Yaw rate (body rate) | rad/s |
| $\tau_z$ | Yaw 토크 | N·m |

> **근사**: hover 근방 ($\phi \approx 0$, $\theta \approx 0$)에서 $\dot{\psi} \approx r$ (body rate ≈ Euler rate)

### 3.2 동역학 모델

**Euler 회전 방정식** (자이로스코픽 커플링 무시):

```math
I_{zz} \dot{r} = \tau_z \quad \Rightarrow \quad \dot{r} = \frac{\tau_z}{I_{zz}}
```

**상태공간 (연속):**

```math
\begin{bmatrix} \dot{\psi} \\ \dot{r} \end{bmatrix} =
\begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}
\begin{bmatrix} \psi \\ r \end{bmatrix} +
\begin{bmatrix} 0 \\ \frac{1}{I_{zz}} \end{bmatrix} \tau_z
```

**이산화 (ZOH):**

```math
A_{d,yaw} = \begin{bmatrix} 1 & dt \\ 0 & 1 \end{bmatrix}, \quad
B_{d,yaw} = \begin{bmatrix} \frac{dt^2}{2 I_{zz}} \\ \frac{dt}{I_{zz}} \end{bmatrix}
```

> **구조**: 이중 적분기 (토크 → 각가속도 → 각속도 → 각도)

### 3.3 PID 제어법칙

```math
\tau_z = K_p \cdot e_\psi + K_i \cdot \int e_\psi \, dt - K_d \cdot r
```

| 항 | 수식 | 설명 |
|---|------|------|
| **P** | $K_p \cdot \text{wrapToPi}(\psi_d - \psi)$ | 각도 오차, ±π 범위 wrapping |
| **I** | $K_i \cdot \int e_\psi \, dt$ | 적분 오차 + Anti-windup |
| **D** | $-K_d \cdot r$ | 현재 rate 사용 (derivative kick 방지) |

**PID 파라미터:**

| 파라미터 | 값 | 단위 | 설계 근거 |
|---------|-----|------|----------|
| $K_p$ | 0.2 | N·m/rad | $I_{zz} \cdot \omega_n^2$ ($\omega_n \approx 2$) |
| $K_i$ | 0.05 | N·m/(rad·s) | 정상상태 오차 제거 |
| $K_d$ | 0.15 | N·m/(rad/s) | $I_{zz} \cdot 2\zeta\omega_n$ ($\zeta \approx 0.8$) |
| $\tau_{max}$ | 0.5 | N·m | 토크 saturation |

---

## 4. 물리 파라미터 (x500 기체 from Gazebo SDF)

| 파라미터 | 기호 | 값 | 단위 | 출처 |
|---------|------|-----|------|------|
| 질량 | $m$ | 2.0 | kg | Gazebo SDF |
| Roll 관성 | $I_{xx}$ | 0.0217 | kg·m² | Gazebo SDF |
| Pitch 관성 | $I_{yy}$ | 0.0217 | kg·m² | Gazebo SDF |
| Yaw 관성 | $I_{zz}$ | 0.0400 | kg·m² | Gazebo SDF |
| 중력가속도 | $g$ | 9.81 | m/s² | 상수 |
| Hover 추력 | $T_{hover}$ | 19.62 | N | $m \cdot g$ |
| 샘플링 시간 | $dt$ | 0.05 | s | 20 Hz |

> 추후 실기체에서 System Identification(시스템 식별)을 통해 이 값을 수정할 예정

---

## 5. Position-Yaw 커플링 처리

### 왜 분리 제어인가?

비선형 병진 가속도에 $\psi$가 포함되어 있어 Position과 Yaw는 본질적으로 커플링됨:

```math
\ddot{p}_x = \frac{T}{m}(\cos\psi \sin\theta \cos\phi + \sin\psi \sin\phi)
```

그러나 **$\psi = 0$ 근방 선형화**를 통해 커플링을 제거:
```math
\ddot{p}_x \approx g \cdot \theta, \quad \ddot{p}_y \approx -g \cdot \phi
```

| 항목 | 현재 (분리 제어) | 통합 MPC |
|------|-----------------|---------|
| Position 모델 | 6-상태 선형 | 8-상태 (비선형 가능) |
| Yaw 모델 | 2-상태 PID | MPC에 통합 |
| Solver | `quadprog` (QP) | QP 또는 NMPC |
| 커플링 처리 | $\psi = 0$ 근방 무시 | 고려 가능 |
| 적용 범위 | Hover 근방 비행 | 넓은 비행 영역 |
| 구현 난이도 | 낮음 | 높음 |

> **현재 방식의 유효 조건**: Hover 근방, 큰 Yaw 기동 없음, Roll/Pitch 작은 각도

---

## 6. PX4 연동 (Level 3)

### 제어 출력 → PX4 메시지

```math
MPC 출력 : \begin{bmatrix} \phi_d \\ \theta_d \\ \Delta T \end{bmatrix} \quad
PID 출력 : \begin{bmatrix} \psi_d \end{bmatrix}
```
```math
\Rightarrow VehicleAttitudeSetpoint : \begin{bmatrix} q_d \\ thrust_{body} \end{bmatrix}
```

```cpp
// OffboardControlMode
mode.attitude = true;   // Level 3 활성화

// VehicleAttitudeSetpoint
msg.q_d = eul2quat(ψ_d, θ_d, φ_d);           // ZYX 순서
msg.thrust_body[2] = -(T_hover + ΔT) / T_max; // 정규화된 추력 (음수)
```

---

## 7. 파일 구조

| 파일 | 역할 |
|------|------|
| `matlab/quadcopter_model.m` | 6-상태 선형 모델 정의 & 이산화 |
| `matlab/mpc_setup.m` | MPC 파라미터 (Q, R, N, 제약조건) 설정 |
| `matlab/mpc_controller.m` | QP 문제 구성 & `quadprog` 호출 |
| `matlab/yaw_setup.m` | Yaw PID 파라미터 & 이산 모델 설정 |
| `matlab/yaw_pid_controller.m` | Yaw PID 제어 함수 |
| `matlab/mpc_waypoint_tracking.m` | Waypoint 추적 시뮬레이션 |
| `matlab/mpc_path_tracking.m` | 경로 추적 시뮬레이션 |

---

## 8. 핵심 가정 & 제한사항

| 가정 | 근거 | 제한 |
|------|------|------|
| 작은 각도 ($\phi, \theta \ll 1$) | Hover 근방 비행 | 급격한 기동 불가 |
| $\psi = 0$ 근방 | 커플링 무시 | 큰 Yaw 변화 시 성능 저하 |
| $\dot\psi \approx r$ | $\phi, \theta \approx 0$ | Body rate ≈ Euler rate 근사 |
| 내부 자세 제어 즉시 추종 | PX4 Rate Control 충분히 빠름 | 자세 제어 bandwidth 의존 |
| 강체 기체 | 진동/탄성 무시 | 구조적 유연성 미고려 |
