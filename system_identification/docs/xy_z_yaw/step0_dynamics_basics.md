;# Step 0 — 드론 동역학 기본 개념 및 식 정리

## 1️⃣ 드론의 기본 좌표계

### 🔹 지구 기준 좌표계 (ENU or NED)
- **ENU:** +X → East, +Y → North, +Z → Up  
- **NED:** +X → North, +Y → East, +Z → Down (PX4 내부 기준)

### 🔹 기체 기준 좌표계 (Body Frame)
- +X: 전방, +Y: 우측, +Z: 하방  

### 🔹 회전행렬 (Body → Earth)
$$
R_{b}^{e} =
\begin{bmatrix}
c_\psi c_\theta & c_\psi s_\theta s_\phi - s_\psi c_\phi & c_\psi s_\theta c_\phi + s_\psi s_\phi \\\\
s_\psi c_\theta & s_\psi s_\theta s_\phi + c_\psi c_\phi & s_\psi s_\theta c_\phi - c_\psi s_\phi \\\\
-s_\theta & c_\theta s_\phi & c_\theta c_\phi
\end{bmatrix}
$$

여기서 $ \phi, \theta, \psi $ 는 각각 **roll, pitch, yaw**를 의미한다.

---

## 2️⃣ 6자유도(6DOF) 방정식

### 🔹 병진 운동 (Translational Motion)
$$
m \dot{v} = R_{b}^{e} f_b + mg
$$

- $m$: 드론 질량  
- $v = [\dot{x}, \dot{y}, \dot{z}]^T$: 속도 벡터  
- $f_b$: 기체 좌표계에서의 총 추력 (주로 z축 방향)  
- $R_{b}^{e}$: 바디 → 지구 회전행렬  
- $g$: 중력 가속도 벡터

### 🔹 회전 운동 (Rotational Motion)
$$
I \dot{\omega} = \tau - \omega \times (I \omega)
$$

- $I$: 관성 모멘트 행렬  
- $\omega = [p, q, r]^T$: 각속도 (roll, pitch, yaw rate)  
- $\tau$: 모터들에 의해 발생하는 토크 벡터

---

## 3️⃣ 모터 추력과 토크 관계

쿼드콥터(X형, +형)에 따라 다르지만 일반적으로:
$$
\begin{bmatrix}
T \\\\
\tau_\phi \\\\
\tau_\theta \\\\
\tau_\psi
\end{bmatrix}
=
\begin{bmatrix}
k_T & k_T & k_T & k_T \\\\
0 & -l k_T & 0 & l k_T \\\\
-l k_T & 0 & l k_T & 0 \\\\
k_Q & -k_Q & k_Q & -k_Q
\end{bmatrix}
\begin{bmatrix}
\omega_1^2 \\\\
\omega_2^2 \\\\
\omega_3^2 \\\\
\omega_4^2
\end{bmatrix}
$$

- $k_T$: 추력 상수  
- $k_Q$: 항력 상수  
- $l$: 프로펠러 중심에서 회전축까지의 거리  
- $\omega_i$: 각 모터의 회전속도  

---

## 4️⃣ 단순 선형화된 모델 (Hover 근처)

Hover(정지 비행) 근처에서는 비선형 항이 작다고 가정하여 선형 근사 가능하다.

### 🔹 Roll/Pitch Dynamics (근사)
$$
\ddot{\phi} = \frac{1}{I_x} \tau_\phi, \quad
\ddot{\theta} = \frac{1}{I_y} \tau_\theta
$$

- 모터 명령에 의해 생성된 토크가 각가속도에 비례  
- 공기저항이나 모멘트 커플링이 작을 때 2차 시스템으로 근사

### 🔹 Yaw Dynamics
$$
\dot{r} = \frac{1}{I_z} \tau_\psi
$$

Yaw는 상대적으로 느리고, 1차 시스템 근사 가능.

### 🔹 Altitude Dynamics
$$
m \ddot{z} = T - mg
$$

정상상태에서 $T = mg$, 작은 변동 시 1차/2차 시스템으로 선형 근사 가능.

---

## 5️⃣ 선형 상태공간 표현 예시

Hover 근처 선형화 후 상태벡터를 $x = [\phi, \dot{\phi}]^T$ 로 정의하면:
$$
\dot{x} =
\begin{bmatrix}
0 & 1 \\\\
0 & -\frac{d_\phi}{I_x}
\end{bmatrix} x +
\begin{bmatrix}
0 \\\\
\frac{1}{I_x}
\end{bmatrix} u
$$

출력은 $y = [1 \ 0]x$ 로 표현된다.  
이 식은 Step 4의 SISO 식별 모델 (2차 시스템)과 직접 대응된다.

---

## 6️⃣ 실제 시스템 식별 시 참고

| 축 | 입력 | 출력 | 추천 모델 형태 |
|----|------|------|----------------|
| Roll | roll_cmd 또는 vx_cmd | roll angle or rate | 2차 |
| Pitch | pitch_cmd 또는 vy_cmd | pitch angle or rate | 2차 |
| Yaw | yaw_rate_cmd | yaw_rate | 1차 |
| Altitude | vz_cmd | z | 1차~2차 |

---

이 문서는 step1~6을 수행하기 전 필수적으로 이해해야 할 동역학 기초를 담고 있다.
