# Step 4 — Yaw-axis System Identification (Rotational Motion)

## 🎯 목적
드론의 **Yaw(회전)** 축에 대한 **1차 모델 파라미터**(`k_r`, `τ_r`)를 식별합니다.

---

## 1️⃣ 실험 설정
- Hover 상태 유지
- `r_cmd`에 ±30°/s (≈ ±0.52 rad/s) step 입력을 줍니다.
- 기록:
  - `/uav/odom/twist/twist.angular.z`
  - `r_cmd`

---

## 2️⃣ 모델
$$
\dot{r} = -\frac{1}{\tau_r}r + \frac{k_r}{\tau_r}r^{cmd}
$$
$$
r[k+1] = a_r r[k] + b_r r^{cmd}[k]
$$

---

## 3️⃣ 최소자승법
$$
\theta = [a_r, b_r]^T = (\Phi^T\Phi)^{-1}\Phi^TY
$$

---

## 4️⃣ 파라미터 변환
$$
\tau_r = -\frac{T_s}{\ln(a_r)}, \quad k_r = \frac{b_r}{1 - a_r}
$$

---

| 파라미터 | 기호 | 예시값 |
|-----------|-------|--------|
| 시정수 | τ_r | 0.30 s |
| 이득 | k_r | 1.10 |
| 샘플링 주기 | T_s | 0.02 s |

---

🎉 모든 축(Z, X, Y, Yaw)의 모델 식별이 완료되었습니다.
다음 단계는 이를 종합하여 **State-Space 모델 구성 및 MPC 설계**로 이어집니다.
