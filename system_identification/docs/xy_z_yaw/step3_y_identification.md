# Step 3 — Y-axis System Identification (Lateral Motion)

## 🎯 목적
드론의 **좌우(Y)축**에 대한 **1차 모델 파라미터**(`k_y`, `τ_y`)를 식별합니다.

---

## 1️⃣ 실험 설정
- Z축 고도를 일정하게 유지합니다.
- `v_y_cmd`에 ±1 m/s 정도의 step 입력을 줍니다.
- 기록:
  - `/uav/odom/twist/twist.linear.y`
  - `v_y_cmd`

---

## 2️⃣ 모델
$$
\dot{v}_y = -\frac{1}{\tau_y}v_y + \frac{k_y}{\tau_y}v_y^{cmd}
$$
$$
v_y[k+1] = a_y v_y[k] + b_y v_y^{cmd}[k]
$$

---

## 3️⃣ 최소자승법
$$
\theta = [a_y, b_y]^T = (\Phi^T\Phi)^{-1}\Phi^TY
$$

---

## 4️⃣ 파라미터 변환
$$
\tau_y = -\frac{T_s}{\ln(a_y)}, \quad k_y = \frac{b_y}{1 - a_y}
$$

---

| 파라미터 | 기호 | 예시값 |
|-----------|-------|--------|
| 시정수 | τ_y | 0.45 s |
| 이득 | k_y | 1.02 |
| 샘플링 주기 | T_s | 0.02 s |

---

✅ 다음: **Step 4 — Yaw축 식별**
