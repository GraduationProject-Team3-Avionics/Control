# Step 2 — X-axis System Identification (Forward Motion)

## 🎯 목적
드론의 **전후(X)축**에 대한 **1차 모델 파라미터**(`k_x`, `τ_x`)를 식별합니다.

---

## 1️⃣ 실험 설정
- 고도를 일정하게 유지합니다.
- `v_x_cmd`에 ±1 m/s 정도의 step 또는 sine 명령을 줍니다.
- 기록 항목:
  - `/uav/odom/twist/twist.linear.x` → 실제 `v_x`
  - `v_x_cmd`

---

## 2️⃣ 모델
$$
\dot{v}_x = -\frac{1}{\tau_x}v_x + \frac{k_x}{\tau_x}v_x^{cmd}
$$
$$
v_x[k+1] = a_x v_x[k] + b_x v_x^{cmd}[k]
$$

---

## 3️⃣ 최소자승법
$$
\theta = [a_x, b_x]^T = (\Phi^T\Phi)^{-1}\Phi^TY
$$

---

## 4️⃣ 파라미터 변환
$$
\tau_x = -\frac{T_s}{\ln(a_x)}, \quad k_x = \frac{b_x}{1 - a_x}
$$

---

| 파라미터 | 기호 | 예시값 |
|-----------|-------|--------|
| 시정수 | τ_x | 0.42 s |
| 이득 | k_x | 0.98 |
| 샘플링 주기 | T_s | 0.02 s |

---

✅ 다음: **Step 3 — Y축 식별**
