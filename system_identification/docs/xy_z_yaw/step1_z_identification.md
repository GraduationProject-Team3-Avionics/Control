# Step 1 — Z-axis System Identification (Vertical Motion)

## 🎯 목적
드론의 **상하(Z)축**에 대한 **1차 동특성 모델** 파라미터(`k_z`, `τ_z`)를 식별합니다.

---

## 1️⃣ 실험 설정
- 드론을 hover 상태로 유지합니다. (`/uav/odom` 활용)
- 수직 속도 명령 `v_z_cmd`에 대해 step 입력 (+0.5 m/s → 0 → -0.5 m/s 등)을 줍니다.
- 다음 데이터를 기록합니다:
  - `/uav/odom/twist/twist.linear.z` → 실제 속도 `v_z`
  - 명령 입력 `v_z_cmd`

---

## 2️⃣ 모델 가정
$$
\dot{v}_z = -\frac{1}{\tau_z}v_z + \frac{k_z}{\tau_z}v_z^{cmd}
$$
이 식을 이산화하면:
$$
v_z[k+1] = a_z v_z[k] + b_z v_z^{cmd}[k]
$$
여기서  
$a_z = e^{-T_s/\tau_z}$, $b_z = k_z(1 - e^{-T_s/\tau_z})$

---

## 3️⃣ 최소자승법 (Least Squares)
1. N개의 샘플 수집
2. 행렬 구성  
   $Y = [v_z[2], v_z[3], ..., v_z[N]]^T$  
   $\Phi = [[v_z[1], v_z^{cmd}[1]], ..., [v_z[N-1], v_z^{cmd}[N-1]]]$
3. 파라미터 추정  
   $$
   \theta = [a_z, b_z]^T = (\Phi^T\Phi)^{-1}\Phi^TY
   $$

---

## 4️⃣ 파라미터 변환
$$
\tau_z = -\frac{T_s}{\ln(a_z)}, \quad k_z = \frac{b_z}{1 - a_z}
$$

---

## 5️⃣ 검증
- 실제 vs 모델 예측 `v_z` 비교 (Python plot)
- RMSE 계산
- 불일치 시 step 크기 조정

---

| 파라미터 | 기호 | 예시값 |
|-----------|-------|--------|
| 시정수 | τ_z | 0.35 s |
| 이득 | k_z | 1.05 |
| 샘플링 주기 | T_s | 0.02 s |

---

✅ 다음: **Step 2 — X축 식별**
