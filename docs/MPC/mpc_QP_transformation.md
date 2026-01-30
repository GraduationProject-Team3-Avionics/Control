# MPC QP 변환 가이드

MPC 비용함수를 QP(Quadratic Programming) 문제로 변환하는 과정을 설명합니다.

---

## 1. 원래 비용함수

```math
J = \sum_{k=0}^{N-1} \left[ (x_k - x_{ref})^T Q (x_k - x_{ref}) + u_k^T R \, u_k \right]
```

- $Q$: 상태 오차 가중치 (6×6)
- $R$: 입력 가중치 (3×3)
- $N$: Prediction Horizon

---

## 2. 미래 상태 예측

시스템 모델 $x_{k+1} = A_d x_k + B_d u_k$ 를 반복 적용:

```math
\begin{aligned}
x_1 &= A_d x_0 + B_d u_0 \\
x_2 &= A_d^2 x_0 + A_d B_d u_0 + B_d u_1 \\
x_3 &= A_d^3 x_0 + A_d^2 B_d u_0 + A_d B_d u_1 + B_d u_2 \\
&\vdots
\end{aligned}
```

### 행렬 형태로 정리

```math
\underbrace{\left[\begin{array}{c} x_1 \\ x_2 \\ x_3 \\ \vdots \\ x_N \end{array}\right]}_{X}
=
\underbrace{\left[\begin{array}{c} A_d \\ A_d^2 \\ A_d^3 \\ \vdots \\ A_d^N \end{array}\right]}_{\Phi} x_0
+
\underbrace{\left[\begin{array}{cccc} B_d & 0 & \cdots & 0 \\ A_d B_d & B_d & \cdots & 0 \\ A_d^2 B_d & A_d B_d & \cdots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ A_d^{N-1} B_d & A_d^{N-2} B_d & \cdots & B_d \end{array}\right]}_{\Gamma}
\underbrace{\left[\begin{array}{c} u_0 \\ u_1 \\ \vdots \\ u_{N-1} \end{array}\right]}_{U}
```

**간단히:**

```math
X = \Phi \cdot x_0 + \Gamma \cdot U
```

---

## 3. 확장 가중치 행렬

N스텝 전체에 대해 Q, R을 대각 블록으로 확장:

```math
\bar{Q} = \text{blkdiag}(Q, Q, \ldots, Q) \quad \text{(N번, 크기: } n_x N \times n_x N \text{)}
```

```math
\bar{R} = \text{blkdiag}(R, R, \ldots, R) \quad \text{(N번, 크기: } n_u N \times n_u N \text{)}
```

---

## 4. 비용함수 대입

### Step 4.1: X 대입

```math
J = (X - X_{ref})^T \bar{Q} (X - X_{ref}) + U^T \bar{R} U
```

$X = \Phi x_0 + \Gamma U$ 대입:

```math
J = (\Phi x_0 + \Gamma U - X_{ref})^T \bar{Q} (\Phi x_0 + \Gamma U - X_{ref}) + U^T \bar{R} U
```

### Step 4.2: 치환

$e_0 = \Phi x_0 - X_{ref}$ (상수) 로 치환:

```math
J = (e_0 + \Gamma U)^T \bar{Q} (e_0 + \Gamma U) + U^T \bar{R} U
```

### Step 4.3: 전개

$(a + b)^T Q (a + b) = a^T Q a + a^T Q b + b^T Q a + b^T Q b$

```math
J = e_0^T \bar{Q} e_0 + e_0^T \bar{Q} \Gamma U + (\Gamma U)^T \bar{Q} e_0 + (\Gamma U)^T \bar{Q} \Gamma U + U^T \bar{R} U
```

### Step 4.4: 정리

- $e_0^T \bar{Q} e_0$ → **상수** (U 없음)
- $e_0^T \bar{Q} \Gamma U + U^T \Gamma^T \bar{Q} e_0 = 2 e_0^T \bar{Q} \Gamma U$ → **U의 1차항**
- $U^T \Gamma^T \bar{Q} \Gamma U + U^T \bar{R} U = U^T (\Gamma^T \bar{Q} \Gamma + \bar{R}) U$ → **U의 2차항**

```math
J = \text{(상수)} + 2 e_0^T \bar{Q} \Gamma \, U + U^T (\Gamma^T \bar{Q} \Gamma + \bar{R}) U
```

---

## 5. QP 표준형

```math
J = \frac{1}{2} U^T H \, U + f^T U + \text{(상수)}
```

비교하면:

```math
H = 2(\Gamma^T \bar{Q} \Gamma + \bar{R})
```

```math
f = 2 \Gamma^T \bar{Q} (\Phi x_0 - X_{ref})
```

---

## 6. 제약 조건

입력 제약 $u_{min} \le u_k \le u_{max}$ 를 행렬 형태로:

```math
\left[\begin{array}{c} I \\ -I \end{array}\right] U \le \left[\begin{array}{c} U_{max} \\ -U_{min} \end{array}\right]
```

**표준형:** $A_{ineq} \cdot U \le b_{ineq}$

---

## 7. Solver 호출

```matlab
[U_opt, J_opt] = quadprog(H, f, A_ineq, b_ineq);

% 첫 번째 입력만 적용
u_opt = U_opt(1:nu);
```

---

## 요약

| 단계 | 내용 | 계산 시점 |
|-----|------|----------|
| Φ, Γ 계산 | 예측 행렬 | 한 번만 (모델 고정) |
| Q̄, R̄ 계산 | 확장 가중치 | 한 번만 |
| H 계산 | Hessian | 한 번만 |
| **f 계산** | 선형 항 | **매 스텝** (x₀ 변함) |
| quadprog | QP 풀기 | 매 스텝 |
