# LQR 기반 제어 개념 정리 및 현재 구현 방식

## 1. 개요

이 문서는 현재 작성한 ROS 2 노드(`lqr_z_controller`)가 어떤 LQR 개념을 기반으로 구성되어 있는지 정리한 것이다.  
현재 노드는 식별된 이산시간 모델을 사용해 LQR 이득을 노드 내부에서 계산하고, PX4에 offboard 속도 명령을 퍼블리시한다.  
지금은 $z$ 축만 제어하지만, 구조는 $x,y,z$ 다축으로 확장할 수 있는 일반적인 형태다.

---

## 2. LQR 기본 문제

LQR(Linear Quadratic Regulator)은 다음과 같은 선형 이산 시스템을 대상으로 한다.

$$
x_{k+1} = A x_k + B u_k
$$

여기서
- $x_k \in \mathbb{R}^n$: 상태(state)
- $u_k \in \mathbb{R}^m$: 제어입력(input)

LQR은 아래의 2차 비용함수를 최소화하는 $u_k$를 찾는다.

$$
J = \sum_{k=0}^{\infty} \left( x_k^\top Q x_k + u_k^\top R u_k \right)
$$

- $Q ≥ 0$: 상태에 대한 가중치  
- $R > 0$: 입력에 대한 가중치

이 문제의 해는 항상 선형 피드백으로 주어지며

$$
u_k = -K x_k
$$

형태가 된다. 여기서 $K$는 DARE(이산 리카티 방정식)를 풀어 얻는 이득 행렬이다.

---

## 3. 참조추종을 위한 오차 상태

위 기본식은 상태를 원점($x=0$)으로 보내는 문제다. 실제로는 “어떤 목표 상태 $x_{\text{ref}}$로 가라”가 필요하므로, 다음과 같이 상태를 오차로 정의해 사용한다.

$$
e_k = x_k - x_{\text{ref}}
$$

이 상태에 대해 LQR을 적용하면 제어입력은

$$
u_k = -K (x_k - x_{\text{ref}})
$$

가 되고, 결과적으로 $x_k$는 $x_{\text{ref}}$로 수렴한다.

현재 구현에서는
$$
x_{\text{ref}} =
\begin{bmatrix}
z_{\text{goal}} \\
0
\end{bmatrix}
$$
으로 두어, 특정 고도에 정지하고 수직 속도는 0이 되도록 한다.

---

## 4. 모델 구성 (상태 확장)

식별 단계에서 얻은 모델은 $z$축 속도에 대한 1차 이산 모델이었다.

$$
v_{k+1} = a v_k + b u_k
$$

이 모델은 속도만 설명하므로, 위치까지 제어하려면 상태를 확장(augment)해야 한다. 샘플링 시간 $T_s$를 알고 있으므로 다음과 같이 위치를 적분 형태로 넣을 수 있다.

1. 위치 갱신:
   $$
   z_{k+1} = z_k + T_s v_k
   $$
2. 속도 갱신(식별 결과 사용):
   $$
   v_{k+1} = a v_k + b u_k
   $$

이를 묶으면 상태를
$$
x_k =
\begin{bmatrix}
z_k \\
v_k
\end{bmatrix}
$$
로 두고,

$$
\begin{bmatrix}
z_{k+1} \\
v_{k+1}
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
1 & T_s \\
0 & a
\end{bmatrix}}_{A}
\begin{bmatrix}
z_k \\
v_k
\end{bmatrix}
+
\underbrace{
\begin{bmatrix}
0 \\
b
\end{bmatrix}}_{B}
u_k
$$

형태의 2차(2-state) 이산 선형 시스템을 얻는다. 이는 속도만 알던 black-box 모델을 LQR이 다룰 수 있는 상태공간형으로 확장한 것이다.

---

## 5. 가중치 선택

현재 상태는 $x = [z, v]^\top$ 이므로, 가중치는 다음과 같이 대각 행렬로 두는 것이 자연스럽다.

$$
Q =
\begin{bmatrix}
Q_z & 0 \\
0 & Q_v
\end{bmatrix},
\quad
R = [\, R_u \,].
$$

- $Q_z$: 고도 오차를 얼마나 싫어하는지
- $Q_v$: 수직 속도 변화를 얼마나 싫어하는지
- $R_u$: 제어입력(보낼 속도 명령)을 얼마나 아끼는지

현재 노드에서는 파라미터로 다음과 같은 기본값을 선언했다.

- $Q_z = 5.0$
- $Q_v = 1.0$
- $R_u = 0.3$

이는 “고도 오차를 속도보다 더 강하게 줄이고 싶고, 입력은 너무 거칠게 쓰지 않는다”라는 의도가 들어간 값이다. ROS 파라미터로 선언했으므로 launch나 YAML에서 조정 가능하다.

---

## 6. LQR 이득 계산

확장한 시스템

$$
x_{k+1} = A x_k + B u_k
$$

에 대해 이산 리카티 방정식을 반복(iteration)으로 풀어 $P$를 구한 후,

$$
K = (R + B^\top P B)^{-1} B^\top P A
$$

공식으로 이득 $K$를 계산한다. 2차 시스템이므로 $K$는

$$
K = [\, K_z \; K_v \,]
$$

와 같은 1행 2열이 되고, 실제 제어입력은

$$
u_k = -K_z (z_k - z_{\text{goal}}) - K_v (v_k - 0)
$$

형태가 된다.

코드에서는 이 과정을 `computeLqrGain()` 안에서 수행하고 있으며, 파라미터로 들어온 $a, b, T_s, Q_z, Q_v, R_u$ 를 사용해 매 노드 시작 시점에 $K_z, K_v$를 계산한다.

---

## 7. ROS 2 및 PX4 연동 흐름

노드의 주기적인 `tick()`에서 다음 순서로 처리한다.

1. `/uav/odom`에서 ENU 기준의 $z$, $v_z$ 를 읽어 현재 상태를 구성한다.
2. 고도 목표 `goal_z_`와의 오차를 계산한다.
3. LQR 피드백 법칙 $u = -K (x - x_{\text{ref}})$ 으로 ENU 기준 수직 속도 명령을 계산한다.
4. PX4는 NED(+down) 기준을 사용하므로 부호를 반대로 하여
   $$
   v_{z,\text{ned}} = - v_{z,\text{enu}}
   $$
   로 변환한다.
5. 변환된 속도 명령을 `/fmu/in/trajectory_setpoint` 에 퍼블리시한다.
6. 동시에 `/fmu/in/offboard_control_mode` 를 반복적으로 퍼블리시해 offboard 모드를 유지한다.
7. 초기 잠깐의 warmup 이후 ARM 및 OFFBOARD 요청을 `/fmu/in/vehicle_command` 로 전송한다.
8. RViz/PlotJuggler에서 확인할 수 있도록 목표 고도를 `/lqr/goal_z`, `/lqr/goal_pose`, `/lqr/goal_marker` 로 퍼블리시한다.

이로써 “식별된 모델 → LQR 이득 → PX4 속도 명령 → 시각화”가 한 노드 안에서 닫힌다.

---

## 8. 다축 확장 관점

현재는 $z$축에 한정되어 있지만 절차 자체는 일반적이다.

1. $x,y,z$ 각각에 대해 선형(또는 선형화/식별) 모델을 확보한다.
2. 상태를
   $$
   x = [x, y, z, \dot x, \dot y, \dot z]^\top
   $$
   처럼 위치+속도로 확장한다.
3. 위와 같은 방식으로 $A, B$를 블록 형태로 구성한다.
4. LQR 가중치 $Q$에서 위치 성분에 더 큰 값을 주어 위치 오차를 우선시한다.
5. 동일한 제어법칙
   $$
   u = -K (x - x_{\text{ref}})
   $$
   을 적용한다.

따라서 현재 노드는 “z축에 대해 모델 기반 제어를 검증하는 최소 예제”로 볼 수 있으며, 구조 그대로 다축으로 확대 가능하다.