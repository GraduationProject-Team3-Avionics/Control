#!/usr/bin/env python3
"""
MPC Core Module
MATLAB MPC 컨트롤러를 Python으로 이식

Classes:
    QuadcopterModel: 선형화된 쿼드콥터 모델 (Ad, Bd 행렬)
    MPCParams: MPC 파라미터 (N, Q, R, 제약조건)

Functions:
    mpc_solve: QP 문제 구성 및 풀이
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Tuple, Optional
from scipy.linalg import expm


@dataclass
class QuadcopterModel:
    """
    선형화된 쿼드콥터 모델 (Hover 근방)
    
    상태: x = [px, py, pz, vx, vy, vz]' (6차원)
    입력: u = [phi_d, theta_d, delta_T]' (3차원)
    
    선형화 (Small angle approximation):
        ddot_px ≈ g * theta
        ddot_py ≈ -g * phi  
        ddot_pz ≈ delta_T / m
    """
    # 물리 파라미터
    mass: float = 2.0           # [kg]
    gravity: float = 9.81       # [m/s^2]
    dt: float = 0.05            # [s] 샘플링 시간 (20 Hz)
    
    # 이산 시스템 행렬 (초기화 후 계산)
    Ad: np.ndarray = field(default=None, repr=False)
    Bd: np.ndarray = field(default=None, repr=False)
    
    def __post_init__(self):
        """연속 시스템을 이산화하여 Ad, Bd 계산"""
        g = self.gravity
        m = self.mass
        dt = self.dt
        
        # 연속 시간 A, B 행렬
        # 상태: [px, py, pz, vx, vy, vz]
        # 입력: [phi_d, theta_d, delta_T]
        A = np.array([
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0]
        ], dtype=float)
        
        B = np.array([
            [0,    0,    0],
            [0,    0,    0],
            [0,    0,    0],
            [0,    g,    0],      # ddot_px = g * theta_d
            [-g,   0,    0],      # ddot_py = -g * phi_d
            [0,    0,    1/m]     # ddot_pz = delta_T / m
        ], dtype=float)
        
        # ZOH 이산화
        # Ad = exp(A * dt)
        # Bd = integral_0^dt exp(A*s) ds * B
        self.Ad = expm(A * dt)
        
        # A가 nilpotent이므로 간단히 계산
        # Bd = (I*dt + A*dt^2/2) * B (for this specific A)
        I = np.eye(6)
        self.Bd = (I * dt + A * (dt**2 / 2)) @ B
    
    @property
    def nx(self) -> int:
        """상태 차원"""
        return 6
    
    @property
    def nu(self) -> int:
        """입력 차원"""
        return 3
    
    @property
    def T_hover(self) -> float:
        """호버링 추력 [N]"""
        return self.mass * self.gravity


@dataclass
class MPCParams:
    """MPC 파라미터"""
    # Horizon
    N: int = 20                 # Prediction horizon
    dt: float = 0.05            # 샘플링 시간 [s]
    
    # 가중치 행렬
    Q: np.ndarray = field(default=None, repr=False)  # 상태 가중치 (6x6)
    R: np.ndarray = field(default=None, repr=False)  # 입력 가중치 (3x3)
    
    # 입력 제약
    phi_max: float = np.deg2rad(30)      # [rad] Roll 제한
    theta_max: float = np.deg2rad(30)    # [rad] Pitch 제한
    delta_T_max: float = 19.62           # [N] 추력 변화 제한 (= T_hover)
    
    # 상태 제약 (속도)
    v_max: float = 3.0                   # [m/s] 최대 속도
    enable_state_constraints: bool = True
    
    def __post_init__(self):
        if self.Q is None:
            # 위치 > 속도 가중치
            self.Q = np.diag([50.0, 50.0, 100.0, 5.0, 5.0, 5.0])
        if self.R is None:
            self.R = np.diag([0.5, 0.5, 0.05])
    
    @property
    def u_min(self) -> np.ndarray:
        return np.array([-self.phi_max, -self.theta_max, -self.delta_T_max])
    
    @property
    def u_max(self) -> np.ndarray:
        return np.array([self.phi_max, self.theta_max, self.delta_T_max])


def build_prediction_matrices(model: QuadcopterModel, N: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    예측 행렬 Phi, Gamma 구성
    
    X = Phi * x0 + Gamma * U
    
    Returns:
        Phi: (nx*N, nx)
        Gamma: (nx*N, nu*N)
    """
    Ad, Bd = model.Ad, model.Bd
    nx, nu = model.nx, model.nu
    
    Phi = np.zeros((nx * N, nx))
    Gamma = np.zeros((nx * N, nu * N))
    
    # Phi 구성: [A; A^2; ...; A^N]
    A_power = Ad.copy()
    for i in range(N):
        Phi[i*nx:(i+1)*nx, :] = A_power
        A_power = A_power @ Ad
    
    # Gamma 구성: 하삼각 블록 행렬
    for i in range(N):
        for j in range(i + 1):
            if i == j:
                Gamma[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = Bd
            else:
                A_power = np.linalg.matrix_power(Ad, i - j)
                Gamma[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = A_power @ Bd
    
    return Phi, Gamma


def mpc_solve(
    x0: np.ndarray,
    x_ref: np.ndarray,
    model: QuadcopterModel,
    params: MPCParams
) -> Tuple[np.ndarray, bool]:
    """
    MPC QP 문제 풀이
    
    Args:
        x0: 현재 상태 (6,)
        x_ref: 목표 상태 (6,)
        model: 쿼드콥터 모델
        params: MPC 파라미터
    
    Returns:
        u_opt: 최적 입력 (3,)
        success: 최적화 성공 여부
    """
    try:
        import cvxpy as cp
    except ImportError:
        # cvxpy 없으면 scipy 사용
        return _mpc_solve_scipy(x0, x_ref, model, params)
    
    return _mpc_solve_cvxpy(x0, x_ref, model, params)


def _mpc_solve_cvxpy(
    x0: np.ndarray,
    x_ref: np.ndarray,
    model: QuadcopterModel,
    params: MPCParams
) -> Tuple[np.ndarray, bool]:
    """cvxpy를 사용한 MPC QP 풀이"""
    import cvxpy as cp
    
    N = params.N
    nx, nu = model.nx, model.nu
    
    # 예측 행렬
    Phi, Gamma = build_prediction_matrices(model, N)
    
    # 확장 가중치 행렬
    Q_bar = np.kron(np.eye(N), params.Q)
    R_bar = np.kron(np.eye(N), params.R)
    
    # 목표 상태 확장
    X_ref = np.tile(x_ref, N)
    
    # 최적화 변수
    U = cp.Variable(nu * N)
    
    # 예측 상태
    X_pred = Phi @ x0 + Gamma @ U
    
    # 비용 함수
    state_cost = cp.quad_form(X_pred - X_ref, Q_bar)
    input_cost = cp.quad_form(U, R_bar)
    cost = state_cost + input_cost
    
    # 제약 조건
    constraints = []
    
    # 입력 제약
    U_min = np.tile(params.u_min, N)
    U_max = np.tile(params.u_max, N)
    constraints.append(U >= U_min)
    constraints.append(U <= U_max)
    
    # 상태 제약 (속도)
    if params.enable_state_constraints:
        # 속도 추출 행렬
        C_vel_single = np.array([
            [0, 0, 0, 1, 0, 0],  # vx
            [0, 0, 0, 0, 1, 0],  # vy
            [0, 0, 0, 0, 0, 1]   # vz
        ])
        C_vel = np.kron(np.eye(N), C_vel_single)
        
        V_pred = C_vel @ X_pred
        constraints.append(V_pred >= -params.v_max)
        constraints.append(V_pred <= params.v_max)
    
    # 문제 풀이
    problem = cp.Problem(cp.Minimize(cost), constraints)
    
    try:
        problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
        
        if problem.status in ['optimal', 'optimal_inaccurate']:
            u_opt = U.value[:nu]
            return u_opt, True
        else:
            return np.zeros(nu), False
            
    except Exception:
        return np.zeros(nu), False


def _mpc_solve_scipy(
    x0: np.ndarray,
    x_ref: np.ndarray,
    model: QuadcopterModel,
    params: MPCParams
) -> Tuple[np.ndarray, bool]:
    """scipy.optimize.minimize를 사용한 MPC 풀이 (fallback)"""
    import warnings
    from scipy.optimize import minimize
    
    N = params.N
    nx, nu = model.nx, model.nu
    
    Phi, Gamma = build_prediction_matrices(model, N)
    Q_bar = np.kron(np.eye(N), params.Q)
    R_bar = np.kron(np.eye(N), params.R)
    X_ref = np.tile(x_ref, N)
    
    # QP 행렬
    H = Gamma.T @ Q_bar @ Gamma + R_bar
    H = (H + H.T) / 2  # 대칭 보장
    f = Gamma.T @ Q_bar @ (Phi @ x0 - X_ref)
    
    def objective(U):
        return 0.5 * U @ H @ U + f @ U
    
    def gradient(U):
        return H @ U + f
    
    # 경계 조건
    bounds = []
    for _ in range(N):
        bounds.extend([
            (-params.phi_max, params.phi_max),
            (-params.theta_max, params.theta_max),
            (-params.delta_T_max, params.delta_T_max)
        ])
    
    # 초기값
    U0 = np.zeros(nu * N)
    
    # 경고 무시하고 최적화 실행
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        result = minimize(
            objective,
            U0,
            method='SLSQP',
            jac=gradient,
            bounds=bounds,
            options={'maxiter': 100, 'disp': False}
        )
    
    if result.success:
        return result.x[:nu], True
    else:
        return np.zeros(nu), False


# ─────────── 테스트 ───────────
if __name__ == '__main__':
    print("=== MPC Core 테스트 ===\n")
    
    # 모델 생성
    model = QuadcopterModel(mass=2.0, gravity=9.81, dt=0.05)
    print(f"모델 생성 완료")
    print(f"  Ad shape: {model.Ad.shape}")
    print(f"  Bd shape: {model.Bd.shape}")
    print(f"  T_hover: {model.T_hover:.2f} N")
    
    # MPC 파라미터
    params = MPCParams(N=20, delta_T_max=model.T_hover)
    print(f"\nMPC 파라미터")
    print(f"  Horizon N: {params.N}")
    print(f"  입력 제약: phi ±{np.rad2deg(params.phi_max):.0f}°, delta_T ±{params.delta_T_max:.1f}N")
    
    # MPC 풀이 테스트
    x0 = np.array([0, 0, 0, 0, 0, 0])       # 현재: 원점
    x_ref = np.array([1, 0, 2, 0, 0, 0])    # 목표: (1, 0, 2)
    
    print(f"\n현재 상태: {x0}")
    print(f"목표 상태: {x_ref}")
    
    u_opt, success = mpc_solve(x0, x_ref, model, params)
    
    print(f"\n최적 입력:")
    print(f"  phi_d: {np.rad2deg(u_opt[0]):.2f}°")
    print(f"  theta_d: {np.rad2deg(u_opt[1]):.2f}°")
    print(f"  delta_T: {u_opt[2]:.2f} N")
    print(f"  성공: {success}")
