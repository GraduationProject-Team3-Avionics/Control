#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, csv, argparse, math, datetime as dt
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt

DEF_CSV = 'src/Control/system_identification/data/z/vz_step_log.csv'
OUT_DIR = 'src/Control/system_identification/data/z'

def load_csv(path: str):
    cols = {}
    with open(path, 'r', newline='') as f:
        r = csv.DictReader(f)
        rows = list(r)
        for k in r.fieldnames:
            cols[k] = [rows[i].get(k, '') for i in range(len(rows))]

    # time
    t = np.array([float(x) for x in cols['t_sec']])

    # 입력: 우선순위 v_z_cmd_NED_z -> v_z_cmd_ENU (부호 반전)
    if 'v_z_cmd_NED_z' in cols:
        u = np.array([float(x) for x in cols['v_z_cmd_NED_z']])
    elif 'v_z_cmd_ENU' in cols:
        u = -np.array([float(x) for x in cols['v_z_cmd_ENU']])  # ENU +z는 위, NED z는 +D
    else:
        raise ValueError('CSV에 v_z_cmd_NED_z 또는 v_z_cmd_ENU가 필요합니다.')

    # 출력: 우선순위 vz_NED_meas -> vz_ENU_meas (부호 반전)
    if 'vz_NED_meas' in cols:
        y = np.array([float(x) for x in cols['vz_NED_meas']])
    elif 'vz_ENU_meas' in cols:
        y = -np.array([float(x) for x in cols['vz_ENU_meas']])
    else:
        raise ValueError('CSV에 vz_NED_meas 또는 vz_ENU_meas가 필요합니다.')

    # 유효값만 필터
    msk = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    return t[msk], u[msk], y[msk]

def median_dt(t: np.ndarray) -> float:
    return float(np.median(np.diff(t)))

def estimate_lag(u: np.ndarray, y: np.ndarray, max_lag_samp: int) -> int:
    """
    u와 y의 지연을 샘플 단위로 추정(양수 = y가 u보다 뒤에 있음 → y를 앞으로 당김).
    """
    # 길이 맞추기
    n = min(len(u), len(y))
    u = u[:n] - np.mean(u[:n])
    y = y[:n] - np.mean(y[:n])
    lags = np.arange(-max_lag_samp, max_lag_samp + 1)
    corr = []
    for L in lags:
        if L >= 0:
            uu, yy = u[L:], y[:len(u)-L]
        else:
            uu, yy = u[:len(u)+L], y[-L:]
        if len(uu) < 5: 
            corr.append(-np.inf)
            continue
        c = np.dot(uu, yy) / (np.linalg.norm(uu) * np.linalg.norm(yy) + 1e-12)
        corr.append(c)
    lag = int(lags[int(np.argmax(corr))])
    return lag

def align_by_lag(u: np.ndarray, y: np.ndarray, lag: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    lag>0 : y가 u보다 늦음 → y를 앞으로 당김(앞부분 절단)
    lag<0 : y가 u보다 빠름 → y를 뒤로 민다(뒷부분 절단)
    """
    if lag > 0:
        u2 = u[:-lag]
        y2 = y[lag:]
    elif lag < 0:
        u2 = u[-lag:]
        y2 = y[:len(u2)]
    else:
        u2, y2 = u.copy(), y.copy()
    n = min(len(u2), len(y2))
    return u2[:n], y2[:n]

def identify_first_order(u: np.ndarray, y: np.ndarray):
    """
    ARX(1,1): v[k+1] = a*v[k] + b*u[k]
    반환: a, b, one-step RMSE, yhat_one_step(참고용)
    """
    yk   = y[:-1]
    yk1  = y[1:]
    uk   = u[:-1]
    Phi  = np.column_stack([yk, uk])
    theta, *_ = np.linalg.lstsq(Phi, yk1, rcond=None)
    a, b = float(theta[0]), float(theta[1])

    y1 = Phi @ theta
    rmse = float(np.sqrt(np.mean((yk1 - y1)**2)))
    return a, b, rmse

def tau_gain_from_ab(a: float, b: float, Ts: float):
    if a <= 0 or a >= 1:
        # 이 경우 tau가 비정상(부호/크기)일 수 있음 → 정보 표시만
        tau = float('inf') if a >= 1 else -float('inf')
    else:
        tau = -Ts / math.log(a)
    k = b / (1.0 - a) if abs(1.0 - a) > 1e-12 else float('inf')
    return tau, k

def free_run(u: np.ndarray, a: float, b: float, y0: float):
    yhat = np.zeros_like(u)
    yhat[0] = y0
    for k in range(len(u)-1):
        yhat[k+1] = a*yhat[k] + b*u[k]
    return yhat

def main():
    ap = argparse.ArgumentParser(description='Z-axis first-order identification (NED z).')
    ap.add_argument('--csv', default=DEF_CSV, help='input CSV path')
    ap.add_argument('--lag_window_s', type=float, default=2.0, help='max lag search window [s]')
    ap.add_argument('--show', action='store_true', help='show plots')
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    t, u, y = load_csv(args.csv)
    Ts = median_dt(t)
    max_lag = int(round(args.lag_window_s / Ts))

    lag = estimate_lag(u, y, max_lag)
    u_a, y_a = align_by_lag(u, y, lag)
    t_a = t[:len(u_a)]  # 시간축은 대략적으로 맞춰 표시용

    # 식별
    a, b, rmse_1 = identify_first_order(u_a, y_a)

    # 파라미터 변환
    tau, k_gain = tau_gain_from_ab(a, b, Ts)

    # free-run
    yhat = free_run(u_a, a, b, y_a[0])
    rmse_fr = float(np.sqrt(np.mean((y_a - yhat)**2)))

    # --- 출력/저장 ---
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    png1 = os.path.join(OUT_DIR, f'vz_id_{stamp}_input.png')
    png2 = os.path.join(OUT_DIR, f'vz_id_{stamp}_plot.png')
    summ = os.path.join(OUT_DIR, f'vz_id_{stamp}_summary.txt')
    jso  = os.path.join(OUT_DIR, f'vz_id_{stamp}_params.json')

    # 입력 플롯
    plt.figure(figsize=(10,3))
    plt.plot(t_a, u_a, label='u_vz_cmd (NED z)')
    plt.xlabel('t [s]'); plt.ylabel('u [m/s]'); plt.grid(True, alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig(png1, dpi=150)

    # free-run 플롯
    plt.figure(figsize=(10,5))
    ttl = f'Free-run vs Measurement (lag={lag:+d} samp, Ts≈{Ts:.3f}s)'
    plt.title(ttl)
    plt.plot(t_a, u_a,      label='u_vz_cmd (NED z)', alpha=0.6)
    plt.plot(t_a, y_a,      label='v_z_meas (NED z)')
    plt.plot(t_a, yhat, '--',label='v_z_hat (free-run)')
    plt.xlabel('t [s]'); plt.ylabel('v_z [m/s]'); plt.grid(True, alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig(png2, dpi=150)

    # 요약 텍스트
    lines = []
    lines.append('=== Z-axis First-Order Identification (NED z) ===')
    lines.append(f'csv                : {os.path.abspath(args.csv)}')
    lines.append(f'samples_used       : {len(u_a)} (after lag alignment)')
    lines.append(f'dt_median [s]      : {Ts:.6f}')
    lines.append(f'lag [samples]      : {lag:+d}  (~{lag*Ts:+.3f} s)')
    lines.append(f'a_hat              : {a:.6f}')
    lines.append(f'b_hat              : {b:.6f}')
    lines.append(f'tau_hat [s]        : {tau:.6f}')
    lines.append(f'k_hat              : {k_gain:.6f}')
    lines.append(f'RMSE one-step      : {rmse_1:.6f}')
    lines.append(f'RMSE free-run      : {rmse_fr:.6f}')
    lines.append(f'plots              : {png1}, {png2}')
    with open(summ, 'w') as f:
        f.write('\n'.join(lines))

    # JSON 저장
    with open(jso, 'w') as f:
        json.dump({
            'csv': os.path.abspath(args.csv),
            'samples_used': int(len(u_a)),
            'dt_median': Ts,
            'lag_samples': int(lag),
            'lag_seconds': float(lag*Ts),
            'a_hat': a, 'b_hat': b,
            'tau_hat': tau, 'k_hat': k_gain,
            'rmse_one_step': rmse_1,
            'rmse_free_run': rmse_fr,
            'plots': {'input': png1, 'free_run': png2},
            'generated_at': stamp
        }, f, indent=2)

    # 콘솔에도 깔끔히
    print('\n' + '\n'.join(lines) + '\n')

    if args.show:
        plt.show()

if __name__ == '__main__':
    main()
