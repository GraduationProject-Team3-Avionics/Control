#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, csv, argparse, math, datetime as dt
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt

DEF_CSV = 'src/Control/system_identification/data/x/vx_step_log.csv'
OUT_DIR = 'src/Control/system_identification/data/x'


def load_csv(path: str):
    cols = {}
    with open(path, 'r', newline='') as f:
        r = csv.DictReader(f)
        rows = list(r)
        for k in r.fieldnames:
            cols[k] = [rows[i].get(k, '') for i in range(len(rows))]

    # time
    t = np.array([float(x) for x in cols['t_sec']])

    # 입력: NED x 명령
    if 'v_x_cmd_NED_x' in cols:
        u = np.array([float(x) for x in cols['v_x_cmd_NED_x']])
    else:
        raise ValueError('CSV에 v_x_cmd_NED_x 컬럼이 필요합니다.')

    # 출력 우선순위:
    # 1) vx_NED_meas  (vehicle_odometry에서 바로 온 경우)
    # 2) vy_ENU_meas  (odom의 ENU y == NED x)
    # 3) 최후 수단으로 vx_ENU_meas (ENU x) ← 이건 축 다르니 가능하면 안 쓰는 게 좋음
    y = None
    if 'vx_NED_meas' in cols and cols['vx_NED_meas'][0] != '':
        y = np.array([float(x) for x in cols['vx_NED_meas']])
    elif 'vy_ENU_meas' in cols and cols['vy_ENU_meas'][0] != '':
        # ENU y == NED x
        y = np.array([float(x) for x in cols['vy_ENU_meas']])
    elif 'vx_ENU_meas' in cols and cols['vx_ENU_meas'][0] != '':
        # 축이 다르지만 일단 fallback
        y = np.array([float(x) for x in cols['vx_ENU_meas']])
    else:
        raise ValueError('CSV에 vx_NED_meas 또는 vy_ENU_meas (또는 vx_ENU_meas)가 필요합니다.')

    msk = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    return t[msk], u[msk], y[msk]


def median_dt(t: np.ndarray) -> float:
    return float(np.median(np.diff(t)))


def estimate_lag(u: np.ndarray, y: np.ndarray, max_lag_samp: int) -> int:
    """
    u와 y의 지연을 샘플 단위로 추정(양수 = y가 u보다 뒤에 있음 → y를 앞으로 당김).
    """
    n = min(len(u), len(y))
    u = u[:n] - np.mean(u[:n])
    y = y[:n] - np.mean(y[:n])
    lags = np.arange(-max_lag_samp, max_lag_samp + 1)
    corr = []
    for L in lags:
        if L >= 0:
            uu, yy = u[L:], y[:len(u) - L]
        else:
            uu, yy = u[:len(u) + L], y[-L:]
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
    """
    yk = y[:-1]
    yk1 = y[1:]
    uk = u[:-1]
    Phi = np.column_stack([yk, uk])
    theta, *_ = np.linalg.lstsq(Phi, yk1, rcond=None)
    a, b = float(theta[0]), float(theta[1])

    y1 = Phi @ theta
    rmse = float(np.sqrt(np.mean((yk1 - y1) ** 2)))
    return a, b, rmse


def tau_gain_from_ab(a: float, b: float, Ts: float):
    if a <= 0 or a >= 1:
        tau = float('inf') if a >= 1 else -float('inf')
    else:
        tau = -Ts / math.log(a)
    k = b / (1.0 - a) if abs(1.0 - a) > 1e-12 else float('inf')
    return tau, k


def free_run(u: np.ndarray, a: float, b: float, y0: float):
    yhat = np.zeros_like(u)
    yhat[0] = y0
    for k in range(len(u) - 1):
        yhat[k + 1] = a * yhat[k] + b * u[k]
    return yhat


def main():
    ap = argparse.ArgumentParser(description='X-axis first-order identification (NED x).')
    ap.add_argument('--csv', default=DEF_CSV, help='input CSV path')
    # x축은 너무 크게 두면 말도 안 되는 lag가 잡혀서 0.4s로 축소
    ap.add_argument('--lag_window_s', type=float, default=0.4, help='max lag search window [s]')
    # 필요하면 추후에 "첫 1초만 식별" 같은 옵션도 추가할 수 있음
    ap.add_argument('--show', action='store_true', help='show plots')
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    t, u, y = load_csv(args.csv)
    Ts = median_dt(t)
    max_lag = int(round(args.lag_window_s / Ts))

    lag = estimate_lag(u, y, max_lag)
    u_a, y_a = align_by_lag(u, y, lag)
    t_a = t[:len(u_a)]

    # 식별
    a, b, rmse_1 = identify_first_order(u_a, y_a)

    # 파라미터 변환
    tau, k_gain = tau_gain_from_ab(a, b, Ts)

    # free-run
    yhat = free_run(u_a, a, b, y_a[0])
    rmse_fr = float(np.sqrt(np.mean((y_a - yhat) ** 2)))

    # --- 출력/저장 ---
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    png1 = os.path.join(OUT_DIR, f'vx_id_{stamp}_input.png')
    png2 = os.path.join(OUT_DIR, f'vx_id_{stamp}_plot.png')
    summ = os.path.join(OUT_DIR, f'vx_id_{stamp}_summary.txt')
    jso  = os.path.join(OUT_DIR, f'vx_id_{stamp}_params.json')

    # 입력 플롯
    plt.figure(figsize=(10, 3))
    plt.plot(t_a, u_a, label='u_vx_cmd (NED x)')
    plt.xlabel('t [s]'); plt.ylabel('u [m/s]'); plt.grid(True, alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig(png1, dpi=150)

    # free-run 플롯
    plt.figure(figsize=(10, 5))
    ttl = f'Free-run vs Measurement (lag={lag:+d} samp, Ts≈{Ts:.3f}s)'
    plt.title(ttl)
    plt.plot(t_a, u_a, label='u_vx_cmd (NED x)', alpha=0.6)
    plt.plot(t_a, y_a, label='v_x_meas (NED x)')
    plt.plot(t_a, yhat, '--', label='v_x_hat (free-run)')
    plt.xlabel('t [s]'); plt.ylabel('v_x [m/s]'); plt.grid(True, alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig(png2, dpi=150)

    # 요약 텍스트
    lines = []
    lines.append('=== X-axis First-Order Identification (NED x) ===')
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

    print('\n' + '\n'.join(lines) + '\n')

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
