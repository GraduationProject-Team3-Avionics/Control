#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, json, csv, argparse, math, datetime as dt
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt

# ===== Defaults =====
DEF_CSV = 'src/Control/system_identification/data/x/vx_step_log.csv'
OUT_DIR = 'src/Control/system_identification/data/x'


# ========== I/O ==========
def load_csv(path: str):
    """
    기대 컬럼:
      t_sec, v_x_cmd_NED_x, vx_NED_meas
    - 입력 u: NED x (필수)
    - 출력 y: NED x (권장). 없으면 최후 수단으로 vx_ENU_meas 사용(축 해석 주의)
    반환: t, u(NED x), y(NED x)
    """
    cols = {}
    with open(path, 'r', newline='') as f:
        r = csv.DictReader(f)
        rows = list(r)
        if r.fieldnames is None:
            raise ValueError('CSV header가 없습니다.')
        for k in r.fieldnames:
            cols[k] = [rows[i].get(k, '') for i in range(len(rows))]

    t = np.array([float(x) for x in cols['t_sec']])

    if 'v_x_cmd_NED_x' not in cols:
        raise ValueError('CSV에 v_x_cmd_NED_x 컬럼이 필요합니다.')
    u = np.array([float(x) for x in cols['v_x_cmd_NED_x']])

    if 'vx_NED_meas' in cols and cols['vx_NED_meas'][0] != '':
        y = np.array([float(x) for x in cols['vx_NED_meas']])
    elif 'vx_ENU_meas' in cols and cols['vx_ENU_meas'][0] != '':
        # 주의: ENU x는 NED y에 해당. 좌표계 해석에 혼동이 있을 수 있음.
        y = np.array([float(x) for x in cols['vx_ENU_meas']])
    else:
        raise ValueError('CSV에 vx_NED_meas (또는 차선책으로 vx_ENU_meas)가 필요합니다.')

    msk = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    return t[msk], u[msk], y[msk]


def median_dt(t: np.ndarray) -> float:
    dtv = np.diff(t)
    dtv = dtv[np.isfinite(dtv) & (dtv > 0)]
    return float(np.median(dtv)) if dtv.size else float('nan')


# ========== Lag (입출력 지연) ==========
def estimate_lag(u: np.ndarray, y: np.ndarray, max_lag_samp: int) -> int:
    """
    정규화 상호상관으로 라그 추정.
    정의: lag>0 → y가 u보다 늦음(=y를 앞으로 당겨야 맞음)
    """
    n = min(len(u), len(y))
    u0 = u[:n] - np.mean(u[:n])
    y0 = y[:n] - np.mean(y[:n])
    lags = np.arange(-max_lag_samp, max_lag_samp + 1)
    corr = []
    for L in lags:
        if L >= 0:
            uu, yy = u0[L:], y0[:len(u0) - L]
        else:
            uu, yy = u0[:len(u0) + L], y0[-L:]
        if len(uu) < 5:
            corr.append(-np.inf)
            continue
        c = np.dot(uu, yy) / (np.linalg.norm(uu) * np.linalg.norm(yy) + 1e-12)
        corr.append(c)
    return int(lags[int(np.argmax(corr))])


def align_by_lag(u: np.ndarray, y: np.ndarray, lag: int) -> Tuple[np.ndarray, np.ndarray]:
    """
    lag>0 : y가 u보다 늦음 → y를 앞에서 잘라 앞으로 당김
    lag<0 : y가 u보다 빠름 → u를 앞에서 잘라 맞춤
    (최종적으로 동일 길이로 트림)
    """
    if lag > 0:
        u2 = u[:-lag]
        y2 = y[lag:]
    elif lag < 0:
        lag = -lag
        u2 = u[lag:]
        y2 = y[:-lag]
    else:
        u2, y2 = u.copy(), y.copy()
    n = min(len(u2), len(y2))
    return u2[:n], y2[:n]


# ========== 2차 ARX(2,2) ==========
def build_regression_arx2(u: np.ndarray, y: np.ndarray):
    """
    v[k+1] = a1*v[k] + a2*v[k-1] + b1*u[k] + b2*u[k-1]
    유효 인덱스: k = 1 .. N-2
    반환: Phi, Y, 유효 샘플 수
    """
    u = np.asarray(u, dtype=float)
    y = np.asarray(y, dtype=float)
    N = len(y)
    if N < 5 or len(u) != N:
        raise ValueError('u, y 길이가 너무 짧거나 서로 다릅니다.')
    ks = np.arange(1, N - 1, dtype=int)
    Y = y[ks + 1]
    Phi = np.column_stack([y[ks], y[ks - 1], u[ks], u[ks - 1]])
    return Phi, Y, len(ks)


def identify_arx2(u: np.ndarray, y: np.ndarray):
    Phi, Y, n_used = build_regression_arx2(u, y)
    theta, *_ = np.linalg.lstsq(Phi, Y, rcond=None)
    a1, a2, b1, b2 = [float(x) for x in theta]
    Yhat = Phi @ theta
    rmse_1 = float(np.sqrt(np.mean((Yhat - Y) ** 2)))
    return a1, a2, b1, b2, rmse_1, n_used


def free_run(u: np.ndarray, a1: float, a2: float, b1: float, b2: float, y0: float, y1: float):
    u = np.asarray(u, dtype=float)
    N = len(u)
    yhat = np.zeros(N, dtype=float)
    yhat[0] = y0
    yhat[1] = y1
    for k in range(1, N - 1):
        yhat[k + 1] = a1 * yhat[k] + a2 * yhat[k - 1] + b1 * u[k] + b2 * u[k - 1]
    return yhat


# ========== Pole report ==========
def discrete_poles(a1: float, a2: float):
    coeff = [1.0, -a1, -a2]
    lam = np.roots(coeff)
    return lam


def cont_poles_from_discrete(lam, Ts: float):
    with np.errstate(divide='ignore', invalid='ignore'):
        s = np.log(lam) / Ts
    return s


def damping_wn_from_poles(s):
    if np.iscomplex(s[0]) or np.iscomplex(s[1]):
        s0 = s[0]
        wn = np.abs(s0)
        zeta = -s0.real / wn if wn > 0 else np.nan
        return float(zeta), float(wn)
    return float('nan'), float('nan')


# ========== Main ==========
def main():
    ap = argparse.ArgumentParser(description='X-axis second-order ARX(2) identification (NED x).')
    ap.add_argument('--csv', default=DEF_CSV, help='input CSV path')
    ap.add_argument('--lag_window_s', type=float, default=0.4, help='max lag search window [s]')
    ap.add_argument('--show', action='store_true', help='show plots')
    ap.add_argument('--cont_report', action='store_true', help='continuous-time pole report in summary')
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    t, u_raw, y_raw = load_csv(args.csv)
    Ts = median_dt(t)
    if not np.isfinite(Ts) or Ts <= 0:
        raise ValueError('샘플링 주기를 추정할 수 없습니다.')

    max_lag = int(round(args.lag_window_s / Ts))
    lag = estimate_lag(u_raw, y_raw, max_lag)
    u, y = align_by_lag(u_raw, y_raw, lag)

    # 시간축(보기용) : 정렬된 길이에 맞춰 뒤쪽 구간 사용
    t_a = t[-len(u):]

    # 2차 식별
    a1, a2, b1, b2, rmse_1, n_used = identify_arx2(u, y)

    # free-run
    yhat = free_run(u, a1, a2, b1, b2, y0=y[0], y1=y[1])
    rmse_fr = float(np.sqrt(np.mean((y - yhat) ** 2)))

    # 극점 리포트(선택)
    lam = discrete_poles(a1, a2)
    s = cont_poles_from_discrete(lam, Ts)
    zeta, wn = damping_wn_from_poles(s)

    # 파일명
    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
    png1 = os.path.join(OUT_DIR, f'vx_id2_{stamp}_input.png')
    png2 = os.path.join(OUT_DIR, f'vx_id2_{stamp}_plot.png')
    summ = os.path.join(OUT_DIR, f'vx_id2_{stamp}_summary.txt')
    jso  = os.path.join(OUT_DIR, f'vx_id2_{stamp}_params.json')

    # 입력 플롯
    plt.figure(figsize=(10,3))
    plt.plot(t_a, u, label='u_vx_cmd (NED x)')
    plt.xlabel('t [s]'); plt.ylabel('u [m/s]')
    plt.grid(True, alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig(png1, dpi=150)

    # free-run 플롯
    plt.figure(figsize=(10,5))
    ttl = f'ARX(2) Free-run vs Measurement (lag={lag:+d} samp, Ts≈{Ts:.3f}s)'
    plt.title(ttl)
    plt.plot(t_a, u,      label='u_vx_cmd (NED x)', alpha=0.6)
    plt.plot(t_a, y,      label='v_x_meas (NED x)')
    plt.plot(t_a, yhat, '--',label='v_x_hat (free-run, ARX2)')
    plt.xlabel('t [s]'); plt.ylabel('v_x [m/s]')
    plt.grid(True, alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig(png2, dpi=150)

    if args.show:
        plt.show()
    plt.close('all')

    # 요약 텍스트
    lines = []
    lines.append('=== X-axis ARX(2) Identification (NED x) ===')
    lines.append(f'csv                : {os.path.abspath(args.csv)}')
    lines.append(f'samples_used       : {n_used} (after lag alignment; total {len(u)})')
    lines.append(f'dt_median [s]      : {Ts:.6f}')
    lines.append(f'lag [samples]      : {lag:+d}  (~{lag*Ts:+.3f} s)')
    lines.append(f'a1_hat             : {a1:.6f}')
    lines.append(f'a2_hat             : {a2:.6f}')
    lines.append(f'b1_hat             : {b1:.6f}')
    lines.append(f'b2_hat             : {b2:.6f}')
    lines.append(f'RMSE one-step      : {rmse_1:.6f}')
    lines.append(f'RMSE free-run      : {rmse_fr:.6f}')
    lines.append(f'plots              : {png1}, {png2}')
    if args.cont_report:
        lines.append('-- discrete poles (lambda) --')
        lines.append(f'  λ1={lam[0]:.6f}, λ2={lam[1]:.6f}')
        lines.append('-- continuous poles (s=ln(λ)/Ts) --')
        lines.append(f'  s1={s[0].real:.6f} + j{s[0].imag:.6f}')
        lines.append(f'  s2={s[1].real:.6f} + j{s[1].imag:.6f}')
        if np.isfinite(zeta):
            lines.append(f'  ~ ζ(damping)={zeta:.4f}, ω_n={wn:.4f} rad/s')

    with open(summ, 'w') as f:
        f.write('\n'.join(lines))

    # JSON
    with open(jso, 'w') as f:
        json.dump({
            'csv': os.path.abspath(args.csv),
            'samples_used': int(n_used),
            'total_used_after_align': int(len(u)),
            'dt_median': Ts,
            'lag_samples': int(lag),
            'lag_seconds': float(lag*Ts),
            'a1_hat': a1, 'a2_hat': a2, 'b1_hat': b1, 'b2_hat': b2,
            'rmse_one_step': rmse_1,
            'rmse_free_run': rmse_fr,
            'discrete_poles': [complex(lam[0]).__repr__(), complex(lam[1]).__repr__()],
            'continuous_poles': [complex(s[0]).__repr__(), complex(s[1]).__repr__()],
            'damping_est': {'zeta': zeta, 'wn_rad_s': wn},
            'plots': {'input': png1, 'free_run': png2},
            'generated_at': stamp
        }, f, indent=2)

    print('\n' + '\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()

