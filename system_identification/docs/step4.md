# Step 4 — 축별 모델 식별(Fit)

## 목적
각 축에 대해 저차(1–2차) **연속시간** 모델을 우선 식별한 뒤, 필요 시 상태공간으로 변환.

## 권장 모델
- **Roll/Pitch**: 2차
  $$
  G(s)=\frac{K \omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}
  $$
- **Yaw rate**: 1차(또는 2차)
  $$
  G(s)=\frac{K}{\tau s + 1}
  $$
- **Altitude(z)**: 1차~2차

## 방법
- Python: `scipy`, `control`, `sippy`(N4SID), `numpy`  
- (선택) MATLAB: `tfest`, `ssest`, `n4sid`

## 실무 파라미터
- **딜레이** $\tau_d$ 포함 가능 (입력 신호와 상호상관으로 초기 추정)
- 과적합 방지: 차수 1↔2 교차 검증, Val 데이터로 `FIT[%]`, `NRMSE` 비교

## 산출물
- `models/roll_ss.yml` 등 (A,B,C,D, Ts=continuous, delay)
- 적합도 리포트: `reports/fit_summary.md` (표/그래프 링크)

## 샘플 워크플로우
1. `fit_roll.py` 실행 → `models/roll_ss.yml` 저장
2. `fit_pitch.py` …
3. `fit_yaw.py` …
4. `fit_alt.py` …
