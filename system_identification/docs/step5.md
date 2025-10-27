# Step 5 — 검증(Validation) & 로버스트 점검

## 목적
식별된 모델이 보지 못한 데이터에서도 잘 맞는지, 물리적으로 타당한지 검증한다.

## 검증 항목
- **Step/PRBS 재현**: 시뮬 응답 vs 실제 응답
- **잔차 분석**: 잔차의 백색성, 상호상관
- **주파수 특성**: Bode/Nyquist로 안정성/대역폭 점검
- **지연/이득**: 명령–응답 교차상관 peak로 재확인
- **모수 감도**: $K, \zeta, \omega_n, \tau$ ±10% 변동 시 성능 변화

## 합격 기준(예시)
- FIT ≥ 80% (Val 세트)
- 잔차의 Ljung-Box 테스트 통과(유의수준 5%)
- 과도응답 오버슈트/정착시간 예측 오차 < 20%

## 산출물
- `reports/validation_plots/…`
- `reports/validation_summary.md`
