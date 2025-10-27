# Step 6 — 이산화, 바이어스 상태 추가, 내보내기

## 목적
식별 모델을 **MPC/제어기**가 바로 쓸 수 있게 이산화하고, 바람/오프셋 보정을 위한 **바이어스 상태**를 선택적으로 추가한 뒤 내보낸다.

## 절차
1. **연속→이산**: Zero-Order Hold, 샘플링 $T_s=0.02\sim0.1$ s
2. **바이어스 상태(옵션)**:  
   $$
   \dot{v}_x = f(\cdot) + b_x,\ \ \dot{b}_x = 0 \quad (y,z도 동일)
   $$
3. **모델 병합**: 축별 SS를 block-diagonal로 합쳐 통합 $A,B,C,D$ 구성
4. **형식화 내보내기**: `models/combined_ss_discrete.yml`  
   - 메타데이터 포함: 좌표계, Ts, 차수, 유효 범위(hover±소각도)
5. **간단 HIL 테스트**: 기존 PID 노드에 *모델 시뮬*을 병렬 붙여 예측 vs 실제 비교 로그

## 산출물
- `models/combined_ss_discrete.yml` (또는 `.json`)
- `tests/test_sim_compare.py` 결과 리포트

## 다음 단계(선택)
- MPC(Q,R,Np,Nc) 초기 튜닝 시트 생성
- (가속도 입력형 병행) 더블 인티그레이터 형태로 동일 파이프라인 반복
