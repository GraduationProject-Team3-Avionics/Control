# Step 3 — 전처리(동기화, 필터, 도함수)

## 목적
원시 rosbag을 학습 가능한 시계열로 변환하고, 노이즈를 줄여 **식별 안정성**을 확보한다.

## 파이프라인
1. **rosbag → CSV** 변환 (타임스탬프 기준 merge)
2. **리샘플**: 50 Hz 고정 (선형 보간)
3. **좌표계 통일**: odom ENU → map ENU (필요시 yaw 보정), 이후 필요 시 NED 변환
4. **필터링**
   - 저역통과(Butterworth 2–4차, 컷오프 5–10 Hz) 또는
   - Savitzky–Golay(윈도우 9–21, poly 2–3)
5. **도함수 계산**
   - 속도/가속도 필요 시 Savitzky–Golay 미분
6. **Train/Val 분리**
   - 70/30 시간 분할

## 산출물
- `data/roll_train.csv`, `data/roll_val.csv`, … 축별로 저장
- 전처리 파라미터는 `config/ident_params.yaml`에 기록(재현성)

## 품질 지표
- 상자그림/히스토그램으로 이상치 확인
- cmd ↔ 응답 상호상관 peak 지연(딜레이) 추정
