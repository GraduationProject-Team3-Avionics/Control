# System Identification 분석용 가상환경 가이드

이 문서는 **ROS 환경과 충돌 없이** Z축 시스템 식별 분석 스크립트(`vz_identify_plot.py`)를 실행하기 위해, **전용 Python 가상환경(venv)**을 만드는 방법을 정리합니다.

> 권장 이유: 현재 시스템(apt) Matplotlib/Numpy와 `pip`로 설치한 버전이 섞이면 ABI 충돌이 발생합니다. 분석은 ROS 노드와 분리된 **오프라인 작업**이므로 전용 venv 사용이 가장 안전합니다.

---

## 1) 가상환경 생성/활성화

아무 디렉터리에서나 실행해도 됩니다. 아래 예시는 홈 디렉터리 기준입니다.

```bash
# 가상환경 생성
python3 -m venv ~/.venvs/si

# 활성화
source ~/.venvs/si/bin/activate
```

> 활성화 후 프롬프트에 `(si)`가 보이면 성공입니다.  
> 매 세션(새 터미널)마다 활성화해야 합니다.

---

## 2) 필수 패키지 설치

분석 스크립트가 사용하는 **NumPy/Matplotlib/Pandas**를 venv에 설치합니다.

```bash
pip install --upgrade pip
pip install "numpy>=2.0" "matplotlib>=3.9" "pandas>=2.0"
```

> 이렇게 설치하면 NumPy 2.x + Matplotlib 최신 조합으로 ABI 충돌 없이 동작합니다.

---

## 3) 스크립트 실행

절대경로 또는 워크스페이스 루트에서 상대경로 모두 가능합니다.

### 방법 A: 절대경로
```bash
python /home/ihw/workspace/gp_ws/src/Control/system_identification/scripts/vz_identify_plot.py --estimate_delay
```

### 방법 B: 워크스페이스 루트에서 실행
```bash
cd ~/workspace/gp_ws
python src/Control/system_identification/scripts/vz_identify_plot.py --estimate_delay
```

- 기본 입력 CSV: `src/Control/system_identification/data/vz_step_log.csv`
- 결과물(분석 CSV/플롯)은 같은 폴더에 `vz_id_YYYYMMDD_HHMMSS_*.{csv,png}`로 저장됩니다.

---

## 4) 비활성화(종료)

```bash
deactivate
```

---

## 5) 자주 묻는 질문(FAQ)

### Q1. `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` 오류가 나요.
- venv가 **활성화되지 않았거나**, 시스템 파이썬(apt)과 섞여 쓰는 상태일 가능성이 큽니다.
- 해결:
  1) `source ~/.venvs/si/bin/activate`로 venv 활성화
  2) `pip show numpy matplotlib`로 설치 위치가 venv를 가리키는지 확인
  3) 필요 시 `pip install --force-reinstall "numpy>=2.0" "matplotlib>=3.9"`

### Q2. 디스플레이가 없는 환경(원격/헤드리스)인데 플롯 창이 안 떠요.
- 스크립트는 결과 이미지를 파일로 저장합니다. 창 표시가 필요 없다면 `plt.show()`는 무시해도 됩니다.
- 또는 환경 변수로 백엔드를 지정할 수 있습니다(필수 아님).
  ```bash
  export MPLBACKEND=Agg
  ```

### Q3. ROS와 섞이면 안 되나요?
- ROS 패키지는 배포판/apt 버전과 밀접해서, `pip`로 시스템 Numpy/Matplotlib을 건드리면 충돌할 수 있습니다.
- 이 문서처럼 **분석만 venv에서** 진행하세요.

### Q4. 매번 활성화하기 귀찮아요.
- `~/.bashrc`에 alias 추가:
  ```bash
  alias si-venv='source ~/.venvs/si/bin/activate'
  ```
  사용: `si-venv`

---

## 6) 참고: venv 삭제(깨끗이 다시 만들기)
```bash
rm -rf ~/.venvs/si
python3 -m venv ~/.venvs/si
source ~/.venvs/si/bin/activate
pip install --upgrade pip
pip install "numpy>=2.0" "matplotlib>=3.9" "pandas>=2.0"
```

---

## 7) 재현가능 버전 체크
```bash
python -c "import sys, numpy, matplotlib, pandas; print(sys.version); print('numpy', numpy.__version__); print('matplotlib', matplotlib.__version__); print('pandas', pandas.__version__)"
```
예시 출력(달라도 무관):
```
Python 3.10.x
numpy 2.2.x
matplotlib 3.9.x
pandas 2.2.x
```

---

### 끝. 문제가 계속되면 현재 버전/오류 메시지를 붙여 알려주세요!
