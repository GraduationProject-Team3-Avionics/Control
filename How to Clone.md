# Private Organization Repository Clone 가이드 (SSH 권장)

## 상황 요약
- Interface 레포: **PUBLIC** → 인증 없이 clone 가능
- Control 레포: **PRIVATE** → 인증 없으면 `Repository not found` 발생 (정상 동작)
- HTTPS + 토큰은 환경/캐시 문제로 자주 실패
- **SSH 방식이 가장 안정적이며 팀 프로젝트에 권장**

---

## 권장 방법: SSH로 Clone

### 1. SSH 키 존재 여부 확인
```bash
ls ~/.ssh
```
- `id_ed25519`, `id_ed25519.pub` 있으면 → 3번으로
- 없으면 → 2번부터

---

### 2. SSH 키 생성
```bash
ssh-keygen -t ed25519 -C "본인 GitHub 이메일"
```
- 엔터 3번 (기본 경로)
- passphrase는 선택

생성 파일:
- 개인키: `~/.ssh/id_ed25519` (공유 금지)
- 공개키: `~/.ssh/id_ed25519.pub`

---

### 3. ssh-agent에 키 등록
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

---

### 4. GitHub에 SSH 공개키 등록
```bash
cat ~/.ssh/id_ed25519.pub
```

출력 예:
```
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAI... your_email@gmail.com
```

GitHub → 개인 계정 →  
Settings → SSH and GPG keys → New SSH key

- Title: PC 이름 (예: ubuntu-desktop)
- Key: 위 한 줄 전체 붙여넣기
- Key type: Authentication Key

---

### 5. GitHub SSH 연결 테스트
```bash
ssh -T git@github.com
```

정상 출력:
```
Hi <github-id>! You've successfully authenticated, but GitHub does not provide shell access.
```

---

### 6. 레포 Clone
```bash
git clone git@github.com:GraduationProject-Team3-Avionics/Control.git
```

---

## 자주 발생하는 이슈

### Q1. 레포가 있는데 `Repository not found`
- Private 레포 + 인증 안 됐을 때 GitHub 보안 정책상 정상 메시지

### Q2. Organization 만든 사람인데도 안 됨
- 로컬 PC git이 GitHub에 인증 안 된 상태

### Q3. Interface는 되는데 Control은 안 됨
| 레포 | 공개 여부 | 인증 필요 |
|------|-----------|-----------|
| Interface | PUBLIC | ❌ |
| Control | PRIVATE | ✅ |

---

## 요약
> Control 레포는 Private임.  
> **SSH 키 등록 후 아래 명령으로 clone하면 됨**

```bash
git clone git@github.com:GraduationProject-Team3-Avionics/Control.git
```