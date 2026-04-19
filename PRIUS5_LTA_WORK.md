# Toyota Prius 5th Gen (TSS3) openpilot LTA 조향 개발 노트

## 차량 정보
- **차량**: Toyota Prius 5th Generation (KR/TSS3)
- **VIN**: JTDACBAU4S3064780
- **openpilot fingerprint**: `TOYOTA_PRIUS_5TH_GEN`
- **플랫폼**: sunnypilot (James-1012/openpilot, James-1012/opendbc)

---

## CAN 버스 구조

| Bus | 역할 |
|-----|------|
| bus0 | PT CAN (메인) — ECU, 바퀴속도 등 |
| bus1 | CAN-FD — **EPS(전동조향)**, 카메라 |
| bus2 | 포워딩 버스 |
| bus128 | bus0 TX 루프백 |
| bus130 | bus2 TX 루프백 |

---

## 핵심 발견사항

### 1. EPS 위치
- `0x025 (STEER_ANGLE_SENSOR)` → **bus1에만 존재** (32바이트 CAN-FD)
- EPS가 bus1(CAN-FD)에 있다는 결정적 증거

### 2. OEM LTA 명령 메시지: 0x081 (TSS3)
TSS2의 0x191 (STEERING_LTA)와 달리, **TSS3 카메라는 0x081 (32바이트 CAN-FD)** 을 bus1로 전송해 EPS를 제어함.

#### 0x081 메시지 구조 (32바이트)

| 바이트 위치 | 내용 | 비고 |
|------------|------|------|
| [0:4] | `00 00 00 18` | 상수 프리앰블 |
| [4:6] | **조향 각도 명령** (signed int16 BE) | 핵심 신호 |
| [6:8] | 관련 값 (high byte 항상 `0xBF`) | 각도와 연관 |
| [8:16] | `00 00 00 00 00 00 00 00` | 항상 0 |
| [16:18] | 카메라 상태 (`0xFFA4` ≈ -92) | 천천히 변화 |
| [18:26] | `FF BF FF BF FF BF FF BF` | 항상 상수 (-65) |
| [26:28] | 방향 표시자 | 좌회전 시 0x1EED계열, 우회전/비활성 시 0x0000 |
| [28:32] | **SecOC MAC (4바이트)** | AES-CMAC, 매 프레임 변경 |

#### bytes[4:6] — 조향 각도 명령 해석

| 값 | 의미 |
|----|------|
| `0xFFBF` (-65) | **LTA 비활성 센티넬** (카메라가 LTA 꺼진 상태에서 전송) |
| 음수 (예: -2770) | 좌회전 명령 |
| 양수 (예: +2090) | 우회전 명령 |
| ≈ 0 | 직진 명령 |

- 관측 범위: **-2783 ~ +2090**
- 스케일: 미확정 (0.0044°/unit 가설, 추가 검증 필요)

#### bytes[28:32] — SecOC MAC 구조

```
[msg_cnt_flag (2bit)] [reset_flag (2bit)] [AES-CMAC 28bit]
```

- top nibble 순환 확인: `4 → 8 → C → 0 → 4 ...` (msg_cnt mod 4)
- 동일 내용에서도 매 프레임 변경 → 시간기반 freshness 포함
- **CRC32로 재현 불가** → 비밀키 없이는 계산 불가

---

## SecOC 인증 구조

### 알고리즘
`opendbc/car/secoc.py`의 `add_mac()` 함수와 동일한 방식:

```python
# Freshness Value (48 bits)
freshness = [Trip Counter (16bit)] + [Reset Counter (20bit)] + [Msg Counter (8bit)] + [Flags (4bit)]

# 인증 대상 (96 bits)
to_auth = addr(0x081, 16bit) + payload_first_4(32bit) + freshness(48bit)

# AES-CMAC → 28bit 잘라서 사용
mac = AES_CMAC(key, to_auth)[:28bit]
```

### 왜 이것이 문제인가
- 키(128-bit AES)는 차량별로 고유하게 ECU에 저장됨
- 키 없이 올바른 MAC 계산 불가
- 잘못된 MAC(예: `00000000`)으로 0x081 전송 시 → **EPS fault → 대시보드 오작동 경고**
- 시동 재시작 시 fault 클리어됨 (영구 손상 없음 확인)

### 다른 Toyota SECOC 차량과의 차이
- 기존 SECOC 지원 차량(Camry 등): `STEERING_LKA (0x190)` + `STEERING_LTA (0x191)`에 add_mac 적용
- Prius5: **완전히 다른 메시지(0x081, 32바이트)** → 기존 SECOC 구현과 별도 처리 필요
- `values.py`에 `ToyotaFlags.SECOC` 플래그 없음 (키 아직 미추출)

---

## 시도했던 방법들과 결과

### 시도 1: 0x191을 bus0으로 전송 (실패)
- **결과**: EPS 무반응
- **이유**: EPS가 bus1(CAN-FD)에 있고, bus0은 PT CAN

### 시도 2: 0x191을 bus1으로 전송 (실패)
- **결과**: panda에서 메시지 전송 확인 (`STEER_REQUEST=1` 확인), EPS 무반응
- **이유**: TSS3 EPS는 0x191을 LTA 명령으로 사용하지 않음

### 시도 3: 0x081을 zeroed MAC으로 전송 (실패 + 부작용)
- **결과**: 대시보드에 "시스템 오작동 딜러 방문" 경고 발생
- **이유**: EPS가 SecOC MAC을 검증 → 잘못된 MAC → fault 등록
- **복구**: 시동 끄고 재시작 후 경고 사라짐
- **결론**: EPS는 MAC을 **실제로 검증함** → 키 없이는 불가능

---

## 현재 코드 상태

### `opendbc/safety/modes/toyota.h`
- prius5용 TX_MSGS: 0x191을 bus1으로 전송 허용 (0x081은 제거됨)

```c
#define TOYOTA_PRIUS5_BASE_TX_MSGS \
  {0x191, 1, 8, .check_relay = true}, {0x412, 0, 8, .check_relay = true}, \
  {0x1D2, 0, 8, .check_relay = false},
```

### `opendbc/car/toyota/toyotacan.py`
- `create_lta_steer_command()`: bus 파라미터 추가됨 (prius5는 bus=1)
- `create_prius5_lta_steer_command()`: 실험용으로 추가됐으나 현재 미사용

### `opendbc/car/toyota/carcontroller.py`
- prius5에서 `lta_bus = 1` 사용하여 0x191을 bus1으로 전송
- 0x081 전송은 제거됨

---

## 앞으로 할 일 (우선순위 순)

### 필수: SecOC 키 추출
TSS3 EPS에서 AES-CMAC 키를 추출해야 조향 제어 가능.

**방법 A: OBD-II 진단 포트를 통한 키 추출**
- Toyota OBD-II seed-key 프로토콜 이용
- 다른 Toyota SECOC 차량에서 이미 성공한 방법
- openpilot 커뮤니티 도구 참조: `#secoc-cars-with-recoverable-keys`

**방법 B: openpilot/sunnypilot 커뮤니티 기다리기**
- TSS3 차량 공식 지원 시 키 추출 절차 포함될 것

### 선택: 각도 스케일 캘리브레이션
키를 얻은 후, bytes[4:6]의 정확한 스케일 결정 필요
- 현재 가설: 0.0044°/unit (TSS2와 동일)
- 실제값은 테스트로 확인 필요

---

## 0x025 (STEER_ANGLE_SENSOR) 디코딩
```
SG_ STEER_ANGLE : 3|12@0- (1.5,0) [-500|500] "deg"
```
- 32바이트 CAN-FD, bus1
- bit 3 (MSB), 12비트, Motorola 빅엔디안, signed
- 스케일: 1.5°/bit
- 범위: ±500° (스티어링 휠 각도)

---

## 빌드 & 플래시 방법 (device에서)

```bash
# panda firmware 빌드
cd /data/openpilot
PATH=/usr/local/venv/bin:$PATH PYTHONPATH=/data/openpilot \
  /usr/local/venv/bin/scons -C panda -j4

# panda 플래시
PYTHONPATH=/data/openpilot python3 -c "
import sys; sys.path.insert(0, '/data/openpilot/panda')
from panda import Panda
p = Panda()
p.flash(fn='/data/openpilot/panda/board/obj/panda_h7.bin.signed')
print('Done')
"
```

---

## DBC 파일
- `opendbc/dbc/toyota_prius_2025_pt.dbc`
- `STEERING_LTA (0x191)`: 8바이트 (TSS2 포맷과 동일하나 EPS에서 무시됨)
- `STEER_ANGLE_SENSOR (0x025)`: 32바이트 CAN-FD
- `0x081`: DBC 미정의 (32바이트 CAN-FD, 역공학으로 분석)

---

## GitHub
- **opendbc**: https://github.com/James-1012/opendbc
- **openpilot**: https://github.com/James-1012/openpilot

---

## 알려진 이슈
- 시동 끌 때 comma 기기에서 비프음 10회 → 기존 이슈, 우리 변경과 무관
- 잘못된 MAC으로 0x081 전송 시 EPS fault → 시동 재시작으로 복구 가능
