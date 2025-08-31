# 무궁화 게임 시스템

오징어 게임을 모티브로 한 인터랙티브 무궁화 게임 시스템입니다.

## 프로젝트 구조

```
무궁화_게임_시스템/
├── src/                       # PlatformIO 소스 코드
│   ├── master/               # Master Board 소스 (Arduino Mega 2560)
│   │   └── main.cpp         # Master Board 메인 코드
│   └── slave/               # Slave Board 소스 (Arduino Nano)
│       └── main.cpp         # Slave Board 메인 코드
├── include/                  # 공통 헤더 파일
│   ├── master_config.h      # Master Board 설정 및 핀 매핑
│   ├── slave_config.h       # Slave Board 설정 및 핀 매핑
│   └── communication.h      # 통신 프로토콜 정의
├── .kiro/specs/mugunghwa-game/ # 프로젝트 명세서
│   ├── requirements.md      # 요구사항 문서
│   ├── design.md           # 설계 문서
│   └── tasks.md            # 구현 작업 목록
└── platformio.ini          # PlatformIO 설정 파일
```

## 하드웨어 구성

### Master Board (Arduino Mega 2560)
- **주요 기능**: 게임 로직, PIR 센서, LED 제어, 오디오 재생
- **입력**: PIR 센서 16개 (동작 감지)
- **출력**: 천정 조명 4개, 경광등 2개, 이펙트 LED 6개, EM-Lock
- **통신**: DFPlayer Mini (내레이션/BGM), 시리얼 통신 (Slave와)

### Slave Board (Arduino Nano)
- **주요 기능**: 모터 제어, 입력 처리, LED 제어, 효과음 재생
- **입력**: 진동 센서 4개 (발판), 리미트 스위치 2개
- **출력**: DC 모터 2개 (로봇 Head/Body), LED 10개 (문제/점수 표시)
- **통신**: DFPlayer Mini (효과음), I2C (MCP23017), 시리얼 통신 (Master와)

## 개발 환경 설정

### PlatformIO 사용 (권장)

1. **PlatformIO 설치**
   ```bash
   # Python이 설치되어 있다면
   pip install platformio
   
   # 또는 VS Code Extension 설치
   # PlatformIO IDE extension 검색 후 설치
   ```

2. **프로젝트 빌드**
   ```bash
   # Master Board 빌드 (Arduino Mega 2560)
   pio run -e master
   
   # Slave Board 빌드 (Arduino Nano)
   pio run -e slave
   ```

3. **업로드**
   ```bash
   # Master Board 업로드
   pio run -e master -t upload
   
   # Slave Board 업로드
   pio run -e slave -t upload
   ```

4. **시리얼 모니터**
   ```bash
   # Master Board 모니터링
   pio device monitor -e master
   
   # Slave Board 모니터링
   pio device monitor -e slave
   ```

### 🔧 **Multi-Environment Source Filter 방식**

이 프로젝트는 **PlatformIO의 Source Filter 기능**을 사용하여 하나의 프로젝트에서 두 개의 서로 다른 보드를 관리합니다:

- **Master 환경**: `src/master/` 디렉터리만 컴파일
- **Slave 환경**: `src/slave/` 디렉터리만 컴파일
- **공통 헤더**: `include/` 디렉터리의 헤더 파일들을 양쪽에서 공유

이 방식의 장점:
- ✅ 공통 헤더 파일 공유 (통신 프로토콜 등)
- ✅ 각 보드별 완전히 독립적인 컴파일
- ✅ 하나의 프로젝트에서 두 보드 관리
- ✅ 버전 관리 및 협업 용이성

### Arduino IDE 사용 (대안)

1. Arduino IDE 설치
2. 보드 매니저에서 Arduino AVR Boards 설치
3. 라이브러리 매니저에서 필요한 라이브러리 설치:
   - **SoftwareSerial**: DFPlayer Mini 통신용
   - **Wire**: I2C 통신용 (MCP23017)

### 필요한 도구

- **Arduino CLI** (선택사항): 명령줄에서 컴파일/업로드
  ```bash
  winget install ArduinoSA.CLI
  ```

## 게임 규칙

1. **시작**: 시작 버튼을 눌러 게임 시작
2. **조명 시퀀스**: 천정 조명이 순차적으로 점등
3. **로봇 회전**: 로봇이 정면을 향해 환영 인사
4. **패턴 표시**: 로봇이 뒤를 돌고 LED 패턴 표시
5. **플레이어 입력**: 같은 순서로 발판을 밟아야 함
6. **동작 감지**: 로봇이 돌아보는 동안 움직이면 감점
7. **점수 획득**: 올바른 입력 시 1점 획득, 7점 달성 시 클리어

## 통신 프로토콜

Master와 Slave 보드 간 시리얼 통신을 통해 명령과 응답을 주고받습니다.

### 메시지 형식
```
[HEADER][COMMAND][DATA1][DATA2][CHECKSUM]
- HEADER: 0xAA (고정)
- COMMAND: 명령어 코드
- DATA1, DATA2: 추가 데이터
- CHECKSUM: XOR 체크섬
```

### 주요 명령어
- `CMD_ROBOT_HEAD_FRONT`: 로봇 HEAD 정면 회전
- `CMD_ROBOT_BODY_BACK`: 로봇 BODY 후면 회전
- `CMD_SHOW_PATTERN`: LED 패턴 표시
- `CMD_PLAY_EFFECT`: 효과음 재생

## 안전 기능

- **모터 브레이크**: 모터 정지 시 5초간 브레이크 신호 인가
- **리미트 스위치**: 모터 회전 범위 제한
- **통신 타임아웃**: 통신 장애 시 안전 모드 전환
- **체크섬 검증**: 데이터 무결성 보장

## 개발 상태

### 완료된 기능
- ✅ **프로젝트 구조**: PlatformIO Multi-Environment Source Filter 방식으로 완전 정리
- ✅ **Master Board 완전 구현**: 
  - 핀 매핑, 게임 상태 관리
  - 이펙트 LED 패턴 시스템 (6가지 상황별 패턴 엔진)
  - DFPlayer 오디오 시스템 (내레이션 및 BGM 재생)
  - 시리얼 명령어를 통한 디버깅 도구
- ✅ **Slave Board 기본 구현**:
  - L298N 모터 제어 시스템 (Head/Body 모터)
  - MCP23017 I2C 확장 칩 제어 (LED, INPUT/OUTPUT)
  - 진동 센서 모니터링 시스템
  - Master와의 시리얼 통신 구조
- ✅ **공통 시스템**:
  - 통신 프로토콜 정의 및 구현
  - 공통 헤더 파일 공유 구조
  - 각 보드별 독립적인 컴파일 환경

### 진행 중인 작업
- 🔄 **PIR 센서 시스템**: 동작 감지 로직
- 🔄 **천정 조명 제어**: 순차 점등 시스템
- 🔄 **경광등 제어**: 게임 상태 연동
- 🔄 **Slave Board 구현**: 모터 제어, LED 패턴 표시

### 테스트 방법

Master Board에서 시리얼 모니터를 통해 다음 명령어로 테스트 가능:
- `TEST`: 패턴 엔진 전체 검증
- `SUCCESS`: 성공 패턴 재생
- `ERROR`: 오류 패턴 재생
- `LEVELUP`: 레벨업 패턴 재생
- `START`: 게임 시작 패턴 재생
- `CLEAR`: 게임 클리어 패턴 재생
- `TENSION`: 긴장감 패턴 재생
- `STOP`: 모든 패턴 정지
- `STATUS`: 엔진 상태 출력
- `HELP`: 도움말

자세한 구현 계획은 `.kiro/specs/mugunghwa-game/tasks.md` 파일을 참조하세요.