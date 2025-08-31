# 무궁화 게임 시스템 - 프로젝트 구조

## 📁 프로젝트 구조 (정리 완료)

```
무궁화 게임 시스템/
├── 📁 src/                     # 소스 코드 (실제 사용)
│   ├── 📁 master/              # Master Board (Arduino Mega 2560)
│   │   └── main.cpp            # Master 메인 코드 (최적화됨)
│   └── 📁 slave/               # Slave Board (Arduino Nano)
│       └── main.cpp            # Slave 메인 코드 (최적화됨)
│
├── 📁 include/                 # 헤더 파일 (실제 사용)
│   ├── communication.h         # 통신 프로토콜 정의
│   ├── master_config.h         # Master 설정 및 상수
│   └── slave_config.h          # Slave 설정 및 상수
│
├── 📁 backup/                  # 백업 파일들
│   ├── 📁 src/
│   │   ├── 📁 master/
│   │   │   ├── main_original.cpp      # 원본 Master 코드
│   │   │   └── main_optimized.cpp     # 이전 최적화 버전
│   │   └── 📁 slave/
│   │       ├── main_original.cpp      # 원본 Slave 코드
│   │       └── main_optimized.cpp     # 이전 최적화 버전
│   └── 📁 include/
│       ├── communication.h            # 원본 통신 헤더
│       ├── master_config.h            # 원본 Master 설정
│       ├── slave_config.h             # 원본 Slave 설정
│       └── *_optimized.h              # 이전 최적화 헤더들
│
├── 📁 .kiro/                   # Kiro AI 설정
│   ├── 📁 specs/               # 스펙 문서
│   └── 📁 steering/            # AI 가이드 문서
│
├── 📁 .vscode/                 # VS Code 설정
│   ├── c_cpp_properties.json   # IntelliSense 설정
│   └── settings.json           # 에디터 설정
│
├── platformio.ini              # PlatformIO 설정 (정리됨)
├── README.md                   # 프로젝트 개요
├── USER_MANUAL.md              # 사용자 매뉴얼
├── ADMIN_MANUAL.md             # 관리자 매뉴얼
├── DEVELOPER_GUIDE.md          # 개발자 가이드
└── 기타 문서들...
```

## 🔧 빌드 환경

### PlatformIO 환경 (정리됨)
- **master**: Arduino Mega 2560 빌드
- **slave**: Arduino Nano 빌드

### 컴파일 결과
✅ **Master Board (Arduino Mega 2560)**
- RAM 사용량: 13.0% (1062/8192 bytes)
- Flash 사용량: 3.6% (9178/253952 bytes)
- 상태: 컴파일 성공

✅ **Slave Board (Arduino Nano)**
- RAM 사용량: 82.7% (1694/2048 bytes)
- Flash 사용량: 36.6% (11246/30720 bytes)
- 상태: 컴파일 성공

## 📋 주요 변경사항

### 1. 파일 정리
- ✅ 최적화된 `main_clean.cpp` → `main.cpp`로 변경
- ✅ 최적화된 헤더 파일들을 기본 파일명으로 변경
- ✅ 불필요한 중간 버전 파일들 제거
- ✅ 원본 파일들을 `backup/` 폴더로 이동

### 2. PlatformIO 설정 정리
- ✅ `master_clean`, `slave_clean` 환경 제거
- ✅ `master_optimized`, `slave_optimized` 환경 제거
- ✅ 기본 `master`, `slave` 환경만 유지

### 3. 헤더 중복 정의 해결
- ✅ `communication.h`와 `slave_config.h` 중복 상수 정의 해결
- ✅ 조건부 컴파일 지시문 추가

## 🚀 빌드 명령어

```bash
# Master Board 빌드
pio run -e master

# Slave Board 빌드
pio run -e slave

# 전체 빌드
pio run

# Master Board 업로드
pio run -e master -t upload

# Slave Board 업로드
pio run -e slave -t upload
```

## 📝 개발 가이드

### 코드 수정 시
1. `src/master/main.cpp` - Master Board 코드 수정
2. `src/slave/main.cpp` - Slave Board 코드 수정
3. `include/*.h` - 헤더 파일 수정

### 백업 파일 참조
- 원본 코드가 필요한 경우 `backup/` 폴더 참조
- 이전 최적화 버전 비교 시 `backup/` 폴더의 `*_optimized.*` 파일 참조

### 새로운 기능 추가
1. 해당 헤더 파일에 함수 선언 추가
2. main.cpp에 함수 구현 추가
3. 컴파일 테스트 수행

## ⚠️ 주의사항

- **Slave Board RAM 사용량**: 82.7%로 높음 - 추가 기능 시 주의 필요
- **Master Board**: 여유 공간 충분 - 추가 기능 개발 가능
- **백업 파일**: 삭제하지 말고 참조용으로 보관
- **헤더 파일**: 중복 정의 방지를 위해 조건부 컴파일 지시문 사용

## 🎯 다음 단계

1. **기능 테스트**: 실제 하드웨어에서 동작 테스트
2. **성능 최적화**: Slave Board RAM 사용량 최적화
3. **추가 기능**: 새로운 게임 모드 또는 안전 기능 추가
4. **문서화**: 코드 주석 및 API 문서 보완