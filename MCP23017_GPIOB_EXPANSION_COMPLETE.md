# MCP23017 GPIOB 핀 확장 완료

## 📋 **확장된 핀 구성**

### 🔧 **MCP23017 #1 (I2C 주소: 0x20) 핀 할당**

#### **GPIOA (출력용)**
- **비트 0~3**: 사용 안함 (예약)
- **비트 4~7**: 문제 제시 LED (4개)

#### **GPIOB (입력용) - 새로 추가**
- **비트 0**: HEAD_LIMIT_SWITCH (Head 모터 리미트 스위치)
- **비트 1**: BODY_LIMIT_SWITCH (Body 모터 리미트 스위치)
- **비트 2~6**: DIGITAL_INPUT_0~4 (추가 디지털 입력 5개)
- **비트 7**: 사용 안함 (예약)

### 🎯 **MCP23017 #2 (I2C 주소: 0x21) 핀 할당**

#### **GPIOA (출력용)**
- **비트 0~5**: 점수 LED (6개)
- **비트 6~7**: 사용 안함 (예약)

## 🔧 **주요 변경 사항**

### 1. **리미트 스위치 이동**
```cpp
// 기존: Arduino 디지털 핀 사용
#define HEAD_LIMIT_PIN 12    // 제거됨
#define BODY_LIMIT_PIN 13    // 제거됨

// 새로운: MCP23017 #1 GPIOB 사용
#define HEAD_LIMIT_BIT 0     // MCP23017 #1 GPIOB 0번 비트
#define BODY_LIMIT_BIT 1     // MCP23017 #1 GPIOB 1번 비트
```

### 2. **추가 디지털 입력 핀**
```cpp
// 새로 추가된 디지털 입력
#define DIGITAL_INPUT_COUNT 5
#define DIGITAL_INPUT_START_BIT 2  // GPIOB 2번 비트부터 시작
#define DIGITAL_INPUT_MASK 0x7C    // 비트 2,3,4,5,6 = 01111100

// 사용 가능한 디지털 입력
// DIGITAL_INPUT_0: GPIOB 비트 2
// DIGITAL_INPUT_1: GPIOB 비트 3
// DIGITAL_INPUT_2: GPIOB 비트 4
// DIGITAL_INPUT_3: GPIOB 비트 5
// DIGITAL_INPUT_4: GPIOB 비트 6
```

### 3. **새로운 상태 관리 구조체**
```cpp
struct DigitalInputState {
  uint8_t currentState;     // 현재 입력 상태 (비트 플래그)
  uint8_t previousState;    // 이전 입력 상태 (비트 플래그)
  unsigned long lastReadTime;  // 마지막 읽기 시간
  unsigned long debounceTime[DIGITAL_INPUT_COUNT];  // 각 입력의 디바운스 시간
};
```

## 🔧 **새로운 함수들**

### 1. **MCP23017 입력 읽기**
```cpp
uint8_t readMCP23017Input(uint8_t addr, uint8_t reg);
```

### 2. **디지털 입력 모니터링**
```cpp
void monitorDigitalInputs();  // 10ms마다 입력 상태 업데이트 (디바운싱 포함)
```

### 3. **리미트 스위치 읽기**
```cpp
bool readHeadLimitSwitch();   // Head 모터 리미트 스위치 상태
bool readBodyLimitSwitch();   // Body 모터 리미트 스위치 상태
```

### 4. **디지털 입력 읽기**
```cpp
bool readDigitalInput(uint8_t inputNumber);  // 개별 디지털 입력 읽기 (0~4)
uint8_t readAllDigitalInputs();              // 모든 디지털 입력을 한번에 읽기
```

### 5. **테스트 함수**
```cpp
void testDigitalInputs();  // 디지털 입력 실시간 테스트
```

## 🔧 **MCP23017 초기화 변경**

### **GPIOB 입력 설정**
```cpp
// GPIOB: 입력 (리미트 스위치 + 디지털 입력용)
Wire.beginTransmission(MCP23017_1_ADDR);
Wire.write(MCP_IODIRB);
Wire.write(0xFF);  // 모든 비트 입력
Wire.endTransmission();

// GPIOB 풀업 활성화
Wire.beginTransmission(MCP23017_1_ADDR);
Wire.write(0x0D);  // GPPUB 레지스터
Wire.write(0xFF);  // 모든 비트 풀업 활성화
Wire.endTransmission();
```

## 🎯 **기능 특징**

### ✅ **디바운싱 처리**
- 모든 입력에 50ms 디바운싱 적용
- 노이즈 및 채터링 방지

### ✅ **풀업 저항 활성화**
- MCP23017 내장 풀업 저항 사용
- 외부 풀업 저항 불필요

### ✅ **실시간 모니터링**
- 10ms 주기로 입력 상태 업데이트
- loop() 함수에서 자동 호출

### ✅ **비트 플래그 최적화**
- 메모리 효율적인 상태 관리
- 빠른 비트 연산 활용

## 🔍 **사용 방법**

### 1. **리미트 스위치 읽기**
```cpp
if (readHeadLimitSwitch()) {
  // Head 모터 리미트 스위치가 눌림
}

if (readBodyLimitSwitch()) {
  // Body 모터 리미트 스위치가 눌림
}
```

### 2. **개별 디지털 입력 읽기**
```cpp
for (int i = 0; i < DIGITAL_INPUT_COUNT; i++) {
  if (readDigitalInput(i)) {
    Serial.print("디지털 입력 ");
    Serial.print(i);
    Serial.println(" 활성화");
  }
}
```

### 3. **모든 디지털 입력 한번에 읽기**
```cpp
uint8_t inputs = readAllDigitalInputs();
// inputs의 각 비트가 해당 입력 상태를 나타냄
// 비트 0 = DIGITAL_INPUT_0
// 비트 1 = DIGITAL_INPUT_1
// ...
// 비트 4 = DIGITAL_INPUT_4
```

## 🔍 **테스트 명령어**

### **시리얼 모니터 명령어**
```
status       // 시스템 상태 (디지털 입력 포함)
input_test   // 디지털 입력 실시간 테스트
motor_test   // 모터 테스트
led_test     // LED 테스트
stop         // 모든 출력 정지
```

### **input_test 출력 예시**
```
=== 디지털 입력 테스트 ===
Head Limit: OFF | Body Limit: OFF | Digital Inputs: D0:0 D1:1 D2:0 D3:1 D4:0 | Raw: 0x0A
```

## ✅ **컴파일 결과**
- **상태**: 성공 ✅
- **RAM 사용량**: 68.3% (1,398 / 2,048 bytes)
- **Flash 사용량**: 42.6% (13,078 / 30,720 bytes)

## 🎯 **확장된 기능 요약**

### ✅ **기존 기능 유지**
- ✅ 문제 LED 4개 (MCP23017 #1 GPIOA 4~7)
- ✅ 점수 LED 6개 (MCP23017 #2 GPIOA 0~5)
- ✅ 7점 클리어 도미노 효과
- ✅ 진동 센서 4개 (Arduino 아날로그 핀)
- ✅ 모터 제어 (Arduino 디지털 핀)

### ✅ **새로 추가된 기능**
- ✅ Head 리미트 스위치 (MCP23017 #1 GPIOB 0)
- ✅ Body 리미트 스위치 (MCP23017 #1 GPIOB 1)
- ✅ 디지털 입력 5개 (MCP23017 #1 GPIOB 2~6)
- ✅ 실시간 입력 모니터링 및 디바운싱
- ✅ 디지털 입력 테스트 기능

## 🎊 **확장 완료!**

이제 MCP23017 #1의 GPIOB 핀들을 활용하여:
1. **리미트 스위치 2개** (Arduino 핀 절약)
2. **추가 디지털 입력 5개** (새로운 센서 연결 가능)

총 **7개의 추가 입력 핀**을 확보했습니다! 🎉

새로운 센서나 스위치를 연결할 준비가 완료되었습니다!