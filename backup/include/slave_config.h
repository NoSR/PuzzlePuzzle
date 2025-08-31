/*
 * Slave Board 설정 헤더 파일
 * Arduino Nano 핀 매핑 및 상수 정의
 */

#ifndef SLAVE_CONFIG_H
#define SLAVE_CONFIG_H

#include <Arduino.h>

// =============================================================================
// 핀 매핑 정의
// =============================================================================

// 모터 제어 핀 (L298N 드라이버)
// Head 모터 제어 (L298N 모터 A)
#define HEAD_PWM_PIN          5    // ENA - 속도 제어 (PWM)
#define HEAD_DIR1_PIN         7    // IN1 - 방향 제어 1
#define HEAD_DIR2_PIN         8    // IN2 - 방향 제어 2

// Body 모터 제어 (L298N 모터 B)
#define BODY_PWM_PIN          6    // ENB - 속도 제어 (PWM)
#define BODY_DIR1_PIN         9    // IN3 - 방향 제어 1
#define BODY_DIR2_PIN         10   // IN4 - 방향 제어 2

// DFPlayer Mini 통신 핀
#define DFR_RX_PIN            2    // Slave -> DFPlayer
#define DFR_TX_PIN            3    // DFPlayer -> Slave

// 진동 센서 아날로그 입력 핀 (PCB 지정)
#define Analog0               A0   // 발판 1 진동 센서
#define Analog1               A1   // 발판 2 진동 센서
#define Analog2               A2   // 발판 3 진동 센서
#define Analog3               A3   // 발판 4 진동 센서

// 호환성을 위한 별칭
#define VIBRATION_1_PIN       Analog0
#define VIBRATION_2_PIN       Analog1
#define VIBRATION_3_PIN       Analog2
#define VIBRATION_4_PIN       Analog3

// I2C 통신 핀 (MCP23017용)
#define SDA_PIN               A4   // I2C 데이터
#define SCL_PIN               A5   // I2C 클럭

// =============================================================================
// MCP23017 확장 칩 설정 (2개 사용)
// =============================================================================

#define MCP23017_1_ADDR       0x20 // MCP23017 #1 I2C 주소 (필수 장치용)
#define MCP23017_2_ADDR       0x21 // MCP23017 #2 I2C 주소 (예비 장치용)

// MCP23017 레지스터 주소
#define MCP_IODIRA            0x00 // 포트A 방향 설정
#define MCP_IODIRB            0x01 // 포트B 방향 설정
#define MCP_GPIOA             0x12 // 포트A GPIO
#define MCP_GPIOB             0x13 // 포트B GPIO
#define MCP_GPPUA             0x0C // 포트A 풀업
#define MCP_GPPUB             0x0D // 포트B 풀업

// =============================================================================
// MCP23017 #1 핀 매핑 (PCB 지정 핀 배열) - 주소 0x20
// =============================================================================

// MCP1 INPUT 핀 배열 (GPIOB 0-7 + GPIOA 0)
#define IN0                   11   // GPIOB 3번 핀
#define IN1                   10   // GPIOB 2번 핀  
#define IN2                   9    // GPIOB 1번 핀
#define IN3                   8    // GPIOB 0번 핀
#define IN4                   12   // GPIOB 4번 핀
#define IN5                   13   // GPIOB 5번 핀
#define IN6                   14   // GPIOB 6번 핀
#define IN7                   0    // GPIOA 0번 핀

// MCP1 OUTPUT 핀 배열 (ULN2803 연결, 12V 제어) - GPIOA 1-7 + GPIOB 7
#define OutSign0              1    // GPIOA 1번 핀
#define OutSign1              2    // GPIOA 2번 핀
#define OutSign2              3    // GPIOA 3번 핀
#define OutSign3              4    // GPIOA 4번 핀
#define OutSign4              5    // GPIOA 5번 핀
#define OutSign5              6    // GPIOA 6번 핀
#define OutSign6              7    // GPIOA 7번 핀
#define OutSign7              15   // GPIOB 7번 핀

// MCP1 핀 비트 매핑 (실제 MCP23017 핀 번호 -> 비트 번호)
#define IN0_BIT               3    // 11번 -> GPIOB 3번 비트
#define IN1_BIT               2    // 10번 -> GPIOB 2번 비트
#define IN2_BIT               1    // 9번 -> GPIOB 1번 비트
#define IN3_BIT               0    // 8번 -> GPIOB 0번 비트
#define IN4_BIT               4    // 12번 -> GPIOB 4번 비트
#define IN5_BIT               5    // 13번 -> GPIOB 5번 비트
#define IN6_BIT               6    // 14번 -> GPIOB 6번 비트
#define IN7_BIT               0    // 0번 -> GPIOA 0번 비트

#define OutSign0_BIT          1    // 1번 -> GPIOA 1번 비트
#define OutSign1_BIT          2    // 2번 -> GPIOA 2번 비트
#define OutSign2_BIT          3    // 3번 -> GPIOA 3번 비트
#define OutSign3_BIT          4    // 4번 -> GPIOA 4번 비트
#define OutSign4_BIT          5    // 5번 -> GPIOA 5번 비트
#define OutSign5_BIT          6    // 6번 -> GPIOA 6번 비트
#define OutSign6_BIT          7    // 7번 -> GPIOA 7번 비트
#define OutSign7_BIT          7    // 15번 -> GPIOB 7번 비트

// =============================================================================
// MCP23017 #2 핀 매핑 (PCB 지정 핀 배열) - 주소 0x21
// =============================================================================

// MCP2 점수 LED 핀 배열 (fstMulti0~5) - GPIOA 0-5
#define fstMulti0             0    // GPIOA 0번 핀
#define fstMulti1             5    // GPIOA 5번 핀
#define fstMulti2             1    // GPIOA 1번 핀
#define fstMulti3             4    // GPIOA 4번 핀
#define fstMulti4             2    // GPIOA 2번 핀
#define fstMulti5             3    // GPIOA 3번 핀

// MCP2 예비장치용 핀 배열 (sndMulti0~5) - GPIOB 0-5
#define sndMulti0             8    // GPIOB 0번 핀
#define sndMulti1             9    // GPIOB 1번 핀
#define sndMulti2             13   // GPIOB 5번 핀
#define sndMulti3             10   // GPIOB 2번 핀
#define sndMulti4             12   // GPIOB 4번 핀
#define sndMulti5             11   // GPIOB 3번 핀

// MCP2 핀 비트 매핑 (실제 MCP23017 핀 번호 -> 비트 번호)
#define fstMulti0_BIT         0    // 0번 -> GPIOA 0번 비트
#define fstMulti1_BIT         5    // 5번 -> GPIOA 5번 비트
#define fstMulti2_BIT         1    // 1번 -> GPIOA 1번 비트
#define fstMulti3_BIT         4    // 4번 -> GPIOA 4번 비트
#define fstMulti4_BIT         2    // 2번 -> GPIOA 2번 비트
#define fstMulti5_BIT         3    // 3번 -> GPIOA 3번 비트

#define sndMulti0_BIT         0    // 8번 -> GPIOB 0번 비트
#define sndMulti1_BIT         1    // 9번 -> GPIOB 1번 비트
#define sndMulti2_BIT         5    // 13번 -> GPIOB 5번 비트
#define sndMulti3_BIT         2    // 10번 -> GPIOB 2번 비트
#define sndMulti4_BIT         4    // 12번 -> GPIOB 4번 비트
#define sndMulti5_BIT         3    // 11번 -> GPIOB 3번 비트

// =============================================================================
// 핀 배열 정의 (편의성을 위한)
// =============================================================================

// L298N 모터 드라이버 핀 배열 (PWM, DIR1, DIR2 순서)
const uint8_t HEAD_MOTOR_PINS[3] = {
  HEAD_PWM_PIN, HEAD_DIR1_PIN, HEAD_DIR2_PIN
};

const uint8_t BODY_MOTOR_PINS[3] = {
  BODY_PWM_PIN, BODY_DIR1_PIN, BODY_DIR2_PIN
};

// 모든 모터 제어 핀 (초기화 시 사용)
const uint8_t ALL_MOTOR_PINS[6] = {
  HEAD_PWM_PIN, HEAD_DIR1_PIN, HEAD_DIR2_PIN,
  BODY_PWM_PIN, BODY_DIR1_PIN, BODY_DIR2_PIN
};

// MCP23017 #1 핀 배열 (PCB 지정 핀 배열)
// INPUT 핀 배열 (IN0~7)
const uint8_t INPUT_PINS[8] = {
  IN0, IN1, IN2, IN3, IN4, IN5, IN6, IN7
};

const uint8_t INPUT_BITS[8] = {
  IN0_BIT, IN1_BIT, IN2_BIT, IN3_BIT, IN4_BIT, IN5_BIT, IN6_BIT, IN7_BIT
};

// OUTPUT 핀 배열 (OutSign0~7) - ULN2803 연결, 12V 제어
const uint8_t OUTPUT_PINS[8] = {
  OutSign0, OutSign1, OutSign2, OutSign3, OutSign4, OutSign5, OutSign6, OutSign7
};

const uint8_t OUTPUT_BITS[8] = {
  OutSign0_BIT, OutSign1_BIT, OutSign2_BIT, OutSign3_BIT, 
  OutSign4_BIT, OutSign5_BIT, OutSign6_BIT, OutSign7_BIT
};

// MCP23017 #2 핀 배열 (PCB 지정 핀 배열)
// 점수 LED 핀 배열 (fstMulti0~5) - MCP2 GPIOA
const uint8_t SCORE_LED_PINS[6] = {
  fstMulti0, fstMulti1, fstMulti2, fstMulti3, fstMulti4, fstMulti5
};

const uint8_t SCORE_LED_BITS[6] = {
  fstMulti0_BIT, fstMulti1_BIT, fstMulti2_BIT, fstMulti3_BIT, fstMulti4_BIT, fstMulti5_BIT
};

// 예비 장치 핀 배열 (sndMulti0~5) - MCP2 GPIOB
const uint8_t SPARE_DEVICE_PINS[6] = {
  sndMulti0, sndMulti1, sndMulti2, sndMulti3, sndMulti4, sndMulti5
};

const uint8_t SPARE_DEVICE_BITS[6] = {
  sndMulti0_BIT, sndMulti1_BIT, sndMulti2_BIT, sndMulti3_BIT, sndMulti4_BIT, sndMulti5_BIT
};

// 진동 센서 핀 배열 (호환성 유지)
const uint8_t VIBRATION_PINS[4] = {
  Analog0, Analog1, Analog2, Analog3
};

// =============================================================================
// 모터 제어 상수 (L298N 드라이버)
// =============================================================================

// 모터 회전 방향
#define MOTOR_STOP            0
#define MOTOR_FORWARD         1    // IN1=HIGH, IN2=LOW
#define MOTOR_BACKWARD        2    // IN1=LOW, IN2=HIGH

// 모터 속도 (PWM 값, 0-255)
#define MOTOR_SPEED_SLOW      100  // 저속 (약 40%)
#define MOTOR_SPEED_NORMAL    150  // 보통 (약 60%)
#define MOTOR_SPEED_FAST      200  // 고속 (약 80%)
#define MOTOR_SPEED_MAX       255  // 최대 (100%)

// 모터 제어 타이밍
#define MOTOR_BRAKE_DURATION  5000  // 모터 브레이크 지속시간 (ms)
#define MOTOR_TIMEOUT         10000 // 모터 동작 타임아웃 (ms)
#define MOTOR_180_DURATION    2000  // 180도 회전 예상 시간 (ms) - 실제 테스트 후 조정 필요

// =============================================================================
// 센서 설정
// =============================================================================

// 진동 센서 설정
#define VIBRATION_THRESHOLD   512   // 진동 감지 임계값 (0-1023)
#define VIBRATION_DEBOUNCE    50    // 디바운싱 시간 (ms)
#define VIBRATION_SAMPLES     5     // 평균화를 위한 샘플 수

// 리미트 스위치 설정
#define LIMIT_DEBOUNCE        20    // 리미트 스위치 디바운싱 시간 (ms)

// =============================================================================
// LED 패턴 구조체
// =============================================================================

// LED 패턴 구조체
struct LEDPattern {
  uint8_t sequence[6];       // LED 점멸 순서 (최대 6개)
  uint8_t length;            // 패턴 길이 (1-6)
  uint16_t interval;         // 점멸 간격 (ms)
  uint8_t repeatCount;       // 반복 횟수
};

// 점수 LED 상태 구조체
struct ScoreLEDState {
  uint8_t currentScore;      // 현재 점수 (0-7)
  bool isBlinking;           // 점멸 상태 여부
  unsigned long blinkTimer;  // 점멸 타이머
  uint8_t blinkCount;        // 점멸 횟수
};

// =============================================================================
// DFPlayer 관련 상수
// =============================================================================

// 효과음 트랙 번호 (SD카드 파일 번호와 매칭)
#define SFX_LIGHT_ON          1    // 조명 점등 효과음
#define SFX_PATTERN_BEEP      2    // 패턴 표시 비프음
#define SFX_PAD_PRESS         3    // 발판 밟기 효과음
#define SFX_CORRECT           4    // 정답 효과음
#define SFX_WRONG             5    // 오답 효과음
#define SFX_GUNSHOT           6    // 총소리 효과음
#define SFX_SUCCESS           7    // 성공 효과음
#define SFX_LEVEL_UP          8    // 레벨업 효과음

// DFPlayer 명령어
#define DF_CMD_PLAY           0x0F
#define DF_CMD_VOLUME         0x06
#define DF_CMD_STOP           0x16

// =============================================================================
// 게임 로직 상수
// =============================================================================

// 레벨별 패턴 설정
#define LEVEL_1_4_INTERVAL    500   // 레벨 1-4 패턴 간격 (ms)
#define LEVEL_5_10_INTERVAL   300   // 레벨 5-10 패턴 간격 (ms)
#define LEVEL_11_INTERVAL     300   // 레벨 11 패턴 간격 (ms)

// 패턴 길이 설정
#define LEVEL_1_LENGTH        2     // 레벨 1 패턴 길이
#define LEVEL_2_LENGTH        3     // 레벨 2 패턴 길이
#define LEVEL_3_LENGTH        4     // 레벨 3 패턴 길이
#define LEVEL_4_LENGTH        5     // 레벨 4 패턴 길이
#define LEVEL_5_10_LENGTH     5     // 레벨 5-10 패턴 길이
#define LEVEL_11_LENGTH       6     // 레벨 11 패턴 길이

// 점수 LED 점멸 설정 (7점 달성시)
#define SCORE_BLINK_INTERVAL  500   // 점멸 간격 (ms)
#define SCORE_BLINK_COUNT     7     // 점멸 횟수

// =============================================================================
// 유틸리티 매크로
// =============================================================================

// 비트 조작 매크로
#define SET_BIT(reg, bit)     ((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit)   ((reg) &= ~(1 << (bit)))
#define TOGGLE_BIT(reg, bit)  ((reg) ^= (1 << (bit)))
#define CHECK_BIT(reg, bit)   (((reg) >> (bit)) & 1)

// 범위 제한 매크로
#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 배열 크기 매크로
#define ARRAY_SIZE(arr)       (sizeof(arr) / sizeof((arr)[0]))

#endif // SLAVE_CONFIG_H