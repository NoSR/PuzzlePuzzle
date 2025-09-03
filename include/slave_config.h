/*
 * Slave Board 설정 파일 (최적화 버전)
 * Arduino Nano용
 */

#ifndef SLAVE_CONFIG_OPTIMIZED_H
#define SLAVE_CONFIG_OPTIMIZED_H

#include <Arduino.h>

// =============================================================================
// 하드웨어 핀 정의
// =============================================================================

// 모터 제어 핀 (L298N)
#define HEAD_PWM_PIN 5
#define HEAD_DIR1_PIN 6
#define HEAD_DIR2_PIN 7
#define BODY_PWM_PIN 10
#define BODY_DIR1_PIN 8
#define BODY_DIR2_PIN 9

// 진동 센서 핀 (아날로그)
#define VIBRATION_SENSOR_COUNT 4
extern const uint8_t VIBRATION_PINS[VIBRATION_SENSOR_COUNT];

// DFPlayer 핀
#define DF_RX_PIN 3
#define DF_TX_PIN 2

// 리미트 스위치 핀 (MCP23017 #1 GPIOB로 이동)
#define HEAD_LIMIT_BIT 0    // MCP23017 #1 GPIOB 0번 비트
#define BODY_LIMIT_BIT 1    // MCP23017 #1 GPIOB 1번 비트

// 추가 디지털 입력핀 (MCP23017 #1 GPIOB 2~6번 비트)
#define DIGITAL_INPUT_COUNT 5
#define DIGITAL_INPUT_START_BIT 2  // GPIOB 2번 비트부터 시작
#define DIGITAL_INPUT_MASK 0x7C    // 비트 2,3,4,5,6 = 01111100

// =============================================================================
// MCP23017 설정
// =============================================================================

#define MCP23017_1_ADDR 0x20  // 문제 제시 LED용
#define MCP23017_2_ADDR 0x21  // 점수 LED용

// MCP23017 레지스터
#define MCP_IODIRA 0x00
#define MCP_IODIRB 0x01
#define MCP_GPIOA 0x12
#define MCP_GPIOB 0x13

// =============================================================================
// 게임 상수 정의
// =============================================================================

// 모터 상수
#define MOTOR_SPEED_SLOW 100
#define MOTOR_SPEED_NORMAL 150
#define MOTOR_SPEED_FAST 200
#define MOTOR_180_DURATION 3000
#define MOTOR_BRAKE_DURATION 5000
#define MOTOR_TIMEOUT 30000

// 진동 센서 상수
#define VIBRATION_THRESHOLD 512
#define VIBRATION_DEBOUNCE 200
#define VIBRATION_SAMPLES 5

// LED 상수
#define PROBLEM_LED_COUNT 4
#define SCORE_LED_COUNT 6
// 문제 LED 비트 매핑 (MCP23017 #1 GPIOA 4,5,6,7번 비트)
#define PROBLEM_LED_START_BIT 4
#define PROBLEM_LED_MASK 0xF0  // 비트 4,5,6,7 = 11110000
// 점수 LED 비트 매핑 (MCP23017 #2 GPIOA 0,1,2,3,4,5번 비트)
#define SCORE_LED_START_BIT 0
#define SCORE_LED_MASK 0x3F    // 비트 0,1,2,3,4,5 = 00111111
#define LED_BLINK_INTERVAL 500
#define LED_PATTERN_INTERVAL 500
// 7점 클리어 시 도미노 효과 설정
#define CLEAR_DOMINO_INTERVAL 500  // 0.5초 간격
#define CLEAR_DOMINO_CYCLES 2      // 오름차순-내림차순 2회 반복

// 효과음 ID
#define SFX_LIGHT_ON 1
#define SFX_PATTERN_BEEP 2
#define SFX_PAD_PRESS 3
#define SFX_CORRECT 4
#define SFX_WRONG 5
#define SFX_GUNSHOT 6
#define SFX_SUCCESS 7
#define SFX_LEVEL_UP 8

// 입력 상수
#define MAX_INPUT_LENGTH 6
#define INPUT_TIMEOUT 10000
#define INPUT_COMPLETE_DELAY 3000

// =============================================================================
// 상태 구조체 정의
// =============================================================================

struct MotorState {
  bool headActive;
  bool bodyActive;
  unsigned long stopTime;
  bool brakeActive;
};

struct VibrationState {
  unsigned long lastTriggerTime[VIBRATION_SENSOR_COUNT];
  uint8_t inputSequence[MAX_INPUT_LENGTH];
  uint8_t inputLength;
  uint8_t padPressed;  // 비트 플래그로 변경 (4비트만 사용)
  bool inputActive;
};

struct LEDPatternState {
  bool patternActive;
  uint8_t currentStep;
  uint8_t patternLength;
  unsigned long stepTimer;
  uint16_t stepInterval;
  uint8_t currentLevel;
};

struct ScoreLEDState {
  unsigned long timer;  // blinkTimer와 dominoTimer 통합
  uint8_t currentScore;
  uint8_t blinkCount;
  uint8_t dominoStep;       // 현재 도미노 단계 (0~5)
  uint8_t dominoCycle;      // 현재 사이클 (0~1, 총 2회)
  uint8_t flags;            // 비트 플래그: bit0=blinking, bit1=dominoActive, bit2=dominoDirection
};

struct DigitalInputState {
  uint8_t currentState;     // 현재 입력 상태 (비트 플래그)
  uint8_t previousState;    // 이전 입력 상태 (비트 플래그)
  unsigned long lastReadTime;  // 마지막 읽기 시간
  unsigned long debounceTime[DIGITAL_INPUT_COUNT];  // 각 입력의 디바운스 시간
};

// =============================================================================
// 레벨별 패턴 정의
// =============================================================================

// 최대 레벨 수
#define MAX_LEVEL 11

// 레벨별 패턴 길이
extern const uint8_t LEVEL_PATTERN_LENGTHS[MAX_LEVEL];

// 레벨별 패턴 데이터
extern const uint8_t LEVEL_PATTERNS[MAX_LEVEL][MAX_INPUT_LENGTH];

// =============================================================================
// 함수 선언
// =============================================================================

// 초기화 함수들
void initializePins();
void initializeMCP23017();
void initializeDFPlayer();
void initializeStates();

// 모터 제어 함수들
void rotateHeadForward();
void rotateHeadBackward();
void rotateBodyForward();
void rotateBodyBackward();
void stopAllMotors();
void handleMotorBrake();

// 진동 센서 함수들
void monitorVibrationSensors();
void startInputCapture();
void stopInputCapture();
bool checkInputSequence(uint8_t level);

// LED 제어 함수들
void setMCP23017Output(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t readMCP23017Input(uint8_t addr, uint8_t reg);
void showLEDPattern(uint8_t level);
void updateLEDPattern();
void displayScore(uint8_t score);
void startScoreBlinking(uint8_t score);
void updateScoreLEDBlinking();
void startClearDominoEffect();
void updateClearDominoEffect();

// 디지털 입력 함수들
void monitorDigitalInputs();
bool readHeadLimitSwitch();
bool readBodyLimitSwitch();
bool readDigitalInput(uint8_t inputNumber);
uint8_t readAllDigitalInputs();

// DFPlayer 함수들
void sendDFCommand(uint8_t cmd, uint8_t param1, uint8_t param2);
void playEffectSound(uint8_t soundId);

// 통신 함수들
void processMasterComm();
void handleMasterCommand(uint8_t command, uint8_t data1, uint8_t data2);
void handleInputRequest();
void sendResponseToMaster(uint8_t response);

// 유틸리티 함수들
uint8_t getPatternLengthForLevel(uint8_t level);

// 시리얼 명령 함수들
void processSerialCommands();
void printSystemStatus();
void testMotorSystem();
void testLEDSystem();

#endif // SLAVE_CONFIG_OPTIMIZED_H