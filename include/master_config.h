/*
 * Master Board 설정 파일 (최적화 버전)
 * Arduino Mega 2560용
 */

#ifndef MASTER_CONFIG_OPTIMIZED_H
#define MASTER_CONFIG_OPTIMIZED_H

#include <Arduino.h>

// =============================================================================
// 하드웨어 핀 정의
// =============================================================================

// PIR 센서 핀 (4개)
#define PIR_SENSOR_COUNT 4
extern const uint8_t PIR_PINS[PIR_SENSOR_COUNT];

// 천정 조명 핀 (4개)
#define CEILING_LIGHT_COUNT 4
extern const uint8_t CEILING_PINS[CEILING_LIGHT_COUNT];

// 경광등 핀
#define BEACON_GREEN_PIN 10
#define BEACON_RED_PIN 11

// 이펙트 LED 핀 (8개)
#define EFFECT_LED_COUNT 8
extern const uint8_t EFFECT_PINS[EFFECT_LED_COUNT];

// DFPlayer 핀
#define DF_RX_PIN 3
#define DF_TX_PIN 2

// EM Lock 핀
#define EM_LOCK_PIN 14

// =============================================================================
// 게임 상수 정의
// =============================================================================

// 게임 단계
enum GamePhase {
  PHASE_IDLE = 0,
  PHASE_GAME_START,
  PHASE_ROBOT_TURN_FRONT,
  PHASE_NARRATION,
  PHASE_ROBOT_TURN_BACK,
  PHASE_SHOW_PATTERN,
  PHASE_PLAYER_INPUT,
  PHASE_MOTION_DETECT,
  PHASE_SCORE_UPDATE,
  PHASE_GAME_CLEAR
};

// 이펙트 패턴
enum EffectPattern {
  EFFECT_NONE = 0,
  EFFECT_ERROR,
  EFFECT_SUCCESS,
  EFFECT_LEVEL_UP,
  EFFECT_GAME_START,
  EFFECT_GAME_CLEAR,
  EFFECT_TENSION
};

// 타이밍 상수
#define LIGHT_SEQUENCE_INTERVAL 500 // 천정 조명 점등 간격 (ms)
#define MOTION_DETECT_DURATION 5000 // 동작 감지 시간 (ms)
#define PATTERN_DISPLAY_TIME 3000   // 패턴 표시 시간 (ms)
#define INPUT_TIMEOUT 10000         // 입력 대기 시간 (ms)
#define PHASE_TIMEOUT 30000         // 단계별 타임아웃 (ms)

// PIR 센서 상수
#define PIR_DEBOUNCE_TIME 100     // 디바운싱 시간 (ms)
#define PIR_TRIGGER_DURATION 2000 // 트리거 지속 시간 (ms)

// 오디오 트랙 번호
#define AUDIO_GAME_START 10
#define AUDIO_WELCOME 11
#define AUDIO_GAME_CLEAR 12
#define AUDIO_LIGHT_1 1
#define AUDIO_LIGHT_2 2
#define AUDIO_LIGHT_3 3
#define AUDIO_LIGHT_4 4
#define AUDIO_GUNSHOT 6

// =============================================================================
// 상태 구조체 정의
// =============================================================================

struct GameState {
  GamePhase currentPhase;
  uint8_t currentLevel;
  uint8_t currentScore;
  bool isGameActive;
  bool isRobotFacing;
  unsigned long phaseStartTime;
  bool motionDetected;
};

struct PIRSensorState {
  bool sensorStates[PIR_SENSOR_COUNT];
  unsigned long lastTriggerTime[PIR_SENSOR_COUNT];
  bool motionDetected;
  unsigned long lastMotionTime;
};

struct CeilingLightState {
  bool isSequenceActive;
  uint8_t currentStep;
  unsigned long lastStepTime;
  uint16_t stepInterval;
};

struct BeaconLightState {
  bool greenOn;
  bool redOn;
  bool isBlinking;
  uint16_t blinkInterval;
  unsigned long lastBlinkTime;
};

struct EffectLEDState {
  bool ledStates[EFFECT_LED_COUNT];
  EffectPattern currentPattern;
  bool patternActive;
  unsigned long patternTimer;
};

// =============================================================================
// 함수 선언
// =============================================================================

// 초기화 함수들
void initializePins();
void initializeGameState();
void initializeDFPlayer();

// PIR 센서 함수들
void readPIRSensors();
void startMotionDetection();
void stopMotionDetection();
void setMotionDetectionResult(bool detected);

// 천정 조명 함수들
void startCeilingLightSequence();
void updateCeilingLights();
bool isCeilingLightSequenceComplete();

// 경광등 함수들
void setBeaconLight(bool green, bool red);
void updateBeaconLights();

// 이펙트 LED 함수들
void playEffectPattern(EffectPattern pattern);
void stopEffectPattern();
void playErrorEffect();
void playSuccessEffect();

// 게임 상태 함수들
void updateGameState();
void transitionToPhase(GamePhase newPhase);
void resetGameToIdleState();

// 오디오 함수들
void playAudio(uint8_t trackNumber);
void stopAudio();
void playWelcomeNarration();
void playGameClearNarration();

// 통신 함수들
void sendCommandToSlave(uint8_t cmd, uint8_t data1, uint8_t data2);
void processSlaveComm();
void handleSlaveResponse(uint8_t response, uint8_t data1, uint8_t data2);

// 유틸리티 함수들
uint8_t getPatternLengthForLevel(uint8_t level);
void updateScoreLEDs();
void activateEMLock();
void prepareSystemShutdown();

// 시리얼 명령 함수들
void processSerialCommands();
void printGameStatus();

#endif // MASTER_CONFIG_OPTIMIZED_H