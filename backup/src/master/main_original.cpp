/*
 * 무궁화 게임 시스템 - Master Board (PlatformIO)
 * Arduino Mega 2560 기반
 * 
 * 주요 기능:
 * - 게임 상태 관리 및 메인 로직 처리
 * - PIR 센서를 통한 동작 감지
 * - 점수 LED, 경광등, 이펙트 LED 제어
 * - 천정 조명 제어
 * - 내레이션 및 BGM 재생
 * - Slave Board와의 시리얼 통신
 */

#include <Arduino.h>
#include "master_config.h"
#include "communication.h"
#include <SoftwareSerial.h>

// DFPlayer Mini 통신용 소프트웨어 시리얼
SoftwareSerial dfSerial(DFRX_PIN, DFTX_PIN);

// 게임 상태 변수
GameState gameState;
unsigned long lastUpdateTime = 0;
unsigned long effectTimer = 0;
bool effectActive = false;
uint8_t currentEffectStep = 0;

// PIR 센서 상태 관리 변수
struct PIRSensorState {
  bool currentState[PIR_SENSOR_COUNT];         // 현재 센서 상태 (A, B, C, D)
  bool previousState[PIR_SENSOR_COUNT];        // 이전 센서 상태 (A, B, C, D)
  unsigned long lastTriggerTime[PIR_SENSOR_COUNT];  // 마지막 트리거 시간 (A, B, C, D)
  bool motionDetected[PIR_ZONE_COUNT];         // 구역별 동작 감지 상태 (A, B, C, D)
};

PIRSensorState pirState;
unsigned long motionDetectStartTime = 0;
bool isMotionDetectionActive = false;

// PIR 센서 디바운싱 상수
#define PIR_DEBOUNCE_TIME     100   // 디바운싱 시간 (ms)
#define PIR_TRIGGER_DURATION  2000  // 트리거 지속 시간 (ms)

// 천정 조명 상태 관리 변수
struct CeilingLightState {
  bool lightState[4];           // 각 조명의 현재 상태
  bool sequenceActive;          // 순차 점등 진행 중 여부
  uint8_t currentSequenceStep;  // 현재 순차 점등 단계 (0-3)
  unsigned long sequenceTimer;  // 순차 점등 타이머
  unsigned long lastUpdateTime; // 마지막 업데이트 시간
};

CeilingLightState ceilingState;

// 경광등 상태 관리 변수
struct BeaconLightState {
  bool greenLightState;         // 초록색 경광등 상태
  bool redLightState;           // 빨간색 경광등 상태
  unsigned long autoOffTimer;   // 자동 소등 타이머
  bool autoOffActive;           // 자동 소등 활성 여부
  unsigned long lastStateChange; // 마지막 상태 변경 시간
};

BeaconLightState beaconState;

// 이펙트 LED 상태 관리 변수
struct EffectLEDState {
  bool ledState[6];             // 각 LED의 현재 상태
  uint8_t currentMask;          // 현재 LED 마스크 (비트별 제어)
  unsigned long lastUpdateTime; // 마지막 업데이트 시간
  bool patternActive;           // 패턴 재생 중 여부
  unsigned long patternTimer;   // 패턴 타이머
};

EffectLEDState effectLEDState;

// 이펙트 패턴 엔진 변수
struct EffectPatternEngine {
  EffectSequence currentSequence;    // 현재 재생 중인 시퀀스
  uint8_t currentStep;               // 현재 단계
  uint8_t currentRepeat;             // 현재 반복 횟수
  unsigned long stepTimer;           // 단계별 타이머
  bool engineActive;                 // 엔진 활성 상태
  EffectPattern activePattern;       // 현재 활성 패턴
};

EffectPatternEngine patternEngine;

// 통신 상태 관리 변수
CommState commState;
SerialMessage incomingMessage;
SerialMessage outgoingMessage;
unsigned long lastCommCheck = 0;

// 통신 안정성 강화 변수 (Task 17.1)
struct CommReliability {
  uint8_t retryCount;                    // 현재 재시도 횟수
  uint8_t maxRetries;                    // 최대 재시도 횟수
  unsigned long lastSuccessTime;         // 마지막 성공 통신 시간
  unsigned long connectionCheckInterval; // 연결 상태 확인 간격
  uint8_t consecutiveErrors;             // 연속 오류 횟수
  uint8_t errorThreshold;                // 오류 임계값
  bool isRecovering;                     // 복구 모드 여부
  unsigned long recoveryStartTime;       // 복구 시작 시간
  uint8_t lastErrorCode;                 // 마지막 오류 코드
  uint32_t totalCommAttempts;            // 총 통신 시도 횟수
  uint32_t successfulComms;              // 성공한 통신 횟수
  uint32_t failedComms;                  // 실패한 통신 횟수
  unsigned long lastHeartbeatTime;       // 마지막 하트비트 시간
  bool slaveResponsive;                  // Slave 응답 상태
  uint8_t communicationQuality;          // 통신 품질 (0-100%)
  unsigned long recoveryTimeout;         // 복구 타임아웃
  bool diagnosticMode;                   // 진단 모드 여부
};

CommReliability commReliability;

// 통신 안정성 상수 정의
#define COMM_RELIABILITY_MAX_RETRIES        5       // 최대 재시도 횟수
#define COMM_RELIABILITY_ERROR_THRESHOLD    3       // 연속 오류 임계값
#define COMM_RELIABILITY_HEARTBEAT_INTERVAL 5000    // 하트비트 간격 (ms)
#define COMM_RELIABILITY_RECOVERY_TIMEOUT   10000   // 복구 타임아웃 (ms)
#define COMM_RELIABILITY_CONNECTION_CHECK   2000    // 연결 확인 간격 (ms)
#define COMM_RELIABILITY_QUALITY_WINDOW     100     // 통신 품질 계산 윈도우

// 통신 진단 및 모니터링 변수
struct CommDiagnostics {
  unsigned long avgResponseTime;         // 평균 응답 시간
  unsigned long maxResponseTime;         // 최대 응답 시간
  unsigned long minResponseTime;         // 최소 응답 시간
  uint16_t timeoutCount;                 // 타임아웃 발생 횟수
  uint16_t checksumErrorCount;           // 체크섬 오류 횟수
  uint16_t invalidMessageCount;          // 잘못된 메시지 횟수
  uint16_t retransmissionCount;          // 재전송 횟수
  unsigned long lastDiagnosticTime;      // 마지막 진단 시간
  bool enableDetailedLogging;            // 상세 로깅 활성화
};

CommDiagnostics commDiagnostics;

// =============================================================================
// 하드웨어 안전장치 강화 시스템 (Task 17.2)
// =============================================================================

// 전원 안정성 모니터링 구조체
struct PowerMonitoring {
  float currentVoltage;                  // 현재 전압 (V)
  float minVoltage;                      // 최소 전압 (V)
  float maxVoltage;                      // 최대 전압 (V)
  bool lowVoltageWarning;                // 저전압 경고
  bool highVoltageWarning;               // 고전압 경고
  unsigned long lastVoltageCheck;        // 마지막 전압 체크 시간
  uint16_t voltageCheckInterval;         // 전압 체크 간격 (ms)
  uint16_t lowVoltageCount;              // 저전압 발생 횟수
  uint16_t highVoltageCount;             // 고전압 발생 횟수
  bool powerStable;                      // 전원 안정성 상태
  float voltageHistory[10];              // 전압 이력 (이동평균용)
  uint8_t historyIndex;                  // 이력 인덱스
  bool emergencyPowerShutdown;           // 비상 전원 차단 상태
};

PowerMonitoring powerMonitor;

// 센서 안전장치 구조체
struct SensorSafety {
  bool pirSensorStatus[PIR_SENSOR_COUNT]; // PIR 센서 상태 (정상/오작동)
  uint16_t pirErrorCount[PIR_SENSOR_COUNT]; // PIR 센서 오류 횟수
  unsigned long pirLastValidTime[PIR_SENSOR_COUNT]; // 마지막 정상 동작 시간
  bool pirSensorRecovering[PIR_SENSOR_COUNT]; // 센서 복구 중 여부
  unsigned long pirRecoveryStartTime[PIR_SENSOR_COUNT]; // 복구 시작 시간
  
  bool dfPlayerHealthy;                  // DFPlayer 상태
  uint16_t dfPlayerErrorCount;           // DFPlayer 오류 횟수
  unsigned long dfPlayerLastResponse;    // 마지막 DFPlayer 응답 시간
  bool dfPlayerRecovering;               // DFPlayer 복구 중
  
  bool systemOverheated;                 // 시스템 과열 상태
  float systemTemperature;               // 시스템 온도 (추정값)
  unsigned long lastTemperatureCheck;    // 마지막 온도 체크
  
  uint16_t totalSystemErrors;            // 총 시스템 오류 횟수
  bool emergencySafeMode;                // 비상 안전 모드
  unsigned long safeModeTriggerTime;     // 안전 모드 진입 시간
};

SensorSafety sensorSafety;

// 모터 안전장치 구조체 (Slave 모터 모니터링용)
struct MotorSafety {
  bool slaveMotorHealthy;                // Slave 모터 상태
  unsigned long lastMotorResponse;       // 마지막 모터 응답 시간
  uint16_t motorTimeoutCount;            // 모터 타임아웃 횟수
  uint16_t motorErrorCount;              // 모터 오류 횟수
  bool motorOverloadDetected;            // 모터 과부하 감지
  unsigned long motorOverloadTime;       // 과부하 감지 시간
  bool motorEmergencyStop;               // 모터 비상 정지 상태
  uint16_t motorRecoveryAttempts;        // 모터 복구 시도 횟수
  unsigned long lastMotorCommand;        // 마지막 모터 명령 시간
  bool motorSafetyLockout;               // 모터 안전 잠금 상태
};

MotorSafety motorSafety;

// 하드웨어 안전장치 상수 정의
#define POWER_VOLTAGE_MIN           4.5f    // 최소 동작 전압 (V)
#define POWER_VOLTAGE_MAX           5.5f    // 최대 동작 전압 (V)
#define POWER_VOLTAGE_CRITICAL_LOW  4.0f    // 임계 저전압 (V)
#define POWER_VOLTAGE_CRITICAL_HIGH 6.0f    // 임계 고전압 (V)
#define POWER_CHECK_INTERVAL        1000    // 전압 체크 간격 (ms)
#define POWER_HISTORY_SIZE          10      // 전압 이력 크기

#define SENSOR_ERROR_THRESHOLD      5       // 센서 오류 임계값
#define SENSOR_RECOVERY_TIMEOUT     30000   // 센서 복구 타임아웃 (ms)
#define SENSOR_HEALTH_CHECK_INTERVAL 2000   // 센서 상태 체크 간격 (ms)
#define PIR_VALID_RESPONSE_TIMEOUT  5000    // PIR 센서 유효 응답 타임아웃 (ms)

#define MOTOR_RESPONSE_TIMEOUT      5000    // 모터 응답 타임아웃 (ms)
#define MOTOR_ERROR_THRESHOLD       3       // 모터 오류 임계값
#define MOTOR_OVERLOAD_TIMEOUT      10000   // 모터 과부하 타임아웃 (ms)
#define MOTOR_RECOVERY_MAX_ATTEMPTS 3       // 최대 모터 복구 시도 횟수
#define MOTOR_SAFETY_LOCKOUT_TIME   60000   // 모터 안전 잠금 시간 (ms)

#define SYSTEM_TEMPERATURE_MAX      70.0f   // 최대 시스템 온도 (°C)
#define EMERGENCY_SAFE_MODE_TIMEOUT 300000  // 비상 안전 모드 타임아웃 (5분)

// DFPlayer 상태 관리 변수
struct DFPlayerState {
  bool isInitialized;           // 초기화 상태
  bool isPlaying;               // 재생 중 여부
  uint8_t currentTrack;         // 현재 재생 중인 트랙
  uint8_t currentVolume;        // 현재 볼륨 (0-30)
  unsigned long playStartTime;  // 재생 시작 시간
  unsigned long lastCommandTime; // 마지막 명령 전송 시간
  bool waitingForResponse;      // 응답 대기 중 여부
};

DFPlayerState dfPlayerState;

// =============================================================================
// 게임 로직 안정성 검증 시스템 (Task 17.3)
// =============================================================================

// 게임 상태 복구 시스템 구조체
struct GameStateRecovery {
  GameState backupState;                 // 백업된 게임 상태
  unsigned long lastBackupTime;          // 마지막 백업 시간
  uint8_t recoveryAttempts;              // 복구 시도 횟수
  uint8_t maxRecoveryAttempts;           // 최대 복구 시도 횟수
  bool recoveryInProgress;               // 복구 진행 중 여부
  unsigned long recoveryStartTime;       // 복구 시작 시간
  uint8_t lastValidPhase;                // 마지막 유효한 게임 단계
  bool stateCorrupted;                   // 상태 손상 여부
  uint8_t corruptionCount;               // 상태 손상 횟수
  unsigned long stateValidationTime;     // 상태 검증 시간
  bool autoRecoveryEnabled;              // 자동 복구 활성화
  uint8_t criticalErrorCount;            // 치명적 오류 횟수
  bool emergencyMode;                    // 비상 모드 여부
};

GameStateRecovery gameRecovery;

// 사용자 입력 오류 처리 시스템 구조체
struct UserInputErrorHandler {
  uint8_t invalidInputCount;             // 잘못된 입력 횟수
  uint8_t consecutiveErrors;             // 연속 오류 횟수
  unsigned long lastErrorTime;           // 마지막 오류 시간
  bool guidanceActive;                   // 가이드 시스템 활성
  uint8_t guidanceLevel;                 // 가이드 레벨 (0-3)
  unsigned long guidanceStartTime;       // 가이드 시작 시간
  uint8_t errorPattern[10];              // 오류 패턴 기록
  uint8_t errorPatternIndex;             // 오류 패턴 인덱스
  bool adaptiveGuidance;                 // 적응형 가이드 활성
  uint8_t userSkillLevel;                // 사용자 숙련도 (0-5)
  uint16_t totalInputAttempts;           // 총 입력 시도 횟수
  uint16_t successfulInputs;             // 성공한 입력 횟수
  float successRate;                     // 성공률
  bool errorRecoveryMode;                // 오류 복구 모드
  unsigned long timeoutExtension;        // 타임아웃 연장 시간
};

UserInputErrorHandler inputErrorHandler;

// 시스템 리셋 및 초기화 시스템 구조체
struct SystemResetManager {
  bool softResetRequested;               // 소프트 리셋 요청
  bool hardResetRequested;               // 하드 리셋 요청
  unsigned long resetRequestTime;        // 리셋 요청 시간
  uint8_t resetReason;                   // 리셋 사유
  uint8_t resetAttempts;                 // 리셋 시도 횟수
  bool resetInProgress;                  // 리셋 진행 중
  unsigned long resetStartTime;          // 리셋 시작 시간
  uint8_t resetPhase;                    // 리셋 단계
  bool emergencyReset;                   // 비상 리셋 여부
  uint32_t systemUptime;                 // 시스템 가동 시간
  uint32_t totalResets;                  // 총 리셋 횟수
  uint32_t softResets;                   // 소프트 리셋 횟수
  uint32_t hardResets;                   // 하드 리셋 횟수
  bool resetValidation;                  // 리셋 검증 여부
  unsigned long lastSuccessfulReset;     // 마지막 성공한 리셋 시간
  bool factoryResetMode;                 // 공장 초기화 모드
};

SystemResetManager resetManager;

// 게임 로직 안정성 상수 정의
#define GAME_STATE_BACKUP_INTERVAL      5000    // 게임 상태 백업 간격 (ms)
#define GAME_RECOVERY_MAX_ATTEMPTS      3       // 최대 게임 복구 시도
#define GAME_RECOVERY_TIMEOUT           10000   // 게임 복구 타임아웃 (ms)
#define GAME_STATE_VALIDATION_INTERVAL  2000    // 상태 검증 간격 (ms)
#define GAME_CORRUPTION_THRESHOLD       5       // 상태 손상 임계값

#define INPUT_ERROR_THRESHOLD           3       // 입력 오류 임계값
#define INPUT_CONSECUTIVE_ERROR_LIMIT   5       // 연속 오류 한계
#define INPUT_GUIDANCE_TIMEOUT          30000   // 가이드 타임아웃 (ms)
#define INPUT_TIMEOUT_EXTENSION_MAX     10000   // 최대 타임아웃 연장 (ms)
#define INPUT_SUCCESS_RATE_THRESHOLD    0.3f    // 성공률 임계값

#define RESET_REQUEST_TIMEOUT           5000    // 리셋 요청 타임아웃 (ms)
#define RESET_VALIDATION_TIMEOUT        3000    // 리셋 검증 타임아웃 (ms)
#define RESET_MAX_ATTEMPTS              5       // 최대 리셋 시도 횟수

// 리셋 사유 코드
#define RESET_REASON_USER_REQUEST       1       // 사용자 요청
#define RESET_REASON_SYSTEM_ERROR       2       // 시스템 오류
#define RESET_REASON_COMMUNICATION      3       // 통신 오류
#define RESET_REASON_HARDWARE_FAILURE   4       // 하드웨어 오류
#define RESET_REASON_GAME_CORRUPTION    5       // 게임 상태 손상
#define RESET_REASON_EMERGENCY          6       // 비상 상황
#define RESET_REASON_FACTORY            7       // 공장 초기화

// 가이드 레벨 정의
#define GUIDANCE_LEVEL_NONE             0       // 가이드 없음
#define GUIDANCE_LEVEL_BASIC            1       // 기본 가이드
#define GUIDANCE_LEVEL_DETAILED         2       // 상세 가이드
#define GUIDANCE_LEVEL_ASSISTED         3       // 보조 가이드

// =============================================================================
// Arduino 필수 함수들
// =============================================================================

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(SERIAL_BAUD_RATE);
  dfSerial.begin(9600);
  
  Serial.println("=== 무궁화 게임 시스템 시작 (PlatformIO) ===");
  
  // 핀 모드 설정
  initializePins();
  
  // 게임 상태 초기화
  initializeGameState();
  
  // DFPlayer 초기화
  initializeDFPlayer();
  
  // 통신 시스템 초기화
  initComm();
  
  // 통신 안정성 시스템 초기화 (Task 17.1)
  initCommReliability();
  
  // 통신 진단 시스템 초기화
  initCommDiagnostics();
  
  // 하드웨어 안전장치 시스템 초기화 (Task 17.2)
  initHardwareSafety();
  
  // 게임 로직 안정성 검증 시스템 초기화 (Task 17.3)
  initGameLogicStability();
  
  // 시스템 초기화 시 조명 설정 (모든 조명 소등 상태로 시작)
  initializeSystemLights();
  
  Serial.println("Master Board 초기화 완료 (PlatformIO)");
  Serial.println("================================");
}

void loop() {
  // PIR 센서 상태 읽기 및 처리
  readPIRSensors();
  
  // 동작 감지 처리
  processMotionDetection();
  
  // 천정 조명 순차 점등 처리
  updateCeilingLightSequence();
  
  // 경광등 자동 제어 처리
  updateBeaconLights();
  
  // 이펙트 LED 업데이트
  updateEffectLEDs();
  
  // DFPlayer 상태 모니터링 및 오디오 처리
  updateDFPlayerStatus();
  
  // 게임 상태 업데이트
  updateGameState();
  
  // 경광등 상태 동기화 (게임 상태와 일치 확인)
  synchronizeBeaconWithGameState();
  
  // Slave와의 통신 처리
  processSlaveComm();
  
  // 통신 안정성 모니터링 (Task 17.1)
  monitorCommReliability();
  
  // 통신 진단 업데이트
  updateCommDiagnostics();
  
  // 하드웨어 안전장치 모니터링 (Task 17.2)
  monitorHardwareSafety();
  
  // 게임 로직 안정성 모니터링 (Task 17.3)
  monitorGameLogicStability();
  
  // 시리얼 명령어 처리 (디버깅용)
  processSerialCommands();
  
  delay(10); // 메인 루프 딜레이
}

// =============================================================================
// 기본 초기화 함수들
// =============================================================================

void initializePins() {
  // 천정 조명 핀 설정 (OUT0~3)
  for (int i = 0; i < 4; i++) {
    pinMode(CEILING_LIGHT_PINS[i], OUTPUT);
    digitalWrite(CEILING_LIGHT_PINS[i], LOW);
  }
  
  // EM-Lock 핀 설정 (OUT4~7)
  for (int i = 0; i < 4; i++) {
    pinMode(EMLOCK_PINS[i], OUTPUT);
    digitalWrite(EMLOCK_PINS[i], LOW);
  }
  
  // 릴레이 신호 핀 설정 (경광등 + 이펙트 LED)
  for (int i = 0; i < 8; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
  }
  
  // PIR 센서 핀 설정
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    pinMode(PIR_PINS[i], INPUT);
  }
}

void initializeGameState() {
  gameState.currentLevel = 1;
  gameState.currentScore = 0;
  gameState.gamePhase = PHASE_IDLE;
  gameState.isGameActive = false;
  gameState.isRobotFacing = false;
  gameState.phaseTimer = 0;
  
  // PIR 센서 상태 초기화
  initializePIRSensors();
  
  // 천정 조명 상태 초기화
  initializeCeilingLights();
  
  // 경광등 상태 초기화
  initializeBeaconLights();
  
  // 이펙트 LED 상태 초기화
  initializeEffectLEDs();
}

void initializeSystemLights() {
  Serial.println("시스템 조명 초기화 - 모든 조명 소등");
  
  // 천정 조명 모두 소등
  for (int i = 0; i < 4; i++) {
    digitalWrite(CEILING_LIGHT_PINS[i], LOW);
    ceilingState.lightState[i] = false;
  }
  
  // 경광등 모두 소등
  digitalWrite(GREEN_LIGHT_RELAY_PIN, LOW);
  digitalWrite(RED_LIGHT_RELAY_PIN, LOW);
  beaconState.greenLightState = false;
  beaconState.redLightState = false;
  
  // 이펙트 LED 모두 소등
  for (int i = 0; i < 6; i++) {
    digitalWrite(EFFECT_LED_PINS[i], LOW);
    effectLEDState.ledState[i] = false;
  }
  effectLEDState.currentMask = 0;
  
  Serial.println("시스템 조명 초기화 완료");
}

// =============================================================================
// PIR 센서 관련 함수들 (기본 구현)
// =============================================================================

void initializePIRSensors() {
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    pirState.currentState[i] = false;
    pirState.previousState[i] = false;
    pirState.lastTriggerTime[i] = 0;
    pirState.motionDetected[i] = false;
  }
  
  motionDetectStartTime = 0;
  isMotionDetectionActive = false;
  
  Serial.print("PIR 센서 시스템 초기화 완료 (");
  Serial.print(PIR_SENSOR_COUNT);
  Serial.println("개 센서)");
}

/**
 * PIR 센서 구역 이름 반환
 * @param sensorIndex 센서 인덱스 (0-3)
 * @return 구역 이름 문자열
 */
const char* getPIRZoneName(uint8_t sensorIndex) {
  switch (sensorIndex) {
    case 0: return "A구역";
    case 1: return "B구역";
    case 2: return "C구역";
    case 3: return "D구역";
    default: return "알수없음";
  }
}

/**
 * 구역별 동작 감지 통합 처리
 * 요구사항 3.2: 구역별(A,B,C,D) 동작 감지 통합 처리
 */
void updateZoneMotionDetection() {
  unsigned long currentTime = millis();
  
  // 각 구역별로 동작 감지 상태 업데이트 (각 센서가 하나의 구역)
  for (int zone = 0; zone < PIR_ZONE_COUNT; zone++) {
    bool zoneMotionDetected = false;
    
    // 해당 구역의 센서 확인 (각 구역당 1개 센서)
    if (pirState.currentState[zone] && 
        (currentTime - pirState.lastTriggerTime[zone] < PIR_TRIGGER_DURATION)) {
      zoneMotionDetected = true;
    }
    
    // 구역 상태 업데이트
    if (zoneMotionDetected != pirState.motionDetected[zone]) {
      pirState.motionDetected[zone] = zoneMotionDetected;
      
      if (zoneMotionDetected) {
        Serial.print("구역 ");
        Serial.print((char)('A' + zone));
        Serial.println(" 동작 감지 활성화");
      } else {
        Serial.print("구역 ");
        Serial.print((char)('A' + zone));
        Serial.println(" 동작 감지 비활성화");
      }
    }
  }
}

/**
 * 동작 감지 시작
 * 요구사항 3.2: PIR 센서가 5초간 동작감지를 시작
 */
void startMotionDetection() {
  Serial.println("=== 동작 감지 시작 ===");
  
  isMotionDetectionActive = true;
  motionDetectStartTime = millis();
  motionDetected = false; // 결과 초기화
  
  // 모든 구역 동작 감지 상태 초기화
  for (int i = 0; i < PIR_ZONE_COUNT; i++) {
    pirState.motionDetected[i] = false;
  }
  
  Serial.print("동작 감지 시간: ");
  Serial.print(MOTION_DETECT_DURATION / 1000);
  Serial.println("초");
  Serial.println("움직이지 마세요!");
}

/**
 * 동작 감지 중지
 */
void stopMotionDetection() {
  if (!isMotionDetectionActive) {
    return;
  }
  
  Serial.println("=== 동작 감지 종료 ===");
  
  isMotionDetectionActive = false;
  
  // 최종 결과 출력
  Serial.print("동작 감지 결과: ");
  Serial.println(motionDetected ? "감지됨" : "감지안됨");
  
  // 구역별 최종 상태 출력
  for (int i = 0; i < PIR_ZONE_COUNT; i++) {
    Serial.print("구역 ");
    Serial.print((char)('A' + i));
    Serial.print(": ");
    Serial.println(pirState.motionDetected[i] ? "감지됨" : "안전");
  }
}

/**
 * PIR 센서 상태 출력 (디버깅용)
 */
void printPIRSensorStatus() {
  Serial.print("=== PIR 센서 상태 (");
  Serial.print(PIR_SENSOR_COUNT);
  Serial.println("개) ===");
  
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    Serial.print("센서 ");
    Serial.print(getPIRZoneName(i));
    Serial.print(" (핀 ");
    Serial.print(PIR_PINS[i]);
    Serial.print("): ");
    Serial.print(pirState.currentState[i] ? "활성" : "비활성");
    
    if (pirState.currentState[i]) {
      Serial.print(" (");
      Serial.print(millis() - pirState.lastTriggerTime[i]);
      Serial.print("ms 전)");
    }
    Serial.println();
  }
  
  Serial.println("=== 구역별 동작 감지 ===");
  for (int i = 0; i < PIR_ZONE_COUNT; i++) {
    Serial.print("구역 ");
    Serial.print((char)('A' + i));
    Serial.print(": ");
    Serial.println(pirState.motionDetected[i] ? "감지됨" : "안전");
  }
  
  Serial.print("동작 감지 활성: ");
  Serial.println(isMotionDetectionActive ? "예" : "아니오");
  
  if (isMotionDetectionActive) {
    unsigned long elapsed = millis() - motionDetectStartTime;
    Serial.print("경과 시간: ");
    Serial.print(elapsed);
    Serial.print("ms / ");
    Serial.print(MOTION_DETECT_DURATION);
    Serial.println("ms");
  }
  
  Serial.println("==========================");
}

/**
 * 총소리 효과음 재생
 * 요구사항 3.3: 움직임이 감지되면 총소리 효과음이 재생되어야 한다
 */
void playGunShotEffect() {
  Serial.println("총소리 효과음 재생");
  // Master Board의 DFPlayer로 총소리 효과음 재생
  // 실제 구현에서는 적절한 트랙 번호 사용
  playAudio(20); // 총소리 트랙 번호 (예시)
}

/**
 * 오디오 재생 함수 (간단한 래퍼)
 * @param trackNumber 재생할 트랙 번호
 */
void playAudio(uint8_t trackNumber) {
  playAudioTrack(trackNumber);
}

void readPIRSensors() {
  unsigned long currentTime = millis();
  
  // PIR 센서 상태 읽기 및 디바운싱 처리
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    bool currentReading = digitalRead(PIR_PINS[i]);
    
    // 상태 변화 감지
    if (currentReading != pirState.previousState[i]) {
      // 디바운싱: 마지막 변화로부터 일정 시간이 지났는지 확인
      if (currentTime - pirState.lastTriggerTime[i] > PIR_DEBOUNCE_TIME) {
        pirState.previousState[i] = pirState.currentState[i];
        pirState.currentState[i] = currentReading;
        pirState.lastTriggerTime[i] = currentTime;
        
        // 동작 감지 시 로그 출력
        if (currentReading) {
          Serial.print("PIR 센서 ");
          Serial.print(getPIRZoneName(i));
          Serial.println(" 동작 감지!");
        }
      }
    }
  }
  
  // 구역별 동작 감지 통합 처리
  updateZoneMotionDetection();
}

void processMotionDetection() {
  // 동작 감지가 활성화된 상태에서만 처리
  if (!isMotionDetectionActive) {
    return;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - motionDetectStartTime;
  
  // 동작 감지 시간 확인 (5초)
  if (elapsedTime >= MOTION_DETECT_DURATION) {
    Serial.println("동작 감지 시간 완료");
    stopMotionDetection();
    return;
  }
  
  // 실시간 동작 감지 확인
  bool motionDetectedNow = false;
  
  // 구역별 동작 감지 확인
  for (int zone = 0; zone < PIR_ZONE_COUNT; zone++) {
    if (pirState.motionDetected[zone]) {
      motionDetectedNow = true;
      Serial.print("구역 ");
      Serial.print((char)('A' + zone));
      Serial.println("에서 동작 감지됨!");
      break;
    }
  }
  
  // 동작이 감지된 경우 처벌 시스템 실행
  if (motionDetectedNow && !motionDetected) {
    Serial.println("=== 동작 감지 처벌 시스템 실행 ===");
    
    // 요구사항 3.3: 총소리 효과음 재생
    playGunShotEffect();
    
    // 요구사항 3.4: 이펙트 LED 0.2초 간격으로 5회 점멸
    playErrorEffect();
    
    // 요구사항 3.5: 점수 1점 감점 (점수 업데이트 단계에서 처리)
    setMotionDetectionResult(true);
    
    Serial.println("동작 감지 처벌 완료");
  }
}

// =============================================================================
// 천정 조명 관련 함수들 (기본 구현)
// =============================================================================

void initializeCeilingLights() {
  ceilingState.sequenceActive = false;
  ceilingState.currentSequenceStep = 0;
  ceilingState.sequenceTimer = 0;
  ceilingState.lastUpdateTime = 0;
  
  for (int i = 0; i < 4; i++) {
    ceilingState.lightState[i] = false;
  }
  
  Serial.println("천정 조명 시스템 초기화 완료");
}

/**
 * 천정 조명 순차 점등 시작
 * 요구사항 1.2: 천정 조명 4개를 1번부터 4번까지 0.5초 간격으로 순차 점등
 */
void startCeilingLightSequence() {
  Serial.println("=== 천정 조명 순차 점등 시작 ===");
  
  // 모든 조명 소등 상태에서 시작
  for (int i = 0; i < 4; i++) {
    digitalWrite(CEILING_LIGHT_PINS[i], LOW);
    ceilingState.lightState[i] = false;
  }
  
  // 순차 점등 상태 초기화
  ceilingState.sequenceActive = true;
  ceilingState.currentSequenceStep = 0;
  ceilingState.sequenceTimer = millis();
  ceilingState.lastUpdateTime = millis();
  
  Serial.print("점등 간격: ");
  Serial.print(LIGHT_SEQUENCE_INTERVAL);
  Serial.println("ms");
}

/**
 * 천정 조명 순차 점등 완료 확인
 * @return 순차 점등 완료 여부
 */
bool isCeilingLightSequenceComplete() {
  return !ceilingState.sequenceActive && ceilingState.currentSequenceStep >= 4;
}

/**
 * 천정 조명 개별 제어
 * @param lightIndex 조명 인덱스 (0-3)
 * @param state 점등 상태 (true: 점등, false: 소등)
 */
void controlCeilingLight(uint8_t lightIndex, bool state) {
  if (lightIndex >= 4) {
    Serial.print("잘못된 조명 인덱스: ");
    Serial.println(lightIndex);
    return;
  }
  
  digitalWrite(CEILING_LIGHT_PINS[lightIndex], state ? HIGH : LOW);
  ceilingState.lightState[lightIndex] = state;
  
  Serial.print("천정 조명 ");
  Serial.print(lightIndex + 1);
  Serial.print(": ");
  Serial.println(state ? "점등" : "소등");
}

/**
 * 모든 천정 조명 제어
 * @param state 점등 상태 (true: 점등, false: 소등)
 */
void controlAllCeilingLights(bool state) {
  Serial.print("모든 천정 조명 ");
  Serial.println(state ? "점등" : "소등");
  
  for (int i = 0; i < 4; i++) {
    controlCeilingLight(i, state);
  }
}

/**
 * 천정 조명 패턴 제어
 * @param pattern 점등 패턴 (비트마스크: bit0=조명1, bit1=조명2, ...)
 */
void controlCeilingLights(uint8_t pattern) {
  Serial.print("천정 조명 패턴 제어: 0b");
  Serial.println(pattern, BIN);
  
  for (int i = 0; i < 4; i++) {
    bool lightState = (pattern >> i) & 0x01;
    controlCeilingLight(i, lightState);
  }
}

/**
 * 천정 조명 효과음 재생
 * 요구사항 1.3: 각 조명이 점등될 때 해당하는 효과음을 재생
 * @param lightNumber 조명 번호 (1-4)
 */
void playCeilingLightEffect(uint8_t lightNumber) {
  if (lightNumber < 1 || lightNumber > 4) {
    Serial.print("잘못된 조명 번호: ");
    Serial.println(lightNumber);
    return;
  }
  
  Serial.print("천정 조명 ");
  Serial.print(lightNumber);
  Serial.println("번 효과음 재생");
  
  // 조명별 효과음 트랙 재생
  uint8_t audioTrack = AUDIO_LIGHT_1 + (lightNumber - 1);
  playAudio(audioTrack);
}

/**
 * 천정 조명 상태 출력 (디버깅용)
 */
void printCeilingLightStatus() {
  Serial.println("=== 천정 조명 상태 ===");
  
  for (int i = 0; i < 4; i++) {
    Serial.print("조명 ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(ceilingState.lightState[i] ? "점등" : "소등");
  }
  
  Serial.print("순차 점등 활성: ");
  Serial.println(ceilingState.sequenceActive ? "예" : "아니오");
  
  if (ceilingState.sequenceActive) {
    Serial.print("현재 단계: ");
    Serial.print(ceilingState.currentSequenceStep);
    Serial.println("/4");
    
    unsigned long elapsed = millis() - ceilingState.sequenceTimer;
    Serial.print("다음 점등까지: ");
    Serial.print(LIGHT_SEQUENCE_INTERVAL - elapsed);
    Serial.println("ms");
  }
  
  Serial.println("====================");
}

void updateCeilingLightSequence() {
  // 순차 점등이 활성화되지 않은 경우 처리하지 않음
  if (!ceilingState.sequenceActive) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // 순차 점등 타이머 확인
  if (currentTime - ceilingState.sequenceTimer >= LIGHT_SEQUENCE_INTERVAL) {
    // 현재 단계의 조명 점등 및 효과음 재생
    if (ceilingState.currentSequenceStep < 4) {
      Serial.print("천정 조명 ");
      Serial.print(ceilingState.currentSequenceStep + 1);
      Serial.println("번 점등");
      
      // 조명 점등
      digitalWrite(CEILING_LIGHT_PINS[ceilingState.currentSequenceStep], HIGH);
      ceilingState.lightState[ceilingState.currentSequenceStep] = true;
      
      // 요구사항 1.3: 각 조명별 효과음 동기화 재생
      playCeilingLightEffect(ceilingState.currentSequenceStep + 1);
      
      // 다음 단계로 진행
      ceilingState.currentSequenceStep++;
      ceilingState.sequenceTimer = currentTime;
      
      // 모든 조명이 점등되었는지 확인
      if (ceilingState.currentSequenceStep >= 4) {
        Serial.println("천정 조명 순차 점등 완료");
        ceilingState.sequenceActive = false;
        
        // 요구사항 1.4: 점등 완료 후 로봇 회전 신호 전송
        // 게임 상태 머신에서 처리됨 (handleGameStartPhase)
      }
    }
  }
}

// =============================================================================
// 경광등 관련 함수들 (기본 구현)
// =============================================================================

void initializeBeaconLights() {
  beaconState.greenLightState = false;
  beaconState.redLightState = false;
  beaconState.autoOffTimer = 0;
  beaconState.autoOffActive = false;
  beaconState.lastStateChange = 0;
  
  Serial.println("경광등 시스템 초기화 완료");
}

/**
 * 경광등 제어 함수
 * @param greenState 초록색 경광등 상태 (true: 점등, false: 소등)
 * @param redState 빨간색 경광등 상태 (true: 점등, false: 소등)
 */
void setBeaconLight(bool greenState, bool redState) {
  unsigned long currentTime = millis();
  
  // 상태 변화가 있는 경우에만 처리
  if (beaconState.greenLightState != greenState || 
      beaconState.redLightState != redState) {
    
    Serial.print("경광등 상태 변경: 초록=");
    Serial.print(greenState ? "ON" : "OFF");
    Serial.print(", 빨강=");
    Serial.println(redState ? "ON" : "OFF");
    
    // 하드웨어 제어
    digitalWrite(GREEN_LIGHT_RELAY_PIN, greenState ? HIGH : LOW);
    digitalWrite(RED_LIGHT_RELAY_PIN, redState ? HIGH : LOW);
    
    // 상태 업데이트
    beaconState.greenLightState = greenState;
    beaconState.redLightState = redState;
    beaconState.lastStateChange = currentTime;
    
    // 자동 소등 타이머 비활성화 (수동 제어 시)
    beaconState.autoOffActive = false;
  }
}

/**
 * 초록색 경광등 제어
 * 요구사항 2.1: 로봇 후면 회전 시 초록 경광등 자동 점등
 * @param state 점등 상태
 */
void setGreenBeacon(bool state) {
  setBeaconLight(state, beaconState.redLightState);
}

/**
 * 빨간색 경광등 제어
 * 요구사항 3.1: 로봇 정면 회전 시 빨간 경광등 자동 점등
 * @param state 점등 상태
 */
void setRedBeacon(bool state) {
  setBeaconLight(beaconState.greenLightState, state);
}

/**
 * 모든 경광등 소등
 * 요구사항 3.1: 동작 감지 완료 후 경광등 자동 소등
 */
void turnOffAllBeacons() {
  setBeaconLight(false, false);
}

/**
 * 경광등 자동 소등 타이머 설정
 * @param delayMs 소등까지의 지연 시간 (ms)
 */
void setBeaconAutoOff(unsigned long delayMs) {
  beaconState.autoOffTimer = millis();
  beaconState.autoOffActive = true;
  
  Serial.print("경광등 자동 소등 타이머 설정: ");
  Serial.print(delayMs);
  Serial.println("ms");
}

/**
 * 경광등 점멸 효과
 * @param greenBlink 초록색 경광등 점멸 여부
 * @param redBlink 빨간색 경광등 점멸 여부
 * @param blinkCount 점멸 횟수
 * @param blinkInterval 점멸 간격 (ms)
 */
void blinkBeaconLights(bool greenBlink, bool redBlink, uint8_t blinkCount, uint16_t blinkInterval) {
  Serial.print("경광등 점멸 시작: 초록=");
  Serial.print(greenBlink ? "ON" : "OFF");
  Serial.print(", 빨강=");
  Serial.print(redBlink ? "ON" : "OFF");
  Serial.print(", 횟수=");
  Serial.print(blinkCount);
  Serial.print(", 간격=");
  Serial.print(blinkInterval);
  Serial.println("ms");
  
  for (uint8_t i = 0; i < blinkCount; i++) {
    // 점등
    if (greenBlink) setGreenBeacon(true);
    if (redBlink) setRedBeacon(true);
    delay(blinkInterval);
    
    // 소등
    if (greenBlink) setGreenBeacon(false);
    if (redBlink) setRedBeacon(false);
    delay(blinkInterval);
  }
  
  Serial.println("경광등 점멸 완료");
}

/**
 * 경광등 상태 확인
 * @return 경광등 점등 상태 (bit0: 초록, bit1: 빨강)
 */
uint8_t getBeaconStatus() {
  uint8_t status = 0;
  if (beaconState.greenLightState) status |= 0x01;
  if (beaconState.redLightState) status |= 0x02;
  return status;
}

/**
 * 경광등 상태 출력 (디버깅용)
 */
void printBeaconStatus() {
  Serial.println("=== 경광등 상태 ===");
  Serial.print("초록색 경광등: ");
  Serial.println(beaconState.greenLightState ? "점등" : "소등");
  Serial.print("빨간색 경광등: ");
  Serial.println(beaconState.redLightState ? "점등" : "소등");
  
  if (beaconState.autoOffActive) {
    unsigned long remaining = 1000 - (millis() - beaconState.autoOffTimer);
    Serial.print("자동 소등까지: ");
    Serial.print(remaining);
    Serial.println("ms");
  }
  
  Serial.print("마지막 상태 변경: ");
  Serial.print(millis() - beaconState.lastStateChange);
  Serial.println("ms 전");
  
  Serial.println("==================");
}

/**
 * 게임 상태별 경광등 제어 (외부에서 호출)
 * @param phase 게임 단계
 */
void updateBeaconForGamePhase(GamePhase phase) {
  switch (phase) {
    case PHASE_ROBOT_TURN_BACK:
      Serial.println("로봇 후면 회전 - 초록 경광등 점등");
      setBeaconLight(true, false);
      break;
      
    case PHASE_MOTION_DETECT:
      Serial.println("동작 감지 단계 - 빨간 경광등 점등");
      setBeaconLight(false, true);
      break;
      
    case PHASE_SCORE_UPDATE:
      Serial.println("점수 업데이트 - 모든 경광등 소등");
      setBeaconLight(false, false);
      break;
      
    case PHASE_IDLE:
    case PHASE_GAME_CLEAR:
      Serial.println("대기/클리어 상태 - 모든 경광등 소등");
      setBeaconLight(false, false);
      break;
      
    default:
      // 기타 상태에서는 변경하지 않음
      break;
  }
}

void updateBeaconLights() {
  unsigned long currentTime = millis();
  
  // 자동 소등 타이머 처리
  if (beaconState.autoOffActive && 
      currentTime - beaconState.autoOffTimer >= 1000) { // 1초 후 자동 소등
    Serial.println("경광등 자동 소등");
    setBeaconLight(false, false);
    beaconState.autoOffActive = false;
  }
  
  // 경광등 상태 변화 감지 및 로깅
  static bool lastGreenState = false;
  static bool lastRedState = false;
  
  if (beaconState.greenLightState != lastGreenState) {
    Serial.print("초록색 경광등: ");
    Serial.println(beaconState.greenLightState ? "점등" : "소등");
    lastGreenState = beaconState.greenLightState;
  }
  
  if (beaconState.redLightState != lastRedState) {
    Serial.print("빨간색 경광등: ");
    Serial.println(beaconState.redLightState ? "점등" : "소등");
    lastRedState = beaconState.redLightState;
  }
}

void synchronizeBeaconWithGameState() {
  // 게임 상태에 따른 경광등 자동 제어
  GamePhase currentPhase = (GamePhase)gameState.gamePhase;
  
  switch (currentPhase) {
    case PHASE_ROBOT_TURN_BACK:
    case PHASE_SHOW_PATTERN:
    case PHASE_PLAYER_INPUT:
      // 요구사항 2.1: 로봇 후면 회전 시 초록 경광등 자동 점등
      if (!beaconState.greenLightState) {
        Serial.println("게임 상태 동기화: 초록 경광등 점등");
        setBeaconLight(true, false);
      }
      break;
      
    case PHASE_MOTION_DETECT:
      // 요구사항 3.1: 로봇 정면 회전 시 빨간 경광등 자동 점등
      if (!beaconState.redLightState) {
        Serial.println("게임 상태 동기화: 빨간 경광등 점등");
        setBeaconLight(false, true);
      }
      break;
      
    case PHASE_SCORE_UPDATE:
      // 요구사항 3.1: 동작 감지 완료 후 경광등 자동 소등
      if (beaconState.greenLightState || beaconState.redLightState) {
        Serial.println("게임 상태 동기화: 모든 경광등 소등");
        setBeaconLight(false, false);
      }
      break;
      
    case PHASE_IDLE:
    case PHASE_GAME_CLEAR:
      // 대기 상태 및 클리어 상태에서는 모든 경광등 소등
      if (beaconState.greenLightState || beaconState.redLightState) {
        Serial.println("게임 상태 동기화: 대기/클리어 상태 - 모든 경광등 소등");
        setBeaconLight(false, false);
      }
      break;
      
    default:
      // 기타 상태에서는 현재 상태 유지
      break;
  }
}

// =============================================================================
// 게임 상태 관련 함수들 (기본 구현)
// =============================================================================

// =============================================================================
// 게임 상태 머신 구현 (Task 11.1)
// =============================================================================

/**
 * 게임 상태 머신 메인 업데이트 함수
 * 요구사항 1.1, 6.1: 게임 단계별 상태 전환 로직 구현
 */
void updateGameState() {
  unsigned long currentTime = millis();
  
  // 게임 단계별 처리
  switch (gameState.gamePhase) {
    case PHASE_IDLE:
      handleIdlePhase();
      break;
      
    case PHASE_GAME_START:
      handleGameStartPhase();
      break;
      
    case PHASE_ROBOT_TURN_FRONT:
      handleRobotTurnFrontPhase();
      break;
      
    case PHASE_NARRATION:
      handleNarrationPhase();
      break;
      
    case PHASE_ROBOT_TURN_BACK:
      handleRobotTurnBackPhase();
      break;
      
    case PHASE_SHOW_PATTERN:
      handleShowPatternPhase();
      break;
      
    case PHASE_PLAYER_INPUT:
      handlePlayerInputPhase();
      break;
      
    case PHASE_MOTION_DETECT:
      handleMotionDetectPhase();
      break;
      
    case PHASE_SCORE_UPDATE:
      handleScoreUpdatePhase();
      break;
      
    case PHASE_GAME_CLEAR:
      handleGameClearPhase();
      break;
      
    default:
      Serial.print("알 수 없는 게임 단계: ");
      Serial.println(gameState.gamePhase);
      transitionToPhase(PHASE_IDLE);
      break;
  }
}

// =============================================================================
// 통신 안정성 강화 시스템 (Task 17.1)
// =============================================================================

/**
 * 통신 안정성 시스템 초기화
 * 요구사항 7.3: Master-Slave 간 통신 오류 복구 메커니즘 완성
 */
void initCommReliability() {
  commReliability.retryCount = 0;
  commReliability.maxRetries = COMM_RELIABILITY_MAX_RETRIES;
  commReliability.lastSuccessTime = millis();
  commReliability.connectionCheckInterval = COMM_RELIABILITY_CONNECTION_CHECK;
  commReliability.consecutiveErrors = 0;
  commReliability.errorThreshold = COMM_RELIABILITY_ERROR_THRESHOLD;
  commReliability.isRecovering = false;
  commReliability.recoveryStartTime = 0;
  commReliability.lastErrorCode = COMM_ERROR_NONE;
  commReliability.totalCommAttempts = 0;
  commReliability.successfulComms = 0;
  commReliability.failedComms = 0;
  commReliability.lastHeartbeatTime = millis();
  commReliability.slaveResponsive = false;
  commReliability.communicationQuality = 100;
  commReliability.recoveryTimeout = COMM_RELIABILITY_RECOVERY_TIMEOUT;
  commReliability.diagnosticMode = false;
  
  Serial.println("통신 안정성 시스템 초기화 완료");
  Serial.print("최대 재시도: ");
  Serial.println(commReliability.maxRetries);
  Serial.print("오류 임계값: ");
  Serial.println(commReliability.errorThreshold);
  Serial.print("하트비트 간격: ");
  Serial.print(COMM_RELIABILITY_HEARTBEAT_INTERVAL);
  Serial.println("ms");
}

/**
 * 통신 안정성 모니터링 (메인 루프에서 호출)
 * 요구사항 7.3: 통신 상태 모니터링 및 진단 기능 추가
 */
void monitorCommReliability() {
  unsigned long currentTime = millis();
  
  // 하트비트 체크
  if (currentTime - commReliability.lastHeartbeatTime >= COMM_RELIABILITY_HEARTBEAT_INTERVAL) {
    performHeartbeatCheck();
    commReliability.lastHeartbeatTime = currentTime;
  }
  
  // 연결 상태 체크
  if (currentTime - commState.lastCommTime >= commReliability.connectionCheckInterval) {
    checkConnectionHealth();
  }
  
  // 복구 모드 처리
  if (commReliability.isRecovering) {
    handleCommRecovery();
  }
  
  // 통신 품질 업데이트
  updateCommunicationQuality();
  
  // 진단 모드 처리
  if (commReliability.diagnosticMode) {
    performCommDiagnostics();
  }
}

/**
 * 하트비트 체크 수행
 * Slave의 응답성을 확인하여 연결 상태 모니터링
 */
void performHeartbeatCheck() {
  if (commReliability.diagnosticMode) {
    Serial.println("[COMM] 하트비트 체크 수행");
  }
  
  // Slave에게 상태 요청 전송
  bool success = sendCommandToSlaveWithRetry(CMD_GET_STATUS, 0, 0, 1); // 재시도 1회만
  
  if (success) {
    commReliability.slaveResponsive = true;
    commReliability.lastSuccessTime = millis();
    
    if (commReliability.consecutiveErrors > 0) {
      Serial.println("[COMM] Slave 응답 복구됨");
      commReliability.consecutiveErrors = 0;
      
      if (commReliability.isRecovering) {
        Serial.println("[COMM] 통신 복구 완료");
        commReliability.isRecovering = false;
      }
    }
  } else {
    commReliability.slaveResponsive = false;
    recordCommError(COMM_ERROR_NO_RESPONSE);
    
    if (commReliability.diagnosticMode) {
      Serial.println("[COMM] 하트비트 실패 - Slave 무응답");
    }
  }
}

/**
 * 연결 상태 건강성 체크
 */
void checkConnectionHealth() {
  unsigned long timeSinceLastSuccess = millis() - commReliability.lastSuccessTime;
  
  // 마지막 성공 통신으로부터 너무 오랜 시간이 지났는지 확인
  if (timeSinceLastSuccess > COMM_RELIABILITY_RECOVERY_TIMEOUT) {
    if (!commReliability.isRecovering) {
      Serial.println("[COMM] 연결 건강성 악화 - 복구 모드 시작");
      startCommRecovery(COMM_ERROR_TIMEOUT);
    }
  }
  
  // 연속 오류 임계값 체크
  if (commReliability.consecutiveErrors >= commReliability.errorThreshold) {
    if (!commReliability.isRecovering) {
      Serial.print("[COMM] 연속 오류 임계값 초과 (");
      Serial.print(commReliability.consecutiveErrors);
      Serial.println(") - 복구 모드 시작");
      startCommRecovery(COMM_ERROR_UNKNOWN);
    }
  }
}

/**
 * 통신 복구 모드 시작
 * @param errorCode 복구를 트리거한 오류 코드
 */
void startCommRecovery(uint8_t errorCode) {
  commReliability.isRecovering = true;
  commReliability.recoveryStartTime = millis();
  commReliability.lastErrorCode = errorCode;
  
  Serial.println("=== 통신 복구 모드 시작 ===");
  Serial.print("오류 코드: 0x");
  Serial.println(errorCode, HEX);
  Serial.print("연속 오류 횟수: ");
  Serial.println(commReliability.consecutiveErrors);
  Serial.print("통신 품질: ");
  Serial.print(commReliability.communicationQuality);
  Serial.println("%");
  
  // 복구 시도
  performCommRecoveryActions();
}

/**
 * 통신 복구 처리
 */
void handleCommRecovery() {
  unsigned long recoveryTime = millis() - commReliability.recoveryStartTime;
  
  // 복구 타임아웃 체크
  if (recoveryTime > commReliability.recoveryTimeout) {
    Serial.println("[COMM] 복구 타임아웃 - 복구 포기");
    commReliability.isRecovering = false;
    commReliability.slaveResponsive = false;
    return;
  }
  
  // 주기적 복구 시도 (2초마다)
  if (recoveryTime % 2000 < 100) { // 2초마다 한 번씩 실행
    performCommRecoveryActions();
  }
}

/**
 * 통신 복구 액션 수행
 */
void performCommRecoveryActions() {
  Serial.println("[COMM] 복구 액션 수행 중...");
  
  // 1. 시리얼 버퍼 클리어
  while (Serial.available()) {
    Serial.read();
  }
  
  // 2. 통신 상태 리셋
  resetComm();
  
  // 3. Slave에게 리셋 명령 전송
  Serial.println("[COMM] Slave 리셋 명령 전송");
  sendCommandToSlaveWithRetry(CMD_RESET, 0, 0, 2);
  
  delay(500); // Slave 리셋 대기
  
  // 4. 연결 테스트
  Serial.println("[COMM] 연결 테스트 수행");
  bool testResult = sendCommandToSlaveWithRetry(CMD_GET_STATUS, 0, 0, 3);
  
  if (testResult) {
    Serial.println("[COMM] 복구 성공!");
    commReliability.isRecovering = false;
    commReliability.consecutiveErrors = 0;
    commReliability.lastSuccessTime = millis();
    commReliability.slaveResponsive = true;
  } else {
    Serial.println("[COMM] 복구 실패 - 재시도 대기");
  }
}

/**
 * 통신 오류 기록
 * @param errorCode 오류 코드
 */
void recordCommError(uint8_t errorCode) {
  commReliability.lastErrorCode = errorCode;
  commReliability.consecutiveErrors++;
  commReliability.failedComms++;
  
  if (commReliability.diagnosticMode) {
    Serial.print("[COMM] 오류 기록 - 코드: 0x");
    Serial.print(errorCode, HEX);
    Serial.print(", 연속 오류: ");
    Serial.println(commReliability.consecutiveErrors);
  }
}

/**
 * 통신 성공 기록
 */
void recordCommSuccess() {
  commReliability.consecutiveErrors = 0;
  commReliability.successfulComms++;
  commReliability.lastSuccessTime = millis();
  commReliability.slaveResponsive = true;
  
  if (commReliability.isRecovering) {
    Serial.println("[COMM] 통신 복구 성공");
    commReliability.isRecovering = false;
  }
}

/**
 * 통신 품질 업데이트
 * 최근 통신 성공률을 기반으로 품질 계산
 */
void updateCommunicationQuality() {
  if (commReliability.totalCommAttempts == 0) {
    commReliability.communicationQuality = 100;
    return;
  }
  
  // 최근 100회 통신 기준으로 품질 계산
  uint32_t recentAttempts = min(commReliability.totalCommAttempts, COMM_RELIABILITY_QUALITY_WINDOW);
  uint32_t recentSuccesses = min(commReliability.successfulComms, recentAttempts);
  
  commReliability.communicationQuality = (recentSuccesses * 100) / recentAttempts;
  
  // 연속 오류가 있으면 품질 감소
  if (commReliability.consecutiveErrors > 0) {
    uint8_t penalty = min(commReliability.consecutiveErrors * 10, 50);
    commReliability.communicationQuality = max(0, (int)commReliability.communicationQuality - penalty);
  }
}

/**
 * 재시도 기능이 포함된 Slave 명령 전송
 * @param command 명령어
 * @param data1 데이터 1
 * @param data2 데이터 2
 * @param maxRetries 최대 재시도 횟수 (기본값: 설정된 최대값)
 * @return 전송 성공 여부
 */
bool sendCommandToSlaveWithRetry(uint8_t command, uint8_t data1, uint8_t data2, uint8_t maxRetries) {
  if (maxRetries == 0) {
    maxRetries = commReliability.maxRetries;
  }
  
  for (uint8_t attempt = 0; attempt <= maxRetries; attempt++) {
    commReliability.totalCommAttempts++;
    
    if (commReliability.diagnosticMode && attempt > 0) {
      Serial.print("[COMM] 재시도 ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.println(maxRetries);
    }
    
    // 명령 전송
    bool success = sendCommandToSlaveBasic(command, data1, data2);
    
    if (success) {
      recordCommSuccess();
      return true;
    } else {
      recordCommError(COMM_ERROR_NO_RESPONSE);
      
      // 마지막 시도가 아니면 잠시 대기
      if (attempt < maxRetries) {
        delay(100 + (attempt * 50)); // 점진적 백오프
      }
    }
  }
  
  if (commReliability.diagnosticMode) {
    Serial.print("[COMM] 명령 전송 실패 - 0x");
    Serial.println(command, HEX);
  }
  
  return false;
}

/**
 * 통신 진단 수행
 */
void performCommDiagnostics() {
  static unsigned long lastDiagnosticTime = 0;
  unsigned long currentTime = millis();
  
  // 5초마다 진단 정보 출력
  if (currentTime - lastDiagnosticTime >= 5000) {
    printCommDiagnostics();
    lastDiagnosticTime = currentTime;
  }
}

/**
 * 통신 진단 정보 출력
 */
void printCommDiagnostics() {
  Serial.println("=== 통신 진단 정보 ===");
  Serial.print("통신 품질: ");
  Serial.print(commReliability.communicationQuality);
  Serial.println("%");
  
  Serial.print("총 시도: ");
  Serial.print(commReliability.totalCommAttempts);
  Serial.print(", 성공: ");
  Serial.print(commReliability.successfulComms);
  Serial.print(", 실패: ");
  Serial.println(commReliability.failedComms);
  
  if (commReliability.totalCommAttempts > 0) {
    uint8_t successRate = (commReliability.successfulComms * 100) / commReliability.totalCommAttempts;
    Serial.print("전체 성공률: ");
    Serial.print(successRate);
    Serial.println("%");
  }
  
  Serial.print("연속 오류: ");
  Serial.println(commReliability.consecutiveErrors);
  
  Serial.print("Slave 응답: ");
  Serial.println(commReliability.slaveResponsive ? "정상" : "무응답");
  
  Serial.print("복구 모드: ");
  Serial.println(commReliability.isRecovering ? "활성" : "비활성");
  
  if (commReliability.lastErrorCode != COMM_ERROR_NONE) {
    Serial.print("마지막 오류: 0x");
    Serial.println(commReliability.lastErrorCode, HEX);
  }
  
  unsigned long timeSinceSuccess = millis() - commReliability.lastSuccessTime;
  Serial.print("마지막 성공: ");
  Serial.print(timeSinceSuccess);
  Serial.println("ms 전");
  
  Serial.println("====================");
}

/**
 * 통신 진단 모드 토글
 * @param enable 진단 모드 활성화 여부
 */
void setCommDiagnosticMode(bool enable) {
  commReliability.diagnosticMode = enable;
  Serial.print("통신 진단 모드: ");
  Serial.println(enable ? "활성화" : "비활성화");
}

/**
 * 통신 통계 리셋
 */
void resetCommStatistics() {
  commReliability.totalCommAttempts = 0;
  commReliability.successfulComms = 0;
  commReliability.failedComms = 0;
  commReliability.consecutiveErrors = 0;
  commReliability.communicationQuality = 100;
  commReliability.lastErrorCode = COMM_ERROR_NONE;
  
  Serial.println("통신 통계 리셋 완료");
}

/**
 * 게임 단계 전환 함수
 * @param newPhase 전환할 새로운 단계
 */
void transitionToPhase(GamePhase newPhase) {
  GamePhase oldPhase = (GamePhase)gameState.gamePhase;
  
  Serial.print("게임 단계 전환: ");
  Serial.print(getPhaseNameKorean(oldPhase));
  Serial.print(" -> ");
  Serial.println(getPhaseNameKorean(newPhase));
  
  // 이전 단계 정리
  exitPhase(oldPhase);
  
  // 새 단계 설정
  gameState.gamePhase = newPhase;
  gameState.phaseTimer = millis();
  
  // 새 단계 초기화
  enterPhase(newPhase);
}

/**
 * 단계 진입 시 초기화 처리
 * @param phase 진입하는 단계
 */
void enterPhase(GamePhase phase) {
  switch (phase) {
    case PHASE_IDLE:
      gameState.isGameActive = false;
      gameState.isRobotFacing = false;
      Serial.println("대기 모드 진입 - 시작 버튼 대기 중");
      break;
      
    case PHASE_GAME_START:
      gameState.isGameActive = true;
      Serial.println("게임 시작 - 천정 조명 순차 점등 시작");
      startCeilingLightSequence();
      playGameStartEffect();
      break;
      
    case PHASE_ROBOT_TURN_FRONT:
      Serial.println("로봇 정면 회전 시작");
      sendCommandToSlave(CMD_ROBOT_BODY_FRONT, 0, 0);
      break;
      
    case PHASE_NARRATION:
      gameState.isRobotFacing = true;
      Serial.println("환영 내레이션 재생 시작");
      playWelcomeNarration();
      break;
      
    case PHASE_ROBOT_TURN_BACK:
      gameState.isRobotFacing = false;
      Serial.println("로봇 후면 회전 시작");
      sendCommandToSlave(CMD_ROBOT_BODY_BACK, 0, 0);
      setBeaconLight(true, false); // 초록색 경광등 점등
      break;
      
    case PHASE_SHOW_PATTERN:
      Serial.println("LED 패턴 표시 시작");
      showCurrentLevelPattern();
      break;
      
    case PHASE_PLAYER_INPUT:
      Serial.println("플레이어 입력 대기 시작");
      // 입력 대기는 Slave에서 처리
      break;
      
    case PHASE_MOTION_DETECT:
      gameState.isRobotFacing = true;
      Serial.println("동작 감지 단계 시작");
      sendCommandToSlave(CMD_ROBOT_HEAD_FRONT, 0, 0);
      setBeaconLight(false, true); // 빨간색 경광등 점등
      startMotionDetection();
      playTensionEffect();
      break;
      
    case PHASE_SCORE_UPDATE:
      Serial.println("점수 업데이트 단계");
      processScoreUpdate();
      break;
      
    case PHASE_GAME_CLEAR:
      Serial.println("게임 클리어 단계 시작");
      playGameClearEffect();
      playGameClearNarration();
      activateEMLock();
      break;
  }
}

/**
 * 단계 종료 시 정리 처리
 * @param phase 종료하는 단계
 */
void exitPhase(GamePhase phase) {
  switch (phase) {
    case PHASE_GAME_START:
      // 천정 조명 순차 점등 완료 확인
      break;
      
    case PHASE_ROBOT_TURN_FRONT:
    case PHASE_ROBOT_TURN_BACK:
      // 로봇 회전 완료 대기
      break;
      
    case PHASE_NARRATION:
      // 내레이션 재생 완료
      break;
      
    case PHASE_SHOW_PATTERN:
      // 패턴 표시 완료
      break;
      
    case PHASE_PLAYER_INPUT:
      // 입력 대기 종료
      break;
      
    case PHASE_MOTION_DETECT:
      gameState.isRobotFacing = false;
      stopMotionDetection();
      stopTensionEffect();
      setBeaconLight(false, false); // 모든 경광등 소등
      break;
      
    case PHASE_SCORE_UPDATE:
      // 점수 업데이트 완료
      break;
      
    case PHASE_GAME_CLEAR:
      // 클리어 처리 완료
      break;
  }
}

// =============================================================================
// 각 게임 단계별 처리 함수들
// =============================================================================

/**
 * IDLE 단계 처리 - 시작 버튼 대기
 * 요구사항 1.1: 플레이어가 시작 버튼을 누르면 게임 시작
 */
void handleIdlePhase() {
  // 시작 버튼 입력 확인 (시리얼 명령으로 시뮬레이션)
  // 실제 구현에서는 하드웨어 버튼 입력 처리
  static unsigned long lastIdleMessage = 0;
  static bool startButtonPressed = false;
  
  // 시작 버튼 상태 확인 (실제 하드웨어에서는 디지털 핀 읽기)
  // 현재는 시리얼 명령으로 대체
  
  // 10초마다 대기 메시지 출력
  if (millis() - lastIdleMessage > 10000) {
    Serial.println("=== 무궁화 게임 대기 중 ===");
    Serial.println("시작하려면 'start' 명령을 입력하세요");
    Serial.print("현재 점수: ");
    Serial.print(gameState.currentScore);
    Serial.print("/7, 레벨: ");
    Serial.println(gameState.currentLevel);
    lastIdleMessage = millis();
  }
  
  // 게임 상태 검증 및 초기화
  if (gameState.isGameActive) {
    Serial.println("비정상 상태 감지 - 게임 상태 초기화");
    resetGameToIdleState();
  }
}

/**
 * GAME_START 단계 처리 - 천정 조명 순차 점등
 * 요구사항 1.2, 1.3: 천정 조명 순차 점등 및 효과음 재생
 */
void handleGameStartPhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 천정 조명 순차 점등 진행 상태 확인
  if (isCeilingLightSequenceComplete()) {
    Serial.println("=== 천정 조명 순차 점등 완료 ===");
    Serial.println("모든 조명이 점등되었습니다");
    
    // 잠시 대기 후 로봇 회전 시작 (시각적 효과를 위해)
    if (elapsedTime > (LIGHT_SEQUENCE_INTERVAL * 4 + 500)) {
      Serial.println("로봇 정면 회전 시작");
      transitionToPhase(PHASE_ROBOT_TURN_FRONT);
    }
  }
  
  // 타임아웃 처리 (최대 5초)
  if (elapsedTime > 5000) {
    Serial.println("천정 조명 시퀀스 타임아웃 - 강제로 다음 단계 진행");
    transitionToPhase(PHASE_ROBOT_TURN_FRONT);
  }
  
  // 진행 상황 로깅 (1초마다)
  static unsigned long lastProgressLog = 0;
  if (millis() - lastProgressLog > 1000) {
    Serial.print("천정 조명 진행: ");
    Serial.print(ceilingState.currentSequenceStep);
    Serial.println("/4");
    lastProgressLog = millis();
  }
}

/**
 * ROBOT_TURN_FRONT 단계 처리 - 로봇 정면 회전 완료 대기
 * 요구사항 1.4: 로봇 몸체가 플레이어 방향으로 180도 회전
 */
void handleRobotTurnFrontPhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 로봇 회전 명령 전송 (1초 후)
  static bool commandSent = false;
  if (!commandSent && elapsedTime > 1000) {
    Serial.println("로봇 몸체 정면 회전 명령 전송");
    sendCommandToSlave(CMD_ROBOT_BODY_FRONT, 0, 0);
    commandSent = true;
  }
  
  // 진행 상황 로깅 (2초마다)
  static unsigned long lastProgressLog = 0;
  if (millis() - lastProgressLog > 2000) {
    Serial.print("로봇 정면 회전 대기 중... (");
    Serial.print(elapsedTime / 1000);
    Serial.println("초 경과)");
    lastProgressLog = millis();
  }
  
  // 타임아웃 처리 (최대 15초)
  if (elapsedTime > ROBOT_ROTATION_TIMEOUT) {
    Serial.println("=== 로봇 정면 회전 타임아웃 ===");
    Serial.println("강제로 다음 단계 진행");
    gameState.isRobotFacing = true; // 강제로 정면 상태 설정
    transitionToPhase(PHASE_NARRATION);
  }
  
  // Slave 통신에서 RESP_ROBOT_FRONT_COMPLETE 응답을 받으면
  // processSlaveComm() 함수에서 transitionToPhase(PHASE_NARRATION) 호출
}

/**
 * NARRATION 단계 처리 - 내레이션 재생 완료 대기
 * 요구사항 1.5: 환영 내레이션이 재생되어야 한다
 */
void handleNarrationPhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 내레이션 재생 시작 (1초 후)
  static bool narrationStarted = false;
  if (!narrationStarted && elapsedTime > 1000) {
    Serial.println("=== 환영 내레이션 재생 시작 ===");
    playWelcomeNarration();
    narrationStarted = true;
  }
  
  // DFPlayer 재생 상태 확인
  if (narrationStarted && !dfPlayerState.isPlaying && elapsedTime > 3000) {
    // 내레이션 재생이 완료되면 다음 단계로 전환 (최소 3초 후)
    Serial.println("=== 환영 내레이션 재생 완료 ===");
    Serial.println("로봇 후면 회전 시작");
    transitionToPhase(PHASE_ROBOT_TURN_BACK);
  }
  
  // 진행 상황 로깅 (3초마다)
  static unsigned long lastProgressLog = 0;
  if (millis() - lastProgressLog > 3000) {
    if (narrationStarted) {
      Serial.print("내레이션 재생 중... (");
      Serial.print(elapsedTime / 1000);
      Serial.println("초 경과)");
    } else {
      Serial.println("내레이션 재생 준비 중...");
    }
    lastProgressLog = millis();
  }
  
  // 타임아웃 처리 (최대 30초)
  if (elapsedTime > NARRATION_TIMEOUT) {
    Serial.println("=== 내레이션 재생 타임아웃 ===");
    Serial.println("강제로 다음 단계 진행");
    transitionToPhase(PHASE_ROBOT_TURN_BACK);
  }
}

/**
 * ROBOT_TURN_BACK 단계 처리 - 로봇 후면 회전 완료 대기
 * 요구사항 2.1: 로봇 몸체가 후면으로 180도 회전 완료되면 초록색 경광등이 점등
 */
void handleRobotTurnBackPhase() {
  // Slave로부터 로봇 후면 회전 완료 응답 대기
  // 타임아웃 처리 (최대 10초)
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  if (elapsedTime > ROBOT_ROTATION_TIMEOUT) {
    Serial.println("로봇 후면 회전 타임아웃 - 강제로 패턴 표시 시작");
    transitionToPhase(PHASE_SHOW_PATTERN);
  }
  
  // Slave 통신에서 RESP_ROBOT_BACK_COMPLETE 응답을 받으면
  // processSlaveComm() 함수에서 transitionToPhase(PHASE_SHOW_PATTERN) 호출
  
  if (isCeilingLightSequenceComplete()) {
    Serial.println("천정 조명 순차 점등 완료");
    transitionToPhase(PHASE_ROBOT_TURN_FRONT);
  }
}

/**
 * ROBOT_TURN_FRONT 단계 처리 - 로봇 정면 회전 대기
 * 요구사항 1.4: 로봇 몸체가 플레이어 방향으로 180도 회전
 */
void handleRobotTurnFrontPhase() {
  // Slave로부터 로봇 회전 완료 응답 대기
  // 타임아웃 처리 (10초)
  if (millis() - gameState.phaseTimer > 10000) {
    Serial.println("로봇 회전 타임아웃 - 다음 단계로 강제 전환");
    transitionToPhase(PHASE_NARRATION);
  }
}

/**
 * NARRATION 단계 처리 - 환영 내레이션 재생
 * 요구사항 1.5: 환영 내레이션이 재생되어야 한다
 */
void handleNarrationPhase() {
  // 내레이션 재생 완료 확인 (예상 재생 시간: 5초)
  if (millis() - gameState.phaseTimer > 5000) {
    Serial.println("환영 내레이션 완료");
    transitionToPhase(PHASE_ROBOT_TURN_BACK);
  }
}

/**
 * ROBOT_TURN_BACK 단계 처리 - 로봇 후면 회전 대기
 * 요구사항 2.1: 로봇 몸체가 후면으로 180도 회전 완료되면 초록색 경광등 점등
 */
void handleRobotTurnBackPhase() {
  // Slave로부터 로봇 회전 완료 응답 대기
  // 타임아웃 처리 (10초)
  if (millis() - gameState.phaseTimer > 10000) {
    Serial.println("로봇 후면 회전 완료 (타임아웃)");
    transitionToPhase(PHASE_SHOW_PATTERN);
  }
}

/**
 * SHOW_PATTERN 단계 처리 - LED 패턴 표시
 * 요구사항 2.2, 2.3: LED 패턴 점멸 및 효과음 재생
 */
void handleShowPatternPhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 패턴 표시 시작 (1초 후)
  static bool patternStarted = false;
  if (!patternStarted && elapsedTime > 1000) {
    Serial.println("=== LED 패턴 표시 시작 ===");
    Serial.print("현재 레벨: ");
    Serial.print(gameState.currentLevel);
    Serial.print(", 패턴 길이: ");
    
    // 레벨별 패턴 길이 계산
    uint8_t patternLength = getPatternLengthForLevel(gameState.currentLevel);
    Serial.println(patternLength);
    
    // Slave에게 패턴 표시 명령 전송
    showCurrentLevelPattern();
    patternStarted = true;
  }
  
  // 패턴 표시 예상 시간 계산
  unsigned long patternDuration = calculatePatternDuration(gameState.currentLevel);
  
  // 진행 상황 로깅 (3초마다)
  static unsigned long lastProgressLog = 0;
  if (millis() - lastProgressLog > 3000) {
    if (patternStarted) {
      Serial.print("패턴 표시 진행 중... (");
      Serial.print(elapsedTime / 1000);
      Serial.print("초 경과, 예상 완료: ");
      Serial.print(patternDuration / 1000);
      Serial.println("초)");
    } else {
      Serial.println("패턴 표시 준비 중...");
    }
    lastProgressLog = millis();
  }
  
  // 타임아웃 처리 (패턴 표시 예상 시간 + 여유시간)
  if (elapsedTime > patternDuration + PATTERN_DISPLAY_TIMEOUT) {
    Serial.println("=== 패턴 표시 타임아웃 ===");
    Serial.println("강제로 입력 대기 시작");
    transitionToPhase(PHASE_PLAYER_INPUT);
  }
  
  // Slave 통신에서 RESP_PATTERN_DONE 응답을 받으면
  // processSlaveComm() 함수에서 transitionToPhase(PHASE_PLAYER_INPUT) 호출
}

/**
 * PLAYER_INPUT 단계 처리 - 플레이어 입력 대기
 * 요구사항 2.5: 초록색 경광등이 점등된 후 10초가 지나면 로봇 HEAD가 정면으로 회전
 */
void handlePlayerInputPhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 입력 대기 시작 알림 (1초 후)
  static bool inputStarted = false;
  if (!inputStarted && elapsedTime > 1000) {
    Serial.println("=== 플레이어 입력 대기 시작 ===");
    Serial.println("요구사항 2.4: 발판을 밟으면 효과음이 재생됩니다");
    Serial.print("입력 제한시간: ");
    Serial.print(PLAYER_INPUT_TIMEOUT / 1000);
    Serial.println("초");
    inputStarted = true;
  }
  
  // 플레이어 입력 제한시간 확인 (10초)
  if (elapsedTime > PLAYER_INPUT_TIMEOUT) {
    Serial.println("=== 플레이어 입력 시간 초과 ===");
    Serial.println("요구사항 2.5: 10초 후 로봇 HEAD가 정면으로 회전");
    lastInputResult = false; // 타임아웃은 오답 처리
    transitionToPhase(PHASE_MOTION_DETECT);
    return;
  }
  
  // 입력 진행 상황 표시 (2초마다)
  static unsigned long lastProgressReport = 0;
  if (elapsedTime - lastProgressReport > 2000 && inputStarted) {
    unsigned long remainingTime = (PLAYER_INPUT_TIMEOUT - elapsedTime) / 1000;
    Serial.print("플레이어 입력 대기 중... 남은 시간: ");
    Serial.print(remainingTime);
    Serial.println("초");
    lastProgressReport = elapsedTime;
  }
  
  // 8초 경과 시 경고 메시지
  static bool warningShown = false;
  if (!warningShown && elapsedTime > 8000) {
    Serial.println("⚠️ 입력 시간이 얼마 남지 않았습니다!");
    warningShown = true;
  }
  
  // Slave로부터 입력 결과 응답 대기
  // RESP_INPUT_CORRECT, RESP_INPUT_WRONG, RESP_INPUT_TIMEOUT 응답을 받으면
  // processSlaveComm() 함수에서 transitionToPhase(PHASE_MOTION_DETECT) 호출
}

/**
 * MOTION_DETECT 단계 처리 - 동작 감지
 * 요구사항 3.2, 3.3, 3.4, 3.5, 3.6: PIR 센서 동작감지 및 처벌 시스템
 */
void handleMotionDetectPhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 동작 감지 시작 처리 (1초 후)
  static bool motionDetectStarted = false;
  if (!motionDetectStarted && elapsedTime > 1000) {
    Serial.println("=== 동작 감지 단계 시작 ===");
    Serial.println("요구사항 3.1: 빨간색 경광등 점등");
    Serial.println("요구사항 3.2: PIR 센서 5초간 동작감지 시작");
    
    // 로봇 HEAD 정면 회전 명령 전송
    sendCommandToSlave(CMD_ROBOT_HEAD_FRONT, 0, 0);
    
    // 경광등 상태 변경 (초록 -> 빨강)
    setBeaconLight(false, true);
    
    // PIR 센서 동작 감지 시작
    startMotionDetection();
    
    // 긴장감 조성 이펙트 시작
    playTensionEffect();
    
    motionDetectStarted = true;
  }
  
  // 동작 감지 시간 확인 (5초)
  if (elapsedTime > MOTION_DETECT_DURATION + 1000) { // 1초 여유시간 추가
    Serial.println("=== 동작 감지 시간 완료 ===");
    
    // 동작 감지 결과 최종 확인
    checkFinalMotionDetectionResult();
    
    // 긴장 이펙트 중지
    stopTensionEffect();
    
    // 로봇 HEAD를 다시 후면으로 회전
    Serial.println("로봇 HEAD 후면 회전 시작");
    sendCommandToSlave(CMD_ROBOT_HEAD_BACK, 0, 0);
    
    // 경광등 소등
    Serial.println("요구사항 3.1: 동작 감지 완료 후 경광등 자동 소등");
    setBeaconLight(false, false);
    
    transitionToPhase(PHASE_SCORE_UPDATE);
    return;
  }
  
  // 동작 감지 진행 상황 표시 (1초마다)
  static unsigned long lastProgressReport = 0;
  if (elapsedTime - lastProgressReport > 1000 && motionDetectStarted) {
    unsigned long remainingTime = (MOTION_DETECT_DURATION + 1000 - elapsedTime) / 1000;
    Serial.print("🔍 동작 감지 중... 남은 시간: ");
    Serial.print(remainingTime);
    Serial.println("초 (움직이지 마세요!)");
    lastProgressReport = elapsedTime;
  }
  
  // 실시간 동작 감지 처리는 processMotionDetection() 함수에서 수행
}

/**
 * SCORE_UPDATE 단계 처리 - 점수 업데이트 및 다음 라운드 준비
 * 요구사항 4.1, 4.4, 4.5, 4.6: 점수 증감 및 클리어 조건 확인
 */
void handleScoreUpdatePhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 점수 업데이트 처리 (1초 후)
  static bool scoreProcessed = false;
  if (!scoreProcessed && elapsedTime > 1000) {
    Serial.println("=== 점수 업데이트 처리 ===");
    
    // 점수 업데이트 실행
    processScoreUpdate();
    
    // 레벨 진행 처리
    updateLevelProgression();
    
    scoreProcessed = true;
  }
  
  // 점수 변화 확인 시간 후 다음 단계로 전환
  if (elapsedTime > SCORE_UPDATE_TIMEOUT) {
    // 게임 클리어 조건 확인 (요구사항 6.1)
    if (gameState.currentScore >= MAX_SCORE) {
      Serial.println("=== 🎉 게임 클리어 조건 달성! 🎉 ===");
      Serial.print("최종 점수: ");
      Serial.print(gameState.currentScore);
      Serial.print("/");
      Serial.println(MAX_SCORE);
      Serial.print("최종 레벨: ");
      Serial.println(gameState.currentLevel);
      Serial.println("요구사항 6.1: 7점 획득으로 클리어 효과 시작");
      
      transitionToPhase(PHASE_GAME_CLEAR);
    } else {
      Serial.println("=== 다음 라운드 준비 ===");
      Serial.print("현재 점수: ");
      Serial.print(gameState.currentScore);
      Serial.print("/");
      Serial.println(MAX_SCORE);
      Serial.print("현재 레벨: ");
      Serial.println(gameState.currentLevel);
      
      // 레벨 순환 처리 (요구사항 5.5)
      if (gameState.currentLevel > MAX_LEVEL) {
        Serial.println("요구사항 5.5: 레벨 11 완료 후 레벨 1부터 다시 시작");
        gameState.currentLevel = 1;
      }
      
      Serial.println("다음 라운드를 위해 로봇 후면 회전 시작");
      
      // 다음 라운드를 위해 로봇 후면 회전부터 시작
      transitionToPhase(PHASE_ROBOT_TURN_BACK);
    }
  }
  
  // 진행 상황 로깅 (2초마다)
  static unsigned long lastProgressLog = 0;
  if (millis() - lastProgressLog > 2000) {
    if (scoreProcessed) {
      Serial.print("점수 업데이트 완료 - 다음 단계 준비 중... (");
      Serial.print(elapsedTime / 1000);
      Serial.println("초 경과)");
    } else {
      Serial.println("점수 업데이트 준비 중...");
    }
    lastProgressLog = millis();
  }
}

/**
 * GAME_CLEAR 단계 처리 - 게임 클리어 연출
 * 요구사항 6.1, 6.2, 6.3, 6.4: 클리어 효과 및 EM-Lock 작동
 */
void handleGameClearPhase() {
  unsigned long elapsedTime = millis() - gameState.phaseTimer;
  
  // 클리어 연출 단계별 처리
  static bool clearEffectStarted = false;
  static bool scoreLEDBlinkStarted = false;
  static bool emLockActivated = false;
  static bool clearNarrationStarted = false;
  static bool systemShutdownStarted = false;
  
  // 1단계: 클리어 이펙트 시작 (즉시)
  if (!clearEffectStarted) {
    Serial.println("🎉 === 게임 클리어 이펙트 시작 === 🎉");
    Serial.println("요구사항 6.1: 7점 달성으로 클리어 효과 시작");
    
    // 요구사항 6.2: 이펙트 LED가 특별한 패턴으로 연출
    Serial.println("요구사항 6.2: 이펙트 LED 특별 패턴 연출");
    playGameClearEffect();
    
    clearEffectStarted = true;
  }
  
  // 2단계: 점수 LED 클리어 점멸 시작 (1초 후)
  if (!scoreLEDBlinkStarted && elapsedTime > 1000) {
    Serial.println("=== 점수 LED 클리어 점멸 시작 ===");
    Serial.println("요구사항 4.6: 7점 달성 시 점수 LED 0.5초 간격 7회 점멸");
    
    // 점수 LED 클리어 점멸 (요구사항 4.6)
    sendCommandToSlave(CMD_UPDATE_SCORE, SCORE_7, 1); // data2=1은 클리어 점멸 모드
    
    scoreLEDBlinkStarted = true;
  }
  
  // 3단계: EM-Lock 작동 (5초 후)
  if (!emLockActivated && elapsedTime > 5000) {
    Serial.println("🔓 === EM-Lock 작동 ===");
    Serial.println("요구사항 6.3: EM-Lock 작동으로 다음 세션 통로 개방");
    
    // 요구사항 6.3: EM-Lock이 작동하여 다음 세션으로의 통로가 열림
    activateEMLock();
    
    emLockActivated = true;
  }
  
  // 4단계: 클리어 내레이션 시작 (7초 후)
  if (!clearNarrationStarted && elapsedTime > 7000) {
    Serial.println("🎊 === 클리어 내레이션 재생 ===");
    
    // 클리어 축하 내레이션 재생
    playGameClearNarration();
    
    clearNarrationStarted = true;
  }
  
  // 5단계: 시스템 최소 전력 모드 준비 (15초 후)
  if (!systemShutdownStarted && elapsedTime > 15000) {
    Serial.println("💤 === 시스템 최소 전력 모드 준비 ===");
    Serial.println("요구사항 6.4: 모든 장치 최소 전력 대기 상태 전환");
    
    // 요구사항 6.4: 모든 장치가 최소 전력 대기 상태로 전환
    prepareSystemShutdown();
    
    systemShutdownStarted = true;
  }
  
  // 6단계: 클리어 연출 완료 및 대기 모드 전환 (20초 후)
  if (elapsedTime > GAME_CLEAR_TIMEOUT) {
    Serial.println("✅ === 게임 클리어 연출 완료 ===");
    Serial.println("축하합니다! 무궁화 게임을 성공적으로 클리어했습니다!");
    Serial.print("최종 점수: ");
    Serial.print(gameState.currentScore);
    Serial.print("/7, 최종 레벨: ");
    Serial.println(gameState.currentLevel);
    Serial.println("대기 모드로 전환합니다");
    
    // 게임 상태 초기화 및 대기 모드 전환
    resetGameToIdleState();
    transitionToPhase(PHASE_IDLE);
    
    // 정적 변수 초기화
    clearEffectStarted = false;
    scoreLEDBlinkStarted = false;
    emLockActivated = false;
    clearNarrationStarted = false;
    systemShutdownStarted = false;
  }
  
  // 진행 상황 표시 (3초마다)
  static unsigned long lastProgressReport = 0;
  if (elapsedTime - lastProgressReport > 3000) {
    unsigned long remainingTime = (GAME_CLEAR_TIMEOUT - elapsedTime) / 1000;
    
    // 현재 단계 표시
    String currentStage = "준비";
    if (systemShutdownStarted) currentStage = "시스템 종료 준비";
    else if (clearNarrationStarted) currentStage = "내레이션 재생";
    else if (emLockActivated) currentStage = "EM-Lock 작동";
    else if (scoreLEDBlinkStarted) currentStage = "점수 LED 점멸";
    else if (clearEffectStarted) currentStage = "이펙트 연출";
    
    Serial.print("🎮 클리어 연출 진행: ");
    Serial.print(currentStage);
    Serial.print(" (남은 시간: ");
    Serial.print(remainingTime);
    Serial.println("초)");
    Serial.println("초");
    lastProgressReport = elapsedTime;
  }
}

// =============================================================================
// 게임 상태 관리 유틸리티 함수들
// =============================================================================

/**
 * 게임 시작 트리거 (외부에서 호출)
 */
void startGame() {
  if (gameState.gamePhase == PHASE_IDLE) {
    Serial.println("게임 시작 명령 수신");
    resetGameState();
    transitionToPhase(PHASE_GAME_START);
  } else {
    Serial.println("게임이 이미 진행 중입니다");
  }
}

/**
 * 게임 상태 리셋
 */
void resetGameState() {
  Serial.println("게임 상태 초기화");
  
  gameState.currentLevel = 1;
  gameState.currentScore = 0;
  gameState.isGameActive = false;
  gameState.isRobotFacing = false;
  gameState.phaseTimer = 0;
  
  // 모든 조명 소등
  setBeaconLight(false, false);
  stopAllEffects();
  
  // Slave에 정지 명령 전송
  sendCommandToSlave(CMD_STOP_ALL, 0, 0);
  
  Serial.println("게임 상태 초기화 완료");
}

/**
 * 현재 게임 단계 확인
 * @return 현재 게임 단계
 */
GamePhase getCurrentPhase() {
  return (GamePhase)gameState.gamePhase;
}

/**
 * 게임 활성 상태 확인
 * @return 게임 활성 여부
 */
bool isGameActive() {
  return gameState.isGameActive;
}

/**
 * 로봇 정면 향함 상태 확인
 * @return 로봇이 정면을 보고 있는지 여부
 */
bool isRobotFacing() {
  return gameState.isRobotFacing;
}

/**
 * 현재 단계 경과 시간 확인
 * @return 현재 단계 시작 후 경과 시간 (ms)
 */
unsigned long getPhaseElapsedTime() {
  return millis() - gameState.phaseTimer;
}

/**
 * 게임 단계 이름 반환 (한국어)
 * @param phase 게임 단계
 * @return 단계 이름 문자열
 */
const char* getPhaseNameKorean(GamePhase phase) {
  switch (phase) {
    case PHASE_IDLE: return "대기";
    case PHASE_GAME_START: return "게임시작";
    case PHASE_ROBOT_TURN_FRONT: return "로봇정면회전";
    case PHASE_NARRATION: return "내레이션";
    case PHASE_ROBOT_TURN_BACK: return "로봇후면회전";
    case PHASE_SHOW_PATTERN: return "패턴표시";
    case PHASE_PLAYER_INPUT: return "플레이어입력";
    case PHASE_MOTION_DETECT: return "동작감지";
    case PHASE_SCORE_UPDATE: return "점수업데이트";
    case PHASE_GAME_CLEAR: return "게임클리어";
    default: return "알수없음";
  }
}

// =============================================================================
// 외부 이벤트 처리 함수들 (다른 시스템에서 호출)
// =============================================================================

/**
 * Slave로부터 로봇 회전 완료 응답 처리
 * @param isHeadRotation HEAD 회전 여부 (false면 BODY 회전)
 */
void onRobotRotationComplete(bool isHeadRotation) {
  Serial.print(isHeadRotation ? "로봇 HEAD" : "로봇 BODY");
  Serial.println(" 회전 완료 응답 수신");
  
  if (gameState.gamePhase == PHASE_ROBOT_TURN_FRONT && !isHeadRotation) {
    transitionToPhase(PHASE_NARRATION);
  } else if (gameState.gamePhase == PHASE_ROBOT_TURN_BACK && !isHeadRotation) {
    transitionToPhase(PHASE_SHOW_PATTERN);
  }
}

/**
 * Slave로부터 패턴 표시 완료 응답 처리
 */
void onPatternDisplayComplete() {
  Serial.println("패턴 표시 완료 응답 수신");
  
  if (gameState.gamePhase == PHASE_SHOW_PATTERN) {
    transitionToPhase(PHASE_PLAYER_INPUT);
  }
}

/**
 * Slave로부터 플레이어 입력 응답 처리
 * @param isCorrect 입력이 올바른지 여부
 */
void onPlayerInputReceived(bool isCorrect) {
  Serial.print("플레이어 입력 응답 수신: ");
  Serial.println(isCorrect ? "정답" : "오답");
  
  if (gameState.gamePhase == PHASE_PLAYER_INPUT) {
    // 입력 결과 저장 (SCORE_UPDATE 단계에서 사용)
    lastInputResult = isCorrect;
    transitionToPhase(PHASE_MOTION_DETECT);
  }
}

// =============================================================================
// 점수 및 레벨 관리 시스템 구현 (Task 11.2)
// =============================================================================

// 점수 및 레벨 관리 변수
bool lastInputResult = false;        // 마지막 입력 결과
bool motionDetected = false;         // 동작 감지 결과
uint8_t previousScore = 0;           // 이전 점수 (변화 감지용)
uint8_t previousLevel = 1;           // 이전 레벨 (변화 감지용)

/**
 * 점수 증가 처리
 * 요구사항 4.1: 플레이어가 올바른 순서로 발판을 밟으면 1점이 추가되어야 한다
 * @return 점수 증가 성공 여부
 */
bool incrementScore() {
  if (gameState.currentScore >= MAX_SCORE) {
    Serial.println("이미 최대 점수에 도달했습니다");
    return false;
  }
  
  previousScore = gameState.currentScore;
  gameState.currentScore++;
  
  Serial.print("점수 증가: ");
  Serial.print(previousScore);
  Serial.print(" -> ");
  Serial.println(gameState.currentScore);
  
  // Slave에 점수 LED 업데이트 명령 전송
  updateScoreLEDs();
  
  // 성공 이펙트 재생
  playSuccessEffect();
  
  return true;
}

/**
 * 점수 감소 처리
 * 요구사항 3.5, 4.4: 움직임이 감지되면 점수가 1점 감점, 0점 하한선 처리
 * @return 점수 감소 성공 여부
 */
bool decrementScore() {
  if (gameState.currentScore <= MIN_SCORE) {
    Serial.println("점수가 이미 최소값입니다 (0점 하한선)");
    return false;
  }
  
  previousScore = gameState.currentScore;
  gameState.currentScore--;
  
  Serial.print("점수 감소: ");
  Serial.print(previousScore);
  Serial.print(" -> ");
  Serial.println(gameState.currentScore);
  
  // Slave에 점수 LED 업데이트 명령 전송
  updateScoreLEDs();
  
  // 오류 이펙트 재생
  playErrorEffect();
  
  return true;
}

/**
 * 점수 LED 업데이트 (Slave에 명령 전송)
 * 요구사항 4.2, 4.3: 점수 증감에 따른 LED 상태 업데이트
 */
void updateScoreLEDs() {
  Serial.print("점수 LED 업데이트: ");
  Serial.print(gameState.currentScore);
  Serial.println("점");
  
  // Slave에 점수 업데이트 명령 전송
  sendCommandToSlave(CMD_UPDATE_SCORE, gameState.currentScore, 0);
  
  // 7점 달성 시 특별 처리
  if (gameState.currentScore == MAX_SCORE) {
    Serial.println("7점 달성! 특별 점멸 패턴 시작");
    // 요구사항 4.6: 7점 달성 시 특별 점멸 패턴 (0.5초 간격 7회)
    // 실제 점멸은 Slave에서 처리
  }
}

/**
 * 레벨 증가 처리
 * 요구사항 5.1: 한 라운드가 완료되면 레벨이 1씩 증가
 * @return 레벨 증가 성공 여부
 */
bool incrementLevel() {
  previousLevel = gameState.currentLevel;
  gameState.currentLevel++;
  
  // 요구사항 5.5: 레벨 11을 완료해도 7점에 도달하지 못하면 레벨 1부터 다시 시작
  if (gameState.currentLevel > MAX_LEVEL) {
    Serial.println("최대 레벨 도달 - 레벨 1로 순환");
    gameState.currentLevel = 1;
  }
  
  Serial.print("레벨 증가: ");
  Serial.print(previousLevel);
  Serial.print(" -> ");
  Serial.println(gameState.currentLevel);
  
  // 레벨업 이펙트 재생
  playLevelUpEffect();
  
  return true;
}

/**
 * 현재 레벨에 따른 패턴 길이 계산
 * 요구사항 5.2, 5.3, 5.4: 레벨별 난이도 조절 로직
 * @param level 레벨 (1-11)
 * @return 패턴 길이 (LED 개수)
 */
uint8_t calculatePatternLength(uint8_t level) {
  if (level >= 1 && level <= 4) {
    // 요구사항 5.2: 레벨 1-4일 때 LED는 (레벨+1)개만큼 점멸
    return level + 1;
  } else if (level >= 5 && level <= 10) {
    // 요구사항 5.3: 레벨 5-10일 때 LED는 5개만큼 점멸
    return 5;
  } else if (level == 11) {
    // 요구사항 5.4: 레벨 11일 때 LED는 6개만큼 점멸
    return 6;
  }
  
  // 기본값 (오류 방지)
  return 2;
}

/**
 * 현재 레벨에 따른 패턴 간격 계산
 * 요구사항 5.2, 5.3: 레벨별 속도 조절
 * @param level 레벨 (1-11)
 * @return 패턴 간격 (ms)
 */
uint16_t calculatePatternInterval(uint8_t level) {
  if (level >= 1 && level <= 4) {
    // 요구사항 5.2: 레벨 1-4일 때 0.5초 간격
    return PATTERN_DISPLAY_INTERVAL; // 500ms
  } else {
    // 요구사항 5.3: 레벨 5+ 일 때 0.3초 간격
    return PATTERN_FAST_INTERVAL; // 300ms
  }
}

/**
 * 현재 레벨 패턴 표시 시간 계산
 * @param level 레벨 (1-11)
 * @return 총 패턴 표시 시간 (ms)
 */
unsigned long calculatePatternDuration(uint8_t level) {
  uint8_t patternLength = calculatePatternLength(level);
  uint16_t patternInterval = calculatePatternInterval(level);
  
  // 패턴 길이 × 간격 + 여유 시간
  return (patternLength * patternInterval) + 1000;
}

/**
 * 현재 레벨에 맞는 LED 패턴 표시 시작
 * 요구사항 2.2: LED가 레벨에 따른 패턴으로 점멸
 */
void showCurrentLevelPattern() {
  uint8_t patternLength = calculatePatternLength(gameState.currentLevel);
  uint16_t patternInterval = calculatePatternInterval(gameState.currentLevel);
  
  Serial.print("레벨 ");
  Serial.print(gameState.currentLevel);
  Serial.print(" 패턴 표시: 길이=");
  Serial.print(patternLength);
  Serial.print(", 간격=");
  Serial.print(patternInterval);
  Serial.println("ms");
  
  // Slave에 패턴 표시 명령 전송
  sendCommandToSlave(CMD_SHOW_PATTERN, patternLength, patternInterval / 100); // 간격을 100ms 단위로 전송
}

/**
 * 점수 업데이트 처리 (게임 상태 머신에서 호출)
 * 입력 결과와 동작 감지 결과를 종합하여 점수 처리
 */
void processScoreUpdate() {
  Serial.println("=== 점수 업데이트 처리 시작 ===");
  Serial.print("입력 결과: ");
  Serial.println(lastInputResult ? "정답" : "오답");
  Serial.print("동작 감지: ");
  Serial.println(motionDetected ? "감지됨" : "감지안됨");
  
  bool scoreChanged = false;
  
  // 1. 올바른 입력 시 점수 증가
  if (lastInputResult) {
    if (incrementScore()) {
      scoreChanged = true;
      Serial.println("정답으로 인한 점수 증가");
    }
  }
  
  // 2. 동작 감지 시 점수 감소
  if (motionDetected) {
    if (decrementScore()) {
      scoreChanged = true;
      Serial.println("동작 감지로 인한 점수 감소");
    }
  }
  
  // 3. 점수 변화가 없는 경우
  if (!scoreChanged) {
    Serial.println("점수 변화 없음");
  }
  
  // 4. 현재 상태 출력
  Serial.print("현재 점수: ");
  Serial.print(gameState.currentScore);
  Serial.print("/");
  Serial.println(MAX_SCORE);
  Serial.print("현재 레벨: ");
  Serial.println(gameState.currentLevel);
  
  // 5. 결과 변수 초기화
  lastInputResult = false;
  motionDetected = false;
  
  Serial.println("=== 점수 업데이트 처리 완료 ===");
}

/**
 * 현재 점수 반환
 * @return 현재 점수 (0-7)
 */
uint8_t getCurrentScore() {
  return gameState.currentScore;
}

/**
 * 현재 레벨 반환
 * @return 현재 레벨 (1-11)
 */
uint8_t getCurrentLevel() {
  return gameState.currentLevel;
}

/**
 * 게임 클리어 조건 확인
 * 요구사항 6.1: 플레이어가 7점을 획득하면 클리어 효과가 시작
 * @return 클리어 조건 달성 여부
 */
bool isGameClearConditionMet() {
  return gameState.currentScore >= MAX_SCORE;
}

/**
 * 점수 및 레벨 상태 출력 (디버깅용)
 */
void printScoreAndLevelStatus() {
  Serial.println("=== 점수 및 레벨 상태 ===");
  Serial.print("점수: ");
  Serial.print(gameState.currentScore);
  Serial.print("/");
  Serial.println(MAX_SCORE);
  Serial.print("레벨: ");
  Serial.print(gameState.currentLevel);
  Serial.print("/");
  Serial.println(MAX_LEVEL);
  
  uint8_t patternLength = calculatePatternLength(gameState.currentLevel);
  uint16_t patternInterval = calculatePatternInterval(gameState.currentLevel);
  Serial.print("현재 레벨 패턴: 길이=");
  Serial.print(patternLength);
  Serial.print(", 간격=");
  Serial.print(patternInterval);
  Serial.println("ms");
  
  Serial.print("클리어 조건: ");
  Serial.println(isGameClearConditionMet() ? "달성" : "미달성");
  Serial.println("========================");
}

/**
 * 동작 감지 결과 설정 (PIR 센서 시스템에서 호출)
 * @param detected 동작 감지 여부
 */
void setMotionDetectionResult(bool detected) {
  motionDetected = detected;
  Serial.print("동작 감지 결과 설정: ");
  Serial.println(detected ? "감지됨" : "감지안됨");
}

/**
 * 레벨별 난이도 정보 출력
 * @param level 확인할 레벨
 */
void printLevelDifficulty(uint8_t level) {
  Serial.print("레벨 ");
  Serial.print(level);
  Serial.print(" 난이도: ");
  
  uint8_t length = calculatePatternLength(level);
  uint16_t interval = calculatePatternInterval(level);
  
  Serial.print("패턴 ");
  Serial.print(length);
  Serial.print("개, ");
  Serial.print(interval);
  Serial.print("ms 간격");
  
  if (level >= 1 && level <= 4) {
    Serial.println(" (초급)");
  } else if (level >= 5 && level <= 10) {
    Serial.println(" (중급)");
  } else if (level == 11) {
    Serial.println(" (고급)");
  } else {
    Serial.println(" (알수없음)");
  }
}

void updateEffectLEDs() {
  // 이펙트 LED 패턴 처리는 패턴 엔진에서 처리
  updatePatternEngine();
}

// =============================================================================
// 이펙트 패턴 엔진 구현 (Task 3.2)
// =============================================================================

/**
 * 이펙트 패턴 엔진 초기화
 * 요구사항 8.1, 8.3: 적절한 시각적 피드백 제공
 */
void initializeEffectPatternEngine() {
  patternEngine.engineActive = false;
  patternEngine.currentStep = 0;
  patternEngine.currentRepeat = 0;
  patternEngine.stepTimer = 0;
  patternEngine.activePattern = SUCCESS_PATTERN; // 기본값
  
  // 현재 시퀀스 초기화
  memset(&patternEngine.currentSequence, 0, sizeof(EffectSequence));
  
  Serial.println("이펙트 패턴 엔진 초기화 완료");
}

/**
 * 이펙트 LED 상태 초기화
 */
void initializeEffectLEDs() {
  // 이펙트 LED 상태 초기화
  for (int i = 0; i < 6; i++) {
    effectLEDState.ledState[i] = false;
  }
  effectLEDState.currentMask = 0;
  effectLEDState.lastUpdateTime = 0;
  effectLEDState.patternActive = false;
  effectLEDState.patternTimer = 0;
  
  // 패턴 엔진 초기화
  initializeEffectPatternEngine();
  
  Serial.println("이펙트 LED 시스템 초기화 완료");
}

/**
 * SUCCESS 패턴 데이터 생성
 * 요구사항 8.3: 플레이어가 올바른 행동을 하면 긍정적인 피드백 제공
 */
void createSuccessPattern(EffectSequence* sequence) {
  sequence->stepCount = 6;
  sequence->repeatCount = 2;
  sequence->isLooping = false;
  
  // 1-2-3-4-5-6 순차 점등 후 전체 점멸
  sequence->steps[0] = {0b000001, EFFECT_SUCCESS_SPEED, true};  // LED 1
  sequence->steps[1] = {0b000011, EFFECT_SUCCESS_SPEED, true};  // LED 1,2
  sequence->steps[2] = {0b000111, EFFECT_SUCCESS_SPEED, true};  // LED 1,2,3
  sequence->steps[3] = {0b001111, EFFECT_SUCCESS_SPEED, true};  // LED 1,2,3,4
  sequence->steps[4] = {0b011111, EFFECT_SUCCESS_SPEED, true};  // LED 1,2,3,4,5
  sequence->steps[5] = {0b111111, EFFECT_SUCCESS_SPEED, true};  // 전체 점등
}

/**
 * ERROR 패턴 데이터 생성
 * 요구사항 8.4: 플레이어가 잘못된 행동을 하면 명확한 부정적인 피드백 제공
 */
void createErrorPattern(EffectSequence* sequence) {
  sequence->stepCount = 6;
  sequence->repeatCount = 5;
  sequence->isLooping = false;
  
  // 빠른 전체 점멸
  sequence->steps[0] = {0b111111, EFFECT_ERROR_SPEED, true};   // 전체 ON
  sequence->steps[1] = {0b000000, EFFECT_ERROR_SPEED, false};  // 전체 OFF
  sequence->steps[2] = {0b111111, EFFECT_ERROR_SPEED, true};   // 전체 ON
  sequence->steps[3] = {0b000000, EFFECT_ERROR_SPEED, false};  // 전체 OFF
  sequence->steps[4] = {0b111111, EFFECT_ERROR_SPEED, true};   // 전체 ON
  sequence->steps[5] = {0b000000, EFFECT_ERROR_SPEED, false};  // 전체 OFF
}

/**
 * LEVEL_UP 패턴 데이터 생성
 * 요구사항 8.1: 게임의 각 단계가 진행되면 적절한 시각적 피드백 제공
 */
void createLevelUpPattern(EffectSequence* sequence) {
  sequence->stepCount = 8;
  sequence->repeatCount = 1;
  sequence->isLooping = false;
  
  // 웨이브 패턴 (좌->우->좌)
  sequence->steps[0] = {0b000001, EFFECT_LEVELUP_SPEED, true};  // LED 1
  sequence->steps[1] = {0b000010, EFFECT_LEVELUP_SPEED, true};  // LED 2
  sequence->steps[2] = {0b000100, EFFECT_LEVELUP_SPEED, true};  // LED 3
  sequence->steps[3] = {0b001000, EFFECT_LEVELUP_SPEED, true};  // LED 4
  sequence->steps[4] = {0b010000, EFFECT_LEVELUP_SPEED, true};  // LED 5
  sequence->steps[5] = {0b100000, EFFECT_LEVELUP_SPEED, true};  // LED 6
  sequence->steps[6] = {0b010000, EFFECT_LEVELUP_SPEED, true};  // LED 5
  sequence->steps[7] = {0b000000, EFFECT_LEVELUP_SPEED, false}; // 전체 OFF
}

/**
 * GAME_START 패턴 데이터 생성
 * 요구사항 1.2, 1.3: 게임 시작 시 시각적 피드백 제공
 */
void createGameStartPattern(EffectSequence* sequence) {
  sequence->stepCount = 4;
  sequence->repeatCount = 3;
  sequence->isLooping = false;
  
  // 중앙에서 외곽으로 확산
  sequence->steps[0] = {0b001100, EFFECT_START_SPEED, true};   // LED 3,4 (중앙)
  sequence->steps[1] = {0b010010, EFFECT_START_SPEED, true};   // LED 2,5
  sequence->steps[2] = {0b100001, EFFECT_START_SPEED, true};   // LED 1,6 (외곽)
  sequence->steps[3] = {0b000000, EFFECT_START_SPEED, false};  // 전체 OFF
}

/**
 * GAME_CLEAR 패턴 데이터 생성
 * 요구사항 6.2: 클리어 효과가 시작되면 이펙트 LED가 특별한 패턴으로 연출
 */
void createGameClearPattern(EffectSequence* sequence) {
  sequence->stepCount = 10;
  sequence->repeatCount = 1;
  sequence->isLooping = true; // 클리어 시까지 무한 반복
  
  // 화려한 순환 패턴
  sequence->steps[0] = {0b111111, 300, true};   // 전체 점등
  sequence->steps[1] = {0b000000, 200, false};  // 전체 소등
  sequence->steps[2] = {0b101010, 200, true};   // 1,3,5 점등
  sequence->steps[3] = {0b010101, 200, true};   // 2,4,6 점등
  sequence->steps[4] = {0b111111, 200, true};   // 전체 점등
  sequence->steps[5] = {0b000000, 100, false};  // 전체 소등
  sequence->steps[6] = {0b100001, 150, true};   // 1,6 점등
  sequence->steps[7] = {0b010010, 150, true};   // 2,5 점등
  sequence->steps[8] = {0b001100, 150, true};   // 3,4 점등
  sequence->steps[9] = {0b000000, 300, false};  // 전체 소등
}

/**
 * TENSION 패턴 데이터 생성
 * 요구사항 3.4: 움직임이 감지되면 이펙트 LED가 점멸
 */
void createTensionPattern(EffectSequence* sequence) {
  sequence->stepCount = 4;
  sequence->repeatCount = 1;
  sequence->isLooping = true; // 동작 감지 중 계속 반복
  
  // 심장박동 패턴
  sequence->steps[0] = {0b111111, 100, true};   // 빠른 점등
  sequence->steps[1] = {0b000000, 100, false};  // 빠른 소등
  sequence->steps[2] = {0b111111, 100, true};   // 빠른 점등
  sequence->steps[3] = {0b000000, EFFECT_TENSION_BEAT - 300, false}; // 긴 소등
}

/**
 * 패턴 데이터 로드
 * @param pattern 로드할 패턴 타입
 * @param sequence 패턴 데이터를 저장할 시퀀스
 */
void loadPatternData(EffectPattern pattern, EffectSequence* sequence) {
  switch (pattern) {
    case SUCCESS_PATTERN:
      createSuccessPattern(sequence);
      break;
      
    case ERROR_PATTERN:
      createErrorPattern(sequence);
      break;
      
    case LEVEL_UP_PATTERN:
      createLevelUpPattern(sequence);
      break;
      
    case GAME_START_PATTERN:
      createGameStartPattern(sequence);
      break;
      
    case GAME_CLEAR_PATTERN:
      createGameClearPattern(sequence);
      break;
      
    case TENSION_PATTERN:
      createTensionPattern(sequence);
      break;
      
    default:
      Serial.print("알 수 없는 패턴: ");
      Serial.println(pattern);
      // 기본 패턴 (전체 점멸)
      sequence->stepCount = 2;
      sequence->repeatCount = 3;
      sequence->isLooping = false;
      sequence->steps[0] = {0b111111, 500, true};
      sequence->steps[1] = {0b000000, 500, false};
      break;
  }
}

/**
 * 이펙트 패턴 재생 시작
 * @param pattern 재생할 패턴
 * @return 재생 시작 성공 여부
 */
bool playEffectPattern(EffectPattern pattern) {
  // 현재 재생 중인 패턴이 있으면 정지
  if (patternEngine.engineActive) {
    Serial.println("기존 패턴 정지 후 새 패턴 시작");
    stopEffectPattern();
  }
  
  Serial.print("이펙트 패턴 재생 시작: ");
  Serial.println(getPatternName(pattern));
  
  // 패턴 데이터 로드
  loadPatternData(pattern, &patternEngine.currentSequence);
  
  // 엔진 상태 초기화
  patternEngine.activePattern = pattern;
  patternEngine.currentStep = 0;
  patternEngine.currentRepeat = 0;
  patternEngine.stepTimer = millis();
  patternEngine.engineActive = true;
  
  // 첫 번째 단계 즉시 적용
  applyEffectStep(&patternEngine.currentSequence.steps[0]);
  
  Serial.print("패턴 단계 수: ");
  Serial.print(patternEngine.currentSequence.stepCount);
  Serial.print(", 반복 횟수: ");
  Serial.print(patternEngine.currentSequence.repeatCount);
  Serial.print(", 무한 반복: ");
  Serial.println(patternEngine.currentSequence.isLooping ? "예" : "아니오");
  
  return true;
}

/**
 * 이펙트 패턴 정지
 */
void stopEffectPattern() {
  if (!patternEngine.engineActive) {
    return;
  }
  
  Serial.print("이펙트 패턴 정지: ");
  Serial.println(getPatternName(patternEngine.activePattern));
  
  // 엔진 비활성화
  patternEngine.engineActive = false;
  
  // 모든 이펙트 LED 소등
  setEffectLEDMask(0b000000);
  
  Serial.println("이펙트 패턴 정지 완료");
}

/**
 * 패턴 엔진 업데이트 (메인 루프에서 호출)
 * 비동기 패턴 실행 및 타이머 관리
 */
void updatePatternEngine() {
  if (!patternEngine.engineActive) {
    return;
  }
  
  unsigned long currentTime = millis();
  EffectSequence* sequence = &patternEngine.currentSequence;
  
  // 현재 단계의 지속 시간 확인
  if (currentTime - patternEngine.stepTimer >= sequence->steps[patternEngine.currentStep].duration) {
    // 다음 단계로 이동
    patternEngine.currentStep++;
    
    // 시퀀스 완료 확인
    if (patternEngine.currentStep >= sequence->stepCount) {
      patternEngine.currentStep = 0;
      patternEngine.currentRepeat++;
      
      // 반복 완료 확인
      if (!sequence->isLooping && patternEngine.currentRepeat >= sequence->repeatCount) {
        // 패턴 완료
        Serial.print("패턴 완료: ");
        Serial.println(getPatternName(patternEngine.activePattern));
        
        stopEffectPattern();
        return;
      }
      
      Serial.print("패턴 반복: ");
      Serial.print(patternEngine.currentRepeat + 1);
      if (!sequence->isLooping) {
        Serial.print("/");
        Serial.print(sequence->repeatCount);
      } else {
        Serial.print(" (무한)");
      }
      Serial.println();
    }
    
    // 새 단계 적용
    applyEffectStep(&sequence->steps[patternEngine.currentStep]);
    patternEngine.stepTimer = currentTime;
  }
}

/**
 * 이펙트 단계 적용
 * @param step 적용할 단계
 */
void applyEffectStep(const EffectStep* step) {
  if (step->isOn) {
    setEffectLEDMask(step->ledMask);
  } else {
    setEffectLEDMask(0b000000); // 모든 LED 소등
  }
  
  Serial.print("이펙트 단계 적용: 마스크=0b");
  Serial.print(step->ledMask, BIN);
  Serial.print(", 상태=");
  Serial.print(step->isOn ? "ON" : "OFF");
  Serial.print(", 지속시간=");
  Serial.print(step->duration);
  Serial.println("ms");
}

/**
 * 이펙트 LED 마스크 설정
 * @param mask LED 점등 마스크 (비트별 제어)
 */
void setEffectLEDMask(uint8_t mask) {
  effectLEDState.currentMask = mask;
  
  // 각 LED 개별 제어
  for (int i = 0; i < 6; i++) {
    bool ledState = (mask & (1 << i)) != 0;
    setEffectLED(i, ledState);
  }
  
  effectLEDState.lastUpdateTime = millis();
}

/**
 * 개별 이펙트 LED 제어
 * @param ledIndex LED 인덱스 (0-5)
 * @param state LED 상태 (true: ON, false: OFF)
 */
void setEffectLED(uint8_t ledIndex, bool state) {
  if (ledIndex >= 6) {
    Serial.print("잘못된 이펙트 LED 인덱스: ");
    Serial.println(ledIndex);
    return;
  }
  
  // 상태 변경이 있을 때만 처리
  if (effectLEDState.ledState[ledIndex] != state) {
    effectLEDState.ledState[ledIndex] = state;
    
    // 릴레이 제어 (relSig3~8)
    uint8_t relayPin = EFFECT_LED_PINS[ledIndex];
    digitalWrite(relayPin, state ? HIGH : LOW);
    
    Serial.print("이펙트 LED ");
    Serial.print(ledIndex + 1);
    Serial.print(" -> ");
    Serial.println(state ? "ON" : "OFF");
  }
}

/**
 * 패턴 엔진 활성 상태 확인
 * @return 엔진 활성 여부
 */
bool isPatternEngineActive() {
  return patternEngine.engineActive;
}

/**
 * 현재 재생 중인 패턴 확인
 * @return 현재 패턴 (엔진이 비활성이면 0)
 */
EffectPattern getCurrentPattern() {
  return patternEngine.engineActive ? patternEngine.activePattern : (EffectPattern)0;
}

/**
 * 패턴 중단 및 전환
 * @param newPattern 새로 재생할 패턴
 */
void switchEffectPattern(EffectPattern newPattern) {
  Serial.print("패턴 전환: ");
  Serial.print(getPatternName(patternEngine.activePattern));
  Serial.print(" -> ");
  Serial.println(getPatternName(newPattern));
  
  stopEffectPattern();
  delay(50); // 짧은 대기 후 새 패턴 시작
  playEffectPattern(newPattern);
}

/**
 * 패턴 이름 반환 (디버깅용)
 * @param pattern 패턴 타입
 * @return 패턴 이름 문자열
 */
const char* getPatternName(EffectPattern pattern) {
  switch (pattern) {
    case SUCCESS_PATTERN: return "SUCCESS";
    case ERROR_PATTERN: return "ERROR";
    case LEVEL_UP_PATTERN: return "LEVEL_UP";
    case GAME_START_PATTERN: return "GAME_START";
    case GAME_CLEAR_PATTERN: return "GAME_CLEAR";
    case TENSION_PATTERN: return "TENSION";
    default: return "UNKNOWN";
  }
}

// =============================================================================
// 게임 상황별 이펙트 패턴 재생 함수들
// =============================================================================

/**
 * 정답 입력 시 성공 이펙트 재생
 * 요구사항 8.3: 플레이어가 올바른 행동을 하면 긍정적인 피드백 제공
 */
void playSuccessEffect() {
  Serial.println("정답 입력 - 성공 이펙트 재생");
  playEffectPattern(SUCCESS_PATTERN);
}

/**
 * 오답 입력 시 오류 이펙트 재생
 * 요구사항 8.4: 플레이어가 잘못된 행동을 하면 명확한 부정적인 피드백 제공
 */
void playErrorEffect() {
  Serial.println("오답 입력 - 오류 이펙트 재생");
  playEffectPattern(ERROR_PATTERN);
}

/**
 * 레벨 상승 시 레벨업 이펙트 재생
 * 요구사항 8.1: 게임의 각 단계가 진행되면 적절한 시각적 피드백 제공
 */
void playLevelUpEffect() {
  Serial.println("레벨 상승 - 레벨업 이펙트 재생");
  playEffectPattern(LEVEL_UP_PATTERN);
}

/**
 * 게임 시작 시 시작 이펙트 재생
 * 요구사항 1.2, 1.3: 게임 시작 시 시각적 피드백 제공
 */
void playGameStartEffect() {
  Serial.println("게임 시작 - 시작 이펙트 재생");
  playEffectPattern(GAME_START_PATTERN);
}

/**
 * 게임 클리어 시 클리어 이펙트 재생
 * 요구사항 6.2: 클리어 효과가 시작되면 이펙트 LED가 특별한 패턴으로 연출
 */
void playGameClearEffect() {
  Serial.println("게임 클리어 - 클리어 이펙트 재생");
  playEffectPattern(GAME_CLEAR_PATTERN);
}

/**
 * 동작 감지 중 긴장감 이펙트 재생
 * 요구사항 3.4: 움직임이 감지되면 이펙트 LED가 점멸
 */
void playTensionEffect() {
  Serial.println("동작 감지 중 - 긴장감 이펙트 재생");
  playEffectPattern(TENSION_PATTERN);
}

/**
 * 동작 감지 완료 시 긴장감 이펙트 정지
 */
void stopTensionEffect() {
  if (getCurrentPattern() == TENSION_PATTERN) {
    Serial.println("동작 감지 완료 - 긴장감 이펙트 정지");
    stopEffectPattern();
  }
}

/**
 * 모든 이펙트 정지 (비상 정지)
 */
void stopAllEffects() {
  Serial.println("모든 이펙트 정지");
  stopEffectPattern();
}

// =============================================================================
// DFPlayer 오디오 시스템 관련 함수들
// =============================================================================

/**
 * DFPlayer Mini 초기화 및 제어 함수
 * 요구사항 1.5, 6.3, 7.4: 내레이션 및 BGM 재생 기능
 */
void initializeDFPlayer() {
  Serial.println("DFPlayer Mini 초기화 시작...");
  
  // DFPlayer 상태 초기화
  dfPlayerState.isInitialized = false;
  dfPlayerState.isPlaying = false;
  dfPlayerState.currentTrack = 0;
  dfPlayerState.currentVolume = 20;
  dfPlayerState.playStartTime = 0;
  dfPlayerState.lastCommandTime = 0;
  dfPlayerState.waitingForResponse = false;
  
  // DFPlayer 초기화 대기 시간
  delay(1000);
  
  // 볼륨 설정 (0-30, 기본값 20)
  if (sendDFPlayerCommand(DF_CMD_VOLUME, 0, dfPlayerState.currentVolume)) {
    Serial.print("DFPlayer 볼륨 설정: ");
    Serial.println(dfPlayerState.currentVolume);
    delay(200);
    
    // 초기화 완료 테스트 (짧은 무음 트랙 재생)
    if (testDFPlayerConnection()) {
      dfPlayerState.isInitialized = true;
      Serial.println("DFPlayer Mini 초기화 완료");
    } else {
      Serial.println("DFPlayer Mini 초기화 실패 - 연결 확인 필요");
    }
  } else {
    Serial.println("DFPlayer Mini 볼륨 설정 실패");
  }
  
  Serial.print("DFPlayer 초기화 상태: ");
  Serial.println(dfPlayerState.isInitialized ? "성공" : "실패");
}

/**
 * DFPlayer 명령 전송 함수
 * @param command 명령어
 * @param dataHigh 상위 데이터 바이트
 * @param dataLow 하위 데이터 바이트
 * @return 명령 전송 성공 여부
 */
bool sendDFPlayerCommand(uint8_t command, uint8_t dataHigh, uint8_t dataLow) {
  if (!dfSerial) {
    Serial.println("DFPlayer 시리얼 연결 오류");
    return false;
  }
  
  // DFPlayer 명령 패킷 구성 (10바이트)
  uint8_t packet[10] = {
    0x7E,           // Start byte
    0xFF,           // Version
    0x06,           // Length
    command,        // Command
    0x00,           // Feedback (no feedback)
    dataHigh,       // Data high byte
    dataLow,        // Data low byte
    0x00,           // Checksum high (계산됨)
    0x00,           // Checksum low (계산됨)
    0xEF            // End byte
  };
  
  // 체크섬 계산
  uint16_t checksum = 0;
  for (int i = 1; i < 7; i++) {
    checksum += packet[i];
  }
  checksum = 0 - checksum;
  
  packet[7] = (checksum >> 8) & 0xFF;  // Checksum high
  packet[8] = checksum & 0xFF;         // Checksum low
  
  // 명령 전송
  dfSerial.write(packet, 10);
  dfPlayerState.lastCommandTime = millis();
  
  Serial.print("DFPlayer 명령 전송: CMD=0x");
  Serial.print(command, HEX);
  Serial.print(", Data=");
  Serial.print(dataHigh);
  Serial.print(",");
  Serial.println(dataLow);
  
  return true;
}

/**
 * DFPlayer 연결 테스트
 * @return 연결 상태
 */
bool testDFPlayerConnection() {
  Serial.println("DFPlayer 연결 테스트 중...");
  
  // 현재 상태 쿼리 명령 전송
  if (sendDFPlayerCommand(0x42, 0, 0)) { // Query current status
    delay(100);
    
    // 응답 확인 (간단한 테스트)
    if (dfSerial.available()) {
      Serial.println("DFPlayer 응답 수신 - 연결 정상");
      // 응답 데이터 클리어
      while (dfSerial.available()) {
        dfSerial.read();
      }
      return true;
    } else {
      Serial.println("DFPlayer 응답 없음 - 연결 확인 필요");
      return false;
    }
  }
  
  return false;
}

/**
 * 오디오 트랙 재생
 * @param trackNumber 재생할 트랙 번호 (1-255)
 * @return 재생 명령 성공 여부
 */
bool playAudioTrack(uint8_t trackNumber) {
  if (!dfPlayerState.isInitialized) {
    Serial.println("DFPlayer 초기화되지 않음 - 재생 불가");
    return false;
  }
  
  if (trackNumber == 0) {
    Serial.println("잘못된 트랙 번호: 0");
    return false;
  }
  
  Serial.print("오디오 트랙 재생: ");
  Serial.println(trackNumber);
  
  // 현재 재생 중인 트랙이 있으면 정지
  if (dfPlayerState.isPlaying) {
    stopAudio();
    delay(100);
  }
  
  // 트랙 재생 명령 전송
  if (sendDFPlayerCommand(DF_CMD_PLAY_TRACK, 0, trackNumber)) {
    dfPlayerState.isPlaying = true;
    dfPlayerState.currentTrack = trackNumber;
    dfPlayerState.playStartTime = millis();
    
    Serial.print("트랙 ");
    Serial.print(trackNumber);
    Serial.println(" 재생 시작");
    return true;
  } else {
    Serial.println("트랙 재생 명령 전송 실패");
    return false;
  }
}

/**
 * 오디오 재생 정지
 * @return 정지 명령 성공 여부
 */
bool stopAudio() {
  if (!dfPlayerState.isInitialized) {
    Serial.println("DFPlayer 초기화되지 않음");
    return false;
  }
  
  Serial.println("오디오 재생 정지");
  
  if (sendDFPlayerCommand(DF_CMD_STOP, 0, 0)) {
    dfPlayerState.isPlaying = false;
    dfPlayerState.currentTrack = 0;
    dfPlayerState.playStartTime = 0;
    
    Serial.println("오디오 정지 완료");
    return true;
  } else {
    Serial.println("오디오 정지 명령 전송 실패");
    return false;
  }
}

/**
 * 볼륨 제어
 * @param volume 볼륨 레벨 (0-30)
 * @return 볼륨 설정 성공 여부
 */
bool setVolume(uint8_t volume) {
  if (!dfPlayerState.isInitialized) {
    Serial.println("DFPlayer 초기화되지 않음");
    return false;
  }
  
  if (volume > 30) {
    volume = 30;
    Serial.println("볼륨 값 제한: 최대 30으로 설정");
  }
  
  Serial.print("볼륨 설정: ");
  Serial.println(volume);
  
  if (sendDFPlayerCommand(DF_CMD_VOLUME, 0, volume)) {
    dfPlayerState.currentVolume = volume;
    Serial.print("볼륨 변경 완료: ");
    Serial.println(volume);
    return true;
  } else {
    Serial.println("볼륨 설정 실패");
    return false;
  }
}

/**
 * 환영 내레이션 재생
 * 요구사항 1.5: 환영 내레이션이 재생되어야 한다
 */
void playWelcomeNarration() {
  Serial.println("환영 내레이션 재생");
  playAudioTrack(AUDIO_WELCOME);
}

/**
 * 게임 시작 안내 재생
 */
void playGameStartNarration() {
  Serial.println("게임 시작 안내 재생");
  playAudioTrack(AUDIO_GAME_START);
}

/**
 * 게임 클리어 축하 메시지 재생
 * 요구사항 6.3: 클리어 효과가 완료되면 축하 오디오 연출
 */
void playGameClearNarration() {
  Serial.println("게임 클리어 축하 메시지 재생");
  playAudioTrack(AUDIO_GAME_CLEAR);
}

/**
 * 메인 BGM 재생
 */
void playMainBGM() {
  Serial.println("메인 BGM 재생");
  playAudioTrack(AUDIO_BGM_MAIN);
}

/**
 * 긴장감 BGM 재생 (동작 감지 중)
 */
void playTensionBGM() {
  Serial.println("긴장감 BGM 재생");
  playAudioTrack(AUDIO_BGM_TENSION);
}

/**
 * 천정 조명 효과음 재생
 * @param lightIndex 조명 인덱스 (0-3)
 * 요구사항 1.3: 각 조명이 점등될 때 해당하는 효과음을 재생해야 한다
 */
void playCeilingLightSound(uint8_t lightIndex) {
  if (lightIndex >= 4) {
    Serial.print("잘못된 조명 인덱스: ");
    Serial.println(lightIndex);
    return;
  }
  
  uint8_t soundTrack = AUDIO_LIGHT_1 + lightIndex; // 10, 11, 12, 13
  Serial.print("천정 조명 ");
  Serial.print(lightIndex + 1);
  Serial.println(" 효과음 재생");
  
  playAudioTrack(soundTrack);
}

/**
 * DFPlayer 재생 상태 확인
 * @return 현재 재생 중 여부
 */
bool isAudioPlaying() {
  return dfPlayerState.isPlaying;
}

/**
 * 현재 재생 중인 트랙 번호 반환
 * @return 트랙 번호 (재생 중이 아니면 0)
 */
uint8_t getCurrentTrack() {
  return dfPlayerState.isPlaying ? dfPlayerState.currentTrack : 0;
}

/**
 * 재생 시간 확인
 * @return 현재 트랙 재생 시간 (ms)
 */
unsigned long getPlayTime() {
  if (!dfPlayerState.isPlaying) {
    return 0;
  }
  return millis() - dfPlayerState.playStartTime;
}

/**
 * DFPlayer 에러 처리 및 복구
 * 요구사항 7.4: 재생 상태 확인 및 에러 처리
 */
void handleDFPlayerError() {
  Serial.println("DFPlayer 오류 감지 - 복구 시도");
  
  // 현재 상태 리셋
  dfPlayerState.isPlaying = false;
  dfPlayerState.currentTrack = 0;
  dfPlayerState.waitingForResponse = false;
  
  // 재초기화 시도
  delay(500);
  initializeDFPlayer();
  
  if (dfPlayerState.isInitialized) {
    Serial.println("DFPlayer 복구 성공");
  } else {
    Serial.println("DFPlayer 복구 실패 - 하드웨어 점검 필요");
  }
}

/**
 * DFPlayer 상태 모니터링 (메인 루프에서 호출)
 * 재생 상태 확인 및 에러 처리
 */
void updateDFPlayerStatus() {
  static unsigned long lastStatusCheck = 0;
  const unsigned long STATUS_CHECK_INTERVAL = 5000; // 5초마다 상태 확인
  
  unsigned long currentTime = millis();
  
  // 주기적 상태 확인
  if (currentTime - lastStatusCheck >= STATUS_CHECK_INTERVAL) {
    lastStatusCheck = currentTime;
    
    // 초기화 상태 확인
    if (!dfPlayerState.isInitialized) {
      Serial.println("DFPlayer 미초기화 상태 - 재초기화 시도");
      initializeDFPlayer();
      return;
    }
    
    // 응답 대기 타임아웃 확인
    if (dfPlayerState.waitingForResponse) {
      if (currentTime - dfPlayerState.lastCommandTime > 3000) { // 3초 타임아웃
        Serial.println("DFPlayer 응답 타임아웃");
        dfPlayerState.waitingForResponse = false;
        handleDFPlayerError();
      }
    }
  }
  
  // DFPlayer 응답 처리
  processDFPlayerResponse();
}

/**
 * DFPlayer 응답 처리
 * SD카드 상태, 재생 완료 등의 응답 처리
 */
void processDFPlayerResponse() {
  if (!dfSerial.available()) {
    return;
  }
  
  // 응답 데이터 읽기 (10바이트 패킷)
  uint8_t responseBuffer[10];
  int bytesRead = 0;
  
  unsigned long startTime = millis();
  while (bytesRead < 10 && (millis() - startTime) < 100) { // 100ms 타임아웃
    if (dfSerial.available()) {
      responseBuffer[bytesRead] = dfSerial.read();
      bytesRead++;
    }
  }
  
  if (bytesRead < 10) {
    Serial.print("DFPlayer 응답 불완전: ");
    Serial.print(bytesRead);
    Serial.println(" 바이트");
    return;
  }
  
  // 응답 패킷 검증
  if (responseBuffer[0] != 0x7E || responseBuffer[9] != 0xEF) {
    Serial.println("DFPlayer 응답 패킷 오류");
    return;
  }
  
  uint8_t command = responseBuffer[3];
  uint8_t dataHigh = responseBuffer[5];
  uint8_t dataLow = responseBuffer[6];
  
  // 응답 처리
  switch (command) {
    case 0x3D: // 재생 완료
      Serial.print("트랙 재생 완료: ");
      Serial.println(dfPlayerState.currentTrack);
      dfPlayerState.isPlaying = false;
      dfPlayerState.currentTrack = 0;
      break;
      
    case 0x3A: // SD카드 삽입
      Serial.println("SD카드 감지됨");
      break;
      
    case 0x3B: // SD카드 제거
      Serial.println("SD카드 제거됨 - 오디오 기능 비활성화");
      dfPlayerState.isInitialized = false;
      break;
      
    case 0x40: // 에러 응답
      Serial.print("DFPlayer 에러: 0x");
      Serial.println(dataLow, HEX);
      handleDFPlayerError();
      break;
      
    case 0x41: // ACK 응답
      dfPlayerState.waitingForResponse = false;
      break;
      
    default:
      Serial.print("알 수 없는 DFPlayer 응답: 0x");
      Serial.println(command, HEX);
      break;
  }
}

/**
 * 오디오 트랙 관리 - 트랙 존재 여부 확인
 * @param trackNumber 확인할 트랙 번호
 * @return 트랙 존재 여부
 */
bool isTrackAvailable(uint8_t trackNumber) {
  if (!dfPlayerState.isInitialized) {
    return false;
  }
  
  // 트랙 번호 유효성 확인
  if (trackNumber == 0 || trackNumber > 255) {
    return false;
  }
  
  // 기본 트랙들의 존재 여부 (실제 구현에서는 SD카드 쿼리 필요)
  switch (trackNumber) {
    case AUDIO_WELCOME:
    case AUDIO_GAME_START:
    case AUDIO_GAME_CLEAR:
    case AUDIO_BGM_MAIN:
    case AUDIO_BGM_TENSION:
    case AUDIO_LIGHT_1:
    case AUDIO_LIGHT_2:
    case AUDIO_LIGHT_3:
    case AUDIO_LIGHT_4:
      return true;
    default:
      return false;
  }
}

/**
 * 볼륨 증가
 * @param increment 증가량 (기본값: 1)
 */
void volumeUp(uint8_t increment = 1) {
  uint8_t newVolume = dfPlayerState.currentVolume + increment;
  if (newVolume > 30) {
    newVolume = 30;
  }
  setVolume(newVolume);
}

/**
 * 볼륨 감소
 * @param decrement 감소량 (기본값: 1)
 */
void volumeDown(uint8_t decrement = 1) {
  uint8_t newVolume = dfPlayerState.currentVolume;
  if (newVolume >= decrement) {
    newVolume -= decrement;
  } else {
    newVolume = 0;
  }
  setVolume(newVolume);
}

/**
 * 현재 볼륨 반환
 * @return 현재 볼륨 레벨 (0-30)
 */
uint8_t getCurrentVolume() {
  return dfPlayerState.currentVolume;
}

/**
 * 오디오 시스템 상태 출력 (디버깅용)
 */
void printAudioStatus() {
  Serial.println("=== DFPlayer 상태 ===");
  Serial.print("초기화: ");
  Serial.println(dfPlayerState.isInitialized ? "완료" : "실패");
  Serial.print("재생 중: ");
  Serial.println(dfPlayerState.isPlaying ? "예" : "아니오");
  Serial.print("현재 트랙: ");
  Serial.println(dfPlayerState.currentTrack);
  Serial.print("볼륨: ");
  Serial.println(dfPlayerState.currentVolume);
  if (dfPlayerState.isPlaying) {
    Serial.print("재생 시간: ");
    Serial.print(getPlayTime());
    Serial.println("ms");
  }
  Serial.println("==================");
}

// =============================================================================
// 시리얼 통신 관련 함수들 (Master-Slave 통신)
// =============================================================================

/**
 * 통신 시스템 초기화
 * 요구사항 7.3: Master와 Slave 보드 간 명령을 정확히 전달
 */
void initComm() {
  commState.isConnected = false;
  commState.lastCommTime = 0;
  commState.retryCount = 0;
  commState.lastError = COMM_ERROR_NONE;
  
  Serial.println("시리얼 통신 시스템 초기화 완료");
  Serial.print("통신 속도: ");
  Serial.print(SERIAL_BAUD_RATE);
  Serial.println(" bps");
}

/**
 * 체크섬 계산
 * @param msg 메시지 구조체 포인터
 * @return 계산된 체크섬
 */
uint8_t calculateChecksum(const SerialMessage* msg) {
  return msg->header ^ msg->command ^ msg->data1 ^ msg->data2;
}

/**
 * 메시지 유효성 검증
 * @param msg 검증할 메시지
 * @return 유효성 여부
 */
bool validateMessage(const SerialMessage* msg) {
  // 헤더 확인
  if (msg->header != MESSAGE_HEADER) {
    Serial.println("통신 오류: 잘못된 헤더");
    return false;
  }
  
  // 체크섬 확인
  uint8_t expectedChecksum = calculateChecksum(msg);
  if (msg->checksum != expectedChecksum) {
    Serial.print("통신 오류: 체크섬 불일치 (예상: 0x");
    Serial.print(expectedChecksum, HEX);
    Serial.print(", 수신: 0x");
    Serial.print(msg->checksum, HEX);
    Serial.println(")");
    return false;
  }
  
  return true;
}

/**
 * 메시지 생성
 * @param msg 생성할 메시지 구조체
 * @param cmd 명령어
 * @param data1 데이터 1
 * @param data2 데이터 2
 */
void createMessage(SerialMessage* msg, uint8_t cmd, uint8_t data1, uint8_t data2) {
  msg->header = MESSAGE_HEADER;
  msg->command = cmd;
  msg->data1 = data1;
  msg->data2 = data2;
  msg->checksum = calculateChecksum(msg);
}

/**
 * 메시지 전송
 * @param msg 전송할 메시지
 * @return 전송 성공 여부
 */
bool sendMessage(const SerialMessage* msg) {
  if (!validateMessage(msg)) {
    Serial.println("메시지 전송 실패: 유효하지 않은 메시지");
    return false;
  }
  
  // 메시지 전송
  Serial.write((uint8_t*)msg, sizeof(SerialMessage));
  Serial.flush(); // 전송 완료 대기
  
  commState.lastCommTime = millis();
  
  Serial.print("메시지 전송: CMD=0x");
  Serial.print(msg->command, HEX);
  Serial.print(", DATA1=0x");
  Serial.print(msg->data1, HEX);
  Serial.print(", DATA2=0x");
  Serial.print(msg->data2, HEX);
  Serial.print(", CHECKSUM=0x");
  Serial.println(msg->checksum, HEX);
  
  return true;
}

/**
 * 명령 전송 (간편 함수)
 * @param cmd 명령어
 * @param data1 데이터 1 (기본값: 0)
 * @param data2 데이터 2 (기본값: 0)
 * @return 전송 성공 여부
 */
bool sendCommand(uint8_t cmd, uint8_t data1, uint8_t data2) {
  SerialMessage msg;
  createMessage(&msg, cmd, data1, data2);
  return sendMessage(&msg);
}

/**
 * 메시지 수신
 * @param msg 수신된 메시지를 저장할 구조체
 * @param timeout 타임아웃 시간 (ms)
 * @return 수신 성공 여부
 */
bool receiveMessage(SerialMessage* msg, unsigned long timeout) {
  unsigned long startTime = millis();
  uint8_t bytesReceived = 0;
  uint8_t* msgBytes = (uint8_t*)msg;
  
  // 타임아웃까지 메시지 수신 대기
  while (millis() - startTime < timeout) {
    if (Serial.available()) {
      msgBytes[bytesReceived] = Serial.read();
      bytesReceived++;
      
      // 완전한 메시지 수신 완료
      if (bytesReceived >= sizeof(SerialMessage)) {
        commState.lastCommTime = millis();
        
        // 메시지 유효성 검증
        if (validateMessage(msg)) {
          Serial.print("메시지 수신: CMD=0x");
          Serial.print(msg->command, HEX);
          Serial.print(", DATA1=0x");
          Serial.print(msg->data1, HEX);
          Serial.print(", DATA2=0x");
          Serial.println(msg->data2, HEX);
          
          commState.lastError = COMM_ERROR_NONE;
          return true;
        } else {
          Serial.println("메시지 수신 실패: 유효성 검증 실패");
          commState.lastError = COMM_ERROR_CHECKSUM;
          return false;
        }
      }
    }
    
    delay(1); // CPU 부하 감소
  }
  
  // 타임아웃 발생
  Serial.print("메시지 수신 타임아웃 (");
  Serial.print(timeout);
  Serial.print("ms), 수신된 바이트: ");
  Serial.println(bytesReceived);
  
  commState.lastError = COMM_ERROR_TIMEOUT;
  return false;
}

/**
 * 특정 응답 대기
 * @param expectedResponse 예상되는 응답 코드
 * @param timeout 타임아웃 시간 (ms)
 * @return 예상 응답 수신 여부
 */
bool waitForResponse(uint8_t expectedResponse, unsigned long timeout) {
  SerialMessage response;
  
  if (receiveMessage(&response, timeout)) {
    if (response.command == expectedResponse) {
      Serial.print("예상 응답 수신: 0x");
      Serial.println(expectedResponse, HEX);
      return true;
    } else {
      Serial.print("예상하지 않은 응답: 0x");
      Serial.print(response.command, HEX);
      Serial.print(" (예상: 0x");
      Serial.print(expectedResponse, HEX);
      Serial.println(")");
      return false;
    }
  }
  
  return false;
}

/**
 * Slave와의 통신 처리
 */
void processSlaveComm() {
  // 수신된 메시지 처리
  if (Serial.available() >= sizeof(SerialMessage)) {
    if (receiveMessage(&incomingMessage)) {
      handleSlaveResponse(&incomingMessage);
    }
  }
  
  // 통신 상태 업데이트
  updateCommState();
}

// =============================================================================
// 이벤트 핸들러 함수들 (기본 구현)
// =============================================================================

void onRobotMovementComplete(uint8_t robotPart) {
  Serial.print("로봇 움직임 완료: ");
  Serial.println(robotPart == 0 ? "HEAD" : "BODY");
}

void handleRobotTimeout(uint8_t robotPart) {
  Serial.print("로봇 제어 타임아웃: ");
  Serial.println(robotPart == 0 ? "HEAD" : "BODY");
  playErrorEffect();
}

void handleRobotError(uint8_t robotPart, uint8_t errorCode) {
  Serial.print("로봇 제어 오류: ");
  Serial.print(robotPart == 0 ? "HEAD" : "BODY");
  Serial.print(", 오류 코드: 0x");
  Serial.println(errorCode, HEX);
  playErrorEffect();
}

void onPatternDisplayComplete() {
  Serial.println("패턴 표시 완료 - 플레이어 입력 대기");
  gameState.gamePhase = PHASE_PLAYER_INPUT;
  gameState.phaseTimer = millis();
}

void handlePatternError(uint8_t errorCode) {
  Serial.print("패턴 표시 오류: 0x");
  Serial.println(errorCode, HEX);
  playErrorEffect();
}

void onCorrectInput() {
  Serial.println("올바른 입력 - 점수 증가");
  playSuccessEffect();
  
  gameState.currentScore++;
  if (gameState.currentScore >= MAX_SCORE) {
    Serial.println("7점 달성 - 게임 클리어!");
    gameState.gamePhase = PHASE_GAME_CLEAR;
    playGameClearEffect();
  } else {
    gameState.currentLevel++;
    if (gameState.currentLevel > MAX_LEVEL) {
      gameState.currentLevel = 1;
    }
    Serial.print("레벨 상승: ");
    Serial.println(gameState.currentLevel);
  }
}

void onWrongInput() {
  Serial.println("잘못된 입력 - 점수 감소");
  playErrorEffect();
  
  if (gameState.currentScore > MIN_SCORE) {
    gameState.currentScore--;
    Serial.print("점수 감소: ");
    Serial.println(gameState.currentScore);
  } else {
    Serial.println("점수 0점 - 더 이상 감소하지 않음");
  }
}

// =============================================================================
// 디버깅 및 테스트 함수들
// =============================================================================

/**
 * 패턴 엔진 상태 진단 출력 (디버깅용)
 */
void printPatternEngineStatus() {
  Serial.println("=== 이펙트 패턴 엔진 상태 ===");
  
  Serial.print("엔진 활성: ");
  Serial.println(patternEngine.engineActive ? "예" : "아니오");
  
  if (patternEngine.engineActive) {
    Serial.print("현재 패턴: ");
    Serial.println(getPatternName(patternEngine.activePattern));
    
    Serial.print("현재 단계: ");
    Serial.print(patternEngine.currentStep + 1);
    Serial.print("/");
    Serial.println(patternEngine.currentSequence.stepCount);
    
    Serial.print("현재 반복: ");
    Serial.print(patternEngine.currentRepeat + 1);
    if (!patternEngine.currentSequence.isLooping) {
      Serial.print("/");
      Serial.print(patternEngine.currentSequence.repeatCount);
    } else {
      Serial.print(" (무한)");
    }
    Serial.println();
    
    Serial.print("단계 경과 시간: ");
    Serial.print(millis() - patternEngine.stepTimer);
    Serial.println("ms");
  }
  
  Serial.print("현재 LED 마스크: 0b");
  Serial.println(effectLEDState.currentMask, BIN);
  
  Serial.print("LED 상태: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(effectLEDState.ledState[i] ? "1" : "0");
  }
  Serial.println();
  
  Serial.println("============================");
}

/**
 * 시리얼 명령어 처리 (디버깅용)
 * 시리얼 모니터에서 명령어를 입력하여 패턴을 테스트할 수 있음
 */
void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    Serial.print("명령어 수신: ");
    Serial.println(command);
    
    if (command == "TEST") {
      validatePatternEngine();
    } else if (command == "SUCCESS") {
      playSuccessEffect();
    } else if (command == "ERROR") {
      playErrorEffect();
    } else if (command == "LEVELUP") {
      playLevelUpEffect();
    } else if (command == "START") {
      playGameStartEffect();
    } else if (command == "CLEAR") {
      playGameClearEffect();
    } else if (command == "TENSION") {
      playTensionEffect();
    } else if (command == "STOP") {
      stopAllEffects();
    } else if (command == "STATUS") {
      printPatternEngineStatus();
    } else if (command == "HELP") {
      Serial.println("사용 가능한 명령어:");
      Serial.println("TEST - 패턴 엔진 검증");
      Serial.println("SUCCESS - 성공 패턴");
      Serial.println("ERROR - 오류 패턴");
      Serial.println("LEVELUP - 레벨업 패턴");
      Serial.println("START - 게임 시작 패턴");
      Serial.println("CLEAR - 게임 클리어 패턴");
      Serial.println("TENSION - 긴장감 패턴");
      Serial.println("STOP - 모든 패턴 정지");
      Serial.println("STATUS - 엔진 상태 출력");
      Serial.println("HELP - 도움말");
    } else {
      Serial.println("알 수 없는 명령어. HELP를 입력하세요.");
    }
  }
}

/**
 * 이펙트 패턴 엔진 기능 검증
 * 모든 패턴의 정상 작동을 확인하는 통합 테스트
 */
void validatePatternEngine() {
  Serial.println("=== 이펙트 패턴 엔진 검증 시작 ===");
  
  // 1. 엔진 초기화 상태 확인
  Serial.println("1. 엔진 초기화 상태 확인");
  if (!patternEngine.engineActive) {
    Serial.println("✓ 엔진이 올바르게 비활성 상태로 초기화됨");
  } else {
    Serial.println("✗ 엔진 초기화 오류");
    return;
  }
  
  // 2. 개별 LED 제어 테스트
  Serial.println("2. 개별 LED 제어 테스트");
  for (int i = 0; i < 6; i++) {
    Serial.print("LED ");
    Serial.print(i + 1);
    Serial.print(" 테스트... ");
    
    setEffectLED(i, true);
    delay(200);
    setEffectLED(i, false);
    delay(100);
    
    Serial.println("완료");
  }
  
  // 3. LED 마스크 제어 테스트
  Serial.println("3. LED 마스크 제어 테스트");
  uint8_t testMasks[] = {0b000001, 0b000011, 0b000111, 0b001111, 0b011111, 0b111111};
  
  for (int i = 0; i < 6; i++) {
    Serial.print("마스크 0b");
    Serial.print(testMasks[i], BIN);
    Serial.print(" 테스트... ");
    
    setEffectLEDMask(testMasks[i]);
    delay(300);
    
    Serial.println("완료");
  }
  
  // 모든 LED 소등
  setEffectLEDMask(0b000000);
  delay(500);
  
  // 4. 패턴 데이터 생성 테스트
  Serial.println("4. 패턴 데이터 생성 테스트");
  EffectSequence testSequence;
  
  EffectPattern patterns[] = {SUCCESS_PATTERN, ERROR_PATTERN, LEVEL_UP_PATTERN, 
                             GAME_START_PATTERN, GAME_CLEAR_PATTERN, TENSION_PATTERN};
  
  for (int i = 0; i < 6; i++) {
    Serial.print("패턴 ");
    Serial.print(getPatternName(patterns[i]));
    Serial.print(" 데이터 생성... ");
    
    loadPatternData(patterns[i], &testSequence);
    
    if (testSequence.stepCount > 0 && testSequence.stepCount <= 10) {
      Serial.print("완료 (");
      Serial.print(testSequence.stepCount);
      Serial.print("단계, ");
      Serial.print(testSequence.repeatCount);
      Serial.print("회 반복, ");
      Serial.print(testSequence.isLooping ? "무한" : "유한");
      Serial.println(")");
    } else {
      Serial.println("오류 - 잘못된 단계 수");
    }
  }
  
  // 5. 짧은 패턴 재생 테스트
  Serial.println("5. 짧은 패턴 재생 테스트");
  
  Serial.println("SUCCESS 패턴 재생 (3초)");
  playEffectPattern(SUCCESS_PATTERN);
  
  unsigned long startTime = millis();
  while (isPatternEngineActive() && (millis() - startTime < 3000)) {
    updatePatternEngine();
    delay(50);
  }
  
  if (isPatternEngineActive()) {
    stopEffectPattern();
  }
  
  delay(500);
  
  Serial.println("ERROR 패턴 재생 (2초)");
  playEffectPattern(ERROR_PATTERN);
  
  startTime = millis();
  while (isPatternEngineActive() && (millis() - startTime < 2000)) {
    updatePatternEngine();
    delay(50);
  }
  
  if (isPatternEngineActive()) {
    stopEffectPattern();
  }
  
  // 6. 패턴 전환 테스트
  Serial.println("6. 패턴 전환 테스트");
  
  Serial.println("LEVEL_UP 패턴 시작");
  playEffectPattern(LEVEL_UP_PATTERN);
  delay(1000);
  
  Serial.println("SUCCESS 패턴으로 전환");
  switchEffectPattern(SUCCESS_PATTERN);
  delay(1500);
  
  Serial.println("패턴 정지");
  stopEffectPattern();
  
  // 7. 최종 상태 확인
  Serial.println("7. 최종 상태 확인");
  if (!isPatternEngineActive()) {
    Serial.println("✓ 엔진이 올바르게 정지됨");
  } else {
    Serial.println("✗ 엔진 정지 오류");
  }
  
  // 모든 LED 소등 확인
  bool allOff = true;
  for (int i = 0; i < 6; i++) {
    if (effectLEDState.ledState[i]) {
      allOff = false;
      break;
    }
  }
  
  if (allOff) {
    Serial.println("✓ 모든 LED가 올바르게 소등됨");
  } else {
    Serial.println("✗ 일부 LED가 여전히 점등 상태");
  }
  
  Serial.println("=== 이펙트 패턴 엔진 검증 완료 ===");
  Serial.println();
}

// =============================================================================
// 시리얼 명령어 처리 함수들 (디버깅 및 테스트용)
// =============================================================================

/**
 * 시리얼 명령어 처리 (디버깅용)
 * 시리얼 모니터를 통한 수동 테스트 지원
 */
void processSerialCommands() {
  if (!Serial.available()) {
    return;
  }
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();
  
  Serial.print("명령어 수신: ");
  Serial.println(command);
  
  // 오디오 관련 명령어
  if (command == "AUDIO_STATUS") {
    printAudioStatus();
  }
  else if (command == "PLAY_WELCOME") {
    playWelcomeNarration();
  }
  else if (command == "PLAY_START") {
    playGameStartNarration();
  }
  else if (command == "PLAY_CLEAR") {
    playGameClearNarration();
  }
  else if (command == "PLAY_BGM") {
    playMainBGM();
  }
  else if (command == "PLAY_TENSION") {
    playTensionBGM();
  }
  else if (command.startsWith("PLAY_LIGHT_")) {
    int lightIndex = command.substring(11).toInt() - 1; // PLAY_LIGHT_1 -> 0
    if (lightIndex >= 0 && lightIndex < 4) {
      playCeilingLightSound(lightIndex);
    } else {
      Serial.println("잘못된 조명 번호 (1-4)");
    }
  }
  else if (command.startsWith("PLAY_TRACK_")) {
    int trackNumber = command.substring(11).toInt();
    if (trackNumber > 0 && trackNumber <= 255) {
      playAudioTrack(trackNumber);
    } else {
      Serial.println("잘못된 트랙 번호 (1-255)");
    }
  }
  else if (command == "STOP_AUDIO") {
    stopAudio();
  }
  else if (command.startsWith("VOLUME_")) {
    int volume = command.substring(7).toInt();
    if (volume >= 0 && volume <= 30) {
      setVolume(volume);
    } else {
      Serial.println("잘못된 볼륨 값 (0-30)");
    }
  }
  else if (command == "VOLUME_UP") {
    volumeUp();
  }
  else if (command == "VOLUME_DOWN") {
    volumeDown();
  }
  
  // 이펙트 LED 관련 명령어
  else if (command == "EFFECT_SUCCESS") {
    playSuccessEffect();
  }
  else if (command == "EFFECT_ERROR") {
    playErrorEffect();
  }
  else if (command == "EFFECT_LEVELUP") {
    playLevelUpEffect();
  }
  else if (command == "EFFECT_START") {
    playGameStartEffect();
  }
  else if (command == "EFFECT_CLEAR") {
    playGameClearEffect();
  }
  else if (command == "EFFECT_TENSION") {
    playTensionEffect();
  }
  else if (command == "EFFECT_STOP") {
    stopAllEffects();
  }
  
  // 시스템 상태 명령어
  else if (command == "STATUS") {
    printSystemStatus();
  }
  else if (command == "RESET_AUDIO") {
    Serial.println("DFPlayer 재초기화 중...");
    initializeDFPlayer();
  }
  else if (command == "HELP") {
    printCommandHelp();
  }
  
  // 알 수 없는 명령어
  else {
    Serial.print("알 수 없는 명령어: ");
    Serial.println(command);
    Serial.println("HELP 명령어로 사용법을 확인하세요.");
  }
}

/**
 * 시스템 전체 상태 출력
 */
void printSystemStatus() {
  Serial.println("=== 무궁화 게임 시스템 상태 ===");
  
  // 게임 상태
  Serial.print("게임 레벨: ");
  Serial.println(gameState.currentLevel);
  Serial.print("현재 점수: ");
  Serial.println(gameState.currentScore);
  Serial.print("게임 단계: ");
  Serial.println(gameState.gamePhase);
  Serial.print("게임 활성: ");
  Serial.println(gameState.isGameActive ? "예" : "아니오");
  
  // 오디오 상태
  Serial.println();
  printAudioStatus();
  
  // 이펙트 LED 상태
  Serial.println("=== 이펙트 LED 상태 ===");
  Serial.print("패턴 엔진 활성: ");
  Serial.println(isPatternEngineActive() ? "예" : "아니오");
  if (isPatternEngineActive()) {
    Serial.print("현재 패턴: ");
    Serial.println(getPatternName(getCurrentPattern()));
  }
  Serial.print("LED 마스크: 0b");
  Serial.println(effectLEDState.currentMask, BIN);
  
  // 통신 상태
  Serial.println("=== 통신 상태 ===");
  Serial.print("Slave 연결: ");
  Serial.println(commState.isConnected ? "연결됨" : "연결 안됨");
  Serial.print("마지막 통신: ");
  Serial.print(millis() - commState.lastCommTime);
  Serial.println("ms 전");
  
  Serial.println("============================");
}

/**
 * 명령어 도움말 출력
 */
void printCommandHelp() {
  Serial.println("=== 사용 가능한 명령어 ===");
  Serial.println();
  Serial.println("[ 오디오 명령어 ]");
  Serial.println("AUDIO_STATUS     - DFPlayer 상태 확인");
  Serial.println("PLAY_WELCOME     - 환영 내레이션 재생");
  Serial.println("PLAY_START       - 게임 시작 안내 재생");
  Serial.println("PLAY_CLEAR       - 게임 클리어 메시지 재생");
  Serial.println("PLAY_BGM         - 메인 BGM 재생");
  Serial.println("PLAY_TENSION     - 긴장감 BGM 재생");
  Serial.println("PLAY_LIGHT_1~4   - 천정 조명 효과음 재생");
  Serial.println("PLAY_TRACK_N     - 트랙 번호 N 재생 (1-255)");
  Serial.println("STOP_AUDIO       - 오디오 재생 정지");
  Serial.println("VOLUME_N         - 볼륨 설정 (0-30)");
  Serial.println("VOLUME_UP        - 볼륨 증가");
  Serial.println("VOLUME_DOWN      - 볼륨 감소");
  Serial.println();
  Serial.println("[ 이펙트 LED 명령어 ]");
  Serial.println("EFFECT_SUCCESS   - 성공 이펙트 재생");
  Serial.println("EFFECT_ERROR     - 오류 이펙트 재생");
  Serial.println("EFFECT_LEVELUP   - 레벨업 이펙트 재생");
  Serial.println("EFFECT_START     - 게임 시작 이펙트 재생");
  Serial.println("EFFECT_CLEAR     - 게임 클리어 이펙트 재생");
  Serial.println("EFFECT_TENSION   - 긴장감 이펙트 재생");
  Serial.println("EFFECT_STOP      - 모든 이펙트 정지");
  Serial.println();
  Serial.println("[ 시스템 명령어 ]");
  Serial.println("STATUS           - 전체 시스템 상태 확인");
  Serial.println("RESET_AUDIO      - DFPlayer 재초기화");
  Serial.println("HELP             - 이 도움말 표시");
  Serial.println("========================");
}

// =============================================================================
// Slave 통신 처리 함수들
// =============================================================================

/**
 * Slave와의 통신 처리
 * Master-Slave 간 명령 송수신 및 상태 동기화
 */
void processSlaveComm() {
  // 수신된 메시지 처리
  if (Serial.available() >= sizeof(SerialMessage)) {
    if (receiveMessage(&incomingMessage)) {
      handleSlaveResponse(&incomingMessage);
    }
  }
  
  // 통신 상태 업데이트
  updateCommState();
}

/**
 * 메시지 수신
 * @param msg 수신할 메시지 구조체
 * @param timeout 타임아웃 (ms)
 * @return 수신 성공 여부
 */
bool receiveMessage(SerialMessage* msg, unsigned long timeout) {
  unsigned long startTime = millis();
  uint8_t* buffer = (uint8_t*)msg;
  int bytesRead = 0;
  
  while (bytesRead < sizeof(SerialMessage) && (millis() - startTime) < timeout) {
    if (Serial.available()) {
      buffer[bytesRead] = Serial.read();
      bytesRead++;
    }
  }
  
  if (bytesRead < sizeof(SerialMessage)) {
    Serial.print("메시지 수신 불완전: ");
    Serial.print(bytesRead);
    Serial.print("/");
    Serial.print(sizeof(SerialMessage));
    Serial.println(" 바이트");
    return false;
  }
  
  return validateMessage(msg);
}

/**
 * Slave 응답 처리
 * @param msg 수신된 메시지
 */
void handleSlaveResponse(const SerialMessage* msg) {
  Serial.print("Slave 응답 수신: CMD=0x");
  Serial.print(msg->command, HEX);
  Serial.print(", DATA1=0x");
  Serial.print(msg->data1, HEX);
  Serial.print(", DATA2=0x");
  Serial.print(msg->data2, HEX);
  Serial.println();
  
  switch (msg->command) {
    case RESP_ROBOT_READY:
      onRobotReady(msg->data1);
      break;
      
    case RESP_PATTERN_DONE:
      onPatternComplete();
      break;
      
    case RESP_INPUT_CORRECT:
      onCorrectInput();
      break;
      
    case RESP_INPUT_WRONG:
      onWrongInput();
      break;
      
    case RESP_LIMIT_HEAD_ON:
    case RESP_LIMIT_BODY_ON:
      onLimitSwitchActivated(msg->command, msg->data1);
      break;
      
    case RESP_AUDIO_DONE:
      onSlaveAudioComplete(msg->data1);
      break;
      
    case RESP_ERROR:
      onSlaveError(msg->data1);
      break;
      
    default:
      Serial.print("알 수 없는 Slave 응답: 0x");
      Serial.println(msg->command, HEX);
      break;
  }
  
  commState.lastCommTime = millis();
  commState.isConnected = true;
}

/**
 * 통신 상태 업데이트
 */
void updateCommState() {
  unsigned long currentTime = millis();
  
  // 통신 타임아웃 확인
  if (currentTime - commState.lastCommTime > COMM_TIMEOUT * 3) {
    if (commState.isConnected) {
      Serial.println("Slave 통신 타임아웃 - 연결 끊김");
      commState.isConnected = false;
    }
  }
}

/**
 * Slave에게 명령 전송
 * @param cmd 명령어
 * @param data1 데이터 1
 * @param data2 데이터 2
 * @return 전송 성공 여부
 */
/**
 * 기본 Slave 명령 전송 (재시도 없음)
 * @param cmd 명령어
 * @param data1 데이터 1
 * @param data2 데이터 2
 * @return 전송 성공 여부
 */
bool sendCommandToSlaveBasic(uint8_t cmd, uint8_t data1, uint8_t data2) {
  createMessage(&outgoingMessage, cmd, data1, data2);
  
  if (commReliability.diagnosticMode) {
    Serial.print("[COMM] Slave에게 명령 전송: CMD=0x");
    Serial.print(cmd, HEX);
    Serial.print(", DATA1=0x");
    Serial.print(data1, HEX);
    Serial.print(", DATA2=0x");
    Serial.print(data2, HEX);
    Serial.println();
  }
  
  unsigned long startTime = millis();
  bool success = sendMessage(&outgoingMessage);
  unsigned long responseTime = millis() - startTime;
  
  // 통신 진단 정보 업데이트
  if (success) {
    commDiagnostics.avgResponseTime = (commDiagnostics.avgResponseTime + responseTime) / 2;
    if (responseTime > commDiagnostics.maxResponseTime) {
      commDiagnostics.maxResponseTime = responseTime;
    }
    if (commDiagnostics.minResponseTime == 0 || responseTime < commDiagnostics.minResponseTime) {
      commDiagnostics.minResponseTime = responseTime;
    }
  }
  
  return success;
}

bool sendCommandToSlave(uint8_t cmd, uint8_t data1, uint8_t data2) {
  // 모터 명령어인 경우 안전장치 체크 (Task 17.2)
  if (cmd >= CMD_ROBOT_HEAD_FRONT && cmd <= CMD_ROBOT_STOP) {
    // 모터 안전 잠금 상태 체크
    if (motorSafety.motorSafetyLockout) {
      Serial.println("모터 안전 잠금으로 인한 명령 거부");
      return false;
    }
    
    // 모터 명령 시간 기록
    motorSafety.lastMotorCommand = millis();
  }
  
  // 통신 안정성 강화: 재시도 기능이 포함된 버전 사용
  bool result = sendCommandToSlaveWithRetry(cmd, data1, data2, 1); // 기본 1회 재시도
  
  // 모터 명령어 응답 처리 (Task 17.2)
  if (cmd >= CMD_ROBOT_HEAD_FRONT && cmd <= CMD_ROBOT_STOP) {
    if (result) {
      motorSafety.lastMotorResponse = millis();
      motorSafety.slaveMotorHealthy = true;
    } else {
      motorSafety.motorErrorCount++;
      Serial.println("모터 명령 전송 실패");
    }
  }
  
  return result;
}

// =============================================================================
// Slave 응답 처리 콜백 함수들
// =============================================================================

/**
 * 로봇 회전 완료 처리
 * @param robotPart 회전 완료된 로봇 부위 (HEAD/BODY)
 */
void onRobotReady(uint8_t robotPart) {
  Serial.print("로봇 회전 완료: ");
  Serial.println(robotPart == 0 ? "HEAD" : "BODY");
  
  // 게임 상태에 따른 후속 처리
  switch (gameState.gamePhase) {
    case PHASE_ROBOT_TURN_FRONT:
      if (robotPart == 0) { // HEAD 정면 회전 완료
        gameState.isRobotFacing = true;
        gameState.gamePhase = PHASE_NARRATION;
        playWelcomeNarration();
      }
      break;
      
    case PHASE_ROBOT_TURN_BACK:
      if (robotPart == 0) { // HEAD 후면 회전 완료
        gameState.isRobotFacing = false;
        gameState.gamePhase = PHASE_SHOW_PATTERN;
        // 초록색 경광등 점등 및 패턴 표시 시작
        digitalWrite(GREEN_LIGHT_RELAY_PIN, HIGH);
        beaconState.greenLightState = true;
      }
      break;
  }
}

/**
 * 패턴 표시 완료 처리
 */
void onPatternComplete() {
  Serial.println("LED 패턴 표시 완료");
  
  if (gameState.gamePhase == PHASE_SHOW_PATTERN) {
    gameState.gamePhase = PHASE_PLAYER_INPUT;
    gameState.phaseTimer = millis(); // 입력 제한시간 시작
    Serial.println("플레이어 입력 대기 시작");
  }
}

/**
 * 올바른 입력 처리
 */
void onCorrectInput() {
  Serial.println("올바른 입력 - 점수 증가");
  playSuccessEffect();
  
  gameState.currentScore++;
  if (gameState.currentScore >= MAX_SCORE) {
    Serial.println("7점 달성 - 게임 클리어!");
    gameState.gamePhase = PHASE_GAME_CLEAR;
    playGameClearEffect();
    playGameClearNarration();
  } else {
    Serial.print("현재 점수: ");
    Serial.print(gameState.currentScore);
    Serial.print("/");
    Serial.println(MAX_SCORE);
    
    // 다음 라운드 준비
    gameState.currentLevel++;
    if (gameState.currentLevel > MAX_LEVEL) {
      gameState.currentLevel = 1; // 레벨 순환
    }
    
    playLevelUpEffect();
    gameState.gamePhase = PHASE_ROBOT_TURN_FRONT; // 동작 감지 단계로
  }
}

/**
 * 잘못된 입력 처리
 */
void onWrongInput() {
  Serial.println("잘못된 입력 - 점수 감소");
  playErrorEffect();
  
  if (gameState.currentScore > MIN_SCORE) {
    gameState.currentScore--;
    Serial.print("현재 점수: ");
    Serial.print(gameState.currentScore);
    Serial.print("/");
    Serial.println(MAX_SCORE);
  } else {
    Serial.println("점수 0점 유지");
  }
  
  // 다음 라운드 준비
  gameState.gamePhase = PHASE_ROBOT_TURN_FRONT; // 동작 감지 단계로
}

/**
 * 리미트 스위치 활성화 처리
 * @param switchType 스위치 타입 (HEAD/BODY)
 * @param switchState 스위치 상태
 */
void onLimitSwitchActivated(uint8_t switchType, uint8_t switchState) {
  Serial.print("리미트 스위치 활성화: ");
  Serial.print(switchType == RESP_LIMIT_HEAD_ON ? "HEAD" : "BODY");
  Serial.print(", 상태: ");
  Serial.println(switchState);
  
  // 로봇 위치 상태 업데이트
  if (switchType == RESP_LIMIT_HEAD_ON) {
    gameState.isRobotFacing = (switchState == 1);
  }
}

/**
 * Slave 오디오 재생 완료 처리
 * @param trackNumber 완료된 트랙 번호
 */
void onSlaveAudioComplete(uint8_t trackNumber) {
  Serial.print("Slave 오디오 재생 완료: 트랙 ");
  Serial.println(trackNumber);
}

/**
 * Slave 에러 처리
 * @param errorCode 에러 코드
 */
void onSlaveError(uint8_t errorCode) {
  Serial.print("Slave 에러 발생: 0x");
  Serial.println(errorCode, HEX);
  
  // 에러에 따른 복구 처리
  switch (errorCode) {
    case 0x01: // 모터 에러
      Serial.println("모터 에러 - 안전 정지");
      sendCommandToSlave(CMD_ROBOT_STOP, 0, 0);
      break;
      
    case 0x02: // 센서 에러
      Serial.println("센서 에러 - 시스템 점검 필요");
      break;
      
    case 0x03: // 통신 에러
      Serial.println("통신 에러 - 재연결 시도");
      commState.isConnected = false;
      break;
      
    default:
      Serial.println("알 수 없는 에러");
      break;
  }
}

// =============================================================================
// 통신 프로토콜 기본 구현 완료 (Task 5.1)
// =============================================================================

/**
 * Slave 응답 처리
 * @param msg 수신된 메시지
 * 요구사항 7.3: Master와 Slave 보드 간 명령을 정확히 전달
 */
void handleSlaveResponse(const SerialMessage* msg) {
  Serial.print("Slave 응답 수신: CMD=0x");
  Serial.print(msg->command, HEX);
  Serial.print(", DATA1=0x");
  Serial.print(msg->data1, HEX);
  Serial.print(", DATA2=0x");
  Serial.print(msg->data2, HEX);
  Serial.println();
  
  switch (msg->command) {
    case RESP_ACK:
      Serial.println("Slave 명령 수신 확인");
      break;
      
    case RESP_NACK:
      Serial.println("Slave 명령 수신 실패");
      commState.lastError = COMM_ERROR_NO_RESPONSE;
      break;
      
    case RESP_BUSY:
      Serial.println("Slave 시스템 사용 중");
      break;
      
    case RESP_ROBOT_READY:
      onRobotReady(msg->data1);
      break;
      
    case RESP_ROBOT_TIMEOUT:
      handleRobotTimeout(msg->data1);
      break;
      
    case RESP_ROBOT_ERROR:
      handleRobotError(msg->data1, msg->data2);
      break;
      
    case RESP_PATTERN_DONE:
      onPatternDisplayComplete();
      break;
      
    case RESP_PATTERN_ERROR:
      handlePatternError(msg->data1);
      break;
      
    case RESP_INPUT_DETECTED:
      Serial.print("입력 감지: 패드 ");
      Serial.println(msg->data1);
      break;
      
    case RESP_INPUT_CORRECT:
      onCorrectInput();
      break;
      
    case RESP_INPUT_WRONG:
      onWrongInput();
      break;
      
    case RESP_INPUT_TIMEOUT:
      Serial.println("플레이어 입력 타임아웃");
      break;
      
    case RESP_LIMIT_HEAD_ON:
      Serial.println("로봇 HEAD 리미트 스위치 활성화");
      onLimitSwitchActivated(0, true);
      break;
      
    case RESP_LIMIT_HEAD_OFF:
      Serial.println("로봇 HEAD 리미트 스위치 비활성화");
      onLimitSwitchActivated(0, false);
      break;
      
    case RESP_LIMIT_BODY_ON:
      Serial.println("로봇 BODY 리미트 스위치 활성화");
      onLimitSwitchActivated(1, true);
      break;
      
    case RESP_LIMIT_BODY_OFF:
      Serial.println("로봇 BODY 리미트 스위치 비활성화");
      onLimitSwitchActivated(1, false);
      break;
      
    case RESP_AUDIO_DONE:
      Serial.print("Slave 오디오 재생 완료: 트랙 ");
      Serial.println(msg->data1);
      break;
      
    case RESP_AUDIO_ERROR:
      Serial.print("Slave 오디오 재생 오류: 트랙 ");
      Serial.println(msg->data1);
      break;
      
    case RESP_ERROR:
      Serial.print("Slave 일반 오류: 코드 0x");
      Serial.println(msg->data1, HEX);
      commState.lastError = msg->data1;
      break;
      
    case RESP_UNKNOWN:
      Serial.println("Slave에서 알 수 없는 명령 수신");
      break;
      
    default:
      Serial.print("알 수 없는 Slave 응답: 0x");
      Serial.println(msg->command, HEX);
      break;
  }
  
  // 통신 상태 업데이트
  commState.lastCommTime = millis();
  commState.isConnected = true;
  commState.lastError = COMM_ERROR_NONE;
}

/**
 * 통신 상태 확인
 * @return 통신 활성 상태
 */
bool isCommActive() {
  unsigned long currentTime = millis();
  return (currentTime - commState.lastCommTime) < (COMM_TIMEOUT * 3);
}

/**
 * 통신 시스템 리셋
 */
void resetComm() {
  Serial.println("통신 시스템 리셋");
  
  // 시리얼 버퍼 클리어
  while (Serial.available()) {
    Serial.read();
  }
  
  // 통신 상태 초기화
  commState.isConnected = false;
  commState.lastCommTime = 0;
  commState.retryCount = 0;
  commState.lastError = COMM_ERROR_NONE;
  
  Serial.println("통신 시스템 리셋 완료");
}

/**
 * 명령어 인코딩 (명령어를 바이트 배열로 변환)
 * @param cmd 명령어
 * @param data1 데이터 1
 * @param data2 데이터 2
 * @param buffer 출력 버퍼
 * @return 인코딩된 바이트 수
 */
uint8_t encodeCommand(uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t* buffer) {
  SerialMessage msg;
  createMessage(&msg, cmd, data1, data2);
  
  memcpy(buffer, &msg, sizeof(SerialMessage));
  return sizeof(SerialMessage);
}

/**
 * 명령어 디코딩 (바이트 배열을 메시지로 변환)
 * @param buffer 입력 버퍼
 * @param bufferSize 버퍼 크기
 * @param msg 출력 메시지
 * @return 디코딩 성공 여부
 */
bool decodeCommand(const uint8_t* buffer, uint8_t bufferSize, SerialMessage* msg) {
  if (bufferSize < sizeof(SerialMessage)) {
    Serial.println("디코딩 실패: 버퍼 크기 부족");
    return false;
  }
  
  memcpy(msg, buffer, sizeof(SerialMessage));
  
  if (!validateMessage(msg)) {
    Serial.println("디코딩 실패: 메시지 유효성 검증 실패");
    return false;
  }
  
  return true;
}

/**
 * 재시도 메커니즘을 포함한 명령 전송
 * @param cmd 명령어
 * @param data1 데이터 1
 * @param data2 데이터 2
 * @param expectedResponse 예상 응답 (0이면 응답 대기 안함)
 * @return 전송 및 응답 수신 성공 여부
 */
bool sendCommandWithRetry(uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t expectedResponse) {
  for (uint8_t attempt = 0; attempt < MAX_RETRY_COUNT; attempt++) {
    Serial.print("명령 전송 시도 ");
    Serial.print(attempt + 1);
    Serial.print("/");
    Serial.println(MAX_RETRY_COUNT);
    
    // 명령 전송
    if (!sendCommandToSlave(cmd, data1, data2)) {
      Serial.println("명령 전송 실패");
      continue;
    }
    
    // 응답이 필요하지 않은 경우
    if (expectedResponse == 0) {
      Serial.println("명령 전송 완료 (응답 대기 안함)");
      return true;
    }
    
    // 응답 대기
    if (waitForResponse(expectedResponse, COMM_TIMEOUT)) {
      Serial.println("명령 전송 및 응답 수신 성공");
      commState.retryCount = 0;
      return true;
    }
    
    Serial.print("응답 대기 실패 (시도 ");
    Serial.print(attempt + 1);
    Serial.println(")");
    
    // 재시도 전 대기
    if (attempt < MAX_RETRY_COUNT - 1) {
      delay(100);
    }
  }
  
  Serial.println("최대 재시도 횟수 초과 - 명령 전송 실패");
  commState.retryCount = MAX_RETRY_COUNT;
  commState.lastError = COMM_ERROR_NO_RESPONSE;
  return false;
}

/**
 * 통신 에러 처리
 * @param errorCode 에러 코드
 */
void handleCommError(uint8_t errorCode) {
  Serial.print("통신 에러 처리: 코드 0x");
  Serial.println(errorCode, HEX);
  
  switch (errorCode) {
    case COMM_ERROR_TIMEOUT:
      Serial.println("통신 타임아웃 - 연결 확인 필요");
      resetComm();
      break;
      
    case COMM_ERROR_CHECKSUM:
      Serial.println("체크섬 오류 - 데이터 무결성 문제");
      break;
      
    case COMM_ERROR_INVALID:
      Serial.println("잘못된 메시지 형식");
      break;
      
    case COMM_ERROR_BUFFER_FULL:
      Serial.println("버퍼 오버플로우 - 시리얼 버퍼 클리어");
      while (Serial.available()) {
        Serial.read();
      }
      break;
      
    case COMM_ERROR_NO_RESPONSE:
      Serial.println("응답 없음 - Slave 보드 상태 확인 필요");
      break;
      
    default:
      Serial.print("알 수 없는 통신 오류: 0x");
      Serial.println(errorCode, HEX);
      break;
  }
  
  commState.lastError = errorCode;
}

/**
 * 통신 상태 진단
 */
void diagnoseCommunication() {
  Serial.println("=== 통신 상태 진단 ===");
  Serial.print("연결 상태: ");
  Serial.println(commState.isConnected ? "연결됨" : "연결 안됨");
  
  Serial.print("마지막 통신 시간: ");
  if (commState.lastCommTime == 0) {
    Serial.println("없음");
  } else {
    Serial.print(millis() - commState.lastCommTime);
    Serial.println("ms 전");
  }
  
  Serial.print("재시도 횟수: ");
  Serial.println(commState.retryCount);
  
  Serial.print("마지막 오류: 0x");
  Serial.println(commState.lastError, HEX);
  
  Serial.print("시리얼 버퍼 사용량: ");
  Serial.print(Serial.available());
  Serial.println(" 바이트");
  
  Serial.println("=====================");
}

// =============================================================================
// 이벤트 핸들러 함수들 (Slave 응답 처리용)
// =============================================================================

/**
 * 로봇 준비 완료 이벤트 처리
 * @param robotPart 로봇 부위 (0: HEAD, 1: BODY)
 */
void onRobotReady(uint8_t robotPart) {
  Serial.print("로봇 준비 완료: ");
  Serial.println(robotPart == 0 ? "HEAD" : "BODY");
  
  // 게임 상태에 따른 처리
  if (gameState.gamePhase == PHASE_ROBOT_TURN_FRONT) {
    if (robotPart == 0) { // HEAD가 정면을 향함
      gameState.isRobotFacing = true;
      gameState.gamePhase = PHASE_MOTION_DETECT;
      gameState.phaseTimer = millis();
      
      // 빨간 경광등 점등
      digitalWrite(RED_LIGHT_RELAY_PIN, HIGH);
      digitalWrite(GREEN_LIGHT_RELAY_PIN, LOW);
      beaconState.redLightState = true;
      beaconState.greenLightState = false;
      
      Serial.println("동작 감지 단계 시작");
    }
  } else if (gameState.gamePhase == PHASE_ROBOT_TURN_BACK) {
    if (robotPart == 0) { // HEAD가 후면을 향함
      gameState.isRobotFacing = false;
      gameState.gamePhase = PHASE_SHOW_PATTERN;
      gameState.phaseTimer = millis();
      
      // 초록 경광등 점등
      digitalWrite(GREEN_LIGHT_RELAY_PIN, HIGH);
      digitalWrite(RED_LIGHT_RELAY_PIN, LOW);
      beaconState.greenLightState = true;
      beaconState.redLightState = false;
      
      Serial.println("패턴 표시 단계 시작");
    }
  }
}

/**
 * 리미트 스위치 활성화 이벤트 처리
 * @param robotPart 로봇 부위 (0: HEAD, 1: BODY)
 * @param isActive 활성화 상태
 */
void onLimitSwitchActivated(uint8_t robotPart, bool isActive) {
  Serial.print("리미트 스위치 ");
  Serial.print(robotPart == 0 ? "HEAD" : "BODY");
  Serial.print(": ");
  Serial.println(isActive ? "활성화" : "비활성화");
  
  if (isActive) {
    // 리미트 스위치 활성화 시 해당 로봇 부위 정지
    if (robotPart == 0) {
      Serial.println("HEAD 리미트 도달 - 정지");
    } else {
      Serial.println("BODY 리미트 도달 - 정지");
    }
  }
}

/**
 * 잘못된 입력 이벤트 처리
 */
void onWrongInput() {
  Serial.println("잘못된 입력 - 점수 감점");
  playErrorEffect();
  
  if (gameState.currentScore > 0) {
    gameState.currentScore--;
    Serial.print("점수 감점: ");
    Serial.println(gameState.currentScore);
  } else {
    Serial.println("점수 0점 유지");
  }
  
  // 다음 라운드로 진행
  gameState.currentLevel++;
  if (gameState.currentLevel > MAX_LEVEL) {
    gameState.currentLevel = 1;
  }
  
  gameState.gamePhase = PHASE_ROBOT_TURN_BACK;
  gameState.phaseTimer = millis();
}

/**
 * Slave 오류 이벤트 처리
 * @param errorCode 오류 코드
 */
void onSlaveError(uint8_t errorCode) {
  Serial.print("Slave 오류 발생: 코드 0x");
  Serial.println(errorCode, HEX);
  
  switch (errorCode) {
    case 0x01: // 모터 에러
      Serial.println("모터 에러 - 안전 정지");
      sendCommandToSlave(CMD_ROBOT_STOP, 0, 0);
      playErrorEffect();
      break;
      
    case 0x02: // 센서 에러
      Serial.println("센서 에러 - 시스템 점검 필요");
      break;
      
    case 0x03: // 오디오 에러
      Serial.println("Slave 오디오 에러");
      break;
      
    default:
      Serial.print("알 수 없는 Slave 오류: 0x");
      Serial.println(errorCode, HEX);
      break;
  }
}

// =============================================================================
// 게임 상태 머신 보조 함수들 구현
// =============================================================================

/**
 * 천정 조명 순차 점등 시작
 * 요구사항 1.2: 메인 조명 4개를 1번부터 4번까지 0.5초 간격으로 순차 점등
 */
void startCeilingLightSequence() {
  Serial.println("천정 조명 순차 점등 시작");
  
  ceilingState.sequenceActive = true;
  ceilingState.currentSequenceStep = 0;
  ceilingState.sequenceTimer = millis();
  ceilingState.lastUpdateTime = millis();
  
  // 모든 조명 소등 상태에서 시작
  for (int i = 0; i < 4; i++) {
    digitalWrite(CEILING_LIGHT_PINS[i], LOW);
    ceilingState.lightState[i] = false;
  }
  
  Serial.println("천정 조명 순차 점등 준비 완료");
}

/**
 * 천정 조명 순차 점등 완료 확인
 * @return 순차 점등 완료 여부
 */
bool isCeilingLightSequenceComplete() {
  return !ceilingState.sequenceActive;
}

/**
 * 천정 조명 순차 점등 업데이트 (기존 함수 개선)
 * 요구사항 1.3: 각 조명이 점등될 때 해당하는 효과음을 재생
 */
void updateCeilingLightSequence() {
  if (!ceilingState.sequenceActive) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // 다음 조명 점등 시간 확인
  if (currentTime - ceilingState.sequenceTimer >= LIGHT_SEQUENCE_INTERVAL) {
    if (ceilingState.currentSequenceStep < 4) {
      // 현재 단계 조명 점등
      uint8_t lightIndex = ceilingState.currentSequenceStep;
      digitalWrite(CEILING_LIGHT_PINS[lightIndex], HIGH);
      ceilingState.lightState[lightIndex] = true;
      
      Serial.print("천정 조명 ");
      Serial.print(lightIndex + 1);
      Serial.println(" 점등");
      
      // 해당 조명 효과음 재생
      playCeilingLightSound(lightIndex);
      
      // 다음 단계로 진행
      ceilingState.currentSequenceStep++;
      ceilingState.sequenceTimer = currentTime;
      
    } else {
      // 모든 조명 점등 완료
      Serial.println("천정 조명 순차 점등 완료");
      ceilingState.sequenceActive = false;
    }
  }
}

/**
 * 경광등 제어
 * @param greenOn 초록색 경광등 상태
 * @param redOn 빨간색 경광등 상태
 */
void setBeaconLight(bool greenOn, bool redOn) {
  // 초록색 경광등 제어
  if (beaconState.greenLightState != greenOn) {
    beaconState.greenLightState = greenOn;
    digitalWrite(GREEN_LIGHT_RELAY_PIN, greenOn ? HIGH : LOW);
    
    Serial.print("초록색 경광등: ");
    Serial.println(greenOn ? "점등" : "소등");
  }
  
  // 빨간색 경광등 제어
  if (beaconState.redLightState != redOn) {
    beaconState.redLightState = redOn;
    digitalWrite(RED_LIGHT_RELAY_PIN, redOn ? HIGH : LOW);
    
    Serial.print("빨간색 경광등: ");
    Serial.println(redOn ? "점등" : "소등");
  }
  
  beaconState.lastStateChange = millis();
}

/**
 * 동작 감지 시작
 * 요구사항 3.2: PIR 센서가 5초간 동작감지를 시작
 */
void startMotionDetection() {
  Serial.println("PIR 센서 동작 감지 시작 (5초간)");
  
  isMotionDetectionActive = true;
  motionDetectStartTime = millis();
  
  // PIR 센서 상태 초기화
  for (int i = 0; i < 4; i++) {
    pirState.motionDetected[i] = false;
  }
  
  Serial.println("동작 감지 활성화 완료");
}

/**
 * 동작 감지 정지
 */
void stopMotionDetection() {
  if (!isMotionDetectionActive) {
    return;
  }
  
  Serial.println("PIR 센서 동작 감지 정지");
  
  isMotionDetectionActive = false;
  motionDetectStartTime = 0;
  
  // 감지 결과 확인
  bool anyMotionDetected = false;
  for (int i = 0; i < 4; i++) {
    if (pirState.motionDetected[i]) {
      anyMotionDetected = true;
      break;
    }
  }
  
  // 게임 상태 머신에 결과 전달
  setMotionDetectionResult(anyMotionDetected);
  
  Serial.print("동작 감지 결과: ");
  Serial.println(anyMotionDetected ? "감지됨" : "감지안됨");
}

/**
 * EM-Lock 활성화
 * 요구사항 6.3: EM-Lock이 작동하여 다음 세션으로의 통로가 열려야 한다
 */
void activateEMLock() {
  Serial.println("EM-Lock 활성화 - 다음 세션 통로 개방");
  
  // 모든 EM-Lock 핀 활성화
  for (int i = 0; i < 4; i++) {
    digitalWrite(EMLOCK_PINS[i], HIGH);
  }
  
  Serial.println("EM-Lock 활성화 완료");
}

/**
 * EM-Lock 비활성화
 */
void deactivateEMLock() {
  Serial.println("EM-Lock 비활성화");
  
  // 모든 EM-Lock 핀 비활성화
  for (int i = 0; i < 4; i++) {
    digitalWrite(EMLOCK_PINS[i], LOW);
  }
  
  Serial.println("EM-Lock 비활성화 완료");
}

/**
 * PIR 센서 실제 동작 감지 처리 (기존 함수 개선)
 * 요구사항 3.2, 3.4: 동작감지 시간 중 움직임이 감지되면 처리
 */
void processMotionDetection() {
  if (!isMotionDetectionActive) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // 동작 감지 시간 확인 (5초)
  if (currentTime - motionDetectStartTime >= MOTION_DETECT_DURATION) {
    // 시간 초과로 자동 종료 (게임 상태 머신에서 처리)
    return;
  }
  
  // PIR 센서 읽기 및 동작 감지
  bool motionDetectedNow = false;
  
  for (int zone = 0; zone < 4; zone++) {
    bool zoneMotion = false;
    
    // 각 구역의 4개 센서 확인
    for (int sensor = 0; sensor < 4; sensor++) {
      int sensorIndex = zone * 4 + sensor;
      bool currentState = digitalRead(PIR_PINS[sensorIndex]);
      
      // 상태 변화 감지 (LOW -> HIGH)
      if (currentState && !pirState.previousState[sensorIndex]) {
        // 디바운싱 처리
        if (currentTime - pirState.lastTriggerTime[sensorIndex] > PIR_DEBOUNCE_TIME) {
          Serial.print("PIR 센서 ");
          Serial.print(sensorIndex + 1);
          Serial.print(" (구역 ");
          Serial.print((char)('A' + zone));
          Serial.println(") 동작 감지!");
          
          pirState.lastTriggerTime[sensorIndex] = currentTime;
          zoneMotion = true;
        }
      }
      
      pirState.previousState[sensorIndex] = currentState;
    }
    
    // 구역별 동작 감지 상태 업데이트
    if (zoneMotion) {
      pirState.motionDetected[zone] = true;
      motionDetectedNow = true;
    }
  }
  
  // 동작 감지 시 즉시 처리
  if (motionDetectedNow) {
    Serial.println("동작 감지됨 - 총소리 효과음 재생");
    
    // 요구사항 3.3: 총소리 효과음 재생 (Slave에서 처리)
    sendCommandToSlave(CMD_PLAY_EFFECT, SFX_GUNSHOT, 0);
    
    // 요구사항 3.4: 이펙트 LED 점멸 (현재 TENSION 패턴이 재생 중이므로 추가 처리 불필요)
  }
}

/**
 * PIR 센서 읽기 함수 개선 (기존 함수 개선)
 */
void readPIRSensors() {
  static unsigned long lastReadTime = 0;
  unsigned long currentTime = millis();
  
  // 10ms마다 센서 읽기 (너무 자주 읽지 않도록)
  if (currentTime - lastReadTime < 10) {
    return;
  }
  
  lastReadTime = currentTime;
  
  // 모든 PIR 센서 상태 읽기
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    pirState.currentState[i] = digitalRead(PIR_PINS[i]);
  }
}

// =============================================================================
// 시리얼 명령어 처리 개선 (게임 제어용)
// =============================================================================

/**
 * 시리얼 명령어 처리 함수 개선
 * 게임 제어 명령어 추가
 */
void processSerialCommands() {
  if (!Serial.available()) {
    return;
  }
  
  String command = Serial.readStringUntil('\n');
  command.trim();
  
  if (command.length() == 0) {
    return;
  }
  
  Serial.print("명령어 수신: ");
  Serial.println(command);
  
  // 게임 제어 명령어
  if (command == "start") {
    Serial.println("=== 게임 시작 버튼 입력 감지 ===");
    startGame();
  } else if (command == "reset") {
    resetGameState();
    transitionToPhase(PHASE_IDLE);
  } else if (command == "status") {
    printGameStatus();
  } else if (command == "score") {
    printScoreAndLevelStatus();
  } else if (command == "test_input_correct") {
    // 테스트용: 정답 입력 시뮬레이션
    onPlayerInputReceived(true);
  } else if (command == "test_input_wrong") {
    // 테스트용: 오답 입력 시뮬레이션
    onPlayerInputReceived(false);
  } else if (command == "test_motion") {
    // 테스트용: 동작 감지 시뮬레이션
    setMotionDetectionResult(true);
  } else if (command == "test_robot_complete") {
    // 테스트용: 로봇 회전 완료 시뮬레이션
    onRobotRotationComplete(false);
  } else if (command == "help") {
    printCommandHelp();
  }
  
  // 게임 로직 안정성 명령어 (Task 17.3)
  else if (command == "stability_status") {
    printGameLogicStabilityStatus();
  } else if (command == "stability_reset") {
    resetGameLogicStability();
  } else if (command == "game_recovery") {
    Serial.println("수동 게임 상태 복구 시작");
    startGameStateRecovery(RESET_REASON_USER_REQUEST);
  } else if (command == "emergency_mode") {
    Serial.println("비상 모드 진입");
    enterEmergencyMode();
  } else if (command == "emergency_exit") {
    Serial.println("비상 모드 해제");
    exitEmergencyMode();
  } else if (command == "input_error_test") {
    Serial.println("입력 오류 테스트");
    recordInputError(1); // 잘못된 순서 오류
  } else if (command == "input_success_test") {
    Serial.println("입력 성공 테스트");
    recordInputSuccess();
  } else if (command == "soft_reset") {
    Serial.println("소프트 리셋 요청");
    requestSystemReset(false, RESET_REASON_USER_REQUEST);
  } else if (command == "hard_reset") {
    Serial.println("하드 리셋 요청");
    requestSystemReset(true, RESET_REASON_USER_REQUEST);
  } else if (command == "validate_state") {
    Serial.println("게임 상태 검증");
    if (isGameStateValid(&gameState)) {
      Serial.println("게임 상태 유효");
    } else {
      Serial.println("게임 상태 손상됨");
    }
  } else if (command == "backup_state") {
    Serial.println("게임 상태 백업");
    backupGameState();
  } else if (command == "guidance_test") {
    Serial.println("적응형 가이드 테스트");
    activateAdaptiveGuidance();
  }
  
  // 통신 진단 명령어 (Task 17.1)
  else if (command == "comm_status") {
    printCommDiagnostics();
  } else if (command == "comm_diag_on") {
    setCommDiagnosticMode(true);
  } else if (command == "comm_diag_off") {
    setCommDiagnosticMode(false);
  } else if (command == "comm_reset") {
    resetCommStatistics();
  } else if (command == "comm_test") {
    // 통신 테스트 수행
    Serial.println("통신 테스트 시작...");
    for (int i = 0; i < 10; i++) {
      bool success = sendCommandToSlaveWithRetry(CMD_GET_STATUS, 0, 0, 3);
      Serial.print("테스트 ");
      Serial.print(i + 1);
      Serial.print("/10: ");
      Serial.println(success ? "성공" : "실패");
      delay(500);
    }
    Serial.println("통신 테스트 완료");
  } else if (command == "comm_recovery") {
    // 수동 통신 복구 시작
    Serial.println("수동 통신 복구 시작");
    startCommRecovery(COMM_ERROR_UNKNOWN);
  }
  
  // PIR 센서 관련 명령어
  else if (command == "pir_status") {
    printPIRSensorStatus();
  } else if (command == "pir_start") {
    startMotionDetection();
  } else if (command == "pir_stop") {
    stopMotionDetection();
  }
  
  // 경광등 관련 명령어
  else if (command == "beacon_green_on") {
    setGreenBeacon(true);
  } else if (command == "beacon_green_off") {
    setGreenBeacon(false);
  } else if (command == "beacon_red_on") {
    setRedBeacon(true);
  } else if (command == "beacon_red_off") {
    setRedBeacon(false);
  } else if (command == "beacon_all_off") {
    turnOffAllBeacons();
  } else if (command == "beacon_status") {
    printBeaconStatus();
  }
  
  // 천정 조명 관련 명령어
  else if (command == "ceiling_status") {
    printCeilingLightStatus();
  } else if (command == "ceiling_sequence") {
    startCeilingLightSequence();
  } else if (command == "ceiling_all_on") {
    controlAllCeilingLights(true);
  } else if (command == "ceiling_all_off") {
    controlAllCeilingLights(false);
  }
  
  // 하드웨어 안전장치 명령어 (Task 17.2)
  else if (command == "safety_status") {
    printHardwareSafetyStatus();
  } else if (command == "safety_reset") {
    resetHardwareSafety();
  } else if (command == "power_status") {
    Serial.print("현재 전압: ");
    Serial.print(powerMonitor.currentVoltage);
    Serial.println("V");
    Serial.print("전원 안정성: ");
    Serial.println(powerMonitor.powerStable ? "안정" : "불안정");
  } else if (command == "sensor_status") {
    Serial.println("=== 센서 상태 ===");
    for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
      Serial.print("PIR 센서 ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(sensorSafety.pirSensorStatus[i] ? "정상" : "오작동");
    }
    Serial.print("DFPlayer: ");
    Serial.println(sensorSafety.dfPlayerHealthy ? "정상" : "오작동");
    Serial.print("시스템 온도: ");
    Serial.print(sensorSafety.systemTemperature);
    Serial.println("°C");
  } else if (command == "motor_status") {
    Serial.println("=== 모터 상태 ===");
    Serial.print("Slave 모터: ");
    Serial.println(motorSafety.slaveMotorHealthy ? "정상" : "오작동");
    Serial.print("모터 타임아웃: ");
    Serial.println(motorSafety.motorTimeoutCount);
    Serial.print("모터 오류: ");
    Serial.println(motorSafety.motorErrorCount);
    Serial.print("안전 잠금: ");
    Serial.println(motorSafety.motorSafetyLockout ? "활성" : "비활성");
  } else if (command == "emergency_test") {
    Serial.println("비상 안전 모드 테스트");
    enterEmergencySafeMode("수동 테스트");
  } else if (command == "overheat_test") {
    Serial.println("과열 보호 테스트");
    sensorSafety.systemTemperature = 75.0f; // 임계값 초과로 설정
    triggerOverheatProtection();
  } else if (command == "voltage_test") {
    Serial.println("저전압 보호 테스트");
    powerMonitor.currentVoltage = 3.8f; // 임계값 이하로 설정
    triggerEmergencyPowerShutdown("수동 테스트");
  }
  
  else {
    Serial.println("알 수 없는 명령어. 'help'를 입력하여 도움말을 확인하세요.");
    Serial.println("하드웨어 안전장치 명령어:");
    Serial.println("  safety_status  - 안전장치 상태 확인");
    Serial.println("  safety_reset   - 안전장치 리셋");
    Serial.println("  power_status   - 전원 상태 확인");
    Serial.println("  sensor_status  - 센서 상태 확인");
    Serial.println("  motor_status   - 모터 상태 확인");
    Serial.println("  emergency_test - 비상 모드 테스트");
    Serial.println("  overheat_test  - 과열 보호 테스트");
    Serial.println("  voltage_test   - 저전압 보호 테스트");
  }
}

/**
 * 게임 상태 출력
 */
void printGameStatus() {
  Serial.println("=== 게임 상태 정보 ===");
  Serial.print("현재 단계: ");
  Serial.println(getPhaseNameKorean(getCurrentPhase()));
  Serial.print("게임 활성: ");
  Serial.println(isGameActive() ? "예" : "아니오");
  Serial.print("로봇 정면: ");
  Serial.println(isRobotFacing() ? "예" : "아니오");
  Serial.print("단계 경과시간: ");
  Serial.print(getPhaseElapsedTime());
  Serial.println("ms");
  
  printScoreAndLevelStatus();
  
  Serial.print("패턴 엔진: ");
  Serial.println(isPatternEngineActive() ? "활성" : "비활성");
  if (isPatternEngineActive()) {
    Serial.print("현재 패턴: ");
    Serial.println(getPatternName(getCurrentPattern()));
  }
  
  Serial.print("동작 감지: ");
  Serial.println(isMotionDetectionActive ? "활성" : "비활성");
  
  Serial.println("====================");
}

/**
 * 명령어 도움말 출력
 */
void printCommandHelp() {
  Serial.println("=== 사용 가능한 명령어 ===");
  Serial.println("start - 게임 시작");
  Serial.println("reset - 게임 리셋");
  Serial.println("status - 게임 상태 확인");
  Serial.println("score - 점수 및 레벨 확인");
  Serial.println("help - 도움말 출력");
  Serial.println("");
  Serial.println("=== PIR 센서 명령어 ===");
  Serial.println("pir_status - PIR 센서 상태 확인");
  Serial.println("pir_start - 동작 감지 시작");
  Serial.println("pir_stop - 동작 감지 중지");
  Serial.println("");
  Serial.println("=== 경광등 명령어 ===");
  Serial.println("beacon_green_on - 초록 경광등 점등");
  Serial.println("beacon_green_off - 초록 경광등 소등");
  Serial.println("beacon_red_on - 빨간 경광등 점등");
  Serial.println("beacon_red_off - 빨간 경광등 소등");
  Serial.println("beacon_all_off - 모든 경광등 소등");
  Serial.println("beacon_status - 경광등 상태 확인");
  Serial.println("");
  Serial.println("=== 천정 조명 명령어 ===");
  Serial.println("ceiling_status - 천정 조명 상태 확인");
  Serial.println("ceiling_sequence - 순차 점등 시작");
  Serial.println("ceiling_all_on - 모든 조명 점등");
  Serial.println("ceiling_all_off - 모든 조명 소등");
  Serial.println("");
  Serial.println("=== 테스트 명령어 ===");
  Serial.println("test_input_correct - 정답 입력 시뮬레이션");
  Serial.println("test_input_wrong - 오답 입력 시뮬레이션");
  Serial.println("test_motion - 동작 감지 시뮬레이션");
  Serial.println("test_robot_complete - 로봇 회전 완료 시뮬레이션");
  Serial.println("");
  Serial.println("=== 통신 진단 명령어 (Task 17.1) ===");
  Serial.println("comm_status - 통신 상태 및 통계 확인");
  Serial.println("comm_diag_on - 통신 진단 모드 활성화");
  Serial.println("comm_diag_off - 통신 진단 모드 비활성화");
  Serial.println("comm_reset - 통신 통계 리셋");
  Serial.println("");
  Serial.println("=== 게임 로직 안정성 명령어 (Task 17.3) ===");
  Serial.println("stability_status - 게임 로직 안정성 상태 확인");
  Serial.println("stability_reset - 안정성 시스템 리셋");
  Serial.println("game_recovery - 수동 게임 상태 복구");
  Serial.println("emergency_mode - 비상 모드 진입");
  Serial.println("emergency_exit - 비상 모드 해제");
  Serial.println("input_error_test - 입력 오류 테스트");
  Serial.println("input_success_test - 입력 성공 테스트");
  Serial.println("soft_reset - 소프트 리셋 요청");
  Serial.println("hard_reset - 하드 리셋 요청");
  Serial.println("validate_state - 게임 상태 검증");
  Serial.println("backup_state - 게임 상태 백업");
  Serial.println("guidance_test - 적응형 가이드 테스트");
  Serial.println("comm_test - 통신 테스트 수행 (10회)");
  Serial.println("comm_recovery - 수동 통신 복구 시작");
  Serial.println("========================");
}

// =============================================================================
// 누락된 함수들 구현 (Task 13.1 완성을 위한)
// =============================================================================

/**
 * 게임 단계 한국어 이름 반환
 * @param phase 게임 단계
 * @return 한국어 단계 이름
 */
const char* getPhaseNameKorean(GamePhase phase) {
  switch (phase) {
    case PHASE_IDLE: return "대기";
    case PHASE_GAME_START: return "게임시작";
    case PHASE_ROBOT_TURN_FRONT: return "로봇정면회전";
    case PHASE_NARRATION: return "내레이션";
    case PHASE_ROBOT_TURN_BACK: return "로봇후면회전";
    case PHASE_SHOW_PATTERN: return "패턴표시";
    case PHASE_PLAYER_INPUT: return "플레이어입력";
    case PHASE_MOTION_DETECT: return "동작감지";
    case PHASE_SCORE_UPDATE: return "점수업데이트";
    case PHASE_GAME_CLEAR: return "게임클리어";
    default: return "알수없음";
  }
}

/**
 * 환영 내레이션 재생
 * 요구사항 1.5: 환영 내레이션이 재생되어야 한다
 */
void playWelcomeNarration() {
  Serial.println("환영 내레이션 재생 시작");
  playAudioTrack(AUDIO_WELCOME);
}

/**
 * 게임 클리어 내레이션 재생
 * 요구사항 6.3: 클리어 효과가 완료되면 내레이션 재생
 */
void playGameClearNarration() {
  Serial.println("게임 클리어 내레이션 재생 시작");
  playAudioTrack(AUDIO_GAME_CLEAR);
}

/**
 * 이펙트 LED 패턴 재생 (통합 함수)
 * @param pattern 재생할 이펙트 패턴
 */
void playEffectLED(EffectPattern pattern) {
  Serial.print("이펙트 LED 패턴 재생: ");
  Serial.println(getPatternName(pattern));
  
  // 기존 패턴이 재생 중이면 정지
  if (patternEngine.engineActive) {
    stopEffectLED();
    delay(50); // 짧은 대기
  }
  
  // 패턴별 시퀀스 설정
  setupEffectSequence(pattern);
  
  // 패턴 엔진 시작
  patternEngine.engineActive = true;
  patternEngine.activePattern = pattern;
  patternEngine.currentStep = 0;
  patternEngine.currentRepeat = 0;
  patternEngine.stepTimer = millis();
  
  Serial.println("이펙트 LED 패턴 시작");
}

/**
 * 이펙트 LED 패턴 정지
 */
void stopEffectLED() {
  if (patternEngine.engineActive) {
    Serial.println("이펙트 LED 패턴 정지");
    
    // 패턴 엔진 정지
    patternEngine.engineActive = false;
    patternEngine.currentStep = 0;
    patternEngine.currentRepeat = 0;
    
    // 모든 이펙트 LED 소등
    for (int i = 0; i < 6; i++) {
      digitalWrite(EFFECT_LED_PINS[i], LOW);
      effectLEDState.ledState[i] = false;
    }
    effectLEDState.currentMask = 0;
    effectLEDState.patternActive = false;
  }
}

/**
 * 패턴별 시퀀스 설정
 * @param pattern 설정할 패턴
 */
void setupEffectSequence(EffectPattern pattern) {
  EffectSequence* seq = &patternEngine.currentSequence;
  
  switch (pattern) {
    case SUCCESS_PATTERN:
      // 성공 패턴: 순차 점등 후 전체 점등
      seq->stepCount = 8;
      seq->repeatCount = 1;
      seq->isLooping = false;
      
      // 순차 점등 (6단계)
      for (int i = 0; i < 6; i++) {
        seq->steps[i].ledMask = (1 << i);
        seq->steps[i].duration = EFFECT_SUCCESS_SPEED;
        seq->steps[i].isOn = true;
      }
      // 전체 점등
      seq->steps[6].ledMask = 0x3F; // 모든 LED
      seq->steps[6].duration = EFFECT_SUCCESS_SPEED * 2;
      seq->steps[6].isOn = true;
      // 전체 소등
      seq->steps[7].ledMask = 0x00;
      seq->steps[7].duration = EFFECT_SUCCESS_SPEED;
      seq->steps[7].isOn = false;
      break;
      
    case ERROR_PATTERN:
      // 오류 패턴: 0.2초 간격으로 5회 점멸
      seq->stepCount = 2;
      seq->repeatCount = 5;
      seq->isLooping = false;
      
      // 전체 점등
      seq->steps[0].ledMask = 0x3F; // 모든 LED
      seq->steps[0].duration = 200; // 0.2초
      seq->steps[0].isOn = true;
      // 전체 소등
      seq->steps[1].ledMask = 0x00;
      seq->steps[1].duration = 200; // 0.2초
      seq->steps[1].isOn = false;
      break;
      
    case LEVEL_UP_PATTERN:
      // 레벨업 패턴: 파도 효과
      seq->stepCount = 6;
      seq->repeatCount = 2;
      seq->isLooping = false;
      
      for (int i = 0; i < 6; i++) {
        seq->steps[i].ledMask = (0x07 << (i % 4)); // 3개씩 이동
        seq->steps[i].duration = EFFECT_LEVELUP_SPEED;
        seq->steps[i].isOn = true;
      }
      break;
      
    case GAME_START_PATTERN:
      // 게임 시작 패턴: 중앙에서 외곽으로 확산
      seq->stepCount = 4;
      seq->repeatCount = 1;
      seq->isLooping = false;
      
      seq->steps[0].ledMask = 0x18; // 중앙 2개
      seq->steps[0].duration = EFFECT_START_SPEED;
      seq->steps[0].isOn = true;
      
      seq->steps[1].ledMask = 0x24; // 중간 2개 추가
      seq->steps[1].duration = EFFECT_START_SPEED;
      seq->steps[1].isOn = true;
      
      seq->steps[2].ledMask = 0x3F; // 모든 LED
      seq->steps[2].duration = EFFECT_START_SPEED;
      seq->steps[2].isOn = true;
      
      seq->steps[3].ledMask = 0x00; // 모두 소등
      seq->steps[3].duration = EFFECT_START_SPEED;
      seq->steps[3].isOn = false;
      break;
      
    case GAME_CLEAR_PATTERN:
      // 게임 클리어 패턴: 축하 점멸
      seq->stepCount = 4;
      seq->repeatCount = 10; // 10회 반복
      seq->isLooping = false;
      
      seq->steps[0].ledMask = 0x15; // 1, 3, 5번 LED
      seq->steps[0].duration = 300;
      seq->steps[0].isOn = true;
      
      seq->steps[1].ledMask = 0x2A; // 2, 4, 6번 LED
      seq->steps[1].duration = 300;
      seq->steps[1].isOn = true;
      
      seq->steps[2].ledMask = 0x3F; // 모든 LED
      seq->steps[2].duration = 500;
      seq->steps[2].isOn = true;
      
      seq->steps[3].ledMask = 0x00; // 모두 소등
      seq->steps[3].duration = 200;
      seq->steps[3].isOn = false;
      break;
      
    case TENSION_PATTERN:
      // 긴장 패턴: 심장박동 효과 (무한 반복)
      seq->stepCount = 4;
      seq->repeatCount = 0; // 무한 반복
      seq->isLooping = true;
      
      seq->steps[0].ledMask = 0x18; // 중앙 2개 약하게
      seq->steps[0].duration = 200;
      seq->steps[0].isOn = true;
      
      seq->steps[1].ledMask = 0x3C; // 중간 4개
      seq->steps[1].duration = 300;
      seq->steps[1].isOn = true;
      
      seq->steps[2].ledMask = 0x3F; // 모든 LED 강하게
      seq->steps[2].duration = 100;
      seq->steps[2].isOn = true;
      
      seq->steps[3].ledMask = 0x00; // 소등 (심장박동 간격)
      seq->steps[3].duration = EFFECT_TENSION_BEAT - 600;
      seq->steps[3].isOn = false;
      break;
      
    default:
      // 기본 패턴: 단순 점멸
      seq->stepCount = 2;
      seq->repeatCount = 3;
      seq->isLooping = false;
      
      seq->steps[0].ledMask = 0x3F;
      seq->steps[0].duration = 500;
      seq->steps[0].isOn = true;
      
      seq->steps[1].ledMask = 0x00;
      seq->steps[1].duration = 500;
      seq->steps[1].isOn = false;
      break;
  }
}

/**
 * 게임 시작 이펙트 재생
 * 요구사항 8.1: 게임의 각 단계가 진행되면 적절한 시각적 피드백 제공
 */
void playGameStartEffect() {
  Serial.println("게임 시작 이펙트 재생");
  playEffectLED(GAME_START_PATTERN);
}

/**
 * 오류 이펙트 재생
 * 요구사항 3.4: 움직임이 감지되면 이펙트 LED가 0.2초 간격으로 5회 점멸
 */
void playErrorEffect() {
  Serial.println("오류 이펙트 재생 (동작 감지 처벌)");
  playEffectLED(ERROR_PATTERN);
}

/**
 * 긴장 이펙트 재생
 * 요구사항 8.1: 동작 감지 중 긴장감 조성을 위한 이펙트
 */
void playTensionEffect() {
  Serial.println("긴장 이펙트 재생 시작");
  playEffectLED(TENSION_PATTERN);
}

/**
 * 긴장 이펙트 중지
 */
void stopTensionEffect() {
  Serial.println("긴장 이펙트 중지");
  stopEffectLED();
}

/**
 * 게임 클리어 이펙트 재생
 * 요구사항 6.2: 클리어 효과가 시작되면 이펙트 LED가 특별한 패턴으로 연출
 */
void playGameClearEffect() {
  Serial.println("게임 클리어 이펙트 재생");
  playEffectLED(GAME_CLEAR_PATTERN);
}

/**
 * 현재 레벨 패턴 표시
 * 요구사항 2.2: 4개의 LED가 레벨에 따른 패턴으로 점멸
 */
void showCurrentLevelPattern() {
  Serial.print("현재 레벨 ");
  Serial.print(gameState.currentLevel);
  Serial.println(" 패턴 표시 시작");
  
  // Slave에게 패턴 표시 명령 전송
  uint8_t patternLength = getPatternLengthForLevel(gameState.currentLevel);
  uint8_t patternSpeed = getPatternSpeedForLevel(gameState.currentLevel);
  
  sendCommandToSlave(CMD_SHOW_PATTERN, patternLength, patternSpeed);
}

/**
 * 레벨별 패턴 길이 반환
 * 요구사항 5.2, 5.3, 5.4: 레벨별 패턴 길이 조절
 */
uint8_t getPatternLengthForLevel(uint8_t level) {
  if (level >= 1 && level <= 4) {
    return level + 1; // 레벨 1-4: 2-5개
  } else if (level >= 5 && level <= 10) {
    return 5; // 레벨 5-10: 5개
  } else if (level == 11) {
    return 6; // 레벨 11: 6개
  }
  return 2; // 기본값
}

/**
 * 레벨별 패턴 속도 반환
 * 요구사항 5.2, 5.3: 레벨별 패턴 속도 조절
 */
uint8_t getPatternSpeedForLevel(uint8_t level) {
  if (level >= 1 && level <= 4) {
    return PATTERN_DISPLAY_INTERVAL / 100; // 0.5초 -> 5 (100ms 단위)
  } else {
    return PATTERN_FAST_INTERVAL / 100; // 0.3초 -> 3 (100ms 단위)
  }
}

/**
 * 점수 업데이트 처리
 * 요구사항 4.1, 4.4, 4.5: 점수 증감 로직 및 처리
 */
void processScoreUpdate() {
  Serial.println("=== 점수 업데이트 처리 시작 ===");
  
  Serial.print("마지막 입력 결과: ");
  Serial.println(lastInputResult ? "정답" : "오답");
  Serial.print("동작 감지: ");
  Serial.println(motionDetected ? "감지됨" : "감지안됨");
  
  bool scoreChanged = false;
  
  // 1. 정답 입력 시 점수 증가
  if (lastInputResult) {
    if (incrementScore()) {
      scoreChanged = true;
      Serial.println("정답! 점수 1점 증가");
    }
  }
  
  // 2. 동작 감지 시 점수 감소
  if (motionDetected) {
    if (decrementScore()) {
      scoreChanged = true;
      Serial.println("동작 감지! 점수 1점 감소");
    }
  }
  
  // 3. 점수 변화가 있으면 Slave에게 업데이트 전송
  if (scoreChanged) {
    sendCommandToSlave(CMD_UPDATE_SCORE, gameState.currentScore, 0);
    Serial.print("점수 업데이트 전송: ");
    Serial.println(gameState.currentScore);
  }
  
  // 4. 레벨 진행 (라운드 완료)
  incrementLevel();
  
  // 5. 결과 변수 초기화
  lastInputResult = false;
  motionDetected = false;
  
  Serial.println("=== 점수 업데이트 처리 완료 ===");
  
  // 6. 다음 단계 결정
  if (gameState.currentScore >= MAX_SCORE) {
    // 게임 클리어
    transitionToPhase(PHASE_GAME_CLEAR);
  } else {
    // 다음 라운드 진행
    transitionToPhase(PHASE_ROBOT_TURN_BACK);
  }
}

/**
 * 동작 감지 결과 설정
 * @param detected 감지 여부
 */
void setMotionDetectionResult(bool detected) {
  motionDetected = detected;
  Serial.print("동작 감지 결과 설정: ");
  Serial.println(detected ? "감지됨" : "감지안됨");
}

/**
 * EM-Lock 작동
 * 요구사항 6.3: EM-Lock이 작동하여 다음 세션으로의 통로가 열려야 한다
 */
void activateEMLock() {
  Serial.println("EM-Lock 작동 - 통로 개방");
  
  // 모든 EM-Lock 핀 활성화
  for (int i = 0; i < 4; i++) {
    digitalWrite(EMLOCK_PINS[i], HIGH);
  }
  
  Serial.println("EM-Lock 활성화 완료");
}

/**
 * Slave 통신 처리
 * 요구사항 7.3: Master와 Slave 보드 간 명령을 정확히 전달
 */
void processSlaveComm() {
  // Slave로부터 메시지 수신 확인
  if (Serial.available() > 0) {
    SerialMessage receivedMsg;
    if (receiveMessage(&receivedMsg)) {
      handleSlaveResponse(receivedMsg.command, receivedMsg.data1, receivedMsg.data2);
    }
  }
  
  // 통신 상태 업데이트
  updateCommState();
}

/**
 * Slave 응답 처리
 * @param response 응답 코드
 * @param data1 데이터 1
 * @param data2 데이터 2
 */
void handleSlaveResponse(uint8_t response, uint8_t data1, uint8_t data2) {
  switch (response) {
    case RESP_ROBOT_FRONT_COMPLETE:
      Serial.println("로봇 정면 회전 완료 응답 수신");
      if (gameState.gamePhase == PHASE_ROBOT_TURN_FRONT) {
        transitionToPhase(PHASE_NARRATION);
      }
      break;
      
    case RESP_ROBOT_BACK_COMPLETE:
      Serial.println("로봇 후면 회전 완료 응답 수신");
      if (gameState.gamePhase == PHASE_ROBOT_TURN_BACK) {
        transitionToPhase(PHASE_SHOW_PATTERN);
      }
      break;
      
    case RESP_PATTERN_DONE:
      Serial.println("패턴 표시 완료 응답 수신");
      if (gameState.gamePhase == PHASE_SHOW_PATTERN) {
        transitionToPhase(PHASE_PLAYER_INPUT);
      }
      break;
      
    case RESP_INPUT_CORRECT:
      Serial.println("정답 입력 응답 수신");
      lastInputResult = true;
      if (gameState.gamePhase == PHASE_PLAYER_INPUT) {
        transitionToPhase(PHASE_MOTION_DETECT);
      }
      break;
      
    case RESP_INPUT_WRONG:
      Serial.println("오답 입력 응답 수신");
      lastInputResult = false;
      if (gameState.gamePhase == PHASE_PLAYER_INPUT) {
        transitionToPhase(PHASE_MOTION_DETECT);
      }
      break;
      
    case RESP_INPUT_TIMEOUT:
      Serial.println("입력 타임아웃 응답 수신");
      lastInputResult = false;
      if (gameState.gamePhase == PHASE_PLAYER_INPUT) {
        transitionToPhase(PHASE_MOTION_DETECT);
      }
      break;
      
    default:
      Serial.print("알 수 없는 Slave 응답: 0x");
      Serial.println(response, HEX);
      break;
  }
}

/**
 * DFPlayer 오디오 트랙 재생
 * @param trackNumber 재생할 트랙 번호
 */
void playAudioTrack(uint8_t trackNumber) {
  Serial.print("오디오 트랙 재생: ");
  Serial.println(trackNumber);
  
  // DFPlayer 명령 전송
  uint8_t command[] = {0x7E, 0xFF, 0x06, 0x03, 0x00, 0x00, trackNumber, 0xEF};
  
  for (int i = 0; i < 8; i++) {
    dfSerial.write(command[i]);
  }
  
  // 재생 상태 업데이트
  dfPlayerState.isPlaying = true;
  dfPlayerState.currentTrack = trackNumber;
  dfPlayerState.playStartTime = millis();
  dfPlayerState.lastCommandTime = millis();
}

/**
 * DFPlayer 상태 업데이트
 */
void updateDFPlayerStatus() {
  // DFPlayer로부터 상태 응답 확인
  if (dfSerial.available() > 0) {
    // 간단한 상태 확인 (실제로는 더 복잡한 프로토콜 처리 필요)
    uint8_t response = dfSerial.read();
    
    // 재생 완료 감지 (예시)
    if (response == 0x3D) { // DFPlayer 재생 완료 응답
      dfPlayerState.isPlaying = false;
      Serial.println("오디오 재생 완료");
    }
  }
  
  // 타임아웃 기반 재생 상태 추정
  if (dfPlayerState.isPlaying) {
    unsigned long elapsed = millis() - dfPlayerState.playStartTime;
    
    // 트랙별 예상 재생 시간 (실제로는 트랙 길이에 따라 조정)
    unsigned long expectedDuration = 10000; // 기본 10초
    
    switch (dfPlayerState.currentTrack) {
      case AUDIO_WELCOME:
        expectedDuration = 15000; // 환영 내레이션 15초
        break;
      case AUDIO_GAME_CLEAR:
        expectedDuration = 20000; // 클리어 내레이션 20초
        break;
      default:
        expectedDuration = 5000; // 기타 효과음 5초
        break;
    }
    
    if (elapsed > expectedDuration) {
      dfPlayerState.isPlaying = false;
      Serial.println("오디오 재생 타임아웃으로 완료 처리");
    }
  }
}

// =============================================================================
// 13.2 작업을 위한 보조 함수들
// =============================================================================

/**
 * 레벨별 패턴 표시 시간 계산
 * @param level 현재 레벨
 * @return 패턴 표시 예상 시간 (ms)
 */
unsigned long calculatePatternDuration(uint8_t level) {
  uint8_t patternLength = getPatternLengthForLevel(level);
  uint16_t patternInterval;
  
  if (level >= 1 && level <= 4) {
    patternInterval = PATTERN_DISPLAY_INTERVAL; // 0.5초
  } else {
    patternInterval = PATTERN_FAST_INTERVAL; // 0.3초
  }
  
  // 패턴 길이 * 간격 + 여유시간
  return (patternLength * patternInterval) + 2000; // 2초 여유시간
}

/**
 * 동작 감지 최종 결과 확인
 * 요구사항 3.6: 동작감지 시간 5초가 완료되면 결과 처리
 */
void checkFinalMotionDetectionResult() {
  Serial.println("=== 동작 감지 최종 결과 확인 ===");
  
  // PIR 센서 상태 초기화
  for (int i = 0; i < 4; i++) {
    pirState.motionDetected[i] = false;
  }
  
  // PIR 센서 최종 상태 읽기
  readPIRSensors();
  
  // 감지 결과 확인
  bool anyMotionDetected = false;
  for (int i = 0; i < 4; i++) {
    if (pirState.motionDetected[i]) {
      anyMotionDetected = true;
      Serial.print("구역 ");
      Serial.print((char)('A' + i));
      Serial.println("에서 동작 감지됨");
      break;
    }
  }
  
  // 게임 상태 머신에 결과 전달
  setMotionDetectionResult(anyMotionDetected);
  
  Serial.print("동작 감지 결과: ");
  Serial.println(anyMotionDetected ? "감지됨" : "감지안됨");
}

/**
 * 점수 증가 함수
 * 요구사항 4.1: 플레이어가 올바른 순서로 발판을 밟으면 1점이 추가
 * @return 점수 변화 여부
 */
bool incrementScore() {
  if (gameState.currentScore < MAX_SCORE) {
    gameState.currentScore++;
    Serial.print("점수 증가: ");
    Serial.print(gameState.currentScore);
    Serial.print("/");
    Serial.println(MAX_SCORE);
    return true;
  } else {
    Serial.println("이미 최대 점수에 도달했습니다");
    return false;
  }
}

/**
 * 점수 감소 함수
 * 요구사항 3.5, 4.4: 움직임이 감지되면 점수가 1점 감점, 0점 하한선 처리
 * @return 점수 변화 여부
 */
bool decrementScore() {
  if (gameState.currentScore > MIN_SCORE) {
    gameState.currentScore--;
    Serial.print("점수 감소: ");
    Serial.print(gameState.currentScore);
    Serial.print("/");
    Serial.println(MAX_SCORE);
    return true;
  } else {
    Serial.println("이미 최소 점수입니다 (0점 하한선)");
    return false;
  }
}

/**
 * 레벨 증가 함수
 * 요구사항 5.1, 5.5: 한 라운드가 완료되면 레벨이 1씩 증가, 레벨 11 완료 후 순환
 * @return 레벨 변화 여부
 */
bool incrementLevel() {
  uint8_t previousLevel = gameState.currentLevel;
  
  gameState.currentLevel++;
  
  // 레벨 11 완료 후 레벨 1로 순환
  if (gameState.currentLevel > MAX_LEVEL) {
    gameState.currentLevel = 1;
    Serial.println("레벨 11 완료 - 레벨 1로 순환");
  }
  
  if (gameState.currentLevel != previousLevel) {
    Serial.print("레벨 증가: ");
    Serial.print(previousLevel);
    Serial.print(" -> ");
    Serial.println(gameState.currentLevel);
    return true;
  }
  
  return false;
}

/**
 * 레벨 진행 처리
 * 요구사항 5.1, 5.5: 라운드 완료 시 레벨 증가 및 순환 처리
 */
void updateLevelProgression() {
  Serial.println("=== 레벨 진행 처리 ===");
  
  // 라운드 완료로 레벨 증가
  bool levelChanged = incrementLevel();
  
  if (levelChanged) {
    Serial.println("요구사항 5.1: 한 라운드 완료로 레벨 1 증가");
    
    // 레벨업 이펙트 재생
    playLevelUpEffect();
    
    // 새 레벨의 난이도 정보 출력
    Serial.print("새 레벨 난이도 - 패턴 길이: ");
    Serial.print(getPatternLengthForLevel(gameState.currentLevel));
    Serial.print(", 패턴 속도: ");
    
    if (gameState.currentLevel >= 1 && gameState.currentLevel <= 4) {
      Serial.println("0.5초 간격 (느림)");
    } else if (gameState.currentLevel >= 5 && gameState.currentLevel <= 10) {
      Serial.println("0.3초 간격 (빠름)");
    } else if (gameState.currentLevel == 11) {
      Serial.println("0.3초 간격 (최고 난이도)");
    }
  }
  
  // 레벨 순환 확인
  if (gameState.currentLevel == 1 && levelChanged) {
    Serial.println("요구사항 5.5: 레벨 11 완료 후 레벨 1부터 다시 시작");
  }
}

/**
 * 현재 게임 단계 반환
 * @return 현재 게임 단계
 */
GamePhase getCurrentPhase() {
  return (GamePhase)gameState.gamePhase;
}

/**
 * 게임 활성 상태 확인
 * @return 게임 활성 여부
 */
bool isGameActive() {
  return gameState.isGameActive;
}

/**
 * 로봇 정면 상태 확인
 * @return 로봇이 정면을 보고 있는지 여부
 */
bool isRobotFacing() {
  return gameState.isRobotFacing;
}

/**
 * 현재 단계 경과 시간 반환
 * @return 경과 시간 (ms)
 */
unsigned long getPhaseElapsedTime() {
  return millis() - gameState.phaseTimer;
}

/**
 * 점수 및 레벨 상태 출력
 */
void printScoreAndLevelStatus() {
  Serial.println("=== 점수 및 레벨 상태 ===");
  Serial.print("현재 점수: ");
  Serial.print(gameState.currentScore);
  Serial.print("/");
  Serial.println(MAX_SCORE);
  Serial.print("현재 레벨: ");
  Serial.print(gameState.currentLevel);
  Serial.print("/");
  Serial.println(MAX_LEVEL);
  
  // 다음 레벨 패턴 정보
  uint8_t nextPatternLength = getPatternLengthForLevel(gameState.currentLevel);
  uint8_t nextPatternSpeed = getPatternSpeedForLevel(gameState.currentLevel);
  
  Serial.print("다음 패턴 길이: ");
  Serial.println(nextPatternLength);
  Serial.print("다음 패턴 속도: ");
  Serial.print(nextPatternSpeed * 100);
  Serial.println("ms");
  
  Serial.println("========================");
}

/**
 * 패턴 엔진 활성 상태 확인
 * @return 패턴 엔진 활성 여부
 */
bool isPatternEngineActive() {
  return patternEngine.engineActive;
}

/**
 * 현재 패턴 반환
 * @return 현재 활성 패턴
 */
EffectPattern getCurrentPattern() {
  return patternEngine.activePattern;
}

/**
 * 패턴 이름 반환
 * @param pattern 패턴 타입
 * @return 패턴 이름 문자열
 */
const char* getPatternName(EffectPattern pattern) {
  switch (pattern) {
    case SUCCESS_PATTERN: return "성공";
    case ERROR_PATTERN: return "오류";
    case LEVEL_UP_PATTERN: return "레벨업";
    case GAME_START_PATTERN: return "게임시작";
    case GAME_CLEAR_PATTERN: return "게임클리어";
    case TENSION_PATTERN: return "긴장";
    default: return "알수없음";
  }
}

// =============================================================================
// 13.3 작업을 위한 게임 클리어 및 종료 처리 함수들
// =============================================================================

/**
 * 시스템 최소 전력 모드 준비
 * 요구사항 6.4: 모든 장치가 최소 전력 대기 상태로 전환
 */
void prepareSystemShutdown() {
  Serial.println("💤 === 시스템 최소 전력 모드 준비 시작 ===");
  Serial.println("요구사항 6.4: 모든 장치 최소 전력 대기 상태 전환");
  
  // 1. 모든 이펙트 LED 소등
  Serial.println("1️⃣ 이펙트 LED 시스템 종료");
  stopEffectLED();
  for (int i = 0; i < 6; i++) {
    digitalWrite(EFFECT_LED_PINS[i], LOW);
    effectLEDState.ledState[i] = false;
  }
  Serial.println("   ✅ 이펙트 LED 모두 소등 완료");
  
  // 2. 경광등 소등
  Serial.println("2️⃣ 경광등 시스템 종료");
  setBeaconLight(false, false);
  Serial.println("   ✅ 경광등 모두 소등 완료");
  
  // 3. 천정 조명 소등 (EM-Lock은 유지)
  Serial.println("3️⃣ 천정 조명 시스템 종료 (EM-Lock 제외)");
  for (int i = 0; i < 4; i++) {
    digitalWrite(CEILING_LIGHT_PINS[i], LOW);
    ceilingState.lightState[i] = false;
  }
  Serial.println("   ✅ 천정 조명 모두 소등 완료");
  
  // 4. PIR 센서 시스템 비활성화
  Serial.println("4️⃣ PIR 센서 시스템 비활성화");
  stopMotionDetection();
  isMotionDetectionActive = false;
  Serial.println("   ✅ PIR 센서 시스템 비활성화 완료");
  
  // 5. Slave에게 최소 전력 모드 명령 전송
  Serial.println("5️⃣ Slave 보드 최소 전력 모드 전환");
  sendCommandToSlave(CMD_STOP_ALL, 0, 0);
  delay(100); // 명령 전송 대기
  Serial.println("   ✅ Slave 보드 최소 전력 모드 명령 전송 완료");
  
  // 6. DFPlayer 볼륨 최소화 및 정지
  Serial.println("6️⃣ 오디오 시스템 최소화");
  dfPlayerState.currentVolume = 3; // 최소 볼륨
  dfPlayerState.isPlaying = false;
  Serial.println("   ✅ 오디오 볼륨 최소화 및 재생 정지 완료");
  
  // 7. 게임 상태 변수 정리
  Serial.println("7️⃣ 게임 상태 변수 정리");
  gameState.isGameActive = false;
  gameState.isRobotFacing = false;
  Serial.println("   ✅ 게임 상태 변수 정리 완료");
  
  Serial.println("💤 === 시스템 최소 전력 모드 준비 완료 ===");
  Serial.println("EM-Lock은 활성 상태를 유지합니다");
  Serial.println("새 게임을 시작하려면 'start' 명령을 입력하세요");
}

/**
 * 게임 상태를 대기 모드로 초기화
 * 요구사항 6.4: 게임 종료 후 IDLE 상태로 전환하는 로직 완성
 */
void resetGameToIdleState() {
  Serial.println("🔄 === 게임 상태 대기 모드 초기화 시작 ===");
  Serial.println("요구사항 6.4: 클리어 후 시스템 초기화 및 대기 상태 전환");
  
  // 1. 게임 상태 변수 초기화
  Serial.println("1️⃣ 게임 상태 변수 초기화");
  gameState.currentLevel = 1;
  gameState.currentScore = 0;
  gameState.gamePhase = PHASE_IDLE;
  gameState.isGameActive = false;
  gameState.isRobotFacing = false;
  gameState.phaseTimer = millis();
  Serial.println("   ✅ 게임 상태: 레벨 1, 점수 0, 대기 모드");
  
  // 2. PIR 센서 상태 초기화
  Serial.println("2️⃣ PIR 센서 시스템 초기화");
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    pirState.currentState[i] = false;
    pirState.previousState[i] = false;
    pirState.lastTriggerTime[i] = 0;
    pirState.motionDetected[i] = false;
  }
  isMotionDetectionActive = false;
  motionDetected = false;
  Serial.println("   ✅ PIR 센서 상태 초기화 완료");
  
  // 3. 천정 조명 상태 초기화
  Serial.println("3️⃣ 천정 조명 시스템 초기화");
  ceilingState.sequenceActive = false;
  ceilingState.currentSequenceStep = 0;
  ceilingState.sequenceTimer = 0;
  Serial.println("   ✅ 천정 조명 시퀀스 초기화 완료");
  
  // 4. 경광등 상태 초기화
  Serial.println("4️⃣ 경광등 시스템 초기화");
  beaconState.autoOffActive = false;
  beaconState.autoOffTimer = 0;
  Serial.println("   ✅ 경광등 상태 초기화 완료");
  
  // 5. 이펙트 LED 상태 초기화
  Serial.println("5️⃣ 이펙트 LED 시스템 초기화");
  effectLEDState.patternActive = false;
  effectLEDState.currentMask = 0;
  patternEngine.engineActive = false;
  patternEngine.currentStep = 0;
  patternEngine.currentRepeat = 0;
  Serial.println("   ✅ 이펙트 LED 패턴 엔진 초기화 완료");
  
  // 6. 점수 및 입력 결과 초기화
  Serial.println("6️⃣ 게임 진행 변수 초기화");
  lastInputResult = false;
  previousScore = 0;
  previousLevel = 1;
  Serial.println("   ✅ 점수 및 입력 결과 초기화 완료");
  
  // 7. DFPlayer 상태 초기화
  Serial.println("7️⃣ 오디오 시스템 초기화");
  dfPlayerState.isPlaying = false;
  dfPlayerState.currentTrack = 0;
  dfPlayerState.waitingForResponse = false;
  
  // 8. 통신 상태 초기화
  commState.retryCount = 0;
  commState.lastError = COMM_ERROR_NONE;
  
  Serial.println("게임 상태 대기 모드 초기화 완료");
  Serial.println("새로운 게임을 시작할 준비가 되었습니다");
}

/**
 * 게임 상태 리셋 (기존 함수 개선)
 */
void resetGameState() {
  Serial.println("게임 상태 리셋");
  resetGameToIdleState();
}

/**
 * 게임 시작 트리거 (외부에서 호출)
 * 요구사항 1.1: 플레이어가 시작 버튼을 누르면 게임 시작
 */
void startGame() {
  if (gameState.gamePhase == PHASE_IDLE) {
    Serial.println("=== 새로운 게임 시작 ===");
    Serial.println("요구사항 1.1: 시작 버튼 입력 처리 완료");
    
    // 게임 상태 초기화 (새 게임 시작)
    gameState.currentLevel = 1;
    gameState.currentScore = 0;
    gameState.isGameActive = true;
    gameState.isRobotFacing = false;
    
    // 시스템 조명 초기화 (요구사항 1.1: 모든 조명을 소등 상태에서 시작)
    Serial.println("요구사항 1.1: 모든 조명 소등 상태에서 시작");
    initializeSystemLights();
    
    // 게임 시작 단계로 전환
    transitionToPhase(PHASE_GAME_START);
    
    Serial.print("시작 레벨: ");
    Serial.print(gameState.currentLevel);
    Serial.print(", 시작 점수: ");
    Serial.println(gameState.currentScore);
    
  } else {
    Serial.println("=== 게임 시작 실패 ===");
    Serial.println("게임이 이미 진행 중입니다");
    Serial.print("현재 단계: ");
    Serial.println(getPhaseNameKorean(getCurrentPhase()));
    Serial.print("현재 점수: ");
    Serial.print(gameState.currentScore);
    Serial.print("/7, 레벨: ");
    Serial.println(gameState.currentLevel);
    Serial.println("게임을 리셋하려면 'reset' 명령을 사용하세요");
  }
}

/**
 * 게임 강제 리셋 (디버깅용)
 */
void forceResetGame() {
  Serial.println("=== 게임 강제 리셋 ===");
  
  // 모든 하드웨어 상태 초기화
  initializeSystemLights();
  
  // EM-Lock 비활성화
  for (int i = 0; i < 4; i++) {
    digitalWrite(EMLOCK_PINS[i], LOW);
  }
  
  // Slave에게 리셋 명령 전송
  sendCommandToSlave(CMD_RESET, 0, 0);
  
  // 게임 상태 초기화
  resetGameToIdleState();
  
  Serial.println("게임 강제 리셋 완료");
}

/**
 * 게임 클리어 통계 출력
 */
void printGameClearStats() {
  Serial.println("=== 게임 클리어 통계 ===");
  Serial.print("최종 점수: ");
  Serial.print(gameState.currentScore);
  Serial.print("/");
  Serial.println(MAX_SCORE);
  Serial.print("도달 레벨: ");
  Serial.println(gameState.currentLevel);
  
  // 게임 진행 시간 계산 (대략적)
  unsigned long gameTime = millis() - gameState.phaseTimer;
  Serial.print("게임 진행 시간: ");
  Serial.print(gameTime / 1000);
  Serial.println("초");
  
  Serial.println("축하합니다! 게임을 클리어했습니다!");
  Serial.println("========================");
}

/**
 * 시스템 상태 전체 점검
 */
void performSystemHealthCheck() {
  Serial.println("=== 시스템 상태 점검 ===");
  
  // 1. 게임 상태 확인
  Serial.print("게임 단계: ");
  Serial.println(getPhaseNameKorean(getCurrentPhase()));
  Serial.print("게임 활성: ");
  Serial.println(isGameActive() ? "예" : "아니오");
  
  // 2. 하드웨어 상태 확인
  Serial.println("하드웨어 상태:");
  Serial.print("- 천정 조명: ");
  int lightCount = 0;
  for (int i = 0; i < 4; i++) {
    if (ceilingState.lightState[i]) lightCount++;
  }
  Serial.print(lightCount);
  Serial.println("/4 점등");
  
  Serial.print("- 경광등: ");
  if (beaconState.greenLightState && beaconState.redLightState) {
    Serial.println("초록+빨강");
  } else if (beaconState.greenLightState) {
    Serial.println("초록");
  } else if (beaconState.redLightState) {
    Serial.println("빨강");
  } else {
    Serial.println("소등");
  }
  
  Serial.print("- 이펙트 LED: ");
  Serial.println(isPatternEngineActive() ? "활성" : "비활성");
  
  Serial.print("- DFPlayer: ");
  Serial.println(dfPlayerState.isPlaying ? "재생중" : "정지");
  
  // 3. 통신 상태 확인
  Serial.print("- Slave 통신: ");
  Serial.println(commState.isConnected ? "연결됨" : "연결안됨");
  
  // 4. 센서 상태 확인
  Serial.print("- PIR 센서: ");
  Serial.println(isMotionDetectionActive ? "감지중" : "대기");
  
  Serial.println("시스템 상태 점검 완료");
  Serial.println("====================");
}

// =============================================================================
// 통신 안정성 강화 시스템 (Task 17.1)
// =============================================================================

/**
 * 통신 안정성 시스템 초기화
 */
void initCommReliability() {
  Serial.println("통신 안정성 시스템 초기화...");
  
  // 기본 설정 초기화
  commReliability.retryCount = 0;
  commReliability.maxRetries = MAX_RETRY_COUNT;
  commReliability.lastSuccessTime = millis();
  commReliability.connectionCheckInterval = 5000; // 5초마다 연결 상태 확인
  commReliability.consecutiveErrors = 0;
  commReliability.errorThreshold = 5; // 연속 5회 오류 시 복구 모드
  commReliability.isRecovering = false;
  commReliability.recoveryStartTime = 0;
  commReliability.lastErrorCode = COMM_ERROR_NONE;
  commReliability.totalCommAttempts = 0;
  commReliability.successfulComms = 0;
  commReliability.failedComms = 0;
  commReliability.lastHeartbeatTime = millis();
  commReliability.slaveResponsive = false;
  commReliability.communicationQuality = 100;
  commReliability.recoveryTimeout = 30000; // 30초 복구 타임아웃
  commReliability.diagnosticMode = false;
  
  Serial.println("통신 안정성 시스템 초기화 완료");
  Serial.print("최대 재시도 횟수: ");
  Serial.println(commReliability.maxRetries);
  Serial.print("오류 임계값: ");
  Serial.println(commReliability.errorThreshold);
  Serial.print("연결 확인 간격: ");
  Serial.print(commReliability.connectionCheckInterval);
  Serial.println("ms");
}

/**
 * 통신 진단 시스템 초기화
 */
void initCommDiagnostics() {
  Serial.println("통신 진단 시스템 초기화...");
  
  commDiagnostics.avgResponseTime = 0;
  commDiagnostics.maxResponseTime = 0;
  commDiagnostics.minResponseTime = UINT32_MAX;
  commDiagnostics.timeoutCount = 0;
  commDiagnostics.checksumErrorCount = 0;
  commDiagnostics.invalidMessageCount = 0;
  commDiagnostics.retransmissionCount = 0;
  commDiagnostics.lastDiagnosticTime = millis();
  commDiagnostics.enableDetailedLogging = false;
  
  Serial.println("통신 진단 시스템 초기화 완료");
}

/**
 * 향상된 명령 전송 함수 (재시도 및 오류 복구 포함)
 * @param command 명령어
 * @param data1 데이터 1
 * @param data2 데이터 2
 * @return 전송 성공 여부
 */
bool sendCommandToSlaveReliable(uint8_t command, uint8_t data1, uint8_t data2) {
  unsigned long startTime = millis();
  bool success = false;
  
  // 통신 시도 횟수 증가
  commReliability.totalCommAttempts++;
  
  // 복구 모드 중이면 복구 완료까지 대기
  if (commReliability.isRecovering) {
    if (!waitForRecoveryComplete()) {
      Serial.println("복구 모드 중 - 명령 전송 실패");
      recordCommError(COMM_ERROR_NO_RESPONSE);
      return false;
    }
  }
  
  // 재시도 루프
  for (uint8_t attempt = 0; attempt <= commReliability.maxRetries; attempt++) {
    if (attempt > 0) {
      Serial.print("재시도 ");
      Serial.print(attempt);
      Serial.print("/");
      Serial.println(commReliability.maxRetries);
      commDiagnostics.retransmissionCount++;
      delay(100 * attempt); // 지수 백오프
    }
    
    // 명령 전송
    if (sendCommandToSlave(command, data1, data2)) {
      // 응답 대기
      SerialMessage response;
      if (receiveFromSlaveWithTimeout(&response, COMM_TIMEOUT)) {
        // 응답 검증
        if (validateSlaveResponse(&response, command)) {
          success = true;
          recordCommSuccess(millis() - startTime);
          break;
        } else {
          Serial.println("잘못된 응답 수신");
          recordCommError(COMM_ERROR_INVALID);
        }
      } else {
        Serial.println("응답 타임아웃");
        recordCommError(COMM_ERROR_TIMEOUT);
        commDiagnostics.timeoutCount++;
      }
    } else {
      Serial.println("명령 전송 실패");
      recordCommError(COMM_ERROR_NO_RESPONSE);
    }
    
    // 마지막 시도가 아니면 잠시 대기
    if (attempt < commReliability.maxRetries) {
      delay(50);
    }
  }
  
  // 결과 처리
  if (success) {
    commReliability.consecutiveErrors = 0;
    commReliability.retryCount = 0;
  } else {
    commReliability.consecutiveErrors++;
    commReliability.failedComms++;
    
    // 연속 오류가 임계값을 초과하면 복구 모드 시작
    if (commReliability.consecutiveErrors >= commReliability.errorThreshold) {
      startCommRecovery();
    }
  }
  
  return success;
}

/**
 * 타임아웃 포함 Slave 응답 수신
 * @param message 수신할 메시지 구조체
 * @param timeout 타임아웃 시간 (ms)
 * @return 수신 성공 여부
 */
bool receiveFromSlaveWithTimeout(SerialMessage* message, unsigned long timeout) {
  unsigned long startTime = millis();
  uint8_t bytesReceived = 0;
  uint8_t* msgBytes = (uint8_t*)message;
  
  while (millis() - startTime < timeout && bytesReceived < sizeof(SerialMessage)) {
    if (Serial.available() > 0) {
      msgBytes[bytesReceived] = Serial.read();
      bytesReceived++;
      
      // 헤더 검증
      if (bytesReceived == 1 && msgBytes[0] != MESSAGE_HEADER) {
        Serial.println("잘못된 헤더 - 수신 재시작");
        bytesReceived = 0;
        continue;
      }
    }
    delay(1); // CPU 부하 감소
  }
  
  // 완전한 메시지 수신 확인
  if (bytesReceived == sizeof(SerialMessage)) {
    // 체크섬 검증
    uint8_t expectedChecksum = message->command ^ message->data1 ^ message->data2;
    if (message->checksum == expectedChecksum) {
      return true;
    } else {
      Serial.println("체크섬 오류");
      commDiagnostics.checksumErrorCount++;
      recordCommError(COMM_ERROR_CHECKSUM);
      return false;
    }
  }
  
  Serial.print("불완전한 메시지 수신: ");
  Serial.print(bytesReceived);
  Serial.print("/");
  Serial.println(sizeof(SerialMessage));
  commDiagnostics.invalidMessageCount++;
  return false;
}

/**
 * Slave 응답 검증
 * @param response 수신된 응답
 * @param originalCommand 원본 명령어
 * @return 응답 유효성
 */
bool validateSlaveResponse(const SerialMessage* response, uint8_t originalCommand) {
  // 기본 응답 코드 확인
  switch (response->command) {
    case RESP_ACK:
    case RESP_ROBOT_READY:
    case RESP_ROBOT_FRONT_COMPLETE:
    case RESP_ROBOT_BACK_COMPLETE:
    case RESP_PATTERN_DONE:
    case RESP_INPUT_DETECTED:
    case RESP_INPUT_CORRECT:
    case RESP_INPUT_WRONG:
    case RESP_AUDIO_DONE:
      return true;
      
    case RESP_NACK:
    case RESP_BUSY:
    case RESP_ROBOT_ERROR:
    case RESP_PATTERN_ERROR:
    case RESP_AUDIO_ERROR:
    case RESP_ERROR:
      Serial.print("Slave 오류 응답: 0x");
      Serial.println(response->command, HEX);
      return false;
      
    default:
      Serial.print("알 수 없는 응답: 0x");
      Serial.println(response->command, HEX);
      return false;
  }
}

/**
 * 통신 성공 기록
 * @param responseTime 응답 시간 (ms)
 */
void recordCommSuccess(unsigned long responseTime) {
  commReliability.successfulComms++;
  commReliability.lastSuccessTime = millis();
  commReliability.slaveResponsive = true;
  
  // 응답 시간 통계 업데이트
  if (responseTime > commDiagnostics.maxResponseTime) {
    commDiagnostics.maxResponseTime = responseTime;
  }
  if (responseTime < commDiagnostics.minResponseTime) {
    commDiagnostics.minResponseTime = responseTime;
  }
  
  // 평균 응답 시간 계산 (이동 평균)
  if (commDiagnostics.avgResponseTime == 0) {
    commDiagnostics.avgResponseTime = responseTime;
  } else {
    commDiagnostics.avgResponseTime = (commDiagnostics.avgResponseTime * 9 + responseTime) / 10;
  }
  
  // 통신 품질 업데이트
  updateCommunicationQuality();
  
  if (commDiagnostics.enableDetailedLogging) {
    Serial.print("통신 성공 - 응답시간: ");
    Serial.print(responseTime);
    Serial.println("ms");
  }
}

/**
 * 통신 오류 기록
 * @param errorCode 오류 코드
 */
void recordCommError(uint8_t errorCode) {
  commReliability.lastErrorCode = errorCode;
  commReliability.failedComms++;
  
  // 통신 품질 업데이트
  updateCommunicationQuality();
  
  Serial.print("통신 오류 기록: ");
  switch (errorCode) {
    case COMM_ERROR_TIMEOUT:
      Serial.println("타임아웃");
      break;
    case COMM_ERROR_CHECKSUM:
      Serial.println("체크섬 오류");
      break;
    case COMM_ERROR_INVALID:
      Serial.println("잘못된 메시지");
      break;
    case COMM_ERROR_NO_RESPONSE:
      Serial.println("응답 없음");
      break;
    default:
      Serial.print("알 수 없는 오류: 0x");
      Serial.println(errorCode, HEX);
      break;
  }
}

/**
 * 통신 품질 계산 및 업데이트
 */
void updateCommunicationQuality() {
  if (commReliability.totalCommAttempts == 0) {
    commReliability.communicationQuality = 100;
    return;
  }
  
  // 성공률 기반 품질 계산
  uint8_t successRate = (commReliability.successfulComms * 100) / commReliability.totalCommAttempts;
  
  // 최근 연속 오류 고려
  uint8_t errorPenalty = commReliability.consecutiveErrors * 10;
  if (errorPenalty > 50) errorPenalty = 50;
  
  // 응답 시간 고려
  uint8_t timePenalty = 0;
  if (commDiagnostics.avgResponseTime > COMM_TIMEOUT / 2) {
    timePenalty = 10;
  }
  
  commReliability.communicationQuality = successRate - errorPenalty - timePenalty;
  if (commReliability.communicationQuality > 100) {
    commReliability.communicationQuality = 100;
  }
}

/**
 * 통신 복구 모드 시작
 */
void startCommRecovery() {
  if (commReliability.isRecovering) {
    return; // 이미 복구 중
  }
  
  Serial.println("=== 통신 복구 모드 시작 ===");
  Serial.print("연속 오류 횟수: ");
  Serial.println(commReliability.consecutiveErrors);
  
  commReliability.isRecovering = true;
  commReliability.recoveryStartTime = millis();
  commReliability.slaveResponsive = false;
  
  // 복구 절차 시작
  performCommRecovery();
}

/**
 * 통신 복구 절차 수행
 */
void performCommRecovery() {
  Serial.println("통신 복구 절차 수행 중...");
  
  // 1단계: 시리얼 버퍼 클리어
  Serial.println("1. 시리얼 버퍼 클리어");
  while (Serial.available() > 0) {
    Serial.read();
  }
  delay(100);
  
  // 2단계: 연결 테스트
  Serial.println("2. 연결 테스트");
  for (int i = 0; i < 3; i++) {
    if (testSlaveConnection()) {
      Serial.println("연결 테스트 성공");
      commReliability.isRecovering = false;
      commReliability.consecutiveErrors = 0;
      commReliability.slaveResponsive = true;
      Serial.println("=== 통신 복구 완료 ===");
      return;
    }
    delay(500);
  }
  
  // 3단계: 시스템 리셋 요청
  Serial.println("3. Slave 시스템 리셋 요청");
  sendCommandToSlave(CMD_RESET, 0, 0);
  delay(2000); // 리셋 대기
  
  // 4단계: 재연결 시도
  Serial.println("4. 재연결 시도");
  if (testSlaveConnection()) {
    Serial.println("재연결 성공");
    commReliability.isRecovering = false;
    commReliability.consecutiveErrors = 0;
    commReliability.slaveResponsive = true;
    Serial.println("=== 통신 복구 완료 ===");
  } else {
    Serial.println("통신 복구 실패 - 수동 개입 필요");
    // 복구 실패 시에도 복구 모드 해제 (무한 대기 방지)
    commReliability.isRecovering = false;
  }
}

/**
 * Slave 연결 테스트
 * @return 연결 상태
 */
bool testSlaveConnection() {
  Serial.println("Slave 연결 테스트 중...");
  
  // 상태 요청 명령 전송
  SerialMessage testMsg;
  testMsg.header = MESSAGE_HEADER;
  testMsg.command = CMD_GET_STATUS;
  testMsg.data1 = 0x00;
  testMsg.data2 = 0x00;
  testMsg.checksum = testMsg.command ^ testMsg.data1 ^ testMsg.data2;
  
  // 메시지 전송
  Serial.write((uint8_t*)&testMsg, sizeof(SerialMessage));
  Serial.flush();
  
  // 응답 대기
  SerialMessage response;
  if (receiveFromSlaveWithTimeout(&response, 2000)) { // 2초 타임아웃
    if (response.command == RESP_ACK) {
      Serial.println("Slave 응답 확인");
      return true;
    }
  }
  
  Serial.println("Slave 응답 없음");
  return false;
}

/**
 * 복구 완료 대기
 * @return 복구 성공 여부
 */
bool waitForRecoveryComplete() {
  unsigned long startTime = millis();
  
  while (commReliability.isRecovering && 
         millis() - startTime < commReliability.recoveryTimeout) {
    delay(100);
  }
  
  return !commReliability.isRecovering;
}

/**
 * 하트비트 전송 (연결 상태 확인)
 */
void sendHeartbeat() {
  unsigned long currentTime = millis();
  
  // 하트비트 간격 확인
  if (currentTime - commReliability.lastHeartbeatTime < commReliability.connectionCheckInterval) {
    return;
  }
  
  commReliability.lastHeartbeatTime = currentTime;
  
  // 간단한 상태 요청으로 하트비트 구현
  if (sendCommandToSlave(CMD_GET_STATUS, 0, 0)) {
    SerialMessage response;
    if (receiveFromSlaveWithTimeout(&response, 1000)) {
      commReliability.slaveResponsive = true;
      if (commDiagnostics.enableDetailedLogging) {
        Serial.println("하트비트 성공");
      }
    } else {
      commReliability.slaveResponsive = false;
      Serial.println("하트비트 실패 - Slave 무응답");
    }
  }
}

/**
 * 통신 안정성 모니터링 (메인 루프에서 호출)
 */
void monitorCommReliability() {
  unsigned long currentTime = millis();
  
  // 하트비트 전송
  sendHeartbeat();
  
  // 장시간 통신 없음 감지
  if (currentTime - commReliability.lastSuccessTime > 30000) { // 30초
    if (commReliability.slaveResponsive) {
      Serial.println("장시간 통신 없음 감지");
      commReliability.slaveResponsive = false;
    }
  }
  
  // 복구 모드 타임아웃 체크
  if (commReliability.isRecovering && 
      currentTime - commReliability.recoveryStartTime > commReliability.recoveryTimeout) {
    Serial.println("복구 모드 타임아웃 - 강제 종료");
    commReliability.isRecovering = false;
  }
}

/**
 * 통신 진단 정보 업데이트
 */
void updateCommDiagnostics() {
  unsigned long currentTime = millis();
  
  // 주기적 진단 정보 출력 (1분마다)
  if (currentTime - commDiagnostics.lastDiagnosticTime > 60000) {
    commDiagnostics.lastDiagnosticTime = currentTime;
    
    if (commDiagnostics.enableDetailedLogging) {
      printCommDiagnostics();
    }
  }
}

/**
 * 통신 진단 정보 출력
 */
void printCommDiagnostics() {
  Serial.println("=== 통신 진단 정보 ===");
  
  // 기본 통계
  Serial.print("총 통신 시도: ");
  Serial.println(commReliability.totalCommAttempts);
  Serial.print("성공한 통신: ");
  Serial.println(commReliability.successfulComms);
  Serial.print("실패한 통신: ");
  Serial.println(commReliability.failedComms);
  
  // 성공률 계산
  if (commReliability.totalCommAttempts > 0) {
    float successRate = (float)commReliability.successfulComms / commReliability.totalCommAttempts * 100;
    Serial.print("성공률: ");
    Serial.print(successRate, 1);
    Serial.println("%");
  }
  
  // 통신 품질
  Serial.print("통신 품질: ");
  Serial.print(commReliability.communicationQuality);
  Serial.println("%");
  
  // 응답 시간 통계
  Serial.print("평균 응답시간: ");
  Serial.print(commDiagnostics.avgResponseTime);
  Serial.println("ms");
  Serial.print("최대 응답시간: ");
  Serial.print(commDiagnostics.maxResponseTime);
  Serial.println("ms");
  Serial.print("최소 응답시간: ");
  Serial.print(commDiagnostics.minResponseTime);
  Serial.println("ms");
  
  // 오류 통계
  Serial.print("타임아웃 횟수: ");
  Serial.println(commDiagnostics.timeoutCount);
  Serial.print("체크섬 오류: ");
  Serial.println(commDiagnostics.checksumErrorCount);
  Serial.print("잘못된 메시지: ");
  Serial.println(commDiagnostics.invalidMessageCount);
  Serial.print("재전송 횟수: ");
  Serial.println(commDiagnostics.retransmissionCount);
  
  // 현재 상태
  Serial.print("Slave 응답 상태: ");
  Serial.println(commReliability.slaveResponsive ? "정상" : "무응답");
  Serial.print("복구 모드: ");
  Serial.println(commReliability.isRecovering ? "활성" : "비활성");
  Serial.print("연속 오류 횟수: ");
  Serial.println(commReliability.consecutiveErrors);
  
  Serial.println("====================");
}

/**
 * 통신 상태 리셋
 */
void resetCommReliability() {
  Serial.println("통신 상태 리셋");
  
  commReliability.retryCount = 0;
  commReliability.consecutiveErrors = 0;
  commReliability.isRecovering = false;
  commReliability.lastErrorCode = COMM_ERROR_NONE;
  commReliability.slaveResponsive = false;
  commReliability.communicationQuality = 100;
  
  // 진단 정보도 리셋
  commDiagnostics.timeoutCount = 0;
  commDiagnostics.checksumErrorCount = 0;
  commDiagnostics.invalidMessageCount = 0;
  commDiagnostics.retransmissionCount = 0;
  commDiagnostics.maxResponseTime = 0;
  commDiagnostics.minResponseTime = UINT32_MAX;
  commDiagnostics.avgResponseTime = 0;
  
  Serial.println("통신 상태 리셋 완료");
}

/**
 * 통신 진단 모드 토글
 */
void toggleCommDiagnosticMode() {
  commDiagnostics.enableDetailedLogging = !commDiagnostics.enableDetailedLogging;
  Serial.print("통신 진단 모드: ");
  Serial.println(commDiagnostics.enableDetailedLogging ? "활성화" : "비활성화");
}

/**
 * 통신 품질 확인
 * @return 통신 품질 (0-100%)
 */
uint8_t getCommQuality() {
  return commReliability.communicationQuality;
}

/**
 * Slave 응답 상태 확인
 * @return Slave 응답 여부
 */
bool isSlaveResponsive() {
  return commReliability.slaveResponsive;
}

/**
 * 통신 복구 모드 상태 확인
 * @return 복구 모드 여부
 */
bool isCommRecovering() {
  return commReliability.isRecovering;
}

// =============================================================================
// 하드웨어 안전장치 강화 시스템 구현 (Task 17.2)
// =============================================================================

/**
 * 하드웨어 안전장치 시스템 초기화
 * 요구사항 7.2, 7.6: 하드웨어 안전장치 강화
 */
void initHardwareSafety() {
  Serial.println("하드웨어 안전장치 시스템 초기화 시작...");
  
  // 전원 모니터링 초기화
  initPowerMonitoring();
  
  // 센서 안전장치 초기화
  initSensorSafety();
  
  // 모터 안전장치 초기화
  initMotorSafety();
  
  Serial.println("하드웨어 안전장치 시스템 초기화 완료");
}

/**
 * 전원 모니터링 시스템 초기화
 */
void initPowerMonitoring() {
  powerMonitor.currentVoltage = 5.0f;
  powerMonitor.minVoltage = POWER_VOLTAGE_MIN;
  powerMonitor.maxVoltage = POWER_VOLTAGE_MAX;
  powerMonitor.lowVoltageWarning = false;
  powerMonitor.highVoltageWarning = false;
  powerMonitor.lastVoltageCheck = millis();
  powerMonitor.voltageCheckInterval = POWER_CHECK_INTERVAL;
  powerMonitor.lowVoltageCount = 0;
  powerMonitor.highVoltageCount = 0;
  powerMonitor.powerStable = true;
  powerMonitor.historyIndex = 0;
  powerMonitor.emergencyPowerShutdown = false;
  
  // 전압 이력 초기화
  for (int i = 0; i < POWER_HISTORY_SIZE; i++) {
    powerMonitor.voltageHistory[i] = 5.0f;
  }
  
  Serial.println("전원 모니터링 시스템 초기화 완료");
  Serial.print("동작 전압 범위: ");
  Serial.print(POWER_VOLTAGE_MIN);
  Serial.print("V - ");
  Serial.print(POWER_VOLTAGE_MAX);
  Serial.println("V");
}

/**
 * 센서 안전장치 시스템 초기화
 */
void initSensorSafety() {
  // PIR 센서 안전장치 초기화
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    sensorSafety.pirSensorStatus[i] = true;
    sensorSafety.pirErrorCount[i] = 0;
    sensorSafety.pirLastValidTime[i] = millis();
    sensorSafety.pirSensorRecovering[i] = false;
    sensorSafety.pirRecoveryStartTime[i] = 0;
  }
  
  // DFPlayer 안전장치 초기화
  sensorSafety.dfPlayerHealthy = true;
  sensorSafety.dfPlayerErrorCount = 0;
  sensorSafety.dfPlayerLastResponse = millis();
  sensorSafety.dfPlayerRecovering = false;
  
  // 시스템 온도 모니터링 초기화
  sensorSafety.systemOverheated = false;
  sensorSafety.systemTemperature = 25.0f; // 기본 온도
  sensorSafety.lastTemperatureCheck = millis();
  
  // 전체 시스템 안전장치 초기화
  sensorSafety.totalSystemErrors = 0;
  sensorSafety.emergencySafeMode = false;
  sensorSafety.safeModeTriggerTime = 0;
  
  Serial.println("센서 안전장치 시스템 초기화 완료");
  Serial.print("PIR 센서 수: ");
  Serial.println(PIR_SENSOR_COUNT);
  Serial.print("센서 오류 임계값: ");
  Serial.println(SENSOR_ERROR_THRESHOLD);
}

/**
 * 모터 안전장치 시스템 초기화
 */
void initMotorSafety() {
  motorSafety.slaveMotorHealthy = true;
  motorSafety.lastMotorResponse = millis();
  motorSafety.motorTimeoutCount = 0;
  motorSafety.motorErrorCount = 0;
  motorSafety.motorOverloadDetected = false;
  motorSafety.motorOverloadTime = 0;
  motorSafety.motorEmergencyStop = false;
  motorSafety.motorRecoveryAttempts = 0;
  motorSafety.lastMotorCommand = 0;
  motorSafety.motorSafetyLockout = false;
  
  Serial.println("모터 안전장치 시스템 초기화 완료");
  Serial.print("모터 응답 타임아웃: ");
  Serial.print(MOTOR_RESPONSE_TIMEOUT);
  Serial.println("ms");
  Serial.print("모터 오류 임계값: ");
  Serial.println(MOTOR_ERROR_THRESHOLD);
}

/**
 * 하드웨어 안전장치 모니터링 (메인 루프에서 호출)
 */
void monitorHardwareSafety() {
  // 전원 안정성 모니터링
  monitorPowerStability();
  
  // 센서 상태 모니터링
  monitorSensorHealth();
  
  // 모터 안전장치 모니터링
  monitorMotorSafety();
  
  // 비상 안전 모드 체크
  checkEmergencySafeMode();
}

/**
 * 전원 안정성 모니터링
 * 요구사항 7.2: 전원 안정성 모니터링 및 보호 기능
 */
void monitorPowerStability() {
  unsigned long currentTime = millis();
  
  // 전압 체크 주기 확인
  if (currentTime - powerMonitor.lastVoltageCheck < powerMonitor.voltageCheckInterval) {
    return;
  }
  
  powerMonitor.lastVoltageCheck = currentTime;
  
  // 전압 측정 (아날로그 핀을 통한 전압 분배 회로 사용)
  // 실제 구현에서는 적절한 아날로그 핀과 분배 비율 사용
  float measuredVoltage = measureSystemVoltage();
  
  // 전압 이력에 추가
  powerMonitor.voltageHistory[powerMonitor.historyIndex] = measuredVoltage;
  powerMonitor.historyIndex = (powerMonitor.historyIndex + 1) % POWER_HISTORY_SIZE;
  
  // 이동 평균 계산
  float avgVoltage = 0;
  for (int i = 0; i < POWER_HISTORY_SIZE; i++) {
    avgVoltage += powerMonitor.voltageHistory[i];
  }
  avgVoltage /= POWER_HISTORY_SIZE;
  powerMonitor.currentVoltage = avgVoltage;
  
  // 전압 범위 체크
  bool previousStable = powerMonitor.powerStable;
  powerMonitor.powerStable = true;
  
  // 저전압 체크
  if (powerMonitor.currentVoltage < POWER_VOLTAGE_MIN) {
    if (!powerMonitor.lowVoltageWarning) {
      Serial.print("경고: 저전압 감지 - ");
      Serial.print(powerMonitor.currentVoltage);
      Serial.println("V");
      powerMonitor.lowVoltageWarning = true;
      powerMonitor.lowVoltageCount++;
    }
    powerMonitor.powerStable = false;
    
    // 임계 저전압 체크
    if (powerMonitor.currentVoltage < POWER_VOLTAGE_CRITICAL_LOW) {
      triggerEmergencyPowerShutdown("임계 저전압");
    }
  } else {
    powerMonitor.lowVoltageWarning = false;
  }
  
  // 고전압 체크
  if (powerMonitor.currentVoltage > POWER_VOLTAGE_MAX) {
    if (!powerMonitor.highVoltageWarning) {
      Serial.print("경고: 고전압 감지 - ");
      Serial.print(powerMonitor.currentVoltage);
      Serial.println("V");
      powerMonitor.highVoltageWarning = true;
      powerMonitor.highVoltageCount++;
    }
    powerMonitor.powerStable = false;
    
    // 임계 고전압 체크
    if (powerMonitor.currentVoltage > POWER_VOLTAGE_CRITICAL_HIGH) {
      triggerEmergencyPowerShutdown("임계 고전압");
    }
  } else {
    powerMonitor.highVoltageWarning = false;
  }
  
  // 전원 안정성 상태 변화 로깅
  if (previousStable != powerMonitor.powerStable) {
    Serial.print("전원 안정성 상태 변경: ");
    Serial.println(powerMonitor.powerStable ? "안정" : "불안정");
  }
}

/**
 * 시스템 전압 측정
 * @return 측정된 전압 (V)
 */
float measureSystemVoltage() {
  // 실제 구현에서는 전압 분배 회로를 통해 측정
  // 여기서는 시뮬레이션을 위해 기본값 반환
  // 아날로그 핀 A15를 전압 측정용으로 사용 (예시)
  
  int analogValue = analogRead(A15);
  
  // 전압 분배 비율에 따른 계산 (예: 2:1 분배)
  // 실제 회로에 맞게 조정 필요
  float voltage = (analogValue / 1023.0f) * 5.0f * 2.0f;
  
  // 노이즈 필터링을 위한 범위 제한
  if (voltage < 3.0f) voltage = 3.0f;
  if (voltage > 7.0f) voltage = 7.0f;
  
  return voltage;
}

/**
 * 비상 전원 차단 트리거
 * @param reason 차단 사유
 */
void triggerEmergencyPowerShutdown(const char* reason) {
  if (powerMonitor.emergencyPowerShutdown) {
    return; // 이미 차단된 상태
  }
  
  Serial.print("비상 전원 차단 트리거: ");
  Serial.println(reason);
  
  powerMonitor.emergencyPowerShutdown = true;
  
  // 모든 고전력 장치 즉시 차단
  emergencyShutdownAllDevices();
  
  // 비상 안전 모드 진입
  enterEmergencySafeMode("전원 이상");
}

/**
 * 센서 상태 모니터링
 * 요구사항 7.6: 센서 오작동 감지 및 복구 로직 추가
 */
void monitorSensorHealth() {
  unsigned long currentTime = millis();
  
  // PIR 센서 상태 모니터링
  monitorPIRSensorHealth(currentTime);
  
  // DFPlayer 상태 모니터링
  monitorDFPlayerHealth(currentTime);
  
  // 시스템 온도 모니터링
  monitorSystemTemperature(currentTime);
}

/**
 * PIR 센서 상태 모니터링
 * @param currentTime 현재 시간
 */
void monitorPIRSensorHealth(unsigned long currentTime) {
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    // 센서 응답 타임아웃 체크
    if (currentTime - sensorSafety.pirLastValidTime[i] > PIR_VALID_RESPONSE_TIMEOUT) {
      if (sensorSafety.pirSensorStatus[i]) {
        Serial.print("PIR 센서 ");
        Serial.print(i + 1);
        Serial.println(" 응답 타임아웃 감지");
        
        sensorSafety.pirSensorStatus[i] = false;
        sensorSafety.pirErrorCount[i]++;
        sensorSafety.totalSystemErrors++;
        
        // 센서 복구 시도
        if (sensorSafety.pirErrorCount[i] >= SENSOR_ERROR_THRESHOLD) {
          startPIRSensorRecovery(i);
        }
      }
    }
    
    // 센서 복구 프로세스 모니터링
    if (sensorSafety.pirSensorRecovering[i]) {
      if (currentTime - sensorSafety.pirRecoveryStartTime[i] > SENSOR_RECOVERY_TIMEOUT) {
        Serial.print("PIR 센서 ");
        Serial.print(i + 1);
        Serial.println(" 복구 타임아웃");
        
        sensorSafety.pirSensorRecovering[i] = false;
        // 센서를 비활성화 상태로 유지
      }
    }
  }
}

/**
 * PIR 센서 복구 시작
 * @param sensorIndex 센서 인덱스
 */
void startPIRSensorRecovery(uint8_t sensorIndex) {
  if (sensorIndex >= PIR_SENSOR_COUNT) return;
  
  Serial.print("PIR 센서 ");
  Serial.print(sensorIndex + 1);
  Serial.println(" 복구 시작");
  
  sensorSafety.pirSensorRecovering[sensorIndex] = true;
  sensorSafety.pirRecoveryStartTime[sensorIndex] = millis();
  
  // 센서 재초기화 시도
  // 실제 구현에서는 센서별 재초기화 로직 추가
  
  // 센서 상태 리셋
  pirState.currentState[sensorIndex] = false;
  pirState.previousState[sensorIndex] = false;
  pirState.lastTriggerTime[sensorIndex] = 0;
  pirState.motionDetected[sensorIndex] = false;
}

/**
 * DFPlayer 상태 모니터링
 * @param currentTime 현재 시간
 */
void monitorDFPlayerHealth(unsigned long currentTime) {
  // DFPlayer 응답 타임아웃 체크
  if (currentTime - sensorSafety.dfPlayerLastResponse > 10000) { // 10초 타임아웃
    if (sensorSafety.dfPlayerHealthy) {
      Serial.println("DFPlayer 응답 타임아웃 감지");
      sensorSafety.dfPlayerHealthy = false;
      sensorSafety.dfPlayerErrorCount++;
      sensorSafety.totalSystemErrors++;
      
      // DFPlayer 복구 시도
      if (sensorSafety.dfPlayerErrorCount >= 3) {
        startDFPlayerRecovery();
      }
    }
  }
  
  // DFPlayer 복구 프로세스 모니터링
  if (sensorSafety.dfPlayerRecovering) {
    // 복구 상태 체크 로직
    if (dfPlayerState.isInitialized && dfPlayerState.isPlaying) {
      Serial.println("DFPlayer 복구 성공");
      sensorSafety.dfPlayerRecovering = false;
      sensorSafety.dfPlayerHealthy = true;
      sensorSafety.dfPlayerLastResponse = currentTime;
    }
  }
}

/**
 * DFPlayer 복구 시작
 */
void startDFPlayerRecovery() {
  Serial.println("DFPlayer 복구 시작");
  
  sensorSafety.dfPlayerRecovering = true;
  
  // DFPlayer 재초기화
  initializeDFPlayer();
  
  sensorSafety.dfPlayerLastResponse = millis();
}

/**
 * 시스템 온도 모니터링
 * @param currentTime 현재 시간
 */
void monitorSystemTemperature(unsigned long currentTime) {
  // 온도 체크 주기 (5초마다)
  if (currentTime - sensorSafety.lastTemperatureCheck < 5000) {
    return;
  }
  
  sensorSafety.lastTemperatureCheck = currentTime;
  
  // 시스템 온도 추정 (실제 온도 센서가 없는 경우)
  // CPU 사용률, 동작 시간, 환경 요인을 고려한 추정
  float estimatedTemp = estimateSystemTemperature();
  sensorSafety.systemTemperature = estimatedTemp;
  
  // 과열 체크
  if (sensorSafety.systemTemperature > SYSTEM_TEMPERATURE_MAX) {
    if (!sensorSafety.systemOverheated) {
      Serial.print("시스템 과열 감지: ");
      Serial.print(sensorSafety.systemTemperature);
      Serial.println("°C");
      
      sensorSafety.systemOverheated = true;
      sensorSafety.totalSystemErrors++;
      
      // 과열 보호 조치
      triggerOverheatProtection();
    }
  } else if (sensorSafety.systemTemperature < SYSTEM_TEMPERATURE_MAX - 10) {
    // 온도가 충분히 낮아지면 과열 상태 해제
    if (sensorSafety.systemOverheated) {
      Serial.println("시스템 온도 정상화");
      sensorSafety.systemOverheated = false;
    }
  }
}

/**
 * 시스템 온도 추정
 * @return 추정 온도 (°C)
 */
float estimateSystemTemperature() {
  // 기본 환경 온도
  float baseTemp = 25.0f;
  
  // 동작 시간에 따른 온도 상승 (간단한 모델)
  unsigned long uptime = millis() / 1000; // 초 단위
  float uptimeTemp = uptime * 0.001f; // 초당 0.001도 상승
  
  // 게임 활성 상태에 따른 온도 상승
  float activityTemp = gameState.isGameActive ? 5.0f : 0.0f;
  
  // 모터 동작에 따른 온도 상승 (Slave 모터 상태 추정)
  float motorTemp = (motorSafety.slaveMotorHealthy && gameState.isGameActive) ? 3.0f : 0.0f;
  
  // 전체 온도 계산
  float totalTemp = baseTemp + uptimeTemp + activityTemp + motorTemp;
  
  // 현실적인 범위로 제한
  if (totalTemp > 80.0f) totalTemp = 80.0f;
  if (totalTemp < 20.0f) totalTemp = 20.0f;
  
  return totalTemp;
}

/**
 * 과열 보호 조치
 */
void triggerOverheatProtection() {
  Serial.println("과열 보호 조치 실행");
  
  // 고전력 장치 일시 정지
  // 1. 모든 LED 밝기 감소 또는 일시 정지
  for (int i = 0; i < 6; i++) {
    digitalWrite(EFFECT_LED_PINS[i], LOW);
  }
  
  // 2. 모터 동작 일시 정지 (Slave에 명령 전송)
  sendCommandToSlave(CMD_ROBOT_STOP, 0, 0);
  
  // 3. 오디오 볼륨 감소
  // DFPlayer 볼륨을 절반으로 감소
  
  // 4. 게임 일시 정지 (필요시)
  if (gameState.isGameActive) {
    Serial.println("과열로 인한 게임 일시 정지");
    // 게임 상태를 일시 정지 상태로 변경
  }
  
  Serial.println("과열 보호 조치 완료 - 시스템 냉각 대기");
}

/**
 * 모터 안전장치 모니터링
 * 요구사항 7.2: 모터 과부하 및 타임아웃 보호 강화
 */
void monitorMotorSafety() {
  unsigned long currentTime = millis();
  
  // 모터 응답 타임아웃 체크
  if (motorSafety.lastMotorCommand > 0 && 
      currentTime - motorSafety.lastMotorResponse > MOTOR_RESPONSE_TIMEOUT) {
    
    if (motorSafety.slaveMotorHealthy) {
      Serial.println("모터 응답 타임아웃 감지");
      motorSafety.slaveMotorHealthy = false;
      motorSafety.motorTimeoutCount++;
      motorSafety.motorErrorCount++;
      
      // 모터 복구 시도
      if (motorSafety.motorErrorCount >= MOTOR_ERROR_THRESHOLD) {
        startMotorRecovery();
      }
    }
  }
  
  // 모터 과부하 타임아웃 체크
  if (motorSafety.motorOverloadDetected && 
      currentTime - motorSafety.motorOverloadTime > MOTOR_OVERLOAD_TIMEOUT) {
    
    Serial.println("모터 과부하 타임아웃");
    triggerMotorSafetyLockout();
  }
  
  // 안전 잠금 해제 체크
  if (motorSafety.motorSafetyLockout && 
      currentTime - motorSafety.motorOverloadTime > MOTOR_SAFETY_LOCKOUT_TIME) {
    
    Serial.println("모터 안전 잠금 해제");
    motorSafety.motorSafetyLockout = false;
    motorSafety.motorRecoveryAttempts = 0;
  }
}

/**
 * 모터 복구 시작
 */
void startMotorRecovery() {
  if (motorSafety.motorRecoveryAttempts >= MOTOR_RECOVERY_MAX_ATTEMPTS) {
    Serial.println("모터 복구 시도 횟수 초과 - 안전 잠금");
    triggerMotorSafetyLockout();
    return;
  }
  
  Serial.print("모터 복구 시도 ");
  Serial.print(motorSafety.motorRecoveryAttempts + 1);
  Serial.print("/");
  Serial.println(MOTOR_RECOVERY_MAX_ATTEMPTS);
  
  motorSafety.motorRecoveryAttempts++;
  
  // 모터 리셋 명령 전송
  sendCommandToSlave(CMD_RESET, 0, 0);
  
  // 복구 대기 시간
  delay(1000);
  
  // 모터 상태 확인 명령 전송
  sendCommandToSlave(CMD_GET_STATUS, 0, 0);
  
  motorSafety.lastMotorCommand = millis();
}

/**
 * 모터 안전 잠금 트리거
 */
void triggerMotorSafetyLockout() {
  Serial.println("모터 안전 잠금 트리거");
  
  motorSafety.motorSafetyLockout = true;
  motorSafety.motorEmergencyStop = true;
  motorSafety.motorOverloadTime = millis();
  
  // 모든 모터 즉시 정지
  sendCommandToSlave(CMD_STOP_ALL, 0, 0);
  
  // 게임 중단
  if (gameState.isGameActive) {
    Serial.println("모터 안전 문제로 인한 게임 중단");
    transitionToPhase(PHASE_IDLE);
    gameState.isGameActive = false;
  }
}

/**
 * 비상 안전 모드 체크
 */
void checkEmergencySafeMode() {
  // 총 시스템 오류가 임계값을 초과하면 비상 안전 모드 진입
  if (!sensorSafety.emergencySafeMode && 
      sensorSafety.totalSystemErrors >= 10) {
    
    enterEmergencySafeMode("시스템 오류 임계값 초과");
  }
  
  // 비상 안전 모드 타임아웃 체크
  if (sensorSafety.emergencySafeMode && 
      millis() - sensorSafety.safeModeTriggerTime > EMERGENCY_SAFE_MODE_TIMEOUT) {
    
    Serial.println("비상 안전 모드 타임아웃 - 시스템 재시작 권장");
  }
}

/**
 * 비상 안전 모드 진입
 * @param reason 진입 사유
 */
void enterEmergencySafeMode(const char* reason) {
  if (sensorSafety.emergencySafeMode) {
    return; // 이미 안전 모드
  }
  
  Serial.print("비상 안전 모드 진입: ");
  Serial.println(reason);
  
  sensorSafety.emergencySafeMode = true;
  sensorSafety.safeModeTriggerTime = millis();
  
  // 모든 장치 안전 정지
  emergencyShutdownAllDevices();
  
  // 게임 강제 종료
  if (gameState.isGameActive) {
    transitionToPhase(PHASE_IDLE);
    gameState.isGameActive = false;
  }
  
  // 안전 모드 표시 (경광등 점멸)
  blinkBeaconLights(false, true, 10, 200); // 빨간 경광등 10회 점멸
}

/**
 * 모든 장치 비상 정지
 */
void emergencyShutdownAllDevices() {
  Serial.println("모든 장치 비상 정지 실행");
  
  // 1. 모든 LED 소등
  for (int i = 0; i < 4; i++) {
    digitalWrite(CEILING_LIGHT_PINS[i], LOW);
  }
  for (int i = 0; i < 6; i++) {
    digitalWrite(EFFECT_LED_PINS[i], LOW);
  }
  digitalWrite(GREEN_LIGHT_RELAY_PIN, LOW);
  digitalWrite(RED_LIGHT_RELAY_PIN, LOW);
  
  // 2. 모든 모터 정지
  sendCommandToSlave(CMD_STOP_ALL, 0, 0);
  
  // 3. 오디오 정지
  sendCommandToSlave(CMD_STOP_AUDIO, 0, 0);
  
  // 4. EM-Lock 비활성화 (안전을 위해)
  for (int i = 0; i < 4; i++) {
    digitalWrite(EMLOCK_PINS[i], LOW);
  }
  
  Serial.println("비상 정지 완료");
}

/**
 * 하드웨어 안전장치 상태 출력 (디버깅용)
 */
void printHardwareSafetyStatus() {
  Serial.println("=== 하드웨어 안전장치 상태 ===");
  
  // 전원 모니터링 상태
  Serial.println("--- 전원 모니터링 ---");
  Serial.print("현재 전압: ");
  Serial.print(powerMonitor.currentVoltage);
  Serial.println("V");
  Serial.print("전원 안정성: ");
  Serial.println(powerMonitor.powerStable ? "안정" : "불안정");
  Serial.print("저전압 경고: ");
  Serial.println(powerMonitor.lowVoltageWarning ? "활성" : "비활성");
  Serial.print("고전압 경고: ");
  Serial.println(powerMonitor.highVoltageWarning ? "활성" : "비활성");
  Serial.print("저전압 발생 횟수: ");
  Serial.println(powerMonitor.lowVoltageCount);
  Serial.print("고전압 발생 횟수: ");
  Serial.println(powerMonitor.highVoltageCount);
  
  // 센서 안전장치 상태
  Serial.println("--- 센서 안전장치 ---");
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    Serial.print("PIR 센서 ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(sensorSafety.pirSensorStatus[i] ? "정상" : "오작동");
    Serial.print(" (오류 ");
    Serial.print(sensorSafety.pirErrorCount[i]);
    Serial.println("회)");
  }
  Serial.print("DFPlayer 상태: ");
  Serial.println(sensorSafety.dfPlayerHealthy ? "정상" : "오작동");
  Serial.print("시스템 온도: ");
  Serial.print(sensorSafety.systemTemperature);
  Serial.println("°C");
  Serial.print("과열 상태: ");
  Serial.println(sensorSafety.systemOverheated ? "과열" : "정상");
  
  // 모터 안전장치 상태
  Serial.println("--- 모터 안전장치 ---");
  Serial.print("Slave 모터 상태: ");
  Serial.println(motorSafety.slaveMotorHealthy ? "정상" : "오작동");
  Serial.print("모터 타임아웃 횟수: ");
  Serial.println(motorSafety.motorTimeoutCount);
  Serial.print("모터 오류 횟수: ");
  Serial.println(motorSafety.motorErrorCount);
  Serial.print("과부하 감지: ");
  Serial.println(motorSafety.motorOverloadDetected ? "감지됨" : "정상");
  Serial.print("안전 잠금: ");
  Serial.println(motorSafety.motorSafetyLockout ? "활성" : "비활성");
  
  // 전체 시스템 상태
  Serial.println("--- 전체 시스템 ---");
  Serial.print("총 시스템 오류: ");
  Serial.println(sensorSafety.totalSystemErrors);
  Serial.print("비상 안전 모드: ");
  Serial.println(sensorSafety.emergencySafeMode ? "활성" : "비활성");
  Serial.print("비상 전원 차단: ");
  Serial.println(powerMonitor.emergencyPowerShutdown ? "활성" : "비활성");
  
  Serial.println("=============================");
}

/**
 * 하드웨어 안전장치 리셋
 */
void resetHardwareSafety() {
  Serial.println("하드웨어 안전장치 리셋");
  
  // 전원 모니터링 리셋
  powerMonitor.lowVoltageCount = 0;
  powerMonitor.highVoltageCount = 0;
  powerMonitor.lowVoltageWarning = false;
  powerMonitor.highVoltageWarning = false;
  powerMonitor.emergencyPowerShutdown = false;
  
  // 센서 안전장치 리셋
  for (int i = 0; i < PIR_SENSOR_COUNT; i++) {
    sensorSafety.pirErrorCount[i] = 0;
    sensorSafety.pirSensorStatus[i] = true;
    sensorSafety.pirSensorRecovering[i] = false;
  }
  sensorSafety.dfPlayerErrorCount = 0;
  sensorSafety.dfPlayerHealthy = true;
  sensorSafety.dfPlayerRecovering = false;
  sensorSafety.systemOverheated = false;
  sensorSafety.totalSystemErrors = 0;
  sensorSafety.emergencySafeMode = false;
  
  // 모터 안전장치 리셋
  motorSafety.motorTimeoutCount = 0;
  motorSafety.motorErrorCount = 0;
  motorSafety.motorOverloadDetected = false;
  motorSafety.motorEmergencyStop = false;
  motorSafety.motorRecoveryAttempts = 0;
  motorSafety.motorSafetyLockout = false;
  motorSafety.slaveMotorHealthy = true;
  
  Serial.println("하드웨어 안전장치 리셋 완료");
}

// =============================================================================
// 게임 로직 안정성 검증 시스템 구현 (Task 17.3)
// =============================================================================

/**
 * 게임 로직 안정성 검증 시스템 초기화
 */
void initGameLogicStability() {
  Serial.println("게임 로직 안정성 검증 시스템 초기화...");
  
  // 게임 상태 복구 시스템 초기화
  initGameStateRecovery();
  
  // 사용자 입력 오류 처리 시스템 초기화
  initUserInputErrorHandler();
  
  // 시스템 리셋 관리자 초기화
  initSystemResetManager();
  
  Serial.println("게임 로직 안정성 검증 시스템 초기화 완료");
}

/**
 * 게임 상태 복구 시스템 초기화
 */
void initGameStateRecovery() {
  gameRecovery.backupState = gameState;
  gameRecovery.lastBackupTime = millis();
  gameRecovery.recoveryAttempts = 0;
  gameRecovery.maxRecoveryAttempts = GAME_RECOVERY_MAX_ATTEMPTS;
  gameRecovery.recoveryInProgress = false;
  gameRecovery.recoveryStartTime = 0;
  gameRecovery.lastValidPhase = PHASE_IDLE;
  gameRecovery.stateCorrupted = false;
  gameRecovery.corruptionCount = 0;
  gameRecovery.stateValidationTime = millis();
  gameRecovery.autoRecoveryEnabled = true;
  gameRecovery.criticalErrorCount = 0;
  gameRecovery.emergencyMode = false;
  
  Serial.println("게임 상태 복구 시스템 초기화 완료");
}

/**
 * 사용자 입력 오류 처리 시스템 초기화
 */
void initUserInputErrorHandler() {
  inputErrorHandler.invalidInputCount = 0;
  inputErrorHandler.consecutiveErrors = 0;
  inputErrorHandler.lastErrorTime = 0;
  inputErrorHandler.guidanceActive = false;
  inputErrorHandler.guidanceLevel = GUIDANCE_LEVEL_NONE;
  inputErrorHandler.guidanceStartTime = 0;
  inputErrorHandler.errorPatternIndex = 0;
  inputErrorHandler.adaptiveGuidance = true;
  inputErrorHandler.userSkillLevel = 3; // 중간 레벨로 시작
  inputErrorHandler.totalInputAttempts = 0;
  inputErrorHandler.successfulInputs = 0;
  inputErrorHandler.successRate = 1.0f;
  inputErrorHandler.errorRecoveryMode = false;
  inputErrorHandler.timeoutExtension = 0;
  
  // 오류 패턴 배열 초기화
  for (int i = 0; i < 10; i++) {
    inputErrorHandler.errorPattern[i] = 0;
  }
  
  Serial.println("사용자 입력 오류 처리 시스템 초기화 완료");
}

/**
 * 시스템 리셋 관리자 초기화
 */
void initSystemResetManager() {
  resetManager.softResetRequested = false;
  resetManager.hardResetRequested = false;
  resetManager.resetRequestTime = 0;
  resetManager.resetReason = 0;
  resetManager.resetAttempts = 0;
  resetManager.resetInProgress = false;
  resetManager.resetStartTime = 0;
  resetManager.resetPhase = 0;
  resetManager.emergencyReset = false;
  resetManager.systemUptime = millis();
  resetManager.totalResets = 0;
  resetManager.softResets = 0;
  resetManager.hardResets = 0;
  resetManager.resetValidation = true;
  resetManager.lastSuccessfulReset = millis();
  resetManager.factoryResetMode = false;
  
  Serial.println("시스템 리셋 관리자 초기화 완료");
}

/**
 * 게임 로직 안정성 모니터링 (메인 루프에서 호출)
 */
void monitorGameLogicStability() {
  unsigned long currentTime = millis();
  
  // 게임 상태 백업 및 검증
  if (currentTime - gameRecovery.lastBackupTime >= GAME_STATE_BACKUP_INTERVAL) {
    backupGameState();
    gameRecovery.lastBackupTime = currentTime;
  }
  
  // 게임 상태 유효성 검증
  if (currentTime - gameRecovery.stateValidationTime >= GAME_STATE_VALIDATION_INTERVAL) {
    validateGameState();
    gameRecovery.stateValidationTime = currentTime;
  }
  
  // 게임 상태 복구 처리
  if (gameRecovery.recoveryInProgress) {
    processGameStateRecovery();
  }
  
  // 사용자 입력 오류 모니터링
  monitorUserInputErrors();
  
  // 시스템 리셋 요청 처리
  if (resetManager.resetInProgress) {
    processSystemReset();
  }
  
  // 비상 모드 처리
  if (gameRecovery.emergencyMode) {
    handleEmergencyMode();
  }
}

/**
 * 게임 상태 백업
 */
void backupGameState() {
  // 현재 게임 상태를 백업에 복사
  gameRecovery.backupState = gameState;
  
  // 백업 상태가 유효한지 확인
  if (isGameStateValid(&gameState)) {
    gameRecovery.lastValidPhase = gameState.gamePhase;
    
    // 디버그 출력 (필요시)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 30000) { // 30초마다
      Serial.print("게임 상태 백업 완료 - 단계: ");
      Serial.print(getPhaseNameKorean((GamePhase)gameState.gamePhase));
      Serial.print(", 점수: ");
      Serial.print(gameState.currentScore);
      Serial.print(", 레벨: ");
      Serial.println(gameState.currentLevel);
      lastDebugTime = millis();
    }
  } else {
    Serial.println("경고: 백업할 게임 상태가 유효하지 않음");
    gameRecovery.stateCorrupted = true;
    gameRecovery.corruptionCount++;
  }
}

/**
 * 게임 상태 유효성 검증
 */
void validateGameState() {
  bool isValid = isGameStateValid(&gameState);
  
  if (!isValid) {
    Serial.println("게임 상태 손상 감지!");
    gameRecovery.stateCorrupted = true;
    gameRecovery.corruptionCount++;
    
    // 자동 복구 시도
    if (gameRecovery.autoRecoveryEnabled && !gameRecovery.recoveryInProgress) {
      Serial.println("자동 게임 상태 복구 시작");
      startGameStateRecovery(RESET_REASON_GAME_CORRUPTION);
    }
  } else if (gameRecovery.stateCorrupted) {
    // 상태가 복구됨
    Serial.println("게임 상태 복구 확인됨");
    gameRecovery.stateCorrupted = false;
  }
}

/**
 * 게임 상태 유효성 확인
 * @param state 확인할 게임 상태
 * @return 유효성 여부
 */
bool isGameStateValid(GameState* state) {
  // 기본 범위 검사
  if (state->currentLevel < 1 || state->currentLevel > 11) {
    Serial.print("잘못된 레벨: ");
    Serial.println(state->currentLevel);
    return false;
  }
  
  if (state->currentScore > 7) {
    Serial.print("잘못된 점수: ");
    Serial.println(state->currentScore);
    return false;
  }
  
  if (state->gamePhase > PHASE_GAME_CLEAR) {
    Serial.print("잘못된 게임 단계: ");
    Serial.println(state->gamePhase);
    return false;
  }
  
  // 논리적 일관성 검사
  if (state->currentScore == 7 && state->gamePhase != PHASE_GAME_CLEAR && state->gamePhase != PHASE_IDLE) {
    Serial.println("점수 7점이지만 클리어 단계가 아님");
    return false;
  }
  
  if (!state->isGameActive && state->gamePhase != PHASE_IDLE && state->gamePhase != PHASE_GAME_CLEAR) {
    Serial.println("게임 비활성 상태이지만 진행 중인 단계");
    return false;
  }
  
  // 타이머 유효성 검사
  unsigned long currentTime = millis();
  if (state->phaseTimer > currentTime + 1000) { // 미래 시간은 허용하지 않음
    Serial.println("잘못된 단계 타이머");
    return false;
  }
  
  return true;
}

/**
 * 게임 상태 복구 시작
 * @param reason 복구 사유
 */
void startGameStateRecovery(uint8_t reason) {
  if (gameRecovery.recoveryInProgress) {
    Serial.println("이미 복구가 진행 중입니다");
    return;
  }
  
  if (gameRecovery.recoveryAttempts >= gameRecovery.maxRecoveryAttempts) {
    Serial.println("최대 복구 시도 횟수 초과 - 비상 모드 진입");
    enterEmergencyMode();
    return;
  }
  
  Serial.print("게임 상태 복구 시작 - 사유: ");
  Serial.println(getResetReasonName(reason));
  
  gameRecovery.recoveryInProgress = true;
  gameRecovery.recoveryStartTime = millis();
  gameRecovery.recoveryAttempts++;
  
  // 백업된 상태로 복구 시도
  if (isGameStateValid(&gameRecovery.backupState)) {
    Serial.println("백업 상태로 복구 중...");
    gameState = gameRecovery.backupState;
    
    // 안전한 상태로 조정
    adjustGameStateToSafe();
    
    Serial.println("게임 상태 복구 완료");
    gameRecovery.recoveryInProgress = false;
    gameRecovery.stateCorrupted = false;
  } else {
    Serial.println("백업 상태도 손상됨 - 안전 상태로 초기화");
    initializeGameStateToSafe();
  }
}

/**
 * 게임 상태 복구 처리
 */
void processGameStateRecovery() {
  unsigned long currentTime = millis();
  
  // 복구 타임아웃 확인
  if (currentTime - gameRecovery.recoveryStartTime > GAME_RECOVERY_TIMEOUT) {
    Serial.println("게임 상태 복구 타임아웃");
    gameRecovery.recoveryInProgress = false;
    
    // 강제로 안전 상태로 초기화
    initializeGameStateToSafe();
    return;
  }
  
  // 복구된 상태 검증
  if (isGameStateValid(&gameState)) {
    Serial.println("게임 상태 복구 성공");
    gameRecovery.recoveryInProgress = false;
    gameRecovery.stateCorrupted = false;
  }
}

/**
 * 게임 상태를 안전한 상태로 조정
 */
void adjustGameStateToSafe() {
  // 범위 검사 및 수정
  if (gameState.currentLevel < 1) gameState.currentLevel = 1;
  if (gameState.currentLevel > 11) gameState.currentLevel = 11;
  if (gameState.currentScore > 7) gameState.currentScore = 7;
  
  // 7점이면 클리어 상태로
  if (gameState.currentScore == 7) {
    gameState.gamePhase = PHASE_GAME_CLEAR;
    gameState.isGameActive = false;
  }
  
  // 타이머 초기화
  gameState.phaseTimer = millis();
  
  Serial.println("게임 상태를 안전한 상태로 조정 완료");
}

/**
 * 게임 상태를 안전한 초기 상태로 초기화
 */
void initializeGameStateToSafe() {
  Serial.println("게임 상태를 안전한 초기 상태로 초기화");
  
  gameState.currentLevel = 1;
  gameState.currentScore = 0;
  gameState.gamePhase = PHASE_IDLE;
  gameState.isGameActive = false;
  gameState.isRobotFacing = false;
  gameState.phaseTimer = millis();
  
  // 모든 하드웨어를 안전 상태로
  turnOffAllBeacons();
  controlAllCeilingLights(false);
  stopEffectLED();
  
  // Slave에게 정지 명령 전송
  sendCommandToSlave(CMD_STOP_ALL, 0, 0);
  
  gameRecovery.recoveryInProgress = false;
  gameRecovery.stateCorrupted = false;
  
  Serial.println("안전한 초기 상태로 초기화 완료");
}

/**
 * 사용자 입력 오류 모니터링
 */
void monitorUserInputErrors() {
  unsigned long currentTime = millis();
  
  // 가이드 시스템 타임아웃 처리
  if (inputErrorHandler.guidanceActive && 
      currentTime - inputErrorHandler.guidanceStartTime > INPUT_GUIDANCE_TIMEOUT) {
    Serial.println("입력 가이드 타임아웃");
    deactivateInputGuidance();
  }
  
  // 성공률 계산 및 적응형 가이드 조정
  if (inputErrorHandler.totalInputAttempts > 0) {
    inputErrorHandler.successRate = (float)inputErrorHandler.successfulInputs / inputErrorHandler.totalInputAttempts;
    
    // 성공률이 낮으면 가이드 레벨 상승
    if (inputErrorHandler.successRate < INPUT_SUCCESS_RATE_THRESHOLD && !inputErrorHandler.guidanceActive) {
      activateAdaptiveGuidance();
    }
  }
  
  // 연속 오류 처리
  if (inputErrorHandler.consecutiveErrors >= INPUT_CONSECUTIVE_ERROR_LIMIT) {
    Serial.println("연속 입력 오류 한계 초과 - 오류 복구 모드 진입");
    enterInputErrorRecoveryMode();
  }
}

/**
 * 입력 오류 기록
 * @param errorType 오류 유형
 */
void recordInputError(uint8_t errorType) {
  inputErrorHandler.invalidInputCount++;
  inputErrorHandler.consecutiveErrors++;
  inputErrorHandler.lastErrorTime = millis();
  inputErrorHandler.totalInputAttempts++;
  
  // 오류 패턴 기록
  inputErrorHandler.errorPattern[inputErrorHandler.errorPatternIndex] = errorType;
  inputErrorHandler.errorPatternIndex = (inputErrorHandler.errorPatternIndex + 1) % 10;
  
  Serial.print("입력 오류 기록 - 유형: ");
  Serial.print(errorType);
  Serial.print(", 연속 오류: ");
  Serial.print(inputErrorHandler.consecutiveErrors);
  Serial.print(", 성공률: ");
  Serial.print(inputErrorHandler.successRate * 100);
  Serial.println("%");
  
  // 오류 유형별 처리
  switch (errorType) {
    case 1: // 잘못된 순서
      Serial.println("가이드: LED 점멸 순서를 정확히 기억하세요");
      break;
    case 2: // 타임아웃
      Serial.println("가이드: 시간 내에 입력을 완료하세요");
      extendInputTimeout();
      break;
    case 3: // 중복 입력
      Serial.println("가이드: 같은 발판을 연속으로 밟지 마세요");
      break;
    default:
      Serial.println("가이드: 패턴을 다시 확인하고 천천히 입력하세요");
      break;
  }
  
  // 적응형 가이드 활성화 검토
  if (inputErrorHandler.consecutiveErrors >= INPUT_ERROR_THRESHOLD) {
    activateAdaptiveGuidance();
  }
}

/**
 * 입력 성공 기록
 */
void recordInputSuccess() {
  inputErrorHandler.consecutiveErrors = 0; // 연속 오류 초기화
  inputErrorHandler.successfulInputs++;
  inputErrorHandler.totalInputAttempts++;
  
  // 성공률 업데이트
  inputErrorHandler.successRate = (float)inputErrorHandler.successfulInputs / inputErrorHandler.totalInputAttempts;
  
  Serial.print("입력 성공 - 성공률: ");
  Serial.print(inputErrorHandler.successRate * 100);
  Serial.println("%");
  
  // 성공률이 높아지면 가이드 레벨 감소
  if (inputErrorHandler.successRate > 0.8f && inputErrorHandler.guidanceLevel > GUIDANCE_LEVEL_NONE) {
    inputErrorHandler.guidanceLevel--;
    Serial.print("가이드 레벨 감소: ");
    Serial.println(inputErrorHandler.guidanceLevel);
  }
  
  // 오류 복구 모드 해제
  if (inputErrorHandler.errorRecoveryMode) {
    inputErrorHandler.errorRecoveryMode = false;
    Serial.println("입력 오류 복구 모드 해제");
  }
}

/**
 * 적응형 가이드 활성화
 */
void activateAdaptiveGuidance() {
  if (inputErrorHandler.guidanceActive) {
    return; // 이미 활성화됨
  }
  
  inputErrorHandler.guidanceActive = true;
  inputErrorHandler.guidanceStartTime = millis();
  
  // 성공률에 따른 가이드 레벨 결정
  if (inputErrorHandler.successRate < 0.3f) {
    inputErrorHandler.guidanceLevel = GUIDANCE_LEVEL_ASSISTED;
  } else if (inputErrorHandler.successRate < 0.6f) {
    inputErrorHandler.guidanceLevel = GUIDANCE_LEVEL_DETAILED;
  } else {
    inputErrorHandler.guidanceLevel = GUIDANCE_LEVEL_BASIC;
  }
  
  Serial.print("적응형 가이드 활성화 - 레벨: ");
  Serial.println(inputErrorHandler.guidanceLevel);
  
  // 가이드 레벨별 메시지
  switch (inputErrorHandler.guidanceLevel) {
    case GUIDANCE_LEVEL_BASIC:
      Serial.println("기본 가이드: LED 점멸 순서를 기억하고 같은 순서로 발판을 밟으세요");
      break;
    case GUIDANCE_LEVEL_DETAILED:
      Serial.println("상세 가이드: 1) LED 점멸을 주의깊게 관찰 2) 순서를 기억 3) 천천히 정확하게 입력");
      break;
    case GUIDANCE_LEVEL_ASSISTED:
      Serial.println("보조 가이드: 패턴이 더 천천히 표시되고 입력 시간이 연장됩니다");
      extendInputTimeout();
      break;
  }
}

/**
 * 입력 가이드 비활성화
 */
void deactivateInputGuidance() {
  inputErrorHandler.guidanceActive = false;
  inputErrorHandler.guidanceLevel = GUIDANCE_LEVEL_NONE;
  inputErrorHandler.timeoutExtension = 0;
  
  Serial.println("입력 가이드 비활성화");
}

/**
 * 입력 타임아웃 연장
 */
void extendInputTimeout() {
  if (inputErrorHandler.timeoutExtension < INPUT_TIMEOUT_EXTENSION_MAX) {
    inputErrorHandler.timeoutExtension += 2000; // 2초씩 연장
    Serial.print("입력 타임아웃 연장: +");
    Serial.print(inputErrorHandler.timeoutExtension / 1000);
    Serial.println("초");
  }
}

/**
 * 입력 오류 복구 모드 진입
 */
void enterInputErrorRecoveryMode() {
  inputErrorHandler.errorRecoveryMode = true;
  inputErrorHandler.consecutiveErrors = 0; // 초기화
  
  Serial.println("=== 입력 오류 복구 모드 진입 ===");
  Serial.println("시스템이 더 관대한 입력 처리를 제공합니다");
  
  // 최대 가이드 레벨 활성화
  inputErrorHandler.guidanceLevel = GUIDANCE_LEVEL_ASSISTED;
  inputErrorHandler.guidanceActive = true;
  inputErrorHandler.guidanceStartTime = millis();
  
  // 타임아웃 최대 연장
  inputErrorHandler.timeoutExtension = INPUT_TIMEOUT_EXTENSION_MAX;
  
  Serial.println("복구 모드 활성화 완료");
}

/**
 * 시스템 리셋 요청
 * @param resetType 리셋 유형 (소프트/하드)
 * @param reason 리셋 사유
 */
void requestSystemReset(bool hardReset, uint8_t reason) {
  if (resetManager.resetInProgress) {
    Serial.println("이미 리셋이 진행 중입니다");
    return;
  }
  
  Serial.print("시스템 리셋 요청 - 유형: ");
  Serial.print(hardReset ? "하드" : "소프트");
  Serial.print(", 사유: ");
  Serial.println(getResetReasonName(reason));
  
  if (hardReset) {
    resetManager.hardResetRequested = true;
  } else {
    resetManager.softResetRequested = true;
  }
  
  resetManager.resetReason = reason;
  resetManager.resetRequestTime = millis();
  resetManager.resetInProgress = true;
  resetManager.resetStartTime = millis();
  resetManager.resetPhase = 0;
  resetManager.resetAttempts++;
}

/**
 * 시스템 리셋 처리
 */
void processSystemReset() {
  unsigned long currentTime = millis();
  
  // 리셋 타임아웃 확인
  if (currentTime - resetManager.resetStartTime > RESET_REQUEST_TIMEOUT) {
    Serial.println("시스템 리셋 타임아웃");
    resetManager.resetInProgress = false;
    return;
  }
  
  // 리셋 단계별 처리
  switch (resetManager.resetPhase) {
    case 0: // 준비 단계
      Serial.println("시스템 리셋 준비 중...");
      prepareSystemReset();
      resetManager.resetPhase = 1;
      break;
      
    case 1: // 하드웨어 정지 단계
      Serial.println("하드웨어 안전 정지 중...");
      stopAllHardware();
      resetManager.resetPhase = 2;
      break;
      
    case 2: // 상태 초기화 단계
      Serial.println("시스템 상태 초기화 중...");
      if (resetManager.hardResetRequested) {
        performHardReset();
      } else {
        performSoftReset();
      }
      resetManager.resetPhase = 3;
      break;
      
    case 3: // 검증 단계
      Serial.println("리셋 검증 중...");
      if (validateSystemReset()) {
        completeSystemReset();
      } else {
        Serial.println("리셋 검증 실패 - 재시도");
        resetManager.resetPhase = 1;
      }
      break;
  }
}

/**
 * 시스템 리셋 준비
 */
void prepareSystemReset() {
  Serial.println("시스템 리셋 준비:");
  Serial.println("- 현재 게임 상태 백업");
  Serial.println("- 통신 연결 확인");
  Serial.println("- 하드웨어 상태 점검");
  
  // 현재 상태 백업
  backupGameState();
  
  // Slave와의 통신 확인
  if (!commState.slaveConnected) {
    Serial.println("경고: Slave 연결 끊어짐");
  }
}

/**
 * 모든 하드웨어 정지
 */
void stopAllHardware() {
  Serial.println("모든 하드웨어 안전 정지:");
  
  // 모든 조명 소등
  controlAllCeilingLights(false);
  turnOffAllBeacons();
  stopEffectLED();
  
  // Slave에게 정지 명령
  sendCommandToSlave(CMD_STOP_ALL, 0, 0);
  
  // 오디오 정지
  // DFPlayer 정지 명령은 여기서 처리 (필요시)
  
  Serial.println("하드웨어 정지 완료");
}

/**
 * 소프트 리셋 수행
 */
void performSoftReset() {
  Serial.println("소프트 리셋 수행:");
  
  // 게임 상태만 초기화
  initializeGameState();
  
  // 통신 상태 초기화
  initComm();
  
  // 오류 카운터 초기화
  inputErrorHandler.invalidInputCount = 0;
  inputErrorHandler.consecutiveErrors = 0;
  gameRecovery.corruptionCount = 0;
  
  resetManager.softResets++;
  
  Serial.println("소프트 리셋 완료");
}

/**
 * 하드 리셋 수행
 */
void performHardReset() {
  Serial.println("하드 리셋 수행:");
  
  // 전체 시스템 초기화
  initializeGameState();
  initComm();
  initCommReliability();
  initHardwareSafety();
  initGameLogicStability();
  
  // 하드웨어 재초기화
  initializeSystemLights();
  
  resetManager.hardResets++;
  
  Serial.println("하드 리셋 완료");
}

/**
 * 시스템 리셋 검증
 * @return 검증 성공 여부
 */
bool validateSystemReset() {
  Serial.println("시스템 리셋 검증:");
  
  // 게임 상태 검증
  if (!isGameStateValid(&gameState)) {
    Serial.println("게임 상태 검증 실패");
    return false;
  }
  
  // 하드웨어 상태 검증
  if (beaconState.greenLightState || beaconState.redLightState) {
    Serial.println("경광등이 여전히 켜져 있음");
    return false;
  }
  
  // 통신 상태 검증 (필요시)
  
  Serial.println("시스템 리셋 검증 성공");
  return true;
}

/**
 * 시스템 리셋 완료
 */
void completeSystemReset() {
  resetManager.resetInProgress = false;
  resetManager.softResetRequested = false;
  resetManager.hardResetRequested = false;
  resetManager.lastSuccessfulReset = millis();
  resetManager.totalResets++;
  
  Serial.println("=== 시스템 리셋 완료 ===");
  Serial.print("총 리셋 횟수: ");
  Serial.print(resetManager.totalResets);
  Serial.print(" (소프트: ");
  Serial.print(resetManager.softResets);
  Serial.print(", 하드: ");
  Serial.print(resetManager.hardResets);
  Serial.println(")");
  
  // 리셋 후 안정화 대기
  delay(1000);
}

/**
 * 비상 모드 진입
 */
void enterEmergencyMode() {
  gameRecovery.emergencyMode = true;
  gameRecovery.criticalErrorCount++;
  
  Serial.println("=== 비상 모드 진입 ===");
  Serial.println("시스템이 최소 기능으로 동작합니다");
  
  // 모든 하드웨어 정지
  stopAllHardware();
  
  // 게임을 안전 상태로
  initializeGameStateToSafe();
  
  // 자동 복구 비활성화
  gameRecovery.autoRecoveryEnabled = false;
  
  Serial.println("비상 모드 활성화 완료");
  Serial.println("수동 리셋이 필요합니다");
}

/**
 * 비상 모드 처리
 */
void handleEmergencyMode() {
  // 비상 모드에서는 최소한의 기능만 동작
  static unsigned long lastEmergencyMessage = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastEmergencyMessage > 30000) { // 30초마다
    Serial.println("비상 모드 활성 - 수동 개입 필요");
    lastEmergencyMessage = currentTime;
  }
  
  // 기본적인 안전 기능만 유지
  if (beaconState.greenLightState || beaconState.redLightState) {
    turnOffAllBeacons();
  }
}

/**
 * 비상 모드 해제
 */
void exitEmergencyMode() {
  if (!gameRecovery.emergencyMode) {
    return;
  }
  
  Serial.println("비상 모드 해제");
  
  gameRecovery.emergencyMode = false;
  gameRecovery.autoRecoveryEnabled = true;
  gameRecovery.recoveryAttempts = 0;
  gameRecovery.stateCorrupted = false;
  
  // 시스템 하드 리셋 수행
  requestSystemReset(true, RESET_REASON_EMERGENCY);
  
  Serial.println("비상 모드 해제 완료");
}

/**
 * 리셋 사유 이름 반환
 * @param reason 리셋 사유 코드
 * @return 사유 이름 문자열
 */
const char* getResetReasonName(uint8_t reason) {
  switch (reason) {
    case RESET_REASON_USER_REQUEST: return "사용자 요청";
    case RESET_REASON_SYSTEM_ERROR: return "시스템 오류";
    case RESET_REASON_COMMUNICATION: return "통신 오류";
    case RESET_REASON_HARDWARE_FAILURE: return "하드웨어 오류";
    case RESET_REASON_GAME_CORRUPTION: return "게임 상태 손상";
    case RESET_REASON_EMERGENCY: return "비상 상황";
    case RESET_REASON_FACTORY: return "공장 초기화";
    default: return "알 수 없음";
  }
}

/**
 * 게임 로직 안정성 상태 출력 (디버깅용)
 */
void printGameLogicStabilityStatus() {
  Serial.println("=== 게임 로직 안정성 상태 ===");
  
  // 게임 상태 복구 시스템
  Serial.println("[ 게임 상태 복구 ]");
  Serial.print("상태 손상: ");
  Serial.println(gameRecovery.stateCorrupted ? "예" : "아니오");
  Serial.print("복구 시도: ");
  Serial.print(gameRecovery.recoveryAttempts);
  Serial.print("/");
  Serial.println(gameRecovery.maxRecoveryAttempts);
  Serial.print("손상 횟수: ");
  Serial.println(gameRecovery.corruptionCount);
  Serial.print("비상 모드: ");
  Serial.println(gameRecovery.emergencyMode ? "활성" : "비활성");
  
  // 사용자 입력 오류 처리
  Serial.println("[ 입력 오류 처리 ]");
  Serial.print("총 입력 시도: ");
  Serial.println(inputErrorHandler.totalInputAttempts);
  Serial.print("성공한 입력: ");
  Serial.println(inputErrorHandler.successfulInputs);
  Serial.print("성공률: ");
  Serial.print(inputErrorHandler.successRate * 100);
  Serial.println("%");
  Serial.print("연속 오류: ");
  Serial.println(inputErrorHandler.consecutiveErrors);
  Serial.print("가이드 레벨: ");
  Serial.println(inputErrorHandler.guidanceLevel);
  Serial.print("오류 복구 모드: ");
  Serial.println(inputErrorHandler.errorRecoveryMode ? "활성" : "비활성");
  
  // 시스템 리셋 관리
  Serial.println("[ 시스템 리셋 ]");
  Serial.print("총 리셋: ");
  Serial.println(resetManager.totalResets);
  Serial.print("소프트 리셋: ");
  Serial.println(resetManager.softResets);
  Serial.print("하드 리셋: ");
  Serial.println(resetManager.hardResets);
  Serial.print("리셋 진행 중: ");
  Serial.println(resetManager.resetInProgress ? "예" : "아니오");
  Serial.print("시스템 가동 시간: ");
  Serial.print((millis() - resetManager.systemUptime) / 1000);
  Serial.println("초");
  
  Serial.println("=============================");
}

/**
 * 게임 로직 안정성 시스템 리셋
 */
void resetGameLogicStability() {
  Serial.println("게임 로직 안정성 시스템 리셋");
  
  // 비상 모드 해제
  gameRecovery.emergencyMode = false;
  
  // 복구 시스템 초기화
  gameRecovery.recoveryAttempts = 0;
  gameRecovery.stateCorrupted = false;
  gameRecovery.corruptionCount = 0;
  gameRecovery.recoveryInProgress = false;
  gameRecovery.autoRecoveryEnabled = true;
  
  // 입력 오류 처리 초기화
  inputErrorHandler.invalidInputCount = 0;
  inputErrorHandler.consecutiveErrors = 0;
  inputErrorHandler.guidanceActive = false;
  inputErrorHandler.errorRecoveryMode = false;
  inputErrorHandler.timeoutExtension = 0;
  
  // 리셋 관리자 상태 초기화
  resetManager.resetInProgress = false;
  resetManager.softResetRequested = false;
  resetManager.hardResetRequested = false;
  
  Serial.println("게임 로직 안정성 시스템 리셋 완료");
}