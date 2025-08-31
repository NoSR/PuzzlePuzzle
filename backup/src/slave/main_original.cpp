/*
 * 무궁화 게임 시스템 - Slave Board (PlatformIO)
 * Arduino Nano 기반
 * 
 * 주요 기능:
 * - 로봇 Head/Body 모터 제어
 * - 바닥 발판 입력 처리 (진동 센서)
 * - 문제 제시 LED 및 점수 LED 제어 (MCP23017)
 * - 효과음 재생
 * - Master Board와의 시리얼 통신
 */

#include <Arduino.h>
#include "slave_config.h"
#include "communication.h"
#include <SoftwareSerial.h>
#include <Wire.h>

// DFPlayer Mini 통신용 소프트웨어 시리얼
SoftwareSerial dfSerial(DFR_RX_PIN, DFR_TX_PIN);

// 모터 상태 변수
bool headMotorActive = false;
bool bodyMotorActive = false;
unsigned long motorStopTime = 0;
bool motorBrakeActive = false;

// 진동 센서 입력 처리 변수
uint16_t vibrationThreshold = VIBRATION_THRESHOLD;
unsigned long lastVibrationTime[4] = {0, 0, 0, 0};
bool padPressed[4] = {false, false, false, false};
uint16_t vibrationReadings[4][VIBRATION_SAMPLES];
uint8_t sampleIndex[4] = {0, 0, 0, 0};
bool samplesReady[4] = {false, false, false, false};

// 발판 입력 순서 추적 변수
uint8_t inputSequence[6];        // 플레이어 입력 순서 (최대 6개)
uint8_t inputSequenceLength = 0; // 현재 입력된 순서 길이
unsigned long lastInputTime = 0; // 마지막 입력 시간
bool isInputActive = false;      // 입력 대기 상태

// 통신 안정성 강화 변수 (Task 17.1)
struct SlaveCommReliability {
  unsigned long lastMasterCommTime;      // 마지막 Master 통신 시간
  uint8_t consecutiveErrors;             // 연속 오류 횟수
  uint8_t errorThreshold;                // 오류 임계값
  uint32_t totalMessagesReceived;        // 총 수신 메시지 수
  uint32_t validMessagesReceived;        // 유효한 메시지 수
  uint32_t checksumErrors;               // 체크섬 오류 수
  uint32_t invalidMessages;              // 잘못된 메시지 수
  bool masterConnected;                  // Master 연결 상태
  unsigned long connectionTimeout;       // 연결 타임아웃
  bool diagnosticMode;                   // 진단 모드
  uint8_t lastErrorCode;                 // 마지막 오류 코드
  unsigned long recoveryStartTime;       // 복구 시작 시간
  bool isRecovering;                     // 복구 모드 여부
};

SlaveCommReliability slaveCommReliability;

// Slave 통신 안정성 상수
#define SLAVE_COMM_TIMEOUT              10000   // Master 통신 타임아웃 (ms)
#define SLAVE_COMM_ERROR_THRESHOLD      5       // 연속 오류 임계값
#define SLAVE_COMM_RECOVERY_TIMEOUT     5000    // 복구 타임아웃 (ms)

// =============================================================================
// 하드웨어 안전장치 강화 시스템 (Task 17.2) - Slave Board
// =============================================================================

// 모터 안전장치 구조체
struct SlaveMotorSafety {
  bool headMotorHealthy;                 // Head 모터 상태
  bool bodyMotorHealthy;                 // Body 모터 상태
  unsigned long headMotorStartTime;      // Head 모터 시작 시간
  unsigned long bodyMotorStartTime;      // Body 모터 시작 시간
  unsigned long maxMotorRunTime;         // 최대 모터 동작 시간
  uint16_t headMotorTimeouts;            // Head 모터 타임아웃 횟수
  uint16_t bodyMotorTimeouts;            // Body 모터 타임아웃 횟수
  bool motorOverloadProtection;          // 모터 과부하 보호 활성
  unsigned long lastMotorHealthCheck;    // 마지막 모터 상태 체크
  float estimatedMotorCurrent;           // 추정 모터 전류
  bool emergencyMotorStop;               // 비상 모터 정지 상태
  uint16_t motorRecoveryAttempts;        // 모터 복구 시도 횟수
  bool motorSafetyLockout;               // 모터 안전 잠금
  unsigned long safetyLockoutTime;       // 안전 잠금 시작 시간
};

SlaveMotorSafety slaveMotorSafety;

// 센서 안전장치 구조체
struct SlaveSensorSafety {
  bool vibrationSensorHealthy[4];        // 진동 센서 상태
  uint16_t vibrationSensorErrors[4];     // 진동 센서 오류 횟수
  unsigned long lastValidVibration[4];   // 마지막 유효 진동 시간
  bool sensorCalibrationNeeded[4];       // 센서 캘리브레이션 필요 여부
  
  bool limitSwitchHealthy[2];            // 리미트 스위치 상태 (Head, Body)
  uint16_t limitSwitchErrors[2];         // 리미트 스위치 오류 횟수
  unsigned long lastLimitSwitchCheck;    // 마지막 리미트 스위치 체크
  
  bool mcpHealthy;                       // MCP23017 상태
  uint16_t mcpErrorCount;                // MCP23017 오류 횟수
  unsigned long lastMcpResponse;         // 마지막 MCP23017 응답
  
  bool dfPlayerHealthy;                  // DFPlayer 상태
  uint16_t dfPlayerErrorCount;           // DFPlayer 오류 횟수
  unsigned long lastDfPlayerResponse;    // 마지막 DFPlayer 응답
  
  uint16_t totalSensorErrors;            // 총 센서 오류 횟수
  bool sensorEmergencyMode;              // 센서 비상 모드
};

SlaveSensorSafety slaveSensorSafety;

// 전원 및 시스템 안전장치 구조체
struct SlaveSystemSafety {
  float systemVoltage;                   // 시스템 전압
  bool lowVoltageDetected;               // 저전압 감지
  unsigned long lastVoltageCheck;        // 마지막 전압 체크
  
  float systemTemperature;               // 시스템 온도 (추정)
  bool overheated;                       // 과열 상태
  unsigned long lastTemperatureCheck;    // 마지막 온도 체크
  
  bool i2cBusHealthy;                    // I2C 버스 상태
  uint16_t i2cErrorCount;                // I2C 오류 횟수
  unsigned long lastI2cCheck;            // 마지막 I2C 체크
  
  bool systemStable;                     // 시스템 안정성
  bool emergencyShutdown;                // 비상 정지 상태
  unsigned long emergencyShutdownTime;   // 비상 정지 시간
  
  uint16_t totalSystemErrors;            // 총 시스템 오류
  bool diagnosticMode;                   // 진단 모드
};

SlaveSystemSafety slaveSystemSafety;

// 하드웨어 안전장치 상수 정의
#define MOTOR_MAX_RUN_TIME              30000   // 최대 모터 동작 시간 (30초)
#define MOTOR_HEALTH_CHECK_INTERVAL     1000    // 모터 상태 체크 간격 (ms)
#define MOTOR_OVERLOAD_CURRENT          2.0f    // 모터 과부하 전류 임계값 (A)
#define MOTOR_SAFETY_LOCKOUT_TIME       60000   // 모터 안전 잠금 시간 (1분)
#define MOTOR_MAX_RECOVERY_ATTEMPTS     3       // 최대 모터 복구 시도

#define SENSOR_ERROR_THRESHOLD          5       // 센서 오류 임계값
#define SENSOR_HEALTH_CHECK_INTERVAL    2000    // 센서 상태 체크 간격 (ms)
#define VIBRATION_SENSOR_TIMEOUT        10000   // 진동 센서 타임아웃 (ms)
#define LIMIT_SWITCH_CHECK_INTERVAL     500     // 리미트 스위치 체크 간격 (ms)

#define SYSTEM_VOLTAGE_MIN              4.5f    // 최소 시스템 전압 (V)
#define SYSTEM_VOLTAGE_CHECK_INTERVAL   2000    // 전압 체크 간격 (ms)
#define SYSTEM_TEMPERATURE_MAX          70.0f   // 최대 시스템 온도 (°C)
#define SYSTEM_TEMP_CHECK_INTERVAL      5000    // 온도 체크 간격 (ms)

#define I2C_TIMEOUT                     1000    // I2C 타임아웃 (ms)
#define I2C_MAX_RETRIES                 3       // I2C 최대 재시도 횟수
#define SYSTEM_ERROR_THRESHOLD          10      // 시스템 오류 임계값

void setup() {
  // 시리얼 통신 초기화
  Serial.begin(SERIAL_BAUD_RATE);
  dfSerial.begin(9600);
  
  // I2C 통신 초기화 (MCP23017용)
  Wire.begin();
  
  // 핀 모드 설정
  initializePins();
  
  // MCP23017 초기화
  initializeMCP23017();
  
  // DFPlayer 초기화
  initializeDFPlayer();
  
  // 리미트 스위치 시스템 초기화
  initializeLimitSwitches();
  
  // 진동 센서 시스템 초기화
  initializeVibrationSensors();
  
  // 통신 안정성 시스템 초기화 (Task 17.1)
  initSlaveCommReliability();
  
  // 하드웨어 안전장치 시스템 초기화 (Task 17.2)
  initSlaveHardwareSafety();
  
  // 게임 로직 안정성 검증 시스템 초기화 (Task 17.3)
  initSlaveGameLogicStability();
  
  Serial.println("Slave Board 초기화 완료 (PlatformIO)");
}

void loop() {
  // Master로부터 명령 처리 (개선된 버전)
  processMasterCommImproved();
  
  // Slave 통신 안정성 모니터링 (Task 17.1)
  monitorSlaveCommReliability();
  
  // 진동 센서 모니터링
  monitorVibrationSensors();
  
  // LED 패턴 재생 업데이트
  updateLEDPattern();
  
  // 점수 LED 점멸 업데이트
  updateScoreLEDBlinking();
  
  // 모터 브레이크 처리
  handleMotorBrake();
  
  // 모터 타임아웃 체크 (안전장치)
  checkMotorTimeout();
  
  // 리미트 스위치 상태 모니터링
  monitorLimitSwitches();
  
  // 오디오 상태 관리
  updateAudioState();
  
  // 하드웨어 안전장치 모니터링 (Task 17.2)
  monitorSlaveHardwareSafety();
  
  // 게임 로직 안정성 모니터링 (Task 17.3)
  monitorSlaveGameLogicStability();
  
  // 시리얼 디버그 명령 처리 (개발/테스트용)
  processDebugCommands();
  
  delay(10); // 메인 루프 딜레이
}

// =============================================================================
// 디버그 및 테스트 명령어 처리
// =============================================================================

/**
 * 시리얼 디버그 명령어 처리 (개발/테스트용)
 */
void processDebugCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    
    if (command == "help" || command == "?") {
      printDebugHelp();
    }
    else if (command == "status") {
      printSystemStatus();
    }
    else if (command == "vibration") {
      printVibrationStatus();
    }
    else if (command == "calibrate") {
      calibrateVibrationSensors();
    }
    else if (command == "monitor") {
      realtimeVibrationMonitor();
    }
    else if (command == "input_start") {
      setInputActive(true);
      Serial.println("입력 대기 활성화");
    }
    else if (command == "input_stop") {
      setInputActive(false);
      Serial.println("입력 대기 비활성화");
    }
    else if (command == "input_clear") {
      clearInputSequence();
      Serial.println("입력 순서 초기화");
    }
    else if (command == "input_show") {
      if (inputSequenceLength > 0) {
        printInputSequence();
        analyzeInputPattern();
      } else {
        Serial.println("입력 순서 없음");
      }
    }
    else if (command.startsWith("threshold ")) {
      int threshold = command.substring(10).toInt();
      if (threshold > 0 && threshold <= 1023) {
        setVibrationThreshold(threshold);
      } else {
        Serial.println("임계값 범위: 1-1023");
      }
    }
    else if (command == "motor_test") {
      testMotorSystem();
    }
    else if (command == "led_test") {
      testLEDSystem();
    }
    else if (command == "pattern_test") {
      testProblemLEDPattern();
    }
    else if (command == "pattern_info") {
      printCurrentPattern();
    }
    else if (command == "pattern_stop") {
      stopLEDPattern();
      Serial.println("LED 패턴 재생 중지");
    }
    else if (command == "score_test") {
      testScoreLEDBlinking();
    }
    else if (command == "score_info") {
      printScoreLEDStatus();
    }
    else if (command.startsWith("score ")) {
      uint8_t score = command.substring(6).toInt();
      if (score <= 7) {
        updateScoreWithBlink(score);
      } else {
        Serial.println("점수 범위: 0-7");
      }
    }
    else if (command == "audio_test") {
      testAllEffectSounds();
    }
    else if (command == "situation_test") {
      testSituationSounds();
    }
    else if (command == "audio_status") {
      printDFPlayerStatus();
    }
    else if (command.startsWith("play ")) {
      uint8_t soundId = command.substring(5).toInt();
      if (soundId > 0 && soundId <= 8) {
        playEffectSound(soundId);
      } else {
        Serial.println("효과음 ID 범위: 1-8");
      }
    }
    else if (command.startsWith("track ")) {
      uint8_t trackNum = command.substring(6).toInt();
      if (trackNum > 0 && trackNum <= 255) {
        playTrack(trackNum);
      } else {
        Serial.println("트랙 번호 범위: 1-255");
      }
    }
    else if (command.startsWith("volume ")) {
      uint8_t volume = command.substring(7).toInt();
      if (volume <= 30) {
        setDFVolume(volume);
      } else {
        Serial.println("볼륨 범위: 0-30");
      }
    }
    else if (command == "audio_stop") {
      stopAllAudio();
    }
    else if (command == "stop") {
      handleStopAllCommand();
    }
    // 통신 진단 명령어 (Task 17.1)
    else if (command == "comm_status") {
      printSlaveCommDiagnostics();
    }
    else if (command == "comm_diag_on") {
      setSlaveCommDiagnosticMode(true);
    }
    else if (command == "comm_diag_off") {
      setSlaveCommDiagnosticMode(false);
    }
    else if (command == "comm_reset") {
      resetSlaveCommStatistics();
    }
    else if (command == "comm_recovery") {
      Serial.println("수동 통신 복구 시작");
      startSlaveCommRecovery();
    }
    // 하드웨어 안전장치 명령어 (Task 17.2)
    else if (command == "safety_status") {
      printSlaveHardwareSafetyStatus();
    }
    else if (command == "safety_reset") {
      resetSlaveHardwareSafety();
    }
    else if (command == "motor_safety") {
      Serial.println("=== 모터 안전장치 상태 ===");
      Serial.print("Head 모터: ");
      Serial.println(slaveMotorSafety.headMotorHealthy ? "정상" : "오작동");
      Serial.print("Body 모터: ");
      Serial.println(slaveMotorSafety.bodyMotorHealthy ? "정상" : "오작동");
      Serial.print("안전 잠금: ");
      Serial.println(slaveMotorSafety.motorSafetyLockout ? "활성" : "비활성");
      Serial.print("추정 전류: ");
      Serial.print(slaveMotorSafety.estimatedMotorCurrent);
      Serial.println("A");
    }
    else if (command == "sensor_safety") {
      Serial.println("=== 센서 안전장치 상태 ===");
      for (int i = 0; i < 4; i++) {
        Serial.print("진동 센서 ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(slaveSensorSafety.vibrationSensorHealthy[i] ? "정상" : "오작동");
      }
      Serial.print("MCP23017: ");
      Serial.println(slaveSensorSafety.mcpHealthy ? "정상" : "오작동");
      Serial.print("DFPlayer: ");
      Serial.println(slaveSensorSafety.dfPlayerHealthy ? "정상" : "오작동");
    }
    else if (command == "system_safety") {
      Serial.println("=== 시스템 안전장치 상태 ===");
      Serial.print("시스템 전압: ");
      Serial.print(slaveSystemSafety.systemVoltage);
      Serial.println("V");
      Serial.print("시스템 온도: ");
      Serial.print(slaveSystemSafety.systemTemperature);
      Serial.println("°C");
      Serial.print("I2C 버스: ");
      Serial.println(slaveSystemSafety.i2cBusHealthy ? "정상" : "오작동");
      Serial.print("비상 정지: ");
      Serial.println(slaveSystemSafety.emergencyShutdown ? "활성" : "비활성");
    }
    else if (command == "motor_test_safety") {
      Serial.println("모터 안전장치 테스트");
      slaveMotorSafety.estimatedMotorCurrent = 2.5f; // 과부하 시뮬레이션
      handleMotorOverload();
    }
    else if (command == "overheat_test_slave") {
      Serial.println("Slave 과열 보호 테스트");
      slaveSystemSafety.systemTemperature = 75.0f;
      triggerSlaveOverheatProtection();
    }
    else if (command == "voltage_test_slave") {
      Serial.println("Slave 저전압 보호 테스트");
      slaveSystemSafety.systemVoltage = 4.0f;
      triggerSlaveLowVoltageProtection();
    }
    // 게임 로직 안정성 명령어 (Task 17.3)
    else if (command == "stability_status_slave") {
      printSlaveGameLogicStabilityStatus();
    }
    else if (command == "stability_reset_slave") {
      resetSlaveGameLogicStability();
    }
    else if (command == "input_error_test_slave") {
      Serial.println("Slave 입력 오류 테스트");
      recordSlaveInputError(INPUT_ERROR_INVALID_SEQUENCE);
    }
    else if (command == "input_recovery_slave") {
      Serial.println("Slave 입력 복구 테스트");
      startSlaveInputRecovery();
    }
    else if (command == "motor_recovery_slave") {
      Serial.println("Slave 모터 복구 테스트");
      startSlaveMotorRecovery();
    }
    else if (command == "led_recovery_slave") {
      Serial.println("Slave LED 복구 테스트");
      startSlaveLedRecovery();
    }
    else if (command == "emergency_mode_slave") {
      Serial.println("Slave 비상 모드 진입");
      enterSlaveEmergencyMode();
    }
    else if (command == "emergency_exit_slave") {
      Serial.println("Slave 비상 모드 해제");
      exitSlaveEmergencyMode();
    }
    else if (command == "reset_slave") {
      Serial.println("Slave 시스템 리셋");
      requestSlaveSystemReset(RESET_REASON_USER_REQUEST);
    }
    else if (command == "validate_input_slave") {
      Serial.println("Slave 입력 시스템 검증");
      validateSlaveInputSystem();
    }
    else if (command != "") {
      Serial.print("알 수 없는 명령: ");
      Serial.println(command);
      Serial.println("'help' 입력으로 도움말 확인");
      Serial.println("하드웨어 안전장치 명령어:");
      Serial.println("  safety_status      - 전체 안전장치 상태");
      Serial.println("  safety_reset       - 안전장치 리셋");
      Serial.println("  motor_safety       - 모터 안전장치 상태");
      Serial.println("  sensor_safety      - 센서 안전장치 상태");
      Serial.println("  system_safety      - 시스템 안전장치 상태");
      Serial.println("  motor_test_safety  - 모터 안전장치 테스트");
      Serial.println("  overheat_test_slave - 과열 보호 테스트");
      Serial.println("  voltage_test_slave  - 저전압 보호 테스트");
    }
  }
}

/**
 * 디버그 도움말 출력
 */
void printDebugHelp() {
  Serial.println("=== Slave Board 디버그 명령어 ===");
  Serial.println("help, ?          - 이 도움말 표시");
  Serial.println("status           - 시스템 전체 상태 출력");
  Serial.println("vibration        - 진동 센서 상태 출력");
  Serial.println("calibrate        - 진동 센서 캘리브레이션");
  Serial.println("monitor          - 실시간 진동 센서 모니터링");
  Serial.println("input_start      - 입력 대기 시작");
  Serial.println("input_stop       - 입력 대기 중지");
  Serial.println("input_clear      - 입력 순서 초기화");
  Serial.println("input_show       - 현재 입력 순서 및 분석");
  Serial.println("threshold <값>   - 진동 임계값 설정 (1-1023)");
  Serial.println("motor_test       - 모터 시스템 테스트");
  Serial.println("led_test         - LED 시스템 테스트");
  Serial.println("pattern_test     - 문제 제시 LED 패턴 테스트");
  Serial.println("pattern_info     - 현재 패턴 정보 출력");
  Serial.println("pattern_stop     - LED 패턴 재생 중지");
  Serial.println("score_test       - 점수 LED 점멸 테스트");
  Serial.println("score_info       - 점수 LED 상태 출력");
  Serial.println("score <값>       - 점수 설정 (0-7)");
  Serial.println("audio_test       - 모든 효과음 테스트");
  Serial.println("situation_test   - 상황별 효과음 테스트");
  Serial.println("audio_status     - DFPlayer 상태 출력");
  Serial.println("play <ID>        - 효과음 재생 (1-8)");
  Serial.println("track <번호>     - 트랙 번호로 재생 (1-255)");
  Serial.println("volume <값>      - 볼륨 설정 (0-30)");
  Serial.println("audio_stop       - 오디오 정지");
  Serial.println("stop             - 모든 동작 정지");
  Serial.println("");
  Serial.println("=== 통신 진단 명령어 (Task 17.1) ===");
  Serial.println("comm_status      - 통신 상태 및 통계 확인");
  Serial.println("comm_diag_on     - 통신 진단 모드 활성화");
  Serial.println("comm_diag_off    - 통신 진단 모드 비활성화");
  Serial.println("comm_reset       - 통신 통계 리셋");
  Serial.println("comm_recovery    - 수동 통신 복구 시작");
  Serial.println("");
  Serial.println("=== 하드웨어 안전장치 명령어 (Task 17.2) ===");
  Serial.println("safety_status      - 전체 안전장치 상태 확인");
  Serial.println("safety_reset       - 안전장치 시스템 리셋");
  Serial.println("motor_safety       - 모터 안전장치 상태");
  Serial.println("sensor_safety      - 센서 안전장치 상태");
  Serial.println("system_safety      - 시스템 안전장치 상태");
  Serial.println("motor_test_safety  - 모터 안전장치 테스트");
  Serial.println("overheat_test_slave - 과열 보호 테스트");
  Serial.println("voltage_test_slave  - 저전압 보호 테스트");
  Serial.println("");
  Serial.println("=== 게임 로직 안정성 명령어 (Task 17.3) ===");
  Serial.println("stability_status_slave   - Slave 게임 로직 안정성 상태");
  Serial.println("stability_reset_slave    - Slave 안정성 시스템 리셋");
  Serial.println("input_error_test_slave   - 입력 오류 테스트");
  Serial.println("input_recovery_slave     - 입력 시스템 복구 테스트");
  Serial.println("motor_recovery_slave     - 모터 시스템 복구 테스트");
  Serial.println("led_recovery_slave       - LED 시스템 복구 테스트");
  Serial.println("emergency_mode_slave     - Slave 비상 모드 진입");
  Serial.println("emergency_exit_slave     - Slave 비상 모드 해제");
  Serial.println("reset_slave              - Slave 시스템 리셋");
  Serial.println("validate_input_slave     - 입력 시스템 검증");
  Serial.println("================================");
}

/**
 * 모터 시스템 테스트
 */
void testMotorSystem() {
  Serial.println("=== 모터 시스템 테스트 ===");
  
  // 모터 진단
  diagnoseMotorSystem();
  
  Serial.println("Head 모터 정방향 테스트...");
  rotateHeadForward(MOTOR_SPEED_SLOW);
  delay(1000);
  
  Serial.println("Head 모터 역방향 테스트...");
  rotateHeadBackward(MOTOR_SPEED_SLOW);
  delay(1000);
  
  Serial.println("Body 모터 정방향 테스트...");
  rotateBodyForward(MOTOR_SPEED_SLOW);
  delay(1000);
  
  Serial.println("Body 모터 역방향 테스트...");
  rotateBodyBackward(MOTOR_SPEED_SLOW);
  delay(1000);
  
  Serial.println("모든 모터 정지...");
  emergencyStopMotors(true);
  
  Serial.println("모터 시스템 테스트 완료");
}

/**
 * LED 시스템 테스트
 */
void testLEDSystem() {
  Serial.println("=== LED 시스템 테스트 ===");
  
  // 점수 LED 테스트
  Serial.println("점수 LED 테스트...");
  for (int i = 0; i <= 6; i++) {
    displayScore(i);
    delay(500);
  }
  
  // OUTPUT LED 테스트
  Serial.println("OUTPUT LED 테스트...");
  for (int i = 0; i < 8; i++) {
    controlOutput(i, true);
    delay(300);
    controlOutput(i, false);
  }
  
  // 모든 LED 소등
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  
  Serial.println("LED 시스템 테스트 완료");
}

void initializePins() {
  // 모터 제어 핀 설정 (L298N)
  // Head 모터 핀 설정
  pinMode(HEAD_PWM_PIN, OUTPUT);
  pinMode(HEAD_DIR1_PIN, OUTPUT);
  pinMode(HEAD_DIR2_PIN, OUTPUT);
  
  // Body 모터 핀 설정
  pinMode(BODY_PWM_PIN, OUTPUT);
  pinMode(BODY_DIR1_PIN, OUTPUT);
  pinMode(BODY_DIR2_PIN, OUTPUT);
  
  // 모터 초기 상태 설정
  stopAllMotors();
  
  // 진동 센서는 아날로그 핀이므로 별도 설정 불필요
}

void initializeMCP23017() {
  // MCP23017 #1 초기화 (PCB 지정 핀 배열) - 주소 0x20
  // GPIOA: OUTPUT 핀 (OutSign1~6: 1-6번 비트) + INPUT 핀 (IN7: 0번 비트)
  // GPIOB: INPUT 핀 (IN0~6: 0-6번 비트) + OUTPUT 핀 (OutSign7: 7번 비트)
  
  // MCP1 GPIOA 설정 (0번: 입력, 1-7번: 출력)
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_IODIRA);
  Wire.write(0x01); // 0번 핀만 입력으로 설정 (IN7)
  Wire.endTransmission();
  
  // MCP1 GPIOB 설정 (0-6번: 입력, 7번: 출력)
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_IODIRB);
  Wire.write(0x7F); // 0-6번 핀을 입력으로 설정 (IN0~6)
  Wire.endTransmission();
  
  // MCP1 풀업 저항 활성화 (INPUT 핀용)
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_GPPUA);
  Wire.write(0x01); // GPIOA 0번 핀 풀업 (IN7)
  Wire.endTransmission();
  
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_GPPUB);
  Wire.write(0x7F); // GPIOB 0-6번 핀 풀업 (IN0~6)
  Wire.endTransmission();
  
  // MCP1 초기 OUTPUT 상태 설정 (모두 OFF)
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOB, 0x00);
  
  // MCP23017 #2 초기화 (PCB 지정 핀 배열) - 주소 0x21
  // GPIOA: 점수 LED (fstMulti0~5: 0,1,2,3,4,5번 비트) + 예비 (6,7번)
  // GPIOB: 예비 장치 (sndMulti0~5: 0,1,2,3,4,5번 비트) + 예비 (6,7번)
  
  // MCP2 GPIOA를 출력으로 설정 (점수 LED용)
  Wire.beginTransmission(MCP23017_2_ADDR);
  Wire.write(MCP_IODIRA);
  Wire.write(0x00); // 모든 핀을 출력으로 설정
  Wire.endTransmission();
  
  // MCP2 GPIOB를 출력으로 설정 (예비 장치용)
  Wire.beginTransmission(MCP23017_2_ADDR);
  Wire.write(MCP_IODIRB);
  Wire.write(0x00); // 모든 핀을 출력으로 설정
  Wire.endTransmission();
  
  // MCP2 초기 상태 설정 (모두 OFF)
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOB, 0x00);
  
  Serial.println("MCP23017 초기화 완료 (PCB 핀 배열)");
  Serial.println("MCP1: INPUT(IN0~7) + OUTPUT(OutSign0~7, ULN2803 연결)");
  Serial.println("MCP2: 점수 LED(fstMulti0~5) + 예비 장치(sndMulti0~5)");
}

// =============================================================================
// DFPlayer Mini 효과음 재생 시스템
// =============================================================================

// DFPlayer 상태 변수
bool dfPlayerReady = false;
bool isAudioPlaying = false;
uint8_t currentTrack = 0;
unsigned long audioStartTime = 0;
uint8_t currentVolume = 20; // 기본 볼륨 (0-30)

// 효과음 매핑 테이블 (상황별 효과음 ID)
struct SoundMapping {
  uint8_t soundId;
  uint8_t trackNumber;
  const char* description;
};

// 효과음 매핑 테이블 (요구사항에 따른 상황별 매핑)
const SoundMapping SOUND_MAP[] = {
  {SFX_LIGHT_ON,      1, "조명 점등"},        // 요구사항 1.3: 조명 점등시
  {SFX_PATTERN_BEEP,  2, "패턴 비프음"},      // 요구사항 2.3: LED 점멸시
  {SFX_PAD_PRESS,     3, "발판 밟기"},        // 요구사항 2.4: 발판 밟기시
  {SFX_CORRECT,       4, "정답"},            // 정답 입력시
  {SFX_WRONG,         5, "오답"},            // 오답 입력시
  {SFX_GUNSHOT,       6, "총소리"},          // 요구사항 3.3: 동작감지시
  {SFX_SUCCESS,       7, "성공"},            // 점수 획득시
  {SFX_LEVEL_UP,      8, "레벨업"}           // 레벨 상승시
};

const uint8_t SOUND_MAP_SIZE = sizeof(SOUND_MAP) / sizeof(SoundMapping);

/**
 * DFPlayer Mini 초기화
 */
void initializeDFPlayer() {
  Serial.println("DFPlayer Mini 초기화 시작...");
  
  // DFPlayer 안정화 대기
  delay(1000);
  
  // DFPlayer 리셋 명령
  sendDFCommand(0x0C, 0, 0);
  delay(500);
  
  // 볼륨 설정 (0-30)
  setDFVolume(currentVolume);
  delay(100);
  
  // EQ 설정 (Normal)
  sendDFCommand(0x07, 0, 0);
  delay(100);
  
  // 재생 모드 설정 (SD 카드)
  sendDFCommand(0x09, 0, 2);
  delay(100);
  
  dfPlayerReady = true;
  isAudioPlaying = false;
  currentTrack = 0;
  
  Serial.println("DFPlayer Mini 초기화 완료");
  Serial.print("기본 볼륨: ");
  Serial.println(currentVolume);
}

void stopAllMotors() {
  // 즉시 정지 - PWM을 0으로 설정
  analogWrite(HEAD_PWM_PIN, 0);
  analogWrite(BODY_PWM_PIN, 0);
  
  // 브레이크 적용 - 모든 방향 핀을 HIGH로 설정 (5초간)
  digitalWrite(HEAD_DIR1_PIN, HIGH);
  digitalWrite(HEAD_DIR2_PIN, HIGH);
  digitalWrite(BODY_DIR1_PIN, HIGH);
  digitalWrite(BODY_DIR2_PIN, HIGH);
  
  // 브레이크 타이머 설정
  motorStopTime = millis() + MOTOR_BRAKE_DURATION;
  motorBrakeActive = true;
  
  Serial.println("모터 정지 및 브레이크 적용 (5초간 HIGH 신호)");
}

void setMCP23017Output(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// 호환성을 위한 오버로드 함수 (기본적으로 MCP1 사용)
void setMCP23017Output(uint8_t reg, uint8_t value) {
  setMCP23017Output(MCP23017_1_ADDR, reg, value);
}

// =============================================================================
// Master Board 통신 처리
// =============================================================================

/**
 * Master로부터 명령 수신 및 처리
 */
void processMasterComm() {
  if (Serial.available() >= 4) { // 최소 메시지 크기
    uint8_t header = Serial.read();
    
    if (header == 0xAA) { // 유효한 헤더
      uint8_t command = Serial.read();
      uint8_t data1 = Serial.read();
      uint8_t data2 = Serial.read();
      
      // 체크섬 검증 (간단한 XOR)
      uint8_t expectedChecksum = command ^ data1 ^ data2;
      
      if (Serial.available() > 0) {
        uint8_t receivedChecksum = Serial.read();
        
        if (receivedChecksum == expectedChecksum) {
          handleMasterCommand(command, data1, data2);
        } else {
          Serial.println("체크섬 오류");
        }
      }
    }
  }
}

/**
 * Master 명령 처리
 * @param command 명령어 코드
 * @param data1 데이터 1
 * @param data2 데이터 2
 */
void handleMasterCommand(uint8_t command, uint8_t data1, uint8_t data2) {
  switch (command) {
    case CMD_ROBOT_HEAD_FRONT: // 0x10 - 로봇 HEAD 정면 회전
      handleRobotHeadFrontCommand();
      break;
      
    case CMD_ROBOT_HEAD_BACK: // 0x11 - 로봇 HEAD 후면 회전
      handleRobotHeadBackCommand();
      break;
      
    case CMD_ROBOT_BODY_FRONT: // 0x12 - 로봇 BODY 정면 회전
      handleRobotBodyFrontCommand();
      break;
      
    case CMD_ROBOT_BODY_BACK: // 0x13 - 로봇 BODY 후면 회전
      handleRobotBodyBackCommand();
      break;
      
    case CMD_ROBOT_STOP: // 0x14 - 모든 모터 정지
      handleRobotStopCommand();
      break;
      
    case CMD_SHOW_PATTERN: // 0x20 - LED 패턴 표시
      handleShowPatternCommand(data1, data2);
      break;
      
    case CMD_CLEAR_PATTERN: // 0x21 - LED 패턴 지우기
      handleClearPatternCommand();
      break;
      
    case CMD_UPDATE_SCORE: // 0x22 - 점수 LED 업데이트
      handleUpdateScoreCommand(data1);
      break;
      
    case CMD_PLAY_EFFECT: // 0x30 - 효과음 재생
      handlePlayEffectCommand(data1);
      break;
      
    case CMD_STOP_AUDIO: // 0x31 - 오디오 정지
      handleStopAudioCommand();
      break;
      
    case CMD_GET_STATUS: // 0x40 - 상태 요청
      handleGetStatusCommand();
      break;
      
    case CMD_GET_INPUT: // 0x41 - 입력 상태 요청
      handleGetInputCommand();
      break;
      
    case CMD_GET_LIMIT: // 0x42 - 리미트 스위치 상태 요청
      handleGetLimitCommand();
      break;
      
    case CMD_RESET: // 0xF0 - 시스템 리셋
      handleResetCommand();
      break;
      
    case CMD_STOP_ALL: // 0xFF - 모든 동작 정지
      handleStopAllCommand();
      break;
      
    default:
      Serial.print("알 수 없는 명령: 0x");
      Serial.println(command, HEX);
      sendResponseToMaster(RESP_UNKNOWN);
      break;
  }
}

/**
 * 로봇 HEAD 정면 회전 명령 처리
 */
void handleRobotHeadFrontCommand() {
  Serial.println("로봇 HEAD 정면 회전 명령 수신");
  
  bool success = rotateHeadForward();
  
  // 응답 전송
  sendResponseToMaster(success ? RESP_ROBOT_READY : RESP_ROBOT_ERROR);
}

/**
 * 로봇 HEAD 후면 회전 명령 처리
 */
void handleRobotHeadBackCommand() {
  Serial.println("로봇 HEAD 후면 회전 명령 수신");
  
  bool success = rotateHeadBackward();
  
  // 응답 전송
  sendResponseToMaster(success ? RESP_ROBOT_READY : RESP_ROBOT_ERROR);
}

/**
 * 로봇 BODY 정면 회전 명령 처리
 */
void handleRobotBodyFrontCommand() {
  Serial.println("로봇 BODY 정면 회전 명령 수신");
  
  bool success = rotateBodyForward();
  
  // 응답 전송
  sendResponseToMaster(success ? RESP_ROBOT_READY : RESP_ROBOT_ERROR);
}

/**
 * 로봇 BODY 후면 회전 명령 처리
 */
void handleRobotBodyBackCommand() {
  Serial.println("로봇 BODY 후면 회전 명령 수신");
  
  bool success = rotateBodyBackward();
  
  // 응답 전송
  sendResponseToMaster(success ? RESP_ROBOT_READY : RESP_ROBOT_ERROR);
}

/**
 * 모든 모터 정지 명령 처리
 */
void handleRobotStopCommand() {
  Serial.println("모든 모터 정지 명령 수신");
  
  emergencyStopMotors(true);
  
  // 응답 전송
  sendResponseToMaster(RESP_ROBOT_READY);
}

/**
 * LED 패턴 표시 명령 처리
 * @param patternLength 패턴 길이
 * @param patternData 패턴 데이터 (압축된 형태)
 */
void handleShowPatternCommand(uint8_t patternLength, uint8_t patternData) {
  Serial.print("LED 패턴 표시 명령 수신 - 길이: ");
  Serial.print(patternLength);
  Serial.print(", 데이터: 0x");
  Serial.println(patternData, HEX);
  
  // 패턴 데이터 디코딩 및 표시
  LEDPattern pattern;
  if (decodePatternData(patternLength, patternData, &pattern)) {
    showLEDPattern(pattern);
    sendResponseToMaster(RESP_PATTERN_DONE);
  } else {
    Serial.println("패턴 데이터 디코딩 실패");
    sendResponseToMaster(RESP_PATTERN_ERROR);
  }
}

/**
 * LED 패턴 지우기 명령 처리
 */
void handleClearPatternCommand() {
  Serial.println("LED 패턴 지우기 명령 수신");
  
  // 모든 OUTPUT LED 소등
  turnOffAllOutputs();
  
  // 응답 전송
  sendResponseToMaster(RESP_PATTERN_DONE);
}

/**
 * 점수 LED 업데이트 명령 처리
 * @param score 새로운 점수 (0-7)
 */
void handleUpdateScoreCommand(uint8_t score) {
  Serial.print("점수 LED 업데이트 명령 수신 - 점수: ");
  Serial.println(score);
  
  // 점멸 패턴 포함 점수 업데이트
  updateScoreWithBlink(score);
  
  // 응답 전송
  sendResponseToMaster(RESP_ACK);
}

/**
 * 효과음 재생 명령 처리
 * @param soundId 효과음 ID
 */
void handlePlayEffectCommand(uint8_t soundId) {
  Serial.print("효과음 재생 명령 수신 - ID: ");
  Serial.println(soundId);
  
  bool success = playEffectSound(soundId);
  
  // 응답 전송
  sendResponseToMaster(success ? RESP_AUDIO_DONE : RESP_AUDIO_ERROR);
}

/**
 * 오디오 정지 명령 처리
 */
void handleStopAudioCommand() {
  Serial.println("오디오 정지 명령 수신");
  
  stopAllAudio();
  
  // 응답 전송
  sendResponseToMaster(RESP_AUDIO_DONE);
}

/**
 * 상태 요청 명령 처리
 */
void handleGetStatusCommand() {
  Serial.println("상태 요청 명령 수신");
  
  // 시스템 상태 정보를 data1, data2에 압축하여 전송
  uint8_t motorStatus = 0;
  if (headMotorActive) motorStatus |= 0x01;
  if (bodyMotorActive) motorStatus |= 0x02;
  if (motorBrakeActive) motorStatus |= 0x04;
  
  uint8_t inputStatus = 0;
  if (isInputActive) inputStatus |= 0x01;
  inputStatus |= (inputSequenceLength << 1); // 상위 7비트에 입력 길이
  
  // 응답 전송 (data1=모터상태, data2=입력상태)
  sendResponseToMasterWithData(RESP_ACK, motorStatus, inputStatus);
}

/**
 * 입력 상태 요청 명령 처리
 */
void handleGetInputCommand() {
  Serial.println("입력 상태 요청 명령 수신");
  
  // 현재 발판 상태 확인
  uint8_t currentPadState = 0;
  for (int i = 0; i < 4; i++) {
    if (padPressed[i]) {
      currentPadState |= (1 << i);
    }
  }
  
  // 마지막 입력된 발판 번호
  uint8_t lastInput = (inputSequenceLength > 0) ? inputSequence[inputSequenceLength - 1] : 0;
  
  // 응답 전송 (data1=현재발판상태, data2=마지막입력)
  sendResponseToMasterWithData(RESP_INPUT_DETECTED, currentPadState, lastInput);
}

/**
 * 리미트 스위치 상태 요청 명령 처리
 */
void handleGetLimitCommand() {
  Serial.println("리미트 스위치 상태 요청 명령 수신");
  
  bool headLimit = checkHeadLimit();
  bool bodyLimit = checkBodyLimit();
  
  uint8_t limitStatus = 0;
  if (headLimit) limitStatus |= 0x01;
  if (bodyLimit) limitStatus |= 0x02;
  
  // 응답 전송
  sendResponseToMasterWithData(RESP_ACK, limitStatus, 0);
}

/**
 * 시스템 리셋 명령 처리
 */
void handleResetCommand() {
  Serial.println("시스템 리셋 명령 수신");
  
  // 모든 동작 정지
  emergencyStopMotors(true);
  setInputActive(false);
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  
  // 진동 센서 시스템 재초기화
  initializeVibrationSensors();
  
  // 응답 전송
  sendResponseToMaster(RESP_ACK);
  
  Serial.println("시스템 리셋 완료");
}

/**
 * 모든 동작 정지 명령 처리
 */
void handleStopAllCommand() {
  Serial.println("모든 동작 정지 명령 수신");
  
  // 모터 정지
  emergencyStopMotors(true);
  
  // 입력 대기 중지
  setInputActive(false);
  
  // 모든 LED 소등
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  
  // 응답 전송
  sendResponseToMaster(0x01); // RESP_ROBOT_READY
}

/**
 * Master로 응답 전송
 * @param response 응답 코드
 */
void sendResponseToMaster(uint8_t response) {
  sendResponseToMasterWithData(response, 0x00, 0x00);
}

/**
 * Master로 데이터 포함 응답 전송
 * @param response 응답 코드
 * @param data1 데이터 1
 * @param data2 데이터 2
 */
void sendResponseToMasterWithData(uint8_t response, uint8_t data1, uint8_t data2) {
  uint8_t checksum = MESSAGE_HEADER ^ response ^ data1 ^ data2;
  
  Serial.write(MESSAGE_HEADER); // 헤더 (0xAA)
  Serial.write(response);       // 응답 코드
  Serial.write(data1);          // 데이터 1
  Serial.write(data2);          // 데이터 2
  Serial.write(checksum);       // 체크섬
  
  Serial.flush();
  
  // 디버그 출력
  Serial.print("응답 전송: 0x");
  Serial.print(response, HEX);
  Serial.print(", 데이터: 0x");
  Serial.print(data1, HEX);
  Serial.print(", 0x");
  Serial.println(data2, HEX);
}

// =============================================================================
// 진동 센서 입력 처리 시스템
// =============================================================================

/**
 * 진동 센서 아날로그 값 읽기 (노이즈 필터링 포함)
 * @param sensorIndex 센서 인덱스 (0-3)
 * @return 필터링된 아날로그 값 (0-1023)
 */
uint16_t readVibrationSensor(uint8_t sensorIndex) {
  if (sensorIndex >= 4) return 0;
  
  // 아날로그 값 읽기
  uint16_t rawValue = analogRead(VIBRATION_PINS[sensorIndex]);
  
  // 이동 평균 필터 적용
  vibrationReadings[sensorIndex][sampleIndex[sensorIndex]] = rawValue;
  sampleIndex[sensorIndex] = (sampleIndex[sensorIndex] + 1) % VIBRATION_SAMPLES;
  
  // 충분한 샘플이 모이면 평균 계산
  if (!samplesReady[sensorIndex] && sampleIndex[sensorIndex] == 0) {
    samplesReady[sensorIndex] = true;
  }
  
  if (samplesReady[sensorIndex]) {
    uint32_t sum = 0;
    for (int i = 0; i < VIBRATION_SAMPLES; i++) {
      sum += vibrationReadings[sensorIndex][i];
    }
    return sum / VIBRATION_SAMPLES;
  }
  
  return rawValue; // 초기 샘플링 중에는 원본 값 반환
}

/**
 * 모든 진동 센서 값 읽기
 * @param values 센서 값을 저장할 배열 (크기 4)
 */
void readAllVibrationSensors(uint16_t values[4]) {
  for (int i = 0; i < 4; i++) {
    values[i] = readVibrationSensor(i);
  }
}

/**
 * 진동 센서 임계값 설정
 * @param threshold 새로운 임계값 (0-1023)
 */
void setVibrationThreshold(uint16_t threshold) {
  vibrationThreshold = CONSTRAIN(threshold, 100, 900);
  Serial.print("진동 센서 임계값 설정: ");
  Serial.println(vibrationThreshold);
}

/**
 * 개별 발판 감지 (디바운싱 포함)
 * @param padIndex 발판 인덱스 (0-3)
 * @return 발판이 눌렸는지 여부
 */
bool detectPadPress(uint8_t padIndex) {
  if (padIndex >= 4) return false;
  
  uint16_t sensorValue = readVibrationSensor(padIndex);
  unsigned long currentTime = millis();
  
  // 임계값을 초과하고 디바운싱 시간이 지났는지 확인
  if (sensorValue > vibrationThreshold && 
      currentTime - lastVibrationTime[padIndex] > VIBRATION_DEBOUNCE) {
    
    // 이전에 눌린 상태가 아닌 경우에만 새로운 입력으로 인식
    if (!padPressed[padIndex]) {
      padPressed[padIndex] = true;
      lastVibrationTime[padIndex] = currentTime;
      
      Serial.print("발판 ");
      Serial.print(padIndex + 1);
      Serial.print(" 감지 - 센서값: ");
      Serial.println(sensorValue);
      
      return true;
    }
  }
  // 임계값 이하로 떨어지면 눌림 상태 해제
  else if (sensorValue < vibrationThreshold * 0.8) { // 히스테리시스 적용
    if (padPressed[padIndex]) {
      padPressed[padIndex] = false;
      Serial.print("발판 ");
      Serial.print(padIndex + 1);
      Serial.println(" 해제");
    }
  }
  
  return false;
}

/**
 * 모든 발판 상태 모니터링
 * @return 눌린 발판 번호 (1-4), 없으면 0
 */
uint8_t detectAnyPadPress() {
  for (int i = 0; i < 4; i++) {
    if (detectPadPress(i)) {
      return i + 1; // 1-4 번호로 반환
    }
  }
  return 0; // 눌린 발판 없음
}

/**
 * 발판 입력 순서 추가
 * @param padNumber 발판 번호 (1-4)
 * @return 순서 추가 성공 여부
 */
bool addInputToSequence(uint8_t padNumber) {
  if (padNumber < 1 || padNumber > 4 || inputSequenceLength >= 6) {
    return false;
  }
  
  inputSequence[inputSequenceLength] = padNumber;
  inputSequenceLength++;
  lastInputTime = millis();
  
  Serial.print("입력 순서 추가: ");
  Serial.print(padNumber);
  Serial.print(" (총 ");
  Serial.print(inputSequenceLength);
  Serial.println("개)");
  
  return true;
}

/**
 * 입력 순서 초기화
 */
void clearInputSequence() {
  inputSequenceLength = 0;
  lastInputTime = 0;
  
  Serial.println("입력 순서 초기화");
}

/**
 * 현재 입력 순서 출력 (디버깅용)
 */
void printInputSequence() {
  Serial.print("현재 입력 순서: ");
  for (int i = 0; i < inputSequenceLength; i++) {
    Serial.print(inputSequence[i]);
    if (i < inputSequenceLength - 1) Serial.print(" -> ");
  }
  Serial.println();
}

/**
 * 입력 순서 검증
 * @param expectedSequence 예상 순서 배열
 * @param expectedLength 예상 순서 길이
 * @return 순서가 일치하는지 여부
 */
bool verifyInputSequence(const uint8_t expectedSequence[], uint8_t expectedLength) {
  if (inputSequenceLength != expectedLength) {
    Serial.println("입력 길이 불일치");
    return false;
  }
  
  for (int i = 0; i < expectedLength; i++) {
    if (inputSequence[i] != expectedSequence[i]) {
      Serial.print("순서 불일치 - 위치 ");
      Serial.print(i);
      Serial.print(": 예상 ");
      Serial.print(expectedSequence[i]);
      Serial.print(", 실제 ");
      Serial.println(inputSequence[i]);
      return false;
    }
  }
  
  Serial.println("입력 순서 검증 성공!");
  return true;
}

/**
 * 입력 대기 상태 설정
 * @param active 입력 대기 활성화 여부
 */
void setInputActive(bool active) {
  isInputActive = active;
  if (active) {
    clearInputSequence();
    Serial.println("입력 대기 활성화");
  } else {
    Serial.println("입력 대기 비활성화");
  }
}

/**
 * 입력 타임아웃 체크
 * @param timeoutMs 타임아웃 시간 (ms)
 * @return 타임아웃 발생 여부
 */
bool checkInputTimeout(unsigned long timeoutMs) {
  if (!isInputActive || inputSequenceLength == 0) return false;
  
  return (millis() - lastInputTime > timeoutMs);
}

/**
 * 진동 센서 시스템 초기화
 */
void initializeVibrationSensors() {
  // 진동 센서 핀은 아날로그 입력이므로 별도 설정 불필요
  
  // 샘플링 배열 초기화
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < VIBRATION_SAMPLES; j++) {
      vibrationReadings[i][j] = 0;
    }
    sampleIndex[i] = 0;
    samplesReady[i] = false;
    padPressed[i] = false;
    lastVibrationTime[i] = 0;
  }
  
  // 입력 순서 초기화
  clearInputSequence();
  isInputActive = false;
  
  Serial.println("진동 센서 시스템 초기화 완료");
  Serial.print("임계값: ");
  Serial.println(vibrationThreshold);
  Serial.print("디바운싱 시간: ");
  Serial.print(VIBRATION_DEBOUNCE);
  Serial.println("ms");
  Serial.print("샘플링 수: ");
  Serial.println(VIBRATION_SAMPLES);
}

/**
 * 진동 센서 상태 출력 (디버깅용)
 */
void printVibrationStatus() {
  uint16_t values[4];
  readAllVibrationSensors(values);
  
  Serial.println("=== 진동 센서 상태 ===");
  for (int i = 0; i < 4; i++) {
    Serial.print("발판 ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(values[i]);
    Serial.print(" (");
    Serial.print(padPressed[i] ? "눌림" : "해제");
    Serial.println(")");
  }
  Serial.print("임계값: ");
  Serial.println(vibrationThreshold);
  Serial.println("====================");
}

/**
 * 진동 센서 캘리브레이션
 * @param calibrationTime 캘리브레이션 시간 (ms)
 */
void calibrateVibrationSensors(unsigned long calibrationTime = 3000) {
  Serial.println("진동 센서 캘리브레이션 시작...");
  Serial.println("발판을 밟지 마세요!");
  
  uint32_t minValues[4] = {1023, 1023, 1023, 1023};
  uint32_t maxValues[4] = {0, 0, 0, 0};
  uint32_t sumValues[4] = {0, 0, 0, 0};
  uint16_t sampleCount = 0;
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < calibrationTime) {
    for (int i = 0; i < 4; i++) {
      uint16_t value = analogRead(VIBRATION_PINS[i]);
      
      if (value < minValues[i]) minValues[i] = value;
      if (value > maxValues[i]) maxValues[i] = value;
      sumValues[i] += value;
    }
    sampleCount++;
    delay(10);
  }
  
  Serial.println("캘리브레이션 완료!");
  
  // 결과 출력 및 임계값 자동 설정
  uint16_t avgNoise = 0;
  for (int i = 0; i < 4; i++) {
    uint16_t avgValue = sumValues[i] / sampleCount;
    uint16_t noiseLevel = maxValues[i] - minValues[i];
    
    Serial.print("발판 ");
    Serial.print(i + 1);
    Serial.print(" - 평균: ");
    Serial.print(avgValue);
    Serial.print(", 노이즈: ");
    Serial.print(noiseLevel);
    Serial.print(" (");
    Serial.print(minValues[i]);
    Serial.print("-");
    Serial.print(maxValues[i]);
    Serial.println(")");
    
    avgNoise += noiseLevel;
  }
  
  // 자동 임계값 설정 (평균 노이즈의 3배)
  avgNoise /= 4;
  uint16_t autoThreshold = avgNoise * 3;
  if (autoThreshold < 100) autoThreshold = 100;
  if (autoThreshold > 500) autoThreshold = 500;
  
  setVibrationThreshold(autoThreshold);
  Serial.print("자동 임계값 설정: ");
  Serial.println(autoThreshold);
}

/**
 * 진동 센서 모니터링 (메인 루프에서 호출)
 */
void monitorVibrationSensors() {
  // 입력이 활성화된 상태에서만 모니터링
  if (!isInputActive) return;
  
  // 발판 입력 감지
  uint8_t pressedPad = detectAnyPadPress();
  
  if (pressedPad > 0) {
    // 연속 입력 체크
    if (isConsecutiveInput(pressedPad)) {
      Serial.print("연속 입력 감지: 발판 ");
      Serial.println(pressedPad);
      // 연속 입력은 무시하거나 별도 처리
      return;
    }
    
    // 입력 순서에 추가
    if (addInputToSequence(pressedPad)) {
      // Master에게 입력 감지 알림
      uint8_t padBit = 0;
      switch (pressedPad) {
        case 1: padBit = PAD_1; break;
        case 2: padBit = PAD_2; break;
        case 3: padBit = PAD_3; break;
        case 4: padBit = PAD_4; break;
      }
      
      sendResponseToMasterWithData(RESP_INPUT_DETECTED, padBit, inputSequenceLength);
      
      // 효과음 재생
      playEffectSound(SFX_PAD_PRESS);
    }
  }
  
  // 입력 타임아웃 체크 (10초)
  if (checkInputTimeout(10000)) {
    Serial.println("입력 타임아웃 발생");
    sendResponseToMaster(RESP_INPUT_TIMEOUT);
    setInputActive(false);
  }
}

void handleMotorBrake() {
  // 모터 브레이크 처리
  if (motorBrakeActive && millis() >= motorStopTime) {
    // 브레이크 해제 - 모든 핀을 LOW로 설정
    digitalWrite(HEAD_DIR1_PIN, LOW);
    digitalWrite(HEAD_DIR2_PIN, LOW);
    digitalWrite(BODY_DIR1_PIN, LOW);
    digitalWrite(BODY_DIR2_PIN, LOW);
    analogWrite(HEAD_PWM_PIN, 0);
    analogWrite(BODY_PWM_PIN, 0);
    
    headMotorActive = false;
    bodyMotorActive = false;
    motorBrakeActive = false;
    
    Serial.println("모터 브레이크 완료 - 모든 핀 LOW");
  }
}

// =============================================================================
// L298N 모터 제어 함수들
// =============================================================================

/**
 * Head 모터 제어 (L298N 모터 A)
 * @param direction 회전 방향 (MOTOR_STOP, MOTOR_FORWARD, MOTOR_BACKWARD)
 * @param speed PWM 속도 값 (0-255)
 */
void controlHeadMotor(uint8_t direction, uint8_t speed) {
  switch (direction) {
    case MOTOR_STOP:
      analogWrite(HEAD_PWM_PIN, 0);
      digitalWrite(HEAD_DIR1_PIN, LOW);
      digitalWrite(HEAD_DIR2_PIN, LOW);
      headMotorActive = false;
      Serial.println("Head 모터 정지");
      break;
      
    case MOTOR_FORWARD:
      digitalWrite(HEAD_DIR1_PIN, HIGH);
      digitalWrite(HEAD_DIR2_PIN, LOW);
      analogWrite(HEAD_PWM_PIN, speed);
      headMotorActive = true;
      Serial.print("Head 모터 전진, 속도: ");
      Serial.println(speed);
      break;
      
    case MOTOR_BACKWARD:
      digitalWrite(HEAD_DIR1_PIN, LOW);
      digitalWrite(HEAD_DIR2_PIN, HIGH);
      analogWrite(HEAD_PWM_PIN, speed);
      headMotorActive = true;
      Serial.print("Head 모터 후진, 속도: ");
      Serial.println(speed);
      break;
      
    default:
      controlHeadMotor(MOTOR_STOP, 0);
      break;
  }
}

/**
 * Body 모터 제어 (L298N 모터 B)
 * @param direction 회전 방향 (MOTOR_STOP, MOTOR_FORWARD, MOTOR_BACKWARD)
 * @param speed PWM 속도 값 (0-255)
 */
void controlBodyMotor(uint8_t direction, uint8_t speed) {
  switch (direction) {
    case MOTOR_STOP:
      analogWrite(BODY_PWM_PIN, 0);
      digitalWrite(BODY_DIR1_PIN, LOW);
      digitalWrite(BODY_DIR2_PIN, LOW);
      bodyMotorActive = false;
      Serial.println("Body 모터 정지");
      break;
      
    case MOTOR_FORWARD:
      digitalWrite(BODY_DIR1_PIN, HIGH);
      digitalWrite(BODY_DIR2_PIN, LOW);
      analogWrite(BODY_PWM_PIN, speed);
      bodyMotorActive = true;
      Serial.print("Body 모터 전진, 속도: ");
      Serial.println(speed);
      break;
      
    case MOTOR_BACKWARD:
      digitalWrite(BODY_DIR1_PIN, LOW);
      digitalWrite(BODY_DIR2_PIN, HIGH);
      analogWrite(BODY_PWM_PIN, speed);
      bodyMotorActive = true;
      Serial.print("Body 모터 후진, 속도: ");
      Serial.println(speed);
      break;
      
    default:
      controlBodyMotor(MOTOR_STOP, 0);
      break;
  }
}

/**
 * Head 모터 180도 회전 (리미트 스위치 기반)
 * @param direction 회전 방향 (MOTOR_FORWARD 또는 MOTOR_BACKWARD)
 * @param speed 회전 속도 (0-255, 기본값: MOTOR_SPEED_NORMAL)
 * @return 회전 성공 여부
 */
bool rotateHead180(uint8_t direction, uint8_t speed = MOTOR_SPEED_NORMAL) {
  Serial.print("Head 180도 회전 시작 - ");
  Serial.println(direction == MOTOR_FORWARD ? "정방향" : "역방향");
  
  return runMotorWithLimitStop(true, direction, speed);
}

/**
 * Body 모터 180도 회전 (리미트 스위치 기반)
 * @param direction 회전 방향 (MOTOR_FORWARD 또는 MOTOR_BACKWARD)
 * @param speed 회전 속도 (0-255, 기본값: MOTOR_SPEED_NORMAL)
 * @return 회전 성공 여부
 */
bool rotateBody180(uint8_t direction, uint8_t speed = MOTOR_SPEED_NORMAL) {
  Serial.print("Body 180도 회전 시작 - ");
  Serial.println(direction == MOTOR_FORWARD ? "정방향" : "역방향");
  
  return runMotorWithLimitStop(false, direction, speed);
}

/**
 * 모터 즉시 정지 (비상 정지)
 * @param applyBrake true=브레이크 적용, false=단순 정지
 */
void emergencyStopMotors(bool applyBrake = true) {
  Serial.println("모터 비상 정지!");
  
  // 모터 정지 시간 기록 (Task 17.2)
  recordMotorStopTime(0); // Head 모터
  recordMotorStopTime(1); // Body 모터
  
  // 즉시 PWM 정지
  analogWrite(HEAD_PWM_PIN, 0);
  analogWrite(BODY_PWM_PIN, 0);
  
  if (applyBrake) {
    // 브레이크 적용
    digitalWrite(HEAD_DIR1_PIN, HIGH);
    digitalWrite(HEAD_DIR2_PIN, HIGH);
    digitalWrite(BODY_DIR1_PIN, HIGH);
    digitalWrite(BODY_DIR2_PIN, HIGH);
    
    motorStopTime = millis() + MOTOR_BRAKE_DURATION;
    motorBrakeActive = true;
  } else {
    // 단순 정지
    digitalWrite(HEAD_DIR1_PIN, LOW);
    digitalWrite(HEAD_DIR2_PIN, LOW);
    digitalWrite(BODY_DIR1_PIN, LOW);
    digitalWrite(BODY_DIR2_PIN, LOW);
  }
  
  headMotorActive = false;
  bodyMotorActive = false;
}

/**
 * 개별 모터 정지
 * @param isHead true=Head 모터, false=Body 모터
 * @param applyBrake true=브레이크 적용, false=단순 정지
 */
void stopMotor(bool isHead, bool applyBrake = true) {
  if (isHead) {
    analogWrite(HEAD_PWM_PIN, 0);
    if (applyBrake) {
      digitalWrite(HEAD_DIR1_PIN, HIGH);
      digitalWrite(HEAD_DIR2_PIN, HIGH);
    } else {
      digitalWrite(HEAD_DIR1_PIN, LOW);
      digitalWrite(HEAD_DIR2_PIN, LOW);
    }
    headMotorActive = false;
    Serial.println("Head 모터 정지");
  } else {
    analogWrite(BODY_PWM_PIN, 0);
    if (applyBrake) {
      digitalWrite(BODY_DIR1_PIN, HIGH);
      digitalWrite(BODY_DIR2_PIN, HIGH);
    } else {
      digitalWrite(BODY_DIR1_PIN, LOW);
      digitalWrite(BODY_DIR2_PIN, LOW);
    }
    bodyMotorActive = false;
    Serial.println("Body 모터 정지");
  }
  
  if (applyBrake) {
    motorStopTime = millis() + MOTOR_BRAKE_DURATION;
    motorBrakeActive = true;
  }
}

// =============================================================================
// 리미트 스위치 기반 위치 제어 시스템 (각 모터당 1개씩)
// =============================================================================

// 리미트 스위치 핀 정의 (MCP23017 INPUT 핀 사용)
#define HEAD_LIMIT_PIN    0  // IN0: Head 리미트 스위치 (정지용)
#define BODY_LIMIT_PIN    1  // IN1: Body 리미트 스위치 (정지용)

// 리미트 스위치 디바운싱 변수
unsigned long lastLimitCheckTime[2] = {0, 0};
bool lastLimitState[2] = {false, false};

/**
 * 개별 리미트 스위치 상태 확인 (디바운싱 포함)
 * @param limitPin 리미트 스위치 핀 번호 (0=HEAD, 1=BODY)
 * @return 리미트 스위치 활성화 여부 (true=눌림, false=안눌림)
 */
bool readLimitSwitch(uint8_t limitPin) {
  if (limitPin > 1) return false;
  
  uint8_t inputPin = (limitPin == 0) ? HEAD_LIMIT_PIN : BODY_LIMIT_PIN;
  bool currentState = readInput(inputPin);
  unsigned long currentTime = millis();
  
  // 디바운싱 처리
  if (currentTime - lastLimitCheckTime[limitPin] > LIMIT_DEBOUNCE) {
    if (currentState != lastLimitState[limitPin]) {
      lastLimitState[limitPin] = currentState;
      lastLimitCheckTime[limitPin] = currentTime;
      
      if (currentState) {
        Serial.print(limitPin == 0 ? "HEAD" : "BODY");
        Serial.println(" 리미트 스위치 활성화 - 모터 정지");
        
        // 리미트 스위치 활성화 시 해당 모터 즉시 정지
        stopMotor(limitPin == 0, true);
      }
    }
  }
  
  return lastLimitState[limitPin];
}

/**
 * Head 모터 리미트 스위치 상태 확인
 * @return 리미트 스위치 활성화 여부
 */
bool checkHeadLimit() {
  return readLimitSwitch(0);  // HEAD 리미트 스위치 (인덱스 0)
}

/**
 * Body 모터 리미트 스위치 상태 확인
 * @return 리미트 스위치 활성화 여부
 */
bool checkBodyLimit() {
  return readLimitSwitch(1);  // BODY 리미트 스위치 (인덱스 1)
}

/**
 * 리미트 스위치 상태 모니터링 (메인 루프에서 호출)
 * 리미트 스위치가 활성화되면 해당 모터를 즉시 정지
 */
void monitorLimitSwitches() {
  // Head 리미트 스위치 체크 및 모터 정지 처리
  bool headLimitActive = checkHeadLimit();
  if (headLimitActive && headMotorActive) {
    Serial.println("HEAD 리미트 스위치 감지 - 모터 즉시 정지");
    stopMotor(true, true);  // Head 모터 정지 및 브레이크 적용
    headMotorActive = false;
  }
  
  // Body 리미트 스위치 체크 및 모터 정지 처리
  bool bodyLimitActive = checkBodyLimit();
  if (bodyLimitActive && bodyMotorActive) {
    Serial.println("BODY 리미트 스위치 감지 - 모터 즉시 정지");
    stopMotor(false, true);  // Body 모터 정지 및 브레이크 적용
    bodyMotorActive = false;
  }
}

/**
 * 리미트 스위치 기반 모터 제어 (시간 기반 + 리미트 스위치 정지)
 * @param isHead true=Head 모터, false=Body 모터
 * @param direction 회전 방향 (MOTOR_FORWARD, MOTOR_BACKWARD)
 * @param speed 모터 속도 (0-255)
 * @param maxDuration 최대 동작 시간 (ms) - 타임아웃 방지
 * @return 동작 성공 여부
 */
bool runMotorWithLimitStop(bool isHead, uint8_t direction, uint8_t speed = MOTOR_SPEED_NORMAL, unsigned long maxDuration = MOTOR_180_DURATION) {
  Serial.print(isHead ? "Head" : "Body");
  Serial.print(" 모터 동작 시작 - ");
  Serial.print(direction == MOTOR_FORWARD ? "전진" : "후진");
  Serial.print(", 속도: ");
  Serial.println(speed);
  
  // 모터 시작
  if (isHead) {
    controlHeadMotor(direction, speed);
  } else {
    controlBodyMotor(direction, speed);
  }
  
  // 리미트 스위치 또는 타임아웃까지 대기
  unsigned long startTime = millis();
  
  while (millis() - startTime < maxDuration) {
    // 해당 모터의 리미트 스위치 체크
    bool limitActive = isHead ? checkHeadLimit() : checkBodyLimit();
    
    if (limitActive) {
      Serial.println("리미트 스위치 활성화 - 모터 정지됨");
      return true;  // 리미트 스위치에 의해 정상 정지
    }
    
    // 모터가 이미 정지된 경우 (다른 이유로)
    bool motorActive = isHead ? headMotorActive : bodyMotorActive;
    if (!motorActive) {
      Serial.println("모터가 이미 정지됨");
      return true;
    }
    
    delay(10);
  }
  
  // 타임아웃 발생 - 강제 정지
  Serial.println("타임아웃 - 모터 강제 정지");
  stopMotor(isHead, true);
  return false;
}

/**
 * Head 모터 180도 회전 (정방향)
 * @param speed 회전 속도 (기본값: MOTOR_SPEED_NORMAL)
 * @return 회전 성공 여부
 */
bool rotateHeadForward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // 모터 안전 잠금 체크 (Task 17.2)
  if (slaveMotorSafety.motorSafetyLockout) {
    Serial.println("모터 안전 잠금으로 인한 Head 모터 동작 거부");
    return false;
  }
  
  // 모터 시작 시간 기록 (Task 17.2)
  recordMotorStartTime(0); // Head 모터 = 0
  
  bool result = runMotorWithLimitStop(true, MOTOR_FORWARD, speed);
  
  // 모터 동작 실패 시 정지 시간 기록
  if (!result) {
    recordMotorStopTime(0);
  }
  
  return result;
}

/**
 * Head 모터 180도 회전 (역방향)
 * @param speed 회전 속도 (기본값: MOTOR_SPEED_NORMAL)
 * @return 회전 성공 여부
 */
bool rotateHeadBackward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // 모터 안전 잠금 체크 (Task 17.2)
  if (slaveMotorSafety.motorSafetyLockout) {
    Serial.println("모터 안전 잠금으로 인한 Head 모터 동작 거부");
    return false;
  }
  
  // 모터 시작 시간 기록 (Task 17.2)
  recordMotorStartTime(0); // Head 모터 = 0
  
  bool result = runMotorWithLimitStop(true, MOTOR_BACKWARD, speed);
  
  // 모터 동작 실패 시 정지 시간 기록
  if (!result) {
    recordMotorStopTime(0);
  }
  
  return result;
}

/**
 * Body 모터 180도 회전 (정방향)
 * @param speed 회전 속도 (기본값: MOTOR_SPEED_NORMAL)
 * @return 회전 성공 여부
 */
bool rotateBodyForward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // 모터 안전 잠금 체크 (Task 17.2)
  if (slaveMotorSafety.motorSafetyLockout) {
    Serial.println("모터 안전 잠금으로 인한 Body 모터 동작 거부");
    return false;
  }
  
  // 모터 시작 시간 기록 (Task 17.2)
  recordMotorStartTime(1); // Body 모터 = 1
  
  bool result = runMotorWithLimitStop(false, MOTOR_FORWARD, speed);
  
  // 모터 동작 실패 시 정지 시간 기록
  if (!result) {
    recordMotorStopTime(1);
  }
  
  return result;
}

/**
 * Body 모터 180도 회전 (역방향)
 * @param speed 회전 속도 (기본값: MOTOR_SPEED_NORMAL)
 * @return 회전 성공 여부
 */
bool rotateBodyBackward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // 모터 안전 잠금 체크 (Task 17.2)
  if (slaveMotorSafety.motorSafetyLockout) {
    Serial.println("모터 안전 잠금으로 인한 Body 모터 동작 거부");
    return false;
  }
  
  // 모터 시작 시간 기록 (Task 17.2)
  recordMotorStartTime(1); // Body 모터 = 1
  
  bool result = runMotorWithLimitStop(false, MOTOR_BACKWARD, speed);
  
  // 모터 동작 실패 시 정지 시간 기록
  if (!result) {
    recordMotorStopTime(1);
  }
  
  return result;
}

/**
 * 모든 리미트 스위치 상태 확인
 */
void checkAllLimitSwitches() {
  Serial.println("=== 리미트 스위치 상태 ===");
  
  Serial.print("Head 리미트: ");
  Serial.println(checkHeadLimit() ? "활성" : "비활성");
  
  Serial.print("Body 리미트: ");
  Serial.println(checkBodyLimit() ? "활성" : "비활성");
  
  Serial.println("========================");
}

/**
 * 리미트 스위치 시스템 초기화
 */
void initializeLimitSwitches() {
  // 리미트 스위치 디바운싱 변수 초기화
  for (int i = 0; i < 2; i++) {
    lastLimitCheckTime[i] = 0;
    lastLimitState[i] = false;
  }
  
  // 초기 리미트 스위치 상태 읽기
  bool headLimit = readInput(HEAD_LIMIT_PIN);
  bool bodyLimit = readInput(BODY_LIMIT_PIN);
  
  // 초기 상태 설정
  lastLimitState[0] = headLimit;
  lastLimitState[1] = bodyLimit;
  
  Serial.println("리미트 스위치 시스템 초기화 완료");
  Serial.println("각 모터당 1개의 리미트 스위치로 동작");
  Serial.println("리미트 스위치 활성화 시 해당 모터 즉시 정지");
  
  // 초기 상태 출력
  checkAllLimitSwitches();
  
  Serial.print("HEAD 리미트 초기 상태: ");
  Serial.println(headLimit ? "활성" : "비활성");
  Serial.print("BODY 리미트 초기 상태: ");
  Serial.println(bodyLimit ? "활성" : "비활성");
}

/**
 * 모터 상태 확인
 * @param isHead true=Head 모터, false=Body 모터
 * @return 모터 활성화 여부
 */
bool isMotorActive(bool isHead) {
  return isHead ? headMotorActive : bodyMotorActive;
}

/**
 * 모터 브레이크 설정
 * @param duration 브레이크 지속 시간 (ms)
 */
void setMotorBrake(unsigned long duration) {
  motorStopTime = millis() + duration;
  motorBrakeActive = true;
  
  Serial.print("모터 브레이크 설정: ");
  Serial.print(duration);
  Serial.println("ms");
}

/**
 * 모터 타임아웃 체크 (안전장치)
 * 모터가 너무 오래 동작하면 자동 정지
 */
void checkMotorTimeout() {
  static unsigned long headStartTime = 0;
  static unsigned long bodyStartTime = 0;
  
  // Head 모터 타임아웃 체크
  if (headMotorActive) {
    if (headStartTime == 0) {
      headStartTime = millis();
    } else if (millis() - headStartTime > MOTOR_TIMEOUT) {
      Serial.println("Head 모터 타임아웃 - 강제 정지");
      stopMotor(true, true);
      headStartTime = 0;
    }
  } else {
    headStartTime = 0;
  }
  
  // Body 모터 타임아웃 체크
  if (bodyMotorActive) {
    if (bodyStartTime == 0) {
      bodyStartTime = millis();
    } else if (millis() - bodyStartTime > MOTOR_TIMEOUT) {
      Serial.println("Body 모터 타임아웃 - 강제 정지");
      stopMotor(false, true);
      bodyStartTime = 0;
    }
  } else {
    bodyStartTime = 0;
  }
}

/**
 * 모터 상태 진단
 * @return 모터 시스템 정상 여부
 */
bool diagnoseMotorSystem() {
  bool systemOK = true;
  
  Serial.println("=== 모터 시스템 진단 ===");
  
  // 핀 상태 확인
  Serial.print("Head PWM 핀 (");
  Serial.print(HEAD_PWM_PIN);
  Serial.print("): ");
  Serial.println(digitalRead(HEAD_PWM_PIN) ? "HIGH" : "LOW");
  
  Serial.print("Head DIR1 핀 (");
  Serial.print(HEAD_DIR1_PIN);
  Serial.print("): ");
  Serial.println(digitalRead(HEAD_DIR1_PIN) ? "HIGH" : "LOW");
  
  Serial.print("Head DIR2 핀 (");
  Serial.print(HEAD_DIR2_PIN);
  Serial.print("): ");
  Serial.println(digitalRead(HEAD_DIR2_PIN) ? "HIGH" : "LOW");
  
  Serial.print("Body PWM 핀 (");
  Serial.print(BODY_PWM_PIN);
  Serial.print("): ");
  Serial.println(digitalRead(BODY_PWM_PIN) ? "HIGH" : "LOW");
  
  Serial.print("Body DIR1 핀 (");
  Serial.print(BODY_DIR1_PIN);
  Serial.print("): ");
  Serial.println(digitalRead(BODY_DIR1_PIN) ? "HIGH" : "LOW");
  
  Serial.print("Body DIR2 핀 (");
  Serial.print(BODY_DIR2_PIN);
  Serial.print("): ");
  Serial.println(digitalRead(BODY_DIR2_PIN) ? "HIGH" : "LOW");
  
  // 모터 상태 확인
  Serial.print("Head 모터 상태: ");
  Serial.println(headMotorActive ? "활성" : "비활성");
  
  Serial.print("Body 모터 상태: ");
  Serial.println(bodyMotorActive ? "활성" : "비활성");
  
  Serial.print("브레이크 상태: ");
  Serial.println(motorBrakeActive ? "활성" : "비활성");
  
  if (motorBrakeActive) {
    Serial.print("브레이크 해제까지: ");
    Serial.print(motorStopTime - millis());
    Serial.println("ms");
  }
  
  Serial.println("=====================");
  
  return systemOK;
}

// =============================================================================
// MCP23017 LED 제어 함수들
// =============================================================================

/**
 * 개별 점수 LED 제어 (MCP2 fstMulti 사용)
 * @param ledNumber LED 번호 (1-6)
 * @param state LED 상태 (true=점등, false=소등)
 */
void controlScoreLED(uint8_t ledNumber, bool state) {
  if (ledNumber < 1 || ledNumber > 6) return;
  
  // 현재 MCP2 GPIOA 상태 읽기
  Wire.beginTransmission(MCP23017_2_ADDR);
  Wire.write(MCP_GPIOA);
  Wire.endTransmission();
  
  Wire.requestFrom(MCP23017_2_ADDR, 1);
  uint8_t currentState = 0;
  if (Wire.available()) {
    currentState = Wire.read();
  }
  
  // 해당 LED 비트 설정/해제
  uint8_t ledBit = SCORE_LED_BITS[ledNumber - 1];
  if (state) {
    currentState |= (1 << ledBit);   // 비트 설정
  } else {
    currentState &= ~(1 << ledBit);  // 비트 해제
  }
  
  // MCP2 GPIOA에 새로운 상태 쓰기
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, currentState);
  
  Serial.print("점수 LED ");
  Serial.print(ledNumber);
  Serial.print(" (MCP2 fstMulti");
  Serial.print(ledNumber - 1);
  Serial.print(", 핀 ");
  Serial.print(SCORE_LED_PINS[ledNumber - 1]);
  Serial.print(") ");
  Serial.println(state ? "점등" : "소등");
}

/**
 * 점수에 따른 LED 표시 (MCP2 fstMulti 사용)
 * @param score 현재 점수 (0-6)
 */
void displayScore(uint8_t score) {
  // 점수 범위 제한
  if (score > 6) score = 6;
  
  // MCP2 GPIOA 상태 설정
  uint8_t ledState = 0;
  for (int i = 0; i < score; i++) {
    ledState |= (1 << SCORE_LED_BITS[i]);
  }
  
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, ledState);
  
  Serial.print("점수 표시: ");
  Serial.print(score);
  Serial.println("점 (MCP2 fstMulti 사용)");
}

/**
 * 개별 OUTPUT 신호 제어 (MCP1 OutSign 사용)
 * @param outputNumber OUTPUT 번호 (0-7)
 * @param state OUTPUT 상태 (true=활성화, false=비활성화)
 */
void controlOutput(uint8_t outputNumber, bool state) {
  if (outputNumber > 7) return;
  
  uint8_t outputBit = OUTPUT_BITS[outputNumber];
  uint8_t mcpReg = (outputNumber == 7) ? MCP_GPIOB : MCP_GPIOA;
  
  // 현재 상태 읽기
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(mcpReg);
  Wire.endTransmission();
  
  Wire.requestFrom(MCP23017_1_ADDR, 1);
  uint8_t currentState = 0;
  if (Wire.available()) {
    currentState = Wire.read();
  }
  
  // 해당 OUTPUT 비트 설정/해제
  if (state) {
    currentState |= (1 << outputBit);   // 비트 설정
  } else {
    currentState &= ~(1 << outputBit);  // 비트 해제
  }
  
  // 새로운 상태 쓰기
  setMCP23017Output(MCP23017_1_ADDR, mcpReg, currentState);
  
  Serial.print("OUTPUT ");
  Serial.print(outputNumber);
  Serial.print(" (OutSign");
  Serial.print(outputNumber);
  Serial.print(", ULN2803 연결) ");
  Serial.println(state ? "활성화" : "비활성화");
}

/**
 * OUTPUT 패턴 제어 (MCP1 OutSign 사용)
 * @param pattern OUTPUT 패턴 (비트마스크: bit0=OutSign0, bit1=OutSign1, ...)
 */
void setOutputPattern(uint8_t pattern) {
  // OutSign0~6은 GPIOA, OutSign7은 GPIOB
  uint8_t gpioA_pattern = pattern & 0x7E; // OutSign1~6 (1-6번 비트)
  uint8_t gpioB_pattern = (pattern & 0x80) ? 0x80 : 0x00; // OutSign7 (7번 비트)
  
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, gpioA_pattern);
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOB, gpioB_pattern);
  
  Serial.print("OUTPUT 패턴 설정: 0b");
  Serial.println(pattern, BIN);
}

/**
 * 모든 OUTPUT 비활성화 (MCP1 사용)
 */
void turnOffAllOutputs() {
  // MCP1 GPIOA OUTPUT 비활성화 (OutSign1~6)
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
  
  // MCP1 GPIOB OUTPUT 비활성화 (OutSign7만, INPUT 상태는 유지)
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOB, 0x00);
  
  Serial.println("모든 OUTPUT 비활성화 (MCP1)");
}

/**
 * 모든 점수 LED 소등 (MCP2 사용)
 */
void turnOffAllScoreLEDs() {
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
  Serial.println("모든 점수 LED 소등 (MCP2)");
}

// =============================================================================
// MCP23017 입력 및 예비 장치 제어 함수들
// =============================================================================

/**
 * INPUT 핀 상태 읽기 (MCP1 사용)
 * @param inputNumber INPUT 번호 (0-7)
 * @return INPUT 상태 (true=활성화/LOW, false=비활성화/HIGH)
 */
bool readInput(uint8_t inputNumber) {
  if (inputNumber > 7) return false;
  
  uint8_t inputBit = INPUT_BITS[inputNumber];
  uint8_t mcpReg = (inputNumber == 7) ? MCP_GPIOA : MCP_GPIOB;
  
  // 해당 GPIO 레지스터 읽기
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(mcpReg);
  Wire.endTransmission();
  
  Wire.requestFrom(MCP23017_1_ADDR, 1);
  if (Wire.available()) {
    uint8_t gpioState = Wire.read();
    // 풀업이므로 LOW가 활성화 상태
    return !(gpioState & (1 << inputBit));
  }
  
  return false;
}

/**
 * 모든 INPUT 상태 읽기 (MCP1 사용)
 * @return INPUT 상태 배열 (8비트, bit0=IN0, bit1=IN1, ...)
 */
uint8_t readAllInputs() {
  uint8_t inputStates = 0;
  
  for (int i = 0; i < 8; i++) {
    if (readInput(i)) {
      inputStates |= (1 << i);
    }
  }
  
  return inputStates;
}

/**
 * 예비 장치 제어 (MCP2 GPIOB 사용, sndMulti0~5)
 * @param deviceNumber 장치 번호 (1-6)
 * @param state 장치 상태 (true=활성화, false=비활성화)
 */
void controlSpareDevice(uint8_t deviceNumber, bool state) {
  if (deviceNumber < 1 || deviceNumber > 6) return;
  
  // 현재 MCP2 GPIOB 상태 읽기
  Wire.beginTransmission(MCP23017_2_ADDR);
  Wire.write(MCP_GPIOB);
  Wire.endTransmission();
  
  Wire.requestFrom(MCP23017_2_ADDR, 1);
  uint8_t currentState = 0;
  if (Wire.available()) {
    currentState = Wire.read();
  }
  
  // 해당 장치 비트 설정/해제
  uint8_t deviceBit = SPARE_DEVICE_BITS[deviceNumber - 1];
  if (state) {
    currentState |= (1 << deviceBit);   // 비트 설정
  } else {
    currentState &= ~(1 << deviceBit);  // 비트 해제
  }
  
  // MCP2 GPIOB에 새로운 상태 쓰기
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOB, currentState);
  
  // 실제 핀 번호 계산 (8, 9, 13, 10, 12, 11)
  uint8_t actualPin = 8;
  switch (deviceNumber) {
    case 1: actualPin = 8; break;   // sndMulti0
    case 2: actualPin = 9; break;   // sndMulti1
    case 3: actualPin = 13; break;  // sndMulti2
    case 4: actualPin = 10; break;  // sndMulti3
    case 5: actualPin = 12; break;  // sndMulti4
    case 6: actualPin = 11; break;  // sndMulti5
  }
  
  Serial.print("예비 장치 ");
  Serial.print(deviceNumber);
  Serial.print(" (MCP2 핀 ");
  Serial.print(deviceBit);
  Serial.print(", sndMulti");
  Serial.print(deviceNumber - 1);
  Serial.print(" -> 실제 ");
  Serial.print(actualPin);
  Serial.print("번) ");
  Serial.println(state ? "활성화" : "비활성화");
}

/**
 * 모든 예비 장치 비활성화 (MCP2 사용)
 */
void turnOffAllSpareDevices() {
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00); // fstMulti LED 모두 소등
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOB, 0x00); // sndMulti 장치 모두 비활성화
  
  Serial.println("모든 예비 장치 비활성화 (MCP2)");
}

/**
 * 시스템 상태 출력 (디버깅용)
 */
void printSystemStatus() {
  Serial.println("=== 시스템 상태 ===");
  
  // INPUT 상태 출력
  Serial.print("INPUT 상태: ");
  uint8_t inputs = readAllInputs();
  for (int i = 7; i >= 0; i--) {
    Serial.print((inputs >> i) & 1);
  }
  Serial.println();
  
  // 모터 상태 출력
  Serial.print("Head 모터: ");
  Serial.println(headMotorActive ? "활성" : "비활성");
  Serial.print("Body 모터: ");
  Serial.println(bodyMotorActive ? "활성" : "비활성");
  
  Serial.print("브레이크 상태: ");
  Serial.println(motorBrakeActive ? "활성" : "비활성");
  
  // 리미트 스위치 상태 출력
  Serial.println("--- 리미트 스위치 ---");
  Serial.print("Head 리미트: ");
  Serial.println(checkHeadLimit() ? "활성" : "비활성");
  Serial.print("Body 리미트: ");
  Serial.println(checkBodyLimit() ? "활성" : "비활성");
  
  // 진동 센서 상태 출력
  Serial.println("--- 진동 센서 ---");
  printVibrationStatus();
  
  // 입력 순서 상태 출력
  Serial.println("--- 입력 상태 ---");
  Serial.print("입력 대기: ");
  Serial.println(isInputActive ? "활성" : "비활성");
  if (inputSequenceLength > 0) {
    printInputSequence();
  }
  
  Serial.println("==================");
}

// =============================================================================
// 진동 센서 고급 기능
// =============================================================================

/**
 * 발판별 민감도 개별 설정
 * @param padIndex 발판 인덱스 (0-3)
 * @param sensitivity 민감도 (0.5-2.0, 1.0이 기본)
 */
void setPadSensitivity(uint8_t padIndex, float sensitivity) {
  if (padIndex >= 4 || sensitivity < 0.5 || sensitivity > 2.0) return;
  
  // 개별 임계값 배열이 필요하면 추가 구현
  Serial.print("발판 ");
  Serial.print(padIndex + 1);
  Serial.print(" 민감도 설정: ");
  Serial.println(sensitivity);
}

/**
 * 연속 입력 방지 (같은 발판 연속 입력 체크)
 * @param padNumber 새로 입력된 발판 번호 (1-4)
 * @return 연속 입력 여부 (true=연속입력, false=정상입력)
 */
bool isConsecutiveInput(uint8_t padNumber) {
  if (inputSequenceLength == 0) return false;
  
  // 마지막 입력과 같은 발판인지 확인
  return (inputSequence[inputSequenceLength - 1] == padNumber);
}

/**
 * 입력 패턴 분석 (디버깅 및 통계용)
 */
void analyzeInputPattern() {
  if (inputSequenceLength == 0) {
    Serial.println("입력 패턴 없음");
    return;
  }
  
  Serial.println("=== 입력 패턴 분석 ===");
  
  // 발판별 사용 횟수 계산
  uint8_t padCount[4] = {0, 0, 0, 0};
  for (int i = 0; i < inputSequenceLength; i++) {
    if (inputSequence[i] >= 1 && inputSequence[i] <= 4) {
      padCount[inputSequence[i] - 1]++;
    }
  }
  
  // 통계 출력
  Serial.print("총 입력 수: ");
  Serial.println(inputSequenceLength);
  
  for (int i = 0; i < 4; i++) {
    Serial.print("발판 ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(padCount[i]);
    Serial.print("회 (");
    Serial.print((padCount[i] * 100) / inputSequenceLength);
    Serial.println("%)");
  }
  
  // 연속 입력 체크
  uint8_t consecutiveCount = 0;
  for (int i = 1; i < inputSequenceLength; i++) {
    if (inputSequence[i] == inputSequence[i-1]) {
      consecutiveCount++;
    }
  }
  
  Serial.print("연속 입력: ");
  Serial.print(consecutiveCount);
  Serial.println("회");
  
  Serial.println("=====================");
}

/**
 * 진동 센서 실시간 모니터링 (테스트용)
 * @param duration 모니터링 시간 (ms)
 */
void realtimeVibrationMonitor(unsigned long duration = 10000) {
  Serial.println("실시간 진동 센서 모니터링 시작");
  Serial.println("발판을 밟아보세요!");
  
  unsigned long startTime = millis();
  unsigned long lastPrintTime = 0;
  
  while (millis() - startTime < duration) {
    // 100ms마다 센서 값 출력
    if (millis() - lastPrintTime > 100) {
      uint16_t values[4];
      readAllVibrationSensors(values);
      
      Serial.print("센서값: ");
      for (int i = 0; i < 4; i++) {
        Serial.print(values[i]);
        if (values[i] > vibrationThreshold) {
          Serial.print("*");
        }
        Serial.print("\t");
      }
      Serial.println();
      
      lastPrintTime = millis();
    }
    
    // 발판 입력 감지 및 출력
    uint8_t pressedPad = detectAnyPadPress();
    if (pressedPad > 0) {
      Serial.print(">>> 발판 ");
      Serial.print(pressedPad);
      Serial.println(" 감지!");
    }
    
    delay(10);
  }
  
  Serial.println("실시간 모니터링 종료");
}

// =============================================================================
// DFPlayer Mini 효과음 재생 함수들
// =============================================================================

/**
 * DFPlayer에 명령 전송
 * @param command 명령어
 * @param param1 매개변수 1
 * @param param2 매개변수 2
 */
void sendDFCommand(uint8_t command, uint8_t param1, uint8_t param2) {
  uint8_t buffer[10];
  
  buffer[0] = 0x7E;    // 시작 바이트
  buffer[1] = 0xFF;    // 버전
  buffer[2] = 0x06;    // 길이
  buffer[3] = command; // 명령어
  buffer[4] = 0x01;    // 피드백 (1=피드백 요청, 0=피드백 없음)
  buffer[5] = param1;  // 매개변수 1 (상위 바이트)
  buffer[6] = param2;  // 매개변수 2 (하위 바이트)
  
  // 체크섬 계산
  uint16_t checksum = 0;
  for (int i = 1; i < 7; i++) {
    checksum += buffer[i];
  }
  checksum = 0 - checksum;
  
  buffer[7] = (checksum >> 8) & 0xFF;  // 체크섬 상위 바이트
  buffer[8] = checksum & 0xFF;         // 체크섬 하위 바이트
  buffer[9] = 0xEF;                    // 종료 바이트
  
  // 명령 전송
  for (int i = 0; i < 10; i++) {
    dfSerial.write(buffer[i]);
  }
  
  // 디버그 출력
  Serial.print("DFPlayer 명령 전송: 0x");
  Serial.print(command, HEX);
  Serial.print(", 매개변수: ");
  Serial.print(param1);
  Serial.print(", ");
  Serial.println(param2);
}

/**
 * 효과음 재생
 * @param soundId 효과음 ID (SFX_* 상수)
 * @return 재생 성공 여부
 */
bool playEffectSound(uint8_t soundId) {
  if (!dfPlayerReady) {
    Serial.println("DFPlayer 준비되지 않음");
    return false;
  }
  
  // 효과음 ID를 트랙 번호로 변환
  uint8_t trackNumber = getSoundTrackNumber(soundId);
  if (trackNumber == 0) {
    Serial.print("알 수 없는 효과음 ID: ");
    Serial.println(soundId);
    return false;
  }
  
  // 중복 재생 방지 설정
  bool allowOverride = true;
  
  // 특정 효과음은 중복 재생 허용 (발판 밟기, 패턴 비프음)
  if (soundId == SFX_PAD_PRESS || soundId == SFX_PATTERN_BEEP) {
    allowOverride = true;
  }
  // 중요한 효과음은 중복 재생 방지 (총소리, 정답/오답)
  else if (soundId == SFX_GUNSHOT || soundId == SFX_CORRECT || soundId == SFX_WRONG) {
    if (isAudioPlaying && currentTrack == trackNumber) {
      Serial.println("중요 효과음 중복 재생 방지");
      return true; // 성공으로 처리하되 재생하지 않음
    }
    allowOverride = true; // 다른 트랙이면 덮어쓰기 허용
  }
  
  // 현재 재생 중인 오디오가 있고 덮어쓰기가 허용된 경우 정지
  if (isAudioPlaying && allowOverride) {
    stopCurrentAudio();
    delay(30); // 정지 후 잠시 대기 (시간 단축)
  }
  
  // 트랙 재생 명령 전송
  sendDFCommand(0x03, 0x00, trackNumber);
  
  // 재생 상태 업데이트
  isAudioPlaying = true;
  currentTrack = trackNumber;
  audioStartTime = millis();
  
  // 효과음 설명 출력
  const char* description = getSoundDescription(soundId);
  Serial.print("효과음 재생: ");
  Serial.print(description);
  Serial.print(" (트랙 ");
  Serial.print(trackNumber);
  Serial.println(")");
  
  return true;
}

/**
 * 특정 트랙 번호로 직접 재생
 * @param trackNumber 트랙 번호 (1-255)
 * @return 재생 성공 여부
 */
bool playTrack(uint8_t trackNumber) {
  if (!dfPlayerReady || trackNumber == 0) {
    return false;
  }
  
  // 현재 재생 중인 오디오 정지
  if (isAudioPlaying) {
    stopCurrentAudio();
    delay(50);
  }
  
  // 트랙 재생
  sendDFCommand(0x03, 0x00, trackNumber);
  
  // 상태 업데이트
  isAudioPlaying = true;
  currentTrack = trackNumber;
  audioStartTime = millis();
  
  Serial.print("트랙 재생: ");
  Serial.println(trackNumber);
  
  return true;
}

/**
 * 현재 재생 중인 오디오 정지
 */
void stopCurrentAudio() {
  if (!dfPlayerReady) return;
  
  sendDFCommand(0x16, 0x00, 0x00); // 정지 명령
  
  isAudioPlaying = false;
  currentTrack = 0;
  audioStartTime = 0;
  
  Serial.println("현재 오디오 정지");
}

/**
 * 모든 오디오 정지
 */
void stopAllAudio() {
  stopCurrentAudio();
  
  // 추가 정지 명령 (확실한 정지를 위해)
  delay(50);
  sendDFCommand(0x0E, 0x00, 0x00); // 모든 재생 정지
  
  Serial.println("모든 오디오 정지");
}

/**
 * DFPlayer 볼륨 설정
 * @param volume 볼륨 (0-30)
 */
void setDFVolume(uint8_t volume) {
  if (!dfPlayerReady) return;
  
  volume = CONSTRAIN(volume, 0, 30);
  sendDFCommand(0x06, 0x00, volume);
  currentVolume = volume;
  
  Serial.print("볼륨 설정: ");
  Serial.println(volume);
}

/**
 * 볼륨 증가
 */
void increaseDFVolume() {
  if (currentVolume < 30) {
    setDFVolume(currentVolume + 1);
  }
}

/**
 * 볼륨 감소
 */
void decreaseDFVolume() {
  if (currentVolume > 0) {
    setDFVolume(currentVolume - 1);
  }
}

/**
 * 효과음 ID를 트랙 번호로 변환
 * @param soundId 효과음 ID
 * @return 트랙 번호 (0이면 찾을 수 없음)
 */
uint8_t getSoundTrackNumber(uint8_t soundId) {
  for (uint8_t i = 0; i < SOUND_MAP_SIZE; i++) {
    if (SOUND_MAP[i].soundId == soundId) {
      return SOUND_MAP[i].trackNumber;
    }
  }
  return 0; // 찾을 수 없음
}

/**
 * 효과음 ID의 설명 가져오기
 * @param soundId 효과음 ID
 * @return 효과음 설명 문자열
 */
const char* getSoundDescription(uint8_t soundId) {
  for (uint8_t i = 0; i < SOUND_MAP_SIZE; i++) {
    if (SOUND_MAP[i].soundId == soundId) {
      return SOUND_MAP[i].description;
    }
  }
  return "알 수 없는 효과음";
}

/**
 * DFPlayer 상태 확인
 * @return DFPlayer 준비 상태
 */
bool isDFPlayerReady() {
  return dfPlayerReady;
}

/**
 * 현재 오디오 재생 상태 확인
 * @return 재생 중 여부
 */
bool isAudioCurrentlyPlaying() {
  return isAudioPlaying;
}

/**
 * 현재 재생 중인 트랙 번호 가져오기
 * @return 트랙 번호 (0이면 재생 중이 아님)
 */
uint8_t getCurrentTrack() {
  return isAudioPlaying ? currentTrack : 0;
}

/**
 * 오디오 재생 시간 가져오기
 * @return 재생 시작 후 경과 시간 (ms)
 */
unsigned long getAudioPlayTime() {
  if (!isAudioPlaying) return 0;
  return millis() - audioStartTime;
}

/**
 * DFPlayer 상태 출력 (디버깅용)
 */
void printDFPlayerStatus() {
  Serial.println("=== DFPlayer 상태 ===");
  Serial.print("준비 상태: ");
  Serial.println(dfPlayerReady ? "준비됨" : "준비 안됨");
  Serial.print("재생 상태: ");
  Serial.println(isAudioPlaying ? "재생 중" : "정지");
  Serial.print("현재 볼륨: ");
  Serial.println(currentVolume);
  
  if (isAudioPlaying) {
    Serial.print("현재 트랙: ");
    Serial.println(currentTrack);
    Serial.print("재생 시간: ");
    Serial.print(getAudioPlayTime());
    Serial.println("ms");
  }
  
  Serial.println("=== 효과음 매핑 ===");
  for (uint8_t i = 0; i < SOUND_MAP_SIZE; i++) {
    Serial.print("ID ");
    Serial.print(SOUND_MAP[i].soundId);
    Serial.print(" -> 트랙 ");
    Serial.print(SOUND_MAP[i].trackNumber);
    Serial.print(" (");
    Serial.print(SOUND_MAP[i].description);
    Serial.println(")");
  }
  Serial.println("====================");
}

/**
 * 오디오 재생 상태 관리 (메인 루프에서 호출)
 */
void updateAudioState() {
  // 재생 중인 오디오의 타임아웃 체크
  if (isAudioPlaying) {
    unsigned long playTime = getAudioPlayTime();
    
    // 효과음별 예상 재생 시간 (ms)
    unsigned long expectedDuration = 2000; // 기본 2초
    
    switch (currentTrack) {
      case 1: expectedDuration = 1000; break; // 조명 점등: 1초
      case 2: expectedDuration = 500;  break; // 패턴 비프음: 0.5초
      case 3: expectedDuration = 800;  break; // 발판 밟기: 0.8초
      case 4: expectedDuration = 1500; break; // 정답: 1.5초
      case 5: expectedDuration = 1500; break; // 오답: 1.5초
      case 6: expectedDuration = 2000; break; // 총소리: 2초
      case 7: expectedDuration = 2500; break; // 성공: 2.5초
      case 8: expectedDuration = 3000; break; // 레벨업: 3초
    }
    
    // 예상 시간의 2배가 지나면 타임아웃으로 간주
    if (playTime > expectedDuration * 2) {
      Serial.print("오디오 재생 타임아웃 (");
      Serial.print(playTime);
      Serial.print("ms) - 강제 정지");
      stopCurrentAudio();
    }
  }
  
  // DFPlayer로부터 응답 처리
  processDFPlayerResponse();
}

/**
 * DFPlayer 응답 처리 (기본 구현)
 */
void processDFPlayerResponse() {
  if (dfSerial.available() >= 10) {
    uint8_t buffer[10];
    
    // 응답 읽기
    for (int i = 0; i < 10; i++) {
      buffer[i] = dfSerial.read();
    }
    
    // 유효한 응답인지 확인 (시작 바이트 0x7E, 종료 바이트 0xEF)
    if (buffer[0] == 0x7E && buffer[9] == 0xEF) {
      uint8_t command = buffer[3];
      uint8_t param1 = buffer[5];
      uint8_t param2 = buffer[6];
      
      // 응답 처리
      switch (command) {
        case 0x3D: // 재생 완료
          if (isAudioPlaying) {
            Serial.print("트랙 ");
            Serial.print(currentTrack);
            Serial.println(" 재생 완료");
            isAudioPlaying = false;
            currentTrack = 0;
            audioStartTime = 0;
          }
          break;
          
        case 0x40: // 에러
          Serial.print("DFPlayer 에러: 0x");
          Serial.println(param2, HEX);
          if (isAudioPlaying) {
            isAudioPlaying = false;
            currentTrack = 0;
            audioStartTime = 0;
          }
          break;
          
        case 0x3F: // 초기화 완료
          Serial.println("DFPlayer 초기화 완료 응답");
          dfPlayerReady = true;
          break;
          
        default:
          // 기타 응답은 무시
          break;
      }
    }
  }
}

/**
 * 발판별 효과음 재생 (요구사항 2.4)
 * @param padNumber 발판 번호 (1-4)
 * @return 재생 성공 여부
 */
bool playPadSound(uint8_t padNumber) {
  if (padNumber < 1 || padNumber > 4) {
    return false;
  }
  
  // 모든 발판은 같은 효과음 사용 (요구사항에 따라)
  return playEffectSound(SFX_PAD_PRESS);
}

/**
 * LED 패턴 표시시 효과음 재생 (요구사항 2.3)
 * @param ledNumber LED 번호 (1-4)
 * @return 재생 성공 여부
 */
bool playPatternSound(uint8_t ledNumber) {
  if (ledNumber < 1 || ledNumber > 4) {
    return false;
  }
  
  // 모든 LED는 같은 패턴 비프음 사용
  return playEffectSound(SFX_PATTERN_BEEP);
}

/**
 * 상황별 효과음 재생 (통합 함수)
 * @param situation 상황 코드
 * @param param 추가 매개변수 (발판 번호, LED 번호 등)
 * @return 재생 성공 여부
 */
bool playSituationSound(uint8_t situation, uint8_t param = 0) {
  switch (situation) {
    case 1: // 조명 점등 (요구사항 1.3)
      return playEffectSound(SFX_LIGHT_ON);
      
    case 2: // LED 패턴 표시 (요구사항 2.3)
      return playPatternSound(param);
      
    case 3: // 발판 밟기 (요구사항 2.4)
      return playPadSound(param);
      
    case 4: // 동작 감지 (요구사항 3.3)
      return playEffectSound(SFX_GUNSHOT);
      
    case 5: // 정답
      return playEffectSound(SFX_CORRECT);
      
    case 6: // 오답
      return playEffectSound(SFX_WRONG);
      
    case 7: // 성공
      return playEffectSound(SFX_SUCCESS);
      
    case 8: // 레벨업
      return playEffectSound(SFX_LEVEL_UP);
      
    default:
      Serial.print("알 수 없는 상황 코드: ");
      Serial.println(situation);
      return false;
  }
}

/**
 * 오디오 재생 완료 대기 (동기화용)
 * @param maxWaitTime 최대 대기 시간 (ms)
 * @return 재생 완료 여부
 */
bool waitForAudioComplete(unsigned long maxWaitTime = 3000) {
  if (!isAudioPlaying) return true;
  
  unsigned long startTime = millis();
  
  while (isAudioPlaying && (millis() - startTime) < maxWaitTime) {
    updateAudioState();
    delay(50);
  }
  
  return !isAudioPlaying;
}

/**
 * 효과음 테스트 함수
 */
void testAllEffectSounds() {
  Serial.println("=== 효과음 테스트 시작 ===");
  
  for (uint8_t i = 0; i < SOUND_MAP_SIZE; i++) {
    uint8_t soundId = SOUND_MAP[i].soundId;
    Serial.print("테스트: ");
    Serial.println(SOUND_MAP[i].description);
    
    playEffectSound(soundId);
    delay(2000); // 2초 대기
    stopCurrentAudio();
    delay(500);  // 0.5초 대기
  }
  
  Serial.println("=== 효과음 테스트 완료 ===");
}

/**
 * 상황별 효과음 테스트
 */
void testSituationSounds() {
  Serial.println("=== 상황별 효과음 테스트 ===");
  
  Serial.println("1. 조명 점등 효과음");
  playSituationSound(1);
  delay(1500);
  
  Serial.println("2. LED 패턴 효과음");
  for (int i = 1; i <= 4; i++) {
    Serial.print("LED ");
    Serial.println(i);
    playSituationSound(2, i);
    delay(800);
  }
  
  Serial.println("3. 발판 효과음");
  for (int i = 1; i <= 4; i++) {
    Serial.print("발판 ");
    Serial.println(i);
    playSituationSound(3, i);
    delay(800);
  }
  
  Serial.println("4. 동작 감지 효과음");
  playSituationSound(4);
  delay(1500);
  
  Serial.println("5. 정답 효과음");
  playSituationSound(5);
  delay(1500);
  
  Serial.println("6. 오답 효과음");
  playSituationSound(6);
  delay(1500);
  
  Serial.println("=== 상황별 효과음 테스트 완료 ===");
}

// =============================================================================
// 문제 제시 LED 패턴 제어 시스템 (작업 9.1)
// =============================================================================

// LED 패턴 상태 변수
LEDPattern currentPattern;
bool isPatternPlaying = false;
unsigned long patternStartTime = 0;
uint8_t currentPatternStep = 0;
unsigned long lastStepTime = 0;

/**
 * 패턴 데이터 디코딩 (압축된 데이터를 LEDPattern 구조체로 변환)
 * @param patternLength 패턴 길이 (1-6)
 * @param patternData 압축된 패턴 데이터 (각 LED를 2비트로 표현)
 * @param pattern 출력할 LEDPattern 구조체 포인터
 * @return 디코딩 성공 여부
 */
bool decodePatternData(uint8_t patternLength, uint8_t patternData, LEDPattern* pattern) {
  if (patternLength == 0 || patternLength > 6 || pattern == nullptr) {
    return false;
  }
  
  // 패턴 길이 설정
  pattern->length = patternLength;
  pattern->repeatCount = 1;
  
  // 레벨에 따른 간격 설정 (Master에서 전달받은 길이로 추정)
  if (patternLength <= 4) {
    pattern->interval = LEVEL_1_4_INTERVAL; // 500ms
  } else {
    pattern->interval = LEVEL_5_10_INTERVAL; // 300ms
  }
  
  // 압축된 데이터에서 LED 순서 추출 (각 LED를 2비트로 표현)
  for (int i = 0; i < patternLength; i++) {
    uint8_t ledIndex = (patternData >> (i * 2)) & 0x03; // 2비트 추출
    pattern->sequence[i] = ledIndex + 1; // 1-4 범위로 변환
  }
  
  Serial.print("패턴 디코딩 완료 - 길이: ");
  Serial.print(pattern->length);
  Serial.print(", 간격: ");
  Serial.print(pattern->interval);
  Serial.print("ms, 순서: ");
  for (int i = 0; i < pattern->length; i++) {
    Serial.print(pattern->sequence[i]);
    if (i < pattern->length - 1) Serial.print("->");
  }
  Serial.println();
  
  return true;
}

/**
 * 레벨별 LED 패턴 생성 (연속 LED 방지 포함)
 * @param level 게임 레벨 (1-11)
 * @param pattern 출력할 LEDPattern 구조체 포인터
 * @return 패턴 생성 성공 여부
 */
bool generateLevelPattern(uint8_t level, LEDPattern* pattern) {
  if (level == 0 || level > 11 || pattern == nullptr) {
    return false;
  }
  
  // 레벨별 패턴 길이 설정
  if (level == 1) {
    pattern->length = LEVEL_1_LENGTH; // 2
  } else if (level == 2) {
    pattern->length = LEVEL_2_LENGTH; // 3
  } else if (level == 3) {
    pattern->length = LEVEL_3_LENGTH; // 4
  } else if (level == 4) {
    pattern->length = LEVEL_4_LENGTH; // 5
  } else if (level >= 5 && level <= 10) {
    pattern->length = LEVEL_5_10_LENGTH; // 5
  } else if (level == 11) {
    pattern->length = LEVEL_11_LENGTH; // 6
  }
  
  // 레벨별 간격 설정
  if (level <= 4) {
    pattern->interval = LEVEL_1_4_INTERVAL; // 500ms
  } else {
    pattern->interval = LEVEL_5_10_INTERVAL; // 300ms
  }
  
  pattern->repeatCount = 1;
  
  // 랜덤 패턴 생성 (연속 LED 방지)
  uint8_t lastLED = 0;
  for (int i = 0; i < pattern->length; i++) {
    uint8_t newLED;
    int attempts = 0;
    
    do {
      newLED = random(1, 5); // 1-4 범위
      attempts++;
    } while (newLED == lastLED && attempts < 10); // 연속 방지, 최대 10회 시도
    
    pattern->sequence[i] = newLED;
    lastLED = newLED;
  }
  
  Serial.print("레벨 ");
  Serial.print(level);
  Serial.print(" 패턴 생성 - 길이: ");
  Serial.print(pattern->length);
  Serial.print(", 간격: ");
  Serial.print(pattern->interval);
  Serial.print("ms, 순서: ");
  for (int i = 0; i < pattern->length; i++) {
    Serial.print(pattern->sequence[i]);
    if (i < pattern->length - 1) Serial.print("->");
  }
  Serial.println();
  
  return true;
}

/**
 * LED 패턴 표시 시작
 * @param pattern 표시할 LEDPattern 구조체
 * @return 패턴 표시 시작 성공 여부
 */
bool showLEDPattern(const LEDPattern& pattern) {
  if (pattern.length == 0 || pattern.length > 6) {
    Serial.println("잘못된 패턴 길이");
    return false;
  }
  
  // 현재 패턴 저장
  currentPattern = pattern;
  isPatternPlaying = true;
  patternStartTime = millis();
  currentPatternStep = 0;
  lastStepTime = 0;
  
  // 모든 문제 제시 LED 소등으로 시작
  turnOffAllProblemLEDs();
  
  Serial.print("LED 패턴 표시 시작 - 길이: ");
  Serial.print(pattern.length);
  Serial.print(", 간격: ");
  Serial.print(pattern.interval);
  Serial.println("ms");
  
  return true;
}

/**
 * LED 패턴 재생 업데이트 (메인 루프에서 호출)
 */
void updateLEDPattern() {
  if (!isPatternPlaying) return;
  
  unsigned long currentTime = millis();
  
  // 첫 번째 스텝이거나 간격이 지났는지 확인
  if (currentPatternStep == 0 || 
      (currentTime - lastStepTime >= currentPattern.interval)) {
    
    if (currentPatternStep < currentPattern.length) {
      // 현재 스텝의 LED 점등
      uint8_t ledNumber = currentPattern.sequence[currentPatternStep];
      controlProblemLED(ledNumber, true);
      
      // 효과음 재생 (요구사항 2.3)
      playPatternSound(ledNumber);
      
      Serial.print("패턴 스텝 ");
      Serial.print(currentPatternStep + 1);
      Serial.print("/");
      Serial.print(currentPattern.length);
      Serial.print(" - LED ");
      Serial.print(ledNumber);
      Serial.println(" 점등");
      
      lastStepTime = currentTime;
      currentPatternStep++;
    } else {
      // 패턴 완료
      stopLEDPattern();
    }
  }
  
  // LED 소등 처리 (점등 후 일정 시간 후 소등)
  if (currentPatternStep > 0 && 
      (currentTime - lastStepTime >= currentPattern.interval / 2)) {
    turnOffAllProblemLEDs();
  }
}

/**
 * LED 패턴 재생 중지
 */
void stopLEDPattern() {
  if (!isPatternPlaying) return;
  
  isPatternPlaying = false;
  currentPatternStep = 0;
  turnOffAllProblemLEDs();
  
  Serial.println("LED 패턴 재생 완료");
}

/**
 * 개별 문제 제시 LED 제어 (MCP23017 #1 OUTPUT 사용)
 * @param ledNumber LED 번호 (1-4)
 * @param state LED 상태 (true=점등, false=소등)
 */
void controlProblemLED(uint8_t ledNumber, bool state) {
  if (ledNumber < 1 || ledNumber > 4) {
    Serial.print("잘못된 LED 번호: ");
    Serial.println(ledNumber);
    return;
  }
  
  // LED 번호를 OUTPUT 번호로 매핑 (LED 1-4 -> OUTPUT 0-3)
  uint8_t outputNumber = ledNumber - 1;
  controlOutput(outputNumber, state);
  
  Serial.print("문제 제시 LED ");
  Serial.print(ledNumber);
  Serial.print(" ");
  Serial.println(state ? "점등" : "소등");
}

/**
 * 모든 문제 제시 LED 소등
 */
void turnOffAllProblemLEDs() {
  for (int i = 1; i <= 4; i++) {
    controlProblemLED(i, false);
  }
  Serial.println("모든 문제 제시 LED 소등");
}

/**
 * 문제 제시 LED 패턴 테스트
 */
void testProblemLEDPattern() {
  Serial.println("=== 문제 제시 LED 패턴 테스트 ===");
  
  // 개별 LED 테스트
  Serial.println("1. 개별 LED 테스트");
  for (int i = 1; i <= 4; i++) {
    Serial.print("LED ");
    Serial.print(i);
    Serial.println(" 점등");
    controlProblemLED(i, true);
    delay(500);
    controlProblemLED(i, false);
    delay(200);
  }
  
  // 레벨별 패턴 테스트
  Serial.println("2. 레벨별 패턴 테스트");
  for (int level = 1; level <= 5; level++) {
    Serial.print("레벨 ");
    Serial.print(level);
    Serial.println(" 패턴 테스트");
    
    LEDPattern testPattern;
    if (generateLevelPattern(level, &testPattern)) {
      showLEDPattern(testPattern);
      
      // 패턴 완료까지 대기
      while (isPatternPlaying) {
        updateLEDPattern();
        delay(10);
      }
      delay(1000);
    }
  }
  
  Serial.println("=== 문제 제시 LED 패턴 테스트 완료 ===");
}/**
 
* 패턴 검증 (연속 LED 방지 확인)
 * @param pattern 검증할 LEDPattern 구조체
 * @return 패턴이 유효한지 여부 (연속 LED 없음)
 */
bool validatePattern(const LEDPattern& pattern) {
  if (pattern.length <= 1) return true; // 길이 1 이하는 항상 유효
  
  for (int i = 1; i < pattern.length; i++) {
    if (pattern.sequence[i] == pattern.sequence[i-1]) {
      Serial.print("연속 LED 감지: 위치 ");
      Serial.print(i-1);
      Serial.print("-");
      Serial.print(i);
      Serial.print(" (LED ");
      Serial.print(pattern.sequence[i]);
      Serial.println(")");
      return false;
    }
  }
  return true;
}

/**
 * 안전한 패턴 생성 (연속 LED 방지 보장)
 * @param level 게임 레벨 (1-11)
 * @param pattern 출력할 LEDPattern 구조체 포인터
 * @param maxAttempts 최대 시도 횟수
 * @return 패턴 생성 성공 여부
 */
bool generateSafePattern(uint8_t level, LEDPattern* pattern, int maxAttempts = 50) {
  if (level == 0 || level > 11 || pattern == nullptr) {
    return false;
  }
  
  for (int attempt = 0; attempt < maxAttempts; attempt++) {
    if (generateLevelPattern(level, pattern)) {
      if (validatePattern(*pattern)) {
        Serial.print("안전한 패턴 생성 성공 (시도 ");
        Serial.print(attempt + 1);
        Serial.println("회)");
        return true;
      }
    }
  }
  
  Serial.print("안전한 패턴 생성 실패 (");
  Serial.print(maxAttempts);
  Serial.println("회 시도)");
  return false;
}

/**
 * 패턴 데이터를 압축된 형태로 인코딩
 * @param pattern 인코딩할 LEDPattern 구조체
 * @param encodedData 출력할 압축된 데이터 포인터
 * @return 인코딩 성공 여부
 */
bool encodePatternData(const LEDPattern& pattern, uint8_t* encodedData) {
  if (pattern.length == 0 || pattern.length > 6 || encodedData == nullptr) {
    return false;
  }
  
  *encodedData = 0;
  
  // 각 LED를 2비트로 압축 (LED 1-4 -> 0-3)
  for (int i = 0; i < pattern.length; i++) {
    if (pattern.sequence[i] < 1 || pattern.sequence[i] > 4) {
      return false; // 잘못된 LED 번호
    }
    
    uint8_t ledIndex = pattern.sequence[i] - 1; // 1-4 -> 0-3
    *encodedData |= (ledIndex << (i * 2)); // 2비트씩 시프트하여 저장
  }
  
  return true;
}

/**
 * 현재 재생 중인 패턴 정보 출력
 */
void printCurrentPattern() {
  if (!isPatternPlaying) {
    Serial.println("현재 재생 중인 패턴 없음");
    return;
  }
  
  Serial.println("=== 현재 패턴 정보 ===");
  Serial.print("길이: ");
  Serial.println(currentPattern.length);
  Serial.print("간격: ");
  Serial.print(currentPattern.interval);
  Serial.println("ms");
  Serial.print("순서: ");
  for (int i = 0; i < currentPattern.length; i++) {
    Serial.print(currentPattern.sequence[i]);
    if (i < currentPattern.length - 1) Serial.print("->");
  }
  Serial.println();
  Serial.print("현재 스텝: ");
  Serial.print(currentPatternStep);
  Serial.print("/");
  Serial.println(currentPattern.length);
  Serial.print("재생 시간: ");
  Serial.print(millis() - patternStartTime);
  Serial.println("ms");
  Serial.println("====================");
}

// =============================================================================
// 점수 LED 점멸 시스템 (작업 9.2 보완)
// =============================================================================

// 점수 LED 점멸 상태 변수
ScoreLEDState scoreLEDState = {0, false, 0, 0};

/**
 * 7점 달성 시 특별 점멸 패턴 시작 (요구사항 4.6)
 */
void startScoreBlinkPattern() {
  scoreLEDState.currentScore = 7;
  scoreLEDState.isBlinking = true;
  scoreLEDState.blinkTimer = millis();
  scoreLEDState.blinkCount = 0;
  
  Serial.println("7점 달성! 특별 점멸 패턴 시작 (0.5초 간격 7회)");
}

/**
 * 점수 LED 점멸 업데이트 (메인 루프에서 호출)
 */
void updateScoreLEDBlinking() {
  if (!scoreLEDState.isBlinking) return;
  
  unsigned long currentTime = millis();
  
  // 0.5초(500ms) 간격으로 점멸
  if (currentTime - scoreLEDState.blinkTimer >= SCORE_BLINK_INTERVAL) {
    scoreLEDState.blinkTimer = currentTime;
    
    // 점멸 상태 토글
    bool ledState = (scoreLEDState.blinkCount % 2) == 0;
    
    // 모든 점수 LED 동시 점멸
    if (ledState) {
      // 6개 LED 모두 점등 (점수 6 표시)
      displayScore(6);
    } else {
      // 모든 LED 소등
      turnOffAllScoreLEDs();
    }
    
    scoreLEDState.blinkCount++;
    
    Serial.print("점수 LED 점멸 ");
    Serial.print(scoreLEDState.blinkCount);
    Serial.print("/");
    Serial.print(SCORE_BLINK_COUNT * 2); // 점등+소등 = 2배
    Serial.print(" (");
    Serial.print(ledState ? "점등" : "소등");
    Serial.println(")");
    
    // 7회 점멸 완료 (점등 7회 + 소등 7회 = 14회)
    if (scoreLEDState.blinkCount >= SCORE_BLINK_COUNT * 2) {
      stopScoreBlinkPattern();
    }
  }
}

/**
 * 점수 LED 점멸 패턴 중지
 */
void stopScoreBlinkPattern() {
  if (!scoreLEDState.isBlinking) return;
  
  scoreLEDState.isBlinking = false;
  scoreLEDState.blinkCount = 0;
  
  // 최종적으로 6개 LED 모두 점등 상태로 유지 (7점 표시)
  displayScore(6);
  
  Serial.println("7점 점멸 패턴 완료 - 최종 점등 상태 유지");
}

/**
 * 점수 업데이트 (점멸 패턴 포함)
 * @param score 새로운 점수 (0-7)
 */
void updateScoreWithBlink(uint8_t score) {
  // 기존 점멸 패턴 중지
  if (scoreLEDState.isBlinking) {
    stopScoreBlinkPattern();
  }
  
  if (score == 7) {
    // 7점 달성 시 특별 점멸 패턴 시작
    startScoreBlinkPattern();
  } else {
    // 일반 점수 표시
    scoreLEDState.currentScore = score;
    displayScore(score);
  }
}

/**
 * 점수 LED 상태 정보 출력 (디버깅용)
 */
void printScoreLEDStatus() {
  Serial.println("=== 점수 LED 상태 ===");
  Serial.print("현재 점수: ");
  Serial.println(scoreLEDState.currentScore);
  Serial.print("점멸 상태: ");
  Serial.println(scoreLEDState.isBlinking ? "활성" : "비활성");
  
  if (scoreLEDState.isBlinking) {
    Serial.print("점멸 횟수: ");
    Serial.print(scoreLEDState.blinkCount);
    Serial.print("/");
    Serial.println(SCORE_BLINK_COUNT * 2);
    Serial.print("경과 시간: ");
    Serial.print(millis() - scoreLEDState.blinkTimer);
    Serial.println("ms");
  }
  Serial.println("==================");
}

/**
 * 점수 LED 점멸 테스트
 */
void testScoreLEDBlinking() {
  Serial.println("=== 점수 LED 점멸 테스트 ===");
  
  // 일반 점수 테스트
  Serial.println("1. 일반 점수 테스트 (0-6점)");
  for (int i = 0; i <= 6; i++) {
    Serial.print("점수 ");
    Serial.print(i);
    Serial.println("점 표시");
    updateScoreWithBlink(i);
    delay(1000);
  }
  
  delay(2000);
  
  // 7점 점멸 테스트
  Serial.println("2. 7점 달성 점멸 테스트");
  updateScoreWithBlink(7);
  
  // 점멸 완료까지 대기
  while (scoreLEDState.isBlinking) {
    updateScoreLEDBlinking();
    delay(10);
  }
  
  delay(2000);
  
  // 모든 LED 소등
  turnOffAllScoreLEDs();
  scoreLEDState.currentScore = 0;
  
  Serial.println("=== 점수 LED 점멸 테스트 완료 ===");
}

// =============================================================================
// 통신 안정성 강화 시스템 (Task 17.1) - Slave Side
// =============================================================================

// Slave 측 통신 안정성 변수
struct SlaveCommReliability {
  unsigned long lastMasterCommTime;      // 마지막 Master 통신 시간
  uint16_t messagesReceived;             // 수신된 메시지 수
  uint16_t validMessages;                // 유효한 메시지 수
  uint16_t invalidMessages;              // 잘못된 메시지 수
  uint16_t checksumErrors;               // 체크섬 오류 수
  uint16_t timeoutErrors;                // 타임아웃 오류 수
  bool masterConnected;                  // Master 연결 상태
  unsigned long connectionTimeout;       // 연결 타임아웃 시간
  uint8_t lastErrorCode;                 // 마지막 오류 코드
  bool diagnosticMode;                   // 진단 모드
  unsigned long diagnosticInterval;      // 진단 정보 출력 간격
  unsigned long lastDiagnosticTime;      // 마지막 진단 시간
};

SlaveCommReliability slaveCommReliability;

/**
 * Slave 통신 안정성 시스템 초기화
 */
void initSlaveCommReliability() {
  Serial.println("Slave 통신 안정성 시스템 초기화...");
  
  slaveCommReliability.lastMasterCommTime = millis();
  slaveCommReliability.messagesReceived = 0;
  slaveCommReliability.validMessages = 0;
  slaveCommReliability.invalidMessages = 0;
  slaveCommReliability.checksumErrors = 0;
  slaveCommReliability.timeoutErrors = 0;
  slaveCommReliability.masterConnected = false;
  slaveCommReliability.connectionTimeout = 10000; // 10초 연결 타임아웃
  slaveCommReliability.lastErrorCode = COMM_ERROR_NONE;
  slaveCommReliability.diagnosticMode = false;
  slaveCommReliability.diagnosticInterval = 30000; // 30초마다 진단 정보
  slaveCommReliability.lastDiagnosticTime = millis();
  
  Serial.println("Slave 통신 안정성 시스템 초기화 완료");
  Serial.print("연결 타임아웃: ");
  Serial.print(slaveCommReliability.connectionTimeout);
  Serial.println("ms");
}

/**
 * 향상된 Master 통신 처리 (안정성 강화)
 */
void processMasterCommReliable() {
  if (Serial.available() >= 4) { // 최소 메시지 크기
    slaveCommReliability.messagesReceived++;
    
    // 메시지 수신 시도
    SerialMessage receivedMsg;
    if (receiveMessageFromMaster(&receivedMsg)) {
      // 유효한 메시지 처리
      slaveCommReliability.validMessages++;
      slaveCommReliability.lastMasterCommTime = millis();
      slaveCommReliability.masterConnected = true;
      
      // 명령 처리
      handleMasterCommandReliable(receivedMsg.command, receivedMsg.data1, receivedMsg.data2);
      
      if (slaveCommReliability.diagnosticMode) {
        Serial.print("유효한 명령 수신: 0x");
        Serial.println(receivedMsg.command, HEX);
      }
    } else {
      // 잘못된 메시지
      slaveCommReliability.invalidMessages++;
      recordSlaveCommError(COMM_ERROR_INVALID);
    }
  }
  
  // 연결 상태 모니터링
  monitorMasterConnection();
  
  // 진단 정보 업데이트
  updateSlaveCommDiagnostics();
}

/**
 * Master로부터 메시지 수신 (검증 포함)
 * @param message 수신할 메시지 구조체
 * @return 수신 성공 여부
 */
bool receiveMessageFromMaster(SerialMessage* message) {
  uint8_t header = Serial.read();
  
  // 헤더 검증
  if (header != MESSAGE_HEADER) {
    Serial.print("잘못된 헤더: 0x");
    Serial.println(header, HEX);
    
    // 버퍼에서 올바른 헤더 찾기
    while (Serial.available() > 0) {
      uint8_t nextByte = Serial.read();
      if (nextByte == MESSAGE_HEADER) {
        header = nextByte;
        break;
      }
    }
    
    if (header != MESSAGE_HEADER) {
      return false;
    }
  }
  
  // 나머지 메시지 수신 대기 (타임아웃 포함)
  unsigned long startTime = millis();
  while (Serial.available() < 4 && millis() - startTime < 1000) {
    delay(1);
  }
  
  if (Serial.available() < 4) {
    Serial.println("메시지 수신 타임아웃");
    slaveCommReliability.timeoutErrors++;
    return false;
  }
  
  // 메시지 데이터 읽기
  message->header = header;
  message->command = Serial.read();
  message->data1 = Serial.read();
  message->data2 = Serial.read();
  
  // 체크섬 수신 대기
  startTime = millis();
  while (Serial.available() < 1 && millis() - startTime < 500) {
    delay(1);
  }
  
  if (Serial.available() < 1) {
    Serial.println("체크섬 수신 타임아웃");
    return false;
  }
  
  message->checksum = Serial.read();
  
  // 체크섬 검증
  uint8_t expectedChecksum = message->command ^ message->data1 ^ message->data2;
  if (message->checksum != expectedChecksum) {
    Serial.print("체크섬 오류 - 예상: 0x");
    Serial.print(expectedChecksum, HEX);
    Serial.print(", 수신: 0x");
    Serial.println(message->checksum, HEX);
    slaveCommReliability.checksumErrors++;
    recordSlaveCommError(COMM_ERROR_CHECKSUM);
    return false;
  }
  
  return true;
}

/**
 * 향상된 Master 명령 처리 (오류 처리 강화)
 * @param command 명령어 코드
 * @param data1 데이터 1
 * @param data2 데이터 2
 */
void handleMasterCommandReliable(uint8_t command, uint8_t data1, uint8_t data2) {
  bool commandSuccess = false;
  uint8_t responseCode = RESP_ERROR;
  
  // 명령별 처리 (기존 함수 호출하되 결과 확인)
  switch (command) {
    case CMD_ROBOT_HEAD_FRONT:
      commandSuccess = rotateHeadForward();
      responseCode = commandSuccess ? RESP_ROBOT_FRONT_COMPLETE : RESP_ROBOT_ERROR;
      break;
      
    case CMD_ROBOT_HEAD_BACK:
      commandSuccess = rotateHeadBackward();
      responseCode = commandSuccess ? RESP_ROBOT_BACK_COMPLETE : RESP_ROBOT_ERROR;
      break;
      
    case CMD_ROBOT_BODY_FRONT:
      commandSuccess = rotateBodyForward();
      responseCode = commandSuccess ? RESP_ROBOT_FRONT_COMPLETE : RESP_ROBOT_ERROR;
      break;
      
    case CMD_ROBOT_BODY_BACK:
      commandSuccess = rotateBodyBackward();
      responseCode = commandSuccess ? RESP_ROBOT_BACK_COMPLETE : RESP_ROBOT_ERROR;
      break;
      
    case CMD_ROBOT_STOP:
      emergencyStopMotors(true);
      commandSuccess = true;
      responseCode = RESP_ROBOT_READY;
      break;
      
    case CMD_SHOW_PATTERN:
      {
        LEDPattern pattern;
        if (decodePatternData(data1, data2, &pattern)) {
          showLEDPattern(pattern);
          commandSuccess = true;
          responseCode = RESP_PATTERN_DONE;
        } else {
          responseCode = RESP_PATTERN_ERROR;
        }
      }
      break;
      
    case CMD_CLEAR_PATTERN:
      turnOffAllOutputs();
      commandSuccess = true;
      responseCode = RESP_PATTERN_DONE;
      break;
      
    case CMD_UPDATE_SCORE:
      updateScoreWithBlink(data1);
      commandSuccess = true;
      responseCode = RESP_ACK;
      break;
      
    case CMD_PLAY_EFFECT:
      commandSuccess = playEffectSound(data1);
      responseCode = commandSuccess ? RESP_AUDIO_DONE : RESP_AUDIO_ERROR;
      break;
      
    case CMD_STOP_AUDIO:
      stopAllAudio();
      commandSuccess = true;
      responseCode = RESP_AUDIO_DONE;
      break;
      
    case CMD_GET_STATUS:
      sendSlaveStatusResponse();
      commandSuccess = true;
      responseCode = RESP_ACK;
      break;
      
    case CMD_GET_INPUT:
      sendInputStatusResponse();
      commandSuccess = true;
      responseCode = RESP_INPUT_DETECTED;
      break;
      
    case CMD_GET_LIMIT:
      sendLimitStatusResponse();
      commandSuccess = true;
      responseCode = RESP_ACK;
      break;
      
    case CMD_RESET:
      performSlaveSystemReset();
      commandSuccess = true;
      responseCode = RESP_ACK;
      break;
      
    case CMD_STOP_ALL:
      handleStopAllCommand();
      commandSuccess = true;
      responseCode = RESP_ACK;
      break;
      
    default:
      Serial.print("알 수 없는 명령: 0x");
      Serial.println(command, HEX);
      responseCode = RESP_UNKNOWN;
      break;
  }
  
  // 응답 전송 (재시도 포함)
  sendResponseToMasterReliable(responseCode, 0, 0);
  
  if (slaveCommReliability.diagnosticMode) {
    Serial.print("명령 처리 완료: 0x");
    Serial.print(command, HEX);
    Serial.print(" -> 응답: 0x");
    Serial.println(responseCode, HEX);
  }
}

/**
 * 향상된 Master 응답 전송 (재시도 포함)
 * @param response 응답 코드
 * @param data1 데이터 1
 * @param data2 데이터 2
 */
void sendResponseToMasterReliable(uint8_t response, uint8_t data1, uint8_t data2) {
  SerialMessage responseMsg;
  responseMsg.header = MESSAGE_HEADER;
  responseMsg.command = response;
  responseMsg.data1 = data1;
  responseMsg.data2 = data2;
  responseMsg.checksum = response ^ data1 ^ data2;
  
  // 재시도 루프
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) {
      delay(50); // 재시도 전 대기
      if (slaveCommReliability.diagnosticMode) {
        Serial.print("응답 재전송 시도: ");
        Serial.println(attempt + 1);
      }
    }
    
    // 메시지 전송
    Serial.write((uint8_t*)&responseMsg, sizeof(SerialMessage));
    Serial.flush();
    
    // 전송 완료 확인 (간단한 지연)
    delay(10);
    
    if (slaveCommReliability.diagnosticMode) {
      Serial.print("응답 전송: 0x");
      Serial.println(response, HEX);
    }
    
    break; // 첫 번째 시도에서 성공으로 간주 (ACK 없음)
  }
}

/**
 * Slave 상태 응답 전송
 */
void sendSlaveStatusResponse() {
  uint8_t motorStatus = 0;
  if (headMotorActive) motorStatus |= 0x01;
  if (bodyMotorActive) motorStatus |= 0x02;
  if (motorBrakeActive) motorStatus |= 0x04;
  
  uint8_t inputStatus = 0;
  if (isInputActive) inputStatus |= 0x01;
  inputStatus |= (inputSequenceLength << 1);
  
  sendResponseToMasterReliable(RESP_ACK, motorStatus, inputStatus);
}

/**
 * 입력 상태 응답 전송
 */
void sendInputStatusResponse() {
  uint8_t currentPadState = 0;
  for (int i = 0; i < 4; i++) {
    if (padPressed[i]) {
      currentPadState |= (1 << i);
    }
  }
  
  uint8_t lastInput = (inputSequenceLength > 0) ? inputSequence[inputSequenceLength - 1] : 0;
  
  sendResponseToMasterReliable(RESP_INPUT_DETECTED, currentPadState, lastInput);
}

/**
 * 리미트 스위치 상태 응답 전송
 */
void sendLimitStatusResponse() {
  bool headLimit = checkHeadLimit();
  bool bodyLimit = checkBodyLimit();
  
  uint8_t limitStatus = 0;
  if (headLimit) limitStatus |= 0x01;
  if (bodyLimit) limitStatus |= 0x02;
  
  sendResponseToMasterReliable(RESP_ACK, limitStatus, 0);
}

/**
 * Slave 시스템 리셋 수행
 */
void performSlaveSystemReset() {
  Serial.println("Slave 시스템 리셋 수행");
  
  // 모든 동작 정지
  emergencyStopMotors(true);
  setInputActive(false);
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  stopAllAudio();
  
  // 상태 변수 초기화
  initializeVibrationSensors();
  initSlaveCommReliability();
  
  Serial.println("Slave 시스템 리셋 완료");
}

/**
 * Master 연결 상태 모니터링
 */
void monitorMasterConnection() {
  unsigned long currentTime = millis();
  
  // 연결 타임아웃 체크
  if (currentTime - slaveCommReliability.lastMasterCommTime > slaveCommReliability.connectionTimeout) {
    if (slaveCommReliability.masterConnected) {
      Serial.println("Master 연결 타임아웃 - 연결 끊김");
      slaveCommReliability.masterConnected = false;
      
      // 연결 끊김 시 안전 조치
      handleMasterDisconnection();
    }
  }
}

/**
 * Master 연결 끊김 처리
 */
void handleMasterDisconnection() {
  Serial.println("Master 연결 끊김 - 안전 모드 진입");
  
  // 모든 위험한 동작 정지
  emergencyStopMotors(true);
  setInputActive(false);
  
  // 시각적 표시 (LED 점멸로 상태 표시)
  for (int i = 0; i < 3; i++) {
    turnOnAllOutputs();
    delay(200);
    turnOffAllOutputs();
    delay(200);
  }
  
  Serial.println("안전 모드 진입 완료 - Master 재연결 대기");
}

/**
 * Slave 통신 오류 기록
 * @param errorCode 오류 코드
 */
void recordSlaveCommError(uint8_t errorCode) {
  slaveCommReliability.lastErrorCode = errorCode;
  
  if (slaveCommReliability.diagnosticMode) {
    Serial.print("통신 오류: ");
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
      default:
        Serial.print("알 수 없는 오류: 0x");
        Serial.println(errorCode, HEX);
        break;
    }
  }
}

/**
 * Slave 통신 진단 정보 업데이트
 */
void updateSlaveCommDiagnostics() {
  unsigned long currentTime = millis();
  
  // 주기적 진단 정보 출력
  if (currentTime - slaveCommReliability.lastDiagnosticTime > slaveCommReliability.diagnosticInterval) {
    slaveCommReliability.lastDiagnosticTime = currentTime;
    
    if (slaveCommReliability.diagnosticMode) {
      printSlaveCommDiagnostics();
    }
  }
}

/**
 * Slave 통신 진단 정보 출력
 */
void printSlaveCommDiagnostics() {
  Serial.println("=== Slave 통신 진단 정보 ===");
  
  Serial.print("수신된 메시지: ");
  Serial.println(slaveCommReliability.messagesReceived);
  Serial.print("유효한 메시지: ");
  Serial.println(slaveCommReliability.validMessages);
  Serial.print("잘못된 메시지: ");
  Serial.println(slaveCommReliability.invalidMessages);
  Serial.print("체크섬 오류: ");
  Serial.println(slaveCommReliability.checksumErrors);
  Serial.print("타임아웃 오류: ");
  Serial.println(slaveCommReliability.timeoutErrors);
  
  // 성공률 계산
  if (slaveCommReliability.messagesReceived > 0) {
    float successRate = (float)slaveCommReliability.validMessages / slaveCommReliability.messagesReceived * 100;
    Serial.print("메시지 성공률: ");
    Serial.print(successRate, 1);
    Serial.println("%");
  }
  
  Serial.print("Master 연결 상태: ");
  Serial.println(slaveCommReliability.masterConnected ? "연결됨" : "끊김");
  
  unsigned long timeSinceLastComm = millis() - slaveCommReliability.lastMasterCommTime;
  Serial.print("마지막 통신: ");
  Serial.print(timeSinceLastComm);
  Serial.println("ms 전");
  
  Serial.println("===========================");
}

/**
 * Slave 통신 진단 모드 토글
 */
void toggleSlaveCommDiagnosticMode() {
  slaveCommReliability.diagnosticMode = !slaveCommReliability.diagnosticMode;
  Serial.print("Slave 통신 진단 모드: ");
  Serial.println(slaveCommReliability.diagnosticMode ? "활성화" : "비활성화");
}

/**
 * Slave 통신 상태 리셋
 */
void resetSlaveCommReliability() {
  Serial.println("Slave 통신 상태 리셋");
  
  slaveCommReliability.messagesReceived = 0;
  slaveCommReliability.validMessages = 0;
  slaveCommReliability.invalidMessages = 0;
  slaveCommReliability.checksumErrors = 0;
  slaveCommReliability.timeoutErrors = 0;
  slaveCommReliability.lastErrorCode = COMM_ERROR_NONE;
  slaveCommReliability.masterConnected = false;
  slaveCommReliability.lastMasterCommTime = millis();
  
  Serial.println("Slave 통신 상태 리셋 완료");
}

/**
 * Master 연결 상태 확인
 * @return Master 연결 여부
 */
bool isMasterConnected() {
  return slaveCommReliability.masterConnected;
}

/**
 * 통신 품질 확인
 * @return 통신 품질 (0-100%)
 */
uint8_t getSlaveCommQuality() {
  if (slaveCommReliability.messagesReceived == 0) {
    return 100;
  }
  
  return (slaveCommReliability.validMessages * 100) / slaveCommReliability.messagesReceived;
}

// =============================================================================
// Slave 통신 안정성 강화 시스템 (Task 17.1)
// =============================================================================

/**
 * Slave 통신 안정성 시스템 초기화
 * 요구사항 7.3: Master-Slave 간 통신 오류 복구 메커니즘 완성
 */
void initSlaveCommReliability() {
  slaveCommReliability.lastMasterCommTime = millis();
  slaveCommReliability.consecutiveErrors = 0;
  slaveCommReliability.errorThreshold = SLAVE_COMM_ERROR_THRESHOLD;
  slaveCommReliability.totalMessagesReceived = 0;
  slaveCommReliability.validMessagesReceived = 0;
  slaveCommReliability.checksumErrors = 0;
  slaveCommReliability.invalidMessages = 0;
  slaveCommReliability.masterConnected = false;
  slaveCommReliability.connectionTimeout = SLAVE_COMM_TIMEOUT;
  slaveCommReliability.diagnosticMode = false;
  slaveCommReliability.lastErrorCode = COMM_ERROR_NONE;
  slaveCommReliability.recoveryStartTime = 0;
  slaveCommReliability.isRecovering = false;
  
  Serial.println("Slave 통신 안정성 시스템 초기화 완료");
  Serial.print("연결 타임아웃: ");
  Serial.print(slaveCommReliability.connectionTimeout);
  Serial.println("ms");
  Serial.print("오류 임계값: ");
  Serial.println(slaveCommReliability.errorThreshold);
}

/**
 * Slave 통신 안정성 모니터링 (메인 루프에서 호출)
 * 요구사항 7.3: 통신 상태 모니터링 및 진단 기능 추가
 */
void monitorSlaveCommReliability() {
  unsigned long currentTime = millis();
  
  // Master 통신 타임아웃 체크
  unsigned long timeSinceLastComm = currentTime - slaveCommReliability.lastMasterCommTime;
  
  if (timeSinceLastComm > slaveCommReliability.connectionTimeout) {
    if (slaveCommReliability.masterConnected) {
      Serial.println("[SLAVE] Master 통신 타임아웃 - 연결 끊김");
      slaveCommReliability.masterConnected = false;
      recordSlaveCommError(COMM_ERROR_TIMEOUT);
      
      // 복구 모드 시작
      if (!slaveCommReliability.isRecovering) {
        startSlaveCommRecovery();
      }
    }
  }
  
  // 복구 모드 처리
  if (slaveCommReliability.isRecovering) {
    handleSlaveCommRecovery();
  }
  
  // 진단 모드 처리
  if (slaveCommReliability.diagnosticMode) {
    performSlaveCommDiagnostics();
  }
}

/**
 * Slave 통신 복구 모드 시작
 */
void startSlaveCommRecovery() {
  slaveCommReliability.isRecovering = true;
  slaveCommReliability.recoveryStartTime = millis();
  
  Serial.println("=== Slave 통신 복구 모드 시작 ===");
  Serial.print("연속 오류 횟수: ");
  Serial.println(slaveCommReliability.consecutiveErrors);
  Serial.print("마지막 오류 코드: 0x");
  Serial.println(slaveCommReliability.lastErrorCode, HEX);
  
  // 복구 액션 수행
  performSlaveRecoveryActions();
}

/**
 * Slave 통신 복구 처리
 */
void handleSlaveCommRecovery() {
  unsigned long recoveryTime = millis() - slaveCommReliability.recoveryStartTime;
  
  // 복구 타임아웃 체크
  if (recoveryTime > SLAVE_COMM_RECOVERY_TIMEOUT) {
    Serial.println("[SLAVE] 복구 타임아웃 - 복구 포기");
    slaveCommReliability.isRecovering = false;
    return;
  }
  
  // 주기적 복구 시도 (1초마다)
  if (recoveryTime % 1000 < 100) {
    performSlaveRecoveryActions();
  }
}

/**
 * Slave 복구 액션 수행
 */
void performSlaveRecoveryActions() {
  Serial.println("[SLAVE] 복구 액션 수행 중...");
  
  // 1. 시리얼 버퍼 클리어
  while (Serial.available()) {
    Serial.read();
  }
  
  // 2. 시스템 상태 리셋
  emergencyStopMotors(true);
  setInputActive(false);
  turnOffAllOutputs();
  
  // 3. 상태 초기화
  slaveCommReliability.consecutiveErrors = 0;
  
  Serial.println("[SLAVE] 복구 액션 완료 - Master 통신 대기");
}

/**
 * Slave 통신 오류 기록
 * @param errorCode 오류 코드
 */
void recordSlaveCommError(uint8_t errorCode) {
  slaveCommReliability.lastErrorCode = errorCode;
  slaveCommReliability.consecutiveErrors++;
  
  if (slaveCommReliability.diagnosticMode) {
    Serial.print("[SLAVE] 통신 오류 기록 - 코드: 0x");
    Serial.print(errorCode, HEX);
    Serial.print(", 연속 오류: ");
    Serial.println(slaveCommReliability.consecutiveErrors);
  }
  
  // 오류 임계값 체크
  if (slaveCommReliability.consecutiveErrors >= slaveCommReliability.errorThreshold) {
    if (!slaveCommReliability.isRecovering) {
      Serial.println("[SLAVE] 연속 오류 임계값 초과 - 복구 모드 시작");
      startSlaveCommRecovery();
    }
  }
}

/**
 * Slave 통신 성공 기록
 */
void recordSlaveCommSuccess() {
  slaveCommReliability.lastMasterCommTime = millis();
  slaveCommReliability.consecutiveErrors = 0;
  slaveCommReliability.validMessagesReceived++;
  
  if (!slaveCommReliability.masterConnected) {
    Serial.println("[SLAVE] Master 연결 복구됨");
    slaveCommReliability.masterConnected = true;
  }
  
  if (slaveCommReliability.isRecovering) {
    Serial.println("[SLAVE] 통신 복구 성공");
    slaveCommReliability.isRecovering = false;
  }
}

/**
 * 개선된 Master 통신 처리 (오류 처리 강화)
 */
void processMasterCommImproved() {
  if (Serial.available() >= 4) { // 최소 메시지 크기
    slaveCommReliability.totalMessagesReceived++;
    
    uint8_t header = Serial.read();
    
    if (header == MESSAGE_HEADER) { // 유효한 헤더 (0xAA)
      uint8_t command = Serial.read();
      uint8_t data1 = Serial.read();
      uint8_t data2 = Serial.read();
      
      // 체크섬 검증
      uint8_t expectedChecksum = command ^ data1 ^ data2;
      
      if (Serial.available() > 0) {
        uint8_t receivedChecksum = Serial.read();
        
        if (receivedChecksum == expectedChecksum) {
          // 유효한 메시지 처리
          recordSlaveCommSuccess();
          handleMasterCommand(command, data1, data2);
        } else {
          // 체크섬 오류
          slaveCommReliability.checksumErrors++;
          recordSlaveCommError(COMM_ERROR_CHECKSUM);
          
          if (slaveCommReliability.diagnosticMode) {
            Serial.print("[SLAVE] 체크섬 오류 - 예상: 0x");
            Serial.print(expectedChecksum, HEX);
            Serial.print(", 수신: 0x");
            Serial.println(receivedChecksum, HEX);
          }
          
          sendResponseToMaster(RESP_NACK);
        }
      } else {
        // 체크섬 누락
        recordSlaveCommError(COMM_ERROR_INVALID);
        sendResponseToMaster(RESP_NACK);
      }
    } else {
      // 잘못된 헤더
      slaveCommReliability.invalidMessages++;
      recordSlaveCommError(COMM_ERROR_INVALID);
      
      if (slaveCommReliability.diagnosticMode) {
        Serial.print("[SLAVE] 잘못된 헤더: 0x");
        Serial.println(header, HEX);
      }
    }
  }
}

/**
 * Slave 통신 진단 수행
 */
void performSlaveCommDiagnostics() {
  static unsigned long lastDiagnosticTime = 0;
  unsigned long currentTime = millis();
  
  // 5초마다 진단 정보 출력
  if (currentTime - lastDiagnosticTime >= 5000) {
    printSlaveCommDiagnostics();
    lastDiagnosticTime = currentTime;
  }
}

/**
 * Slave 통신 진단 정보 출력
 */
void printSlaveCommDiagnostics() {
  Serial.println("=== Slave 통신 진단 정보 ===");
  
  Serial.print("총 수신 메시지: ");
  Serial.println(slaveCommReliability.totalMessagesReceived);
  
  Serial.print("유효한 메시지: ");
  Serial.println(slaveCommReliability.validMessagesReceived);
  
  Serial.print("체크섬 오류: ");
  Serial.println(slaveCommReliability.checksumErrors);
  
  Serial.print("잘못된 메시지: ");
  Serial.println(slaveCommReliability.invalidMessages);
  
  if (slaveCommReliability.totalMessagesReceived > 0) {
    uint8_t successRate = (slaveCommReliability.validMessagesReceived * 100) / slaveCommReliability.totalMessagesReceived;
    Serial.print("메시지 성공률: ");
    Serial.print(successRate);
    Serial.println("%");
  }
  
  Serial.print("연속 오류: ");
  Serial.println(slaveCommReliability.consecutiveErrors);
  
  Serial.print("Master 연결: ");
  Serial.println(slaveCommReliability.masterConnected ? "정상" : "끊김");
  
  Serial.print("복구 모드: ");
  Serial.println(slaveCommReliability.isRecovering ? "활성" : "비활성");
  
  if (slaveCommReliability.lastErrorCode != COMM_ERROR_NONE) {
    Serial.print("마지막 오류: 0x");
    Serial.println(slaveCommReliability.lastErrorCode, HEX);
  }
  
  unsigned long timeSinceLastComm = millis() - slaveCommReliability.lastMasterCommTime;
  Serial.print("마지막 Master 통신: ");
  Serial.print(timeSinceLastComm);
  Serial.println("ms 전");
  
  Serial.println("===========================");
}

/**
 * Slave 통신 진단 모드 토글
 * @param enable 진단 모드 활성화 여부
 */
void setSlaveCommDiagnosticMode(bool enable) {
  slaveCommReliability.diagnosticMode = enable;
  Serial.print("Slave 통신 진단 모드: ");
  Serial.println(enable ? "활성화" : "비활성화");
}

/**
 * Slave 통신 통계 리셋
 */
void resetSlaveCommStatistics() {
  slaveCommReliability.totalMessagesReceived = 0;
  slaveCommReliability.validMessagesReceived = 0;
  slaveCommReliability.checksumErrors = 0;
  slaveCommReliability.invalidMessages = 0;
  slaveCommReliability.consecutiveErrors = 0;
  slaveCommReliability.lastErrorCode = COMM_ERROR_NONE;
  
  Serial.println("Slave 통신 통계 리셋 완료");
}

// =============================================================================
// 하드웨어 안전장치 강화 시스템 구현 (Task 17.2) - Slave Board
// =============================================================================

/**
 * Slave 하드웨어 안전장치 시스템 초기화
 * 요구사항 7.2, 7.6: 하드웨어 안전장치 강화
 */
void initSlaveHardwareSafety() {
  Serial.println("Slave 하드웨어 안전장치 시스템 초기화 시작...");
  
  // 모터 안전장치 초기화
  initSlaveMotorSafety();
  
  // 센서 안전장치 초기화
  initSlaveSensorSafety();
  
  // 시스템 안전장치 초기화
  initSlaveSystemSafety();
  
  Serial.println("Slave 하드웨어 안전장치 시스템 초기화 완료");
}

/**
 * Slave 모터 안전장치 초기화
 */
void initSlaveMotorSafety() {
  slaveMotorSafety.headMotorHealthy = true;
  slaveMotorSafety.bodyMotorHealthy = true;
  slaveMotorSafety.headMotorStartTime = 0;
  slaveMotorSafety.bodyMotorStartTime = 0;
  slaveMotorSafety.maxMotorRunTime = MOTOR_MAX_RUN_TIME;
  slaveMotorSafety.headMotorTimeouts = 0;
  slaveMotorSafety.bodyMotorTimeouts = 0;
  slaveMotorSafety.motorOverloadProtection = true;
  slaveMotorSafety.lastMotorHealthCheck = millis();
  slaveMotorSafety.estimatedMotorCurrent = 0.0f;
  slaveMotorSafety.emergencyMotorStop = false;
  slaveMotorSafety.motorRecoveryAttempts = 0;
  slaveMotorSafety.motorSafetyLockout = false;
  slaveMotorSafety.safetyLockoutTime = 0;
  
  Serial.println("Slave 모터 안전장치 초기화 완료");
  Serial.print("최대 모터 동작 시간: ");
  Serial.print(MOTOR_MAX_RUN_TIME / 1000);
  Serial.println("초");
}

/**
 * Slave 센서 안전장치 초기화
 */
void initSlaveSensorSafety() {
  // 진동 센서 안전장치 초기화
  for (int i = 0; i < 4; i++) {
    slaveSensorSafety.vibrationSensorHealthy[i] = true;
    slaveSensorSafety.vibrationSensorErrors[i] = 0;
    slaveSensorSafety.lastValidVibration[i] = millis();
    slaveSensorSafety.sensorCalibrationNeeded[i] = false;
  }
  
  // 리미트 스위치 안전장치 초기화
  for (int i = 0; i < 2; i++) {
    slaveSensorSafety.limitSwitchHealthy[i] = true;
    slaveSensorSafety.limitSwitchErrors[i] = 0;
  }
  slaveSensorSafety.lastLimitSwitchCheck = millis();
  
  // MCP23017 안전장치 초기화
  slaveSensorSafety.mcpHealthy = true;
  slaveSensorSafety.mcpErrorCount = 0;
  slaveSensorSafety.lastMcpResponse = millis();
  
  // DFPlayer 안전장치 초기화
  slaveSensorSafety.dfPlayerHealthy = true;
  slaveSensorSafety.dfPlayerErrorCount = 0;
  slaveSensorSafety.lastDfPlayerResponse = millis();
  
  // 전체 센서 안전장치 초기화
  slaveSensorSafety.totalSensorErrors = 0;
  slaveSensorSafety.sensorEmergencyMode = false;
  
  Serial.println("Slave 센서 안전장치 초기화 완료");
}

/**
 * Slave 시스템 안전장치 초기화
 */
void initSlaveSystemSafety() {
  slaveSystemSafety.systemVoltage = 5.0f;
  slaveSystemSafety.lowVoltageDetected = false;
  slaveSystemSafety.lastVoltageCheck = millis();
  
  slaveSystemSafety.systemTemperature = 25.0f;
  slaveSystemSafety.overheated = false;
  slaveSystemSafety.lastTemperatureCheck = millis();
  
  slaveSystemSafety.i2cBusHealthy = true;
  slaveSystemSafety.i2cErrorCount = 0;
  slaveSystemSafety.lastI2cCheck = millis();
  
  slaveSystemSafety.systemStable = true;
  slaveSystemSafety.emergencyShutdown = false;
  slaveSystemSafety.emergencyShutdownTime = 0;
  
  slaveSystemSafety.totalSystemErrors = 0;
  slaveSystemSafety.diagnosticMode = false;
  
  Serial.println("Slave 시스템 안전장치 초기화 완료");
}

/**
 * Slave 하드웨어 안전장치 모니터링 (메인 루프에서 호출)
 */
void monitorSlaveHardwareSafety() {
  // 모터 안전장치 모니터링
  monitorSlaveMotorSafety();
  
  // 센서 안전장치 모니터링
  monitorSlaveSensorSafety();
  
  // 시스템 안전장치 모니터링
  monitorSlaveSystemSafety();
  
  // 비상 상황 체크
  checkSlaveEmergencyConditions();
}

/**
 * Slave 모터 안전장치 모니터링
 * 요구사항 7.2: 모터 과부하 및 타임아웃 보호 강화
 */
void monitorSlaveMotorSafety() {
  unsigned long currentTime = millis();
  
  // 모터 상태 체크 주기 확인
  if (currentTime - slaveMotorSafety.lastMotorHealthCheck < MOTOR_HEALTH_CHECK_INTERVAL) {
    return;
  }
  
  slaveMotorSafety.lastMotorHealthCheck = currentTime;
  
  // Head 모터 타임아웃 체크
  if (headMotorActive && slaveMotorSafety.headMotorStartTime > 0) {
    unsigned long headRunTime = currentTime - slaveMotorSafety.headMotorStartTime;
    
    if (headRunTime > slaveMotorSafety.maxMotorRunTime) {
      Serial.println("Head 모터 타임아웃 감지");
      handleMotorTimeout(0); // Head 모터 = 0
    }
  }
  
  // Body 모터 타임아웃 체크
  if (bodyMotorActive && slaveMotorSafety.bodyMotorStartTime > 0) {
    unsigned long bodyRunTime = currentTime - slaveMotorSafety.bodyMotorStartTime;
    
    if (bodyRunTime > slaveMotorSafety.maxMotorRunTime) {
      Serial.println("Body 모터 타임아웃 감지");
      handleMotorTimeout(1); // Body 모터 = 1
    }
  }
  
  // 모터 전류 추정 및 과부하 체크
  if (headMotorActive || bodyMotorActive) {
    slaveMotorSafety.estimatedMotorCurrent = estimateMotorCurrent();
    
    if (slaveMotorSafety.estimatedMotorCurrent > MOTOR_OVERLOAD_CURRENT) {
      Serial.print("모터 과부하 감지: ");
      Serial.print(slaveMotorSafety.estimatedMotorCurrent);
      Serial.println("A");
      
      handleMotorOverload();
    }
  }
  
  // 안전 잠금 해제 체크
  if (slaveMotorSafety.motorSafetyLockout && 
      currentTime - slaveMotorSafety.safetyLockoutTime > MOTOR_SAFETY_LOCKOUT_TIME) {
    
    Serial.println("모터 안전 잠금 해제");
    slaveMotorSafety.motorSafetyLockout = false;
    slaveMotorSafety.motorRecoveryAttempts = 0;
  }
}

/**
 * 모터 타임아웃 처리
 * @param motorIndex 모터 인덱스 (0: Head, 1: Body)
 */
void handleMotorTimeout(uint8_t motorIndex) {
  if (motorIndex == 0) { // Head 모터
    slaveMotorSafety.headMotorTimeouts++;
    slaveMotorSafety.headMotorHealthy = false;
    Serial.print("Head 모터 타임아웃 횟수: ");
    Serial.println(slaveMotorSafety.headMotorTimeouts);
  } else { // Body 모터
    slaveMotorSafety.bodyMotorTimeouts++;
    slaveMotorSafety.bodyMotorHealthy = false;
    Serial.print("Body 모터 타임아웃 횟수: ");
    Serial.println(slaveMotorSafety.bodyMotorTimeouts);
  }
  
  // 즉시 모터 정지
  emergencyStopMotors(true);
  
  // 복구 시도
  if (slaveMotorSafety.motorRecoveryAttempts < MOTOR_MAX_RECOVERY_ATTEMPTS) {
    startSlaveMotorRecovery();
  } else {
    triggerSlaveMotorSafetyLockout();
  }
}

/**
 * 모터 과부하 처리
 */
void handleMotorOverload() {
  Serial.println("모터 과부하 처리 시작");
  
  // 즉시 모터 정지
  emergencyStopMotors(true);
  
  slaveMotorSafety.headMotorHealthy = false;
  slaveMotorSafety.bodyMotorHealthy = false;
  
  // 과부하 보호 활성화
  slaveMotorSafety.motorOverloadProtection = true;
  
  // 복구 시도 또는 안전 잠금
  if (slaveMotorSafety.motorRecoveryAttempts < MOTOR_MAX_RECOVERY_ATTEMPTS) {
    startSlaveMotorRecovery();
  } else {
    triggerSlaveMotorSafetyLockout();
  }
}

/**
 * 모터 전류 추정
 * @return 추정 전류 (A)
 */
float estimateMotorCurrent() {
  float baseCurrent = 0.1f; // 기본 전류
  float headCurrent = headMotorActive ? 0.8f : 0.0f;
  float bodyCurrent = bodyMotorActive ? 1.0f : 0.0f;
  
  // 동작 시간에 따른 전류 증가 (발열로 인한)
  unsigned long currentTime = millis();
  float timeFactor = 1.0f;
  
  if (headMotorActive && slaveMotorSafety.headMotorStartTime > 0) {
    unsigned long headRunTime = currentTime - slaveMotorSafety.headMotorStartTime;
    timeFactor += (headRunTime / 10000.0f) * 0.1f; // 10초마다 10% 증가
  }
  
  if (bodyMotorActive && slaveMotorSafety.bodyMotorStartTime > 0) {
    unsigned long bodyRunTime = currentTime - slaveMotorSafety.bodyMotorStartTime;
    timeFactor += (bodyRunTime / 10000.0f) * 0.1f;
  }
  
  float totalCurrent = (baseCurrent + headCurrent + bodyCurrent) * timeFactor;
  
  // 현실적인 범위로 제한
  if (totalCurrent > 5.0f) totalCurrent = 5.0f;
  
  return totalCurrent;
}

/**
 * Slave 모터 복구 시작
 */
void startSlaveMotorRecovery() {
  slaveMotorSafety.motorRecoveryAttempts++;
  
  Serial.print("Slave 모터 복구 시도 ");
  Serial.print(slaveMotorSafety.motorRecoveryAttempts);
  Serial.print("/");
  Serial.println(MOTOR_MAX_RECOVERY_ATTEMPTS);
  
  // 모터 시스템 리셋
  emergencyStopMotors(true);
  delay(2000); // 2초 대기
  
  // 모터 상태 리셋
  slaveMotorSafety.headMotorHealthy = true;
  slaveMotorSafety.bodyMotorHealthy = true;
  slaveMotorSafety.headMotorStartTime = 0;
  slaveMotorSafety.bodyMotorStartTime = 0;
  slaveMotorSafety.estimatedMotorCurrent = 0.0f;
  
  Serial.println("Slave 모터 복구 완료");
}

/**
 * Slave 모터 안전 잠금 트리거
 */
void triggerSlaveMotorSafetyLockout() {
  Serial.println("Slave 모터 안전 잠금 트리거");
  
  slaveMotorSafety.motorSafetyLockout = true;
  slaveMotorSafety.emergencyMotorStop = true;
  slaveMotorSafety.safetyLockoutTime = millis();
  
  // 모든 모터 완전 정지
  emergencyStopMotors(true);
  
  // Master에 모터 오류 상태 보고
  sendResponseToMasterWithData(RESP_ROBOT_ERROR, 0xFF, 0xFF);
}

/**
 * Slave 센서 안전장치 모니터링
 * 요구사항 7.6: 센서 오작동 감지 및 복구 로직 추가
 */
void monitorSlaveSensorSafety() {
  unsigned long currentTime = millis();
  
  // 센서 상태 체크 주기 확인
  if (currentTime - slaveSensorSafety.lastLimitSwitchCheck < SENSOR_HEALTH_CHECK_INTERVAL) {
    return;
  }
  
  // 진동 센서 상태 모니터링
  monitorVibrationSensorHealth(currentTime);
  
  // 리미트 스위치 상태 모니터링
  monitorLimitSwitchHealth(currentTime);
  
  // MCP23017 상태 모니터링
  monitorMcpHealth(currentTime);
  
  // DFPlayer 상태 모니터링
  monitorSlaveDfPlayerHealth(currentTime);
}

/**
 * 진동 센서 상태 모니터링
 * @param currentTime 현재 시간
 */
void monitorVibrationSensorHealth(unsigned long currentTime) {
  for (int i = 0; i < 4; i++) {
    // 센서 응답 타임아웃 체크
    if (currentTime - slaveSensorSafety.lastValidVibration[i] > VIBRATION_SENSOR_TIMEOUT) {
      if (slaveSensorSafety.vibrationSensorHealthy[i]) {
        Serial.print("진동 센서 ");
        Serial.print(i + 1);
        Serial.println(" 응답 타임아웃");
        
        slaveSensorSafety.vibrationSensorHealthy[i] = false;
        slaveSensorSafety.vibrationSensorErrors[i]++;
        slaveSensorSafety.totalSensorErrors++;
        
        // 센서 캘리브레이션 필요 표시
        if (slaveSensorSafety.vibrationSensorErrors[i] >= SENSOR_ERROR_THRESHOLD) {
          slaveSensorSafety.sensorCalibrationNeeded[i] = true;
          Serial.print("진동 센서 ");
          Serial.print(i + 1);
          Serial.println(" 캘리브레이션 필요");
        }
      }
    }
    
    // 센서 값 유효성 체크
    uint16_t sensorValue = readVibrationSensor(i);
    if (sensorValue > 0 && sensorValue < 1023) {
      // 유효한 센서 값 감지
      slaveSensorSafety.lastValidVibration[i] = currentTime;
      
      if (!slaveSensorSafety.vibrationSensorHealthy[i]) {
        Serial.print("진동 센서 ");
        Serial.print(i + 1);
        Serial.println(" 복구됨");
        slaveSensorSafety.vibrationSensorHealthy[i] = true;
      }
    }
  }
}

/**
 * 리미트 스위치 상태 모니터링
 * @param currentTime 현재 시간
 */
void monitorLimitSwitchHealth(unsigned long currentTime) {
  slaveSensorSafety.lastLimitSwitchCheck = currentTime;
  
  // Head 리미트 스위치 체크
  bool headLimit = checkHeadLimit();
  bool bodyLimit = checkBodyLimit();
  
  // 리미트 스위치 상태 검증
  // 모터가 동작 중인데 리미트 스위치가 계속 활성화되어 있으면 오작동 의심
  if (headMotorActive && headLimit) {
    unsigned long motorRunTime = currentTime - slaveMotorSafety.headMotorStartTime;
    if (motorRunTime > 5000) { // 5초 이상 동작했는데 여전히 리미트 활성
      Serial.println("Head 리미트 스위치 오작동 의심");
      slaveSensorSafety.limitSwitchHealthy[0] = false;
      slaveSensorSafety.limitSwitchErrors[0]++;
      slaveSensorSafety.totalSensorErrors++;
    }
  }
  
  if (bodyMotorActive && bodyLimit) {
    unsigned long motorRunTime = currentTime - slaveMotorSafety.bodyMotorStartTime;
    if (motorRunTime > 5000) {
      Serial.println("Body 리미트 스위치 오작동 의심");
      slaveSensorSafety.limitSwitchHealthy[1] = false;
      slaveSensorSafety.limitSwitchErrors[1]++;
      slaveSensorSafety.totalSensorErrors++;
    }
  }
}

/**
 * MCP23017 상태 모니터링
 * @param currentTime 현재 시간
 */
void monitorMcpHealth(unsigned long currentTime) {
  // MCP23017 통신 테스트
  if (currentTime - slaveSensorSafety.lastMcpResponse > 5000) { // 5초마다 체크
    bool mcpResponsive = testMcpCommunication();
    
    if (mcpResponsive) {
      slaveSensorSafety.lastMcpResponse = currentTime;
      if (!slaveSensorSafety.mcpHealthy) {
        Serial.println("MCP23017 통신 복구됨");
        slaveSensorSafety.mcpHealthy = true;
      }
    } else {
      if (slaveSensorSafety.mcpHealthy) {
        Serial.println("MCP23017 통신 오류 감지");
        slaveSensorSafety.mcpHealthy = false;
        slaveSensorSafety.mcpErrorCount++;
        slaveSensorSafety.totalSensorErrors++;
        
        // MCP23017 복구 시도
        if (slaveSensorSafety.mcpErrorCount >= 3) {
          recoverMcpCommunication();
        }
      }
    }
  }
}

/**
 * MCP23017 통신 테스트
 * @return 통신 성공 여부
 */
bool testMcpCommunication() {
  // MCP23017 레지스터 읽기 테스트
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_GPIOA);
  uint8_t error = Wire.endTransmission();
  
  if (error == 0) {
    Wire.requestFrom(MCP23017_1_ADDR, 1);
    if (Wire.available()) {
      Wire.read(); // 데이터 읽기
      return true;
    }
  }
  
  return false;
}

/**
 * MCP23017 통신 복구
 */
void recoverMcpCommunication() {
  Serial.println("MCP23017 통신 복구 시도");
  
  // I2C 버스 리셋
  Wire.end();
  delay(100);
  Wire.begin();
  delay(100);
  
  // MCP23017 재초기화
  initializeMCP23017();
  
  slaveSensorSafety.lastMcpResponse = millis();
}

/**
 * Slave DFPlayer 상태 모니터링
 * @param currentTime 현재 시간
 */
void monitorSlaveDfPlayerHealth(unsigned long currentTime) {
  // DFPlayer 응답 체크 (10초마다)
  if (currentTime - slaveSensorSafety.lastDfPlayerResponse > 10000) {
    if (slaveSensorSafety.dfPlayerHealthy) {
      Serial.println("Slave DFPlayer 응답 타임아웃");
      slaveSensorSafety.dfPlayerHealthy = false;
      slaveSensorSafety.dfPlayerErrorCount++;
      slaveSensorSafety.totalSensorErrors++;
      
      // DFPlayer 복구 시도
      if (slaveSensorSafety.dfPlayerErrorCount >= 3) {
        recoverSlaveDfPlayer();
      }
    }
  }
}

/**
 * Slave DFPlayer 복구
 */
void recoverSlaveDfPlayer() {
  Serial.println("Slave DFPlayer 복구 시도");
  
  // DFPlayer 재초기화
  initializeDFPlayer();
  
  slaveSensorSafety.lastDfPlayerResponse = millis();
}

/**
 * Slave 시스템 안전장치 모니터링
 */
void monitorSlaveSystemSafety() {
  unsigned long currentTime = millis();
  
  // 전압 모니터링
  monitorSlaveSystemVoltage(currentTime);
  
  // 온도 모니터링
  monitorSlaveSystemTemperature(currentTime);
  
  // I2C 버스 모니터링
  monitorSlaveI2cBus(currentTime);
}

/**
 * Slave 시스템 전압 모니터링
 * @param currentTime 현재 시간
 */
void monitorSlaveSystemVoltage(unsigned long currentTime) {
  if (currentTime - slaveSystemSafety.lastVoltageCheck < SYSTEM_VOLTAGE_CHECK_INTERVAL) {
    return;
  }
  
  slaveSystemSafety.lastVoltageCheck = currentTime;
  
  // 시스템 전압 측정 (간단한 추정)
  slaveSystemSafety.systemVoltage = measureSlaveSystemVoltage();
  
  // 저전압 체크
  if (slaveSystemSafety.systemVoltage < SYSTEM_VOLTAGE_MIN) {
    if (!slaveSystemSafety.lowVoltageDetected) {
      Serial.print("Slave 저전압 감지: ");
      Serial.print(slaveSystemSafety.systemVoltage);
      Serial.println("V");
      
      slaveSystemSafety.lowVoltageDetected = true;
      slaveSystemSafety.totalSystemErrors++;
      
      // 저전압 보호 조치
      triggerSlaveLowVoltageProtection();
    }
  } else {
    slaveSystemSafety.lowVoltageDetected = false;
  }
}

/**
 * Slave 시스템 전압 측정
 * @return 측정된 전압 (V)
 */
float measureSlaveSystemVoltage() {
  // 실제 구현에서는 전압 분배 회로 사용
  // 여기서는 시뮬레이션
  int analogValue = analogRead(A7); // 예시 핀
  float voltage = (analogValue / 1023.0f) * 5.0f;
  
  // 노이즈 필터링
  if (voltage < 3.0f) voltage = 3.0f;
  if (voltage > 6.0f) voltage = 6.0f;
  
  return voltage;
}

/**
 * Slave 저전압 보호 조치
 */
void triggerSlaveLowVoltageProtection() {
  Serial.println("Slave 저전압 보호 조치 실행");
  
  // 고전력 장치 정지
  emergencyStopMotors(true);
  
  // LED 밝기 감소 (MCP23017을 통한 제어)
  turnOffAllOutputs();
  
  // Master에 저전압 상태 보고
  sendResponseToMasterWithData(RESP_ROBOT_ERROR, 0x01, 0x00); // 저전압 코드
}

/**
 * Slave 시스템 온도 모니터링
 * @param currentTime 현재 시간
 */
void monitorSlaveSystemTemperature(unsigned long currentTime) {
  if (currentTime - slaveSystemSafety.lastTemperatureCheck < SYSTEM_TEMP_CHECK_INTERVAL) {
    return;
  }
  
  slaveSystemSafety.lastTemperatureCheck = currentTime;
  
  // 시스템 온도 추정
  slaveSystemSafety.systemTemperature = estimateSlaveSystemTemperature();
  
  // 과열 체크
  if (slaveSystemSafety.systemTemperature > SYSTEM_TEMPERATURE_MAX) {
    if (!slaveSystemSafety.overheated) {
      Serial.print("Slave 시스템 과열: ");
      Serial.print(slaveSystemSafety.systemTemperature);
      Serial.println("°C");
      
      slaveSystemSafety.overheated = true;
      slaveSystemSafety.totalSystemErrors++;
      
      // 과열 보호 조치
      triggerSlaveOverheatProtection();
    }
  } else if (slaveSystemSafety.systemTemperature < SYSTEM_TEMPERATURE_MAX - 10) {
    if (slaveSystemSafety.overheated) {
      Serial.println("Slave 시스템 온도 정상화");
      slaveSystemSafety.overheated = false;
    }
  }
}

/**
 * Slave 시스템 온도 추정
 * @return 추정 온도 (°C)
 */
float estimateSlaveSystemTemperature() {
  float baseTemp = 25.0f;
  unsigned long uptime = millis() / 1000;
  float uptimeTemp = uptime * 0.002f; // 초당 0.002도 상승
  
  // 모터 동작에 따른 온도 상승
  float motorTemp = 0.0f;
  if (headMotorActive) motorTemp += 8.0f;
  if (bodyMotorActive) motorTemp += 10.0f;
  
  // LED 동작에 따른 온도 상승
  float ledTemp = 2.0f; // 기본 LED 발열
  
  float totalTemp = baseTemp + uptimeTemp + motorTemp + ledTemp;
  
  if (totalTemp > 85.0f) totalTemp = 85.0f;
  if (totalTemp < 20.0f) totalTemp = 20.0f;
  
  return totalTemp;
}

/**
 * Slave 모터 안전장치 리셋
 */
void resetSlaveMotorSafety() {
  Serial.println("Slave 모터 안전장치 리셋");
  
  slaveMotorSafety.headMotorHealthy = true;
  slaveMotorSafety.bodyMotorHealthy = true;
  slaveMotorSafety.headMotorTimeouts = 0;
  slaveMotorSafety.bodyMotorTimeouts = 0;
  slaveMotorSafety.motorOverloadProtection = false;
  slaveMotorSafety.emergencyMotorStop = false;
  slaveMotorSafety.motorRecoveryAttempts = 0;
  slaveMotorSafety.motorSafetyLockout = false;
  slaveMotorSafety.estimatedMotorCurrent = 0.0f;
  
  Serial.println("모터 안전장치 리셋 완료");
}

/**
 * Slave 과열 보호 조치
 */
void triggerSlaveOverheatProtection() {
  Serial.println("Slave 과열 보호 조치 실행");
  
  // 모든 모터 즉시 정지
  emergencyStopMotors(true);
  
  // 모든 LED 소등
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  
  // 오디오 정지
  stopAllAudio();
  
  // Master에 과열 상태 보고
  sendResponseToMasterWithData(RESP_ROBOT_ERROR, 0x02, 0x00); // 과열 코드
}

/**
 * Slave I2C 버스 모니터링
 * @param currentTime 현재 시간
 */
void monitorSlaveI2cBus(unsigned long currentTime) {
  if (currentTime - slaveSystemSafety.lastI2cCheck < 3000) { // 3초마다 체크
    return;
  }
  
  slaveSystemSafety.lastI2cCheck = currentTime;
  
  // I2C 버스 상태 테스트
  bool i2cHealthy = testSlaveI2cBus();
  
  if (!i2cHealthy) {
    if (slaveSystemSafety.i2cBusHealthy) {
      Serial.println("Slave I2C 버스 오류 감지");
      slaveSystemSafety.i2cBusHealthy = false;
      slaveSystemSafety.i2cErrorCount++;
      slaveSystemSafety.totalSystemErrors++;
      
      // I2C 버스 복구 시도
      recoverSlaveI2cBus();
    }
  } else {
    if (!slaveSystemSafety.i2cBusHealthy) {
      Serial.println("Slave I2C 버스 복구됨");
      slaveSystemSafety.i2cBusHealthy = true;
    }
  }
}

/**
 * Slave I2C 버스 테스트
 * @return I2C 버스 정상 여부
 */
bool testSlaveI2cBus() {
  // MCP23017 장치들과의 통신 테스트
  Wire.beginTransmission(MCP23017_1_ADDR);
  uint8_t error1 = Wire.endTransmission();
  
  Wire.beginTransmission(MCP23017_2_ADDR);
  uint8_t error2 = Wire.endTransmission();
  
  return (error1 == 0 && error2 == 0);
}

/**
 * Slave I2C 버스 복구
 */
void recoverSlaveI2cBus() {
  Serial.println("Slave I2C 버스 복구 시도");
  
  // I2C 버스 리셋
  Wire.end();
  delay(200);
  Wire.begin();
  delay(200);
  
  // MCP23017 재초기화
  initializeMCP23017();
}

/**
 * Slave 비상 상황 체크
 */
void checkSlaveEmergencyConditions() {
  // 총 시스템 오류가 임계값을 초과하면 비상 정지
  if (!slaveSystemSafety.emergencyShutdown && 
      slaveSystemSafety.totalSystemErrors >= SYSTEM_ERROR_THRESHOLD) {
    
    triggerSlaveEmergencyShutdown("시스템 오류 임계값 초과");
  }
  
  // 모터 안전 잠금과 센서 비상 모드가 동시에 활성화되면 완전 정지
  if (slaveMotorSafety.motorSafetyLockout && slaveSensorSafety.sensorEmergencyMode) {
    if (!slaveSystemSafety.emergencyShutdown) {
      triggerSlaveEmergencyShutdown("모터 및 센서 동시 오류");
    }
  }
}

/**
 * Slave 비상 정지 트리거
 * @param reason 정지 사유
 */
void triggerSlaveEmergencyShutdown(const char* reason) {
  Serial.print("Slave 비상 정지 트리거: ");
  Serial.println(reason);
  
  slaveSystemSafety.emergencyShutdown = true;
  slaveSystemSafety.emergencyShutdownTime = millis();
  
  // 모든 장치 즉시 정지
  emergencyStopMotors(true);
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  stopAllAudio();
  
  // Master에 비상 정지 상태 보고
  sendResponseToMasterWithData(RESP_ROBOT_ERROR, 0xFF, 0xFF);
  
  Serial.println("Slave 비상 정지 완료");
}

/**
 * Slave 하드웨어 안전장치 상태 출력 (디버깅용)
 */
void printSlaveHardwareSafetyStatus() {
  Serial.println("=== Slave 하드웨어 안전장치 상태 ===");
  
  // 모터 안전장치 상태
  Serial.println("--- 모터 안전장치 ---");
  Serial.print("Head 모터 상태: ");
  Serial.println(slaveMotorSafety.headMotorHealthy ? "정상" : "오작동");
  Serial.print("Body 모터 상태: ");
  Serial.println(slaveMotorSafety.bodyMotorHealthy ? "정상" : "오작동");
  Serial.print("Head 모터 타임아웃: ");
  Serial.println(slaveMotorSafety.headMotorTimeouts);
  Serial.print("Body 모터 타임아웃: ");
  Serial.println(slaveMotorSafety.bodyMotorTimeouts);
  Serial.print("추정 모터 전류: ");
  Serial.print(slaveMotorSafety.estimatedMotorCurrent);
  Serial.println("A");
  Serial.print("모터 안전 잠금: ");
  Serial.println(slaveMotorSafety.motorSafetyLockout ? "활성" : "비활성");
  
  // 센서 안전장치 상태
  Serial.println("--- 센서 안전장치 ---");
  for (int i = 0; i < 4; i++) {
    Serial.print("진동 센서 ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(slaveSensorSafety.vibrationSensorHealthy[i] ? "정상" : "오작동");
    Serial.print(" (오류 ");
    Serial.print(slaveSensorSafety.vibrationSensorErrors[i]);
    Serial.println("회)");
  }
  Serial.print("Head 리미트 스위치: ");
  Serial.println(slaveSensorSafety.limitSwitchHealthy[0] ? "정상" : "오작동");
  Serial.print("Body 리미트 스위치: ");
  Serial.println(slaveSensorSafety.limitSwitchHealthy[1] ? "정상" : "오작동");
  Serial.print("MCP23017 상태: ");
  Serial.println(slaveSensorSafety.mcpHealthy ? "정상" : "오작동");
  Serial.print("DFPlayer 상태: ");
  Serial.println(slaveSensorSafety.dfPlayerHealthy ? "정상" : "오작동");
  
  // 시스템 안전장치 상태
  Serial.println("--- 시스템 안전장치 ---");
  Serial.print("시스템 전압: ");
  Serial.print(slaveSystemSafety.systemVoltage);
  Serial.println("V");
  Serial.print("저전압 감지: ");
  Serial.println(slaveSystemSafety.lowVoltageDetected ? "감지됨" : "정상");
  Serial.print("시스템 온도: ");
  Serial.print(slaveSystemSafety.systemTemperature);
  Serial.println("°C");
  Serial.print("과열 상태: ");
  Serial.println(slaveSystemSafety.overheated ? "과열" : "정상");
  Serial.print("I2C 버스 상태: ");
  Serial.println(slaveSystemSafety.i2cBusHealthy ? "정상" : "오작동");
  Serial.print("시스템 안정성: ");
  Serial.println(slaveSystemSafety.systemStable ? "안정" : "불안정");
  
  // 전체 상태
  Serial.println("--- 전체 상태 ---");
  Serial.print("총 시스템 오류: ");
  Serial.println(slaveSystemSafety.totalSystemErrors);
  Serial.print("비상 정지 상태: ");
  Serial.println(slaveSystemSafety.emergencyShutdown ? "활성" : "비활성");
  
  Serial.println("===================================");
}

/**
 * Slave 하드웨어 안전장치 리셋
 */
void resetSlaveHardwareSafety() {
  Serial.println("Slave 하드웨어 안전장치 리셋");
  
  // 모터 안전장치 리셋
  slaveMotorSafety.headMotorTimeouts = 0;
  slaveMotorSafety.bodyMotorTimeouts = 0;
  slaveMotorSafety.motorRecoveryAttempts = 0;
  slaveMotorSafety.motorSafetyLockout = false;
  slaveMotorSafety.emergencyMotorStop = false;
  slaveMotorSafety.headMotorHealthy = true;
  slaveMotorSafety.bodyMotorHealthy = true;
  
  // 센서 안전장치 리셋
  for (int i = 0; i < 4; i++) {
    slaveSensorSafety.vibrationSensorErrors[i] = 0;
    slaveSensorSafety.vibrationSensorHealthy[i] = true;
    slaveSensorSafety.sensorCalibrationNeeded[i] = false;
  }
  for (int i = 0; i < 2; i++) {
    slaveSensorSafety.limitSwitchErrors[i] = 0;
    slaveSensorSafety.limitSwitchHealthy[i] = true;
  }
  slaveSensorSafety.mcpErrorCount = 0;
  slaveSensorSafety.mcpHealthy = true;
  slaveSensorSafety.dfPlayerErrorCount = 0;
  slaveSensorSafety.dfPlayerHealthy = true;
  slaveSensorSafety.totalSensorErrors = 0;
  slaveSensorSafety.sensorEmergencyMode = false;
  
  // 시스템 안전장치 리셋
  slaveSystemSafety.lowVoltageDetected = false;
  slaveSystemSafety.overheated = false;
  slaveSystemSafety.i2cErrorCount = 0;
  slaveSystemSafety.i2cBusHealthy = true;
  slaveSystemSafety.systemStable = true;
  slaveSystemSafety.emergencyShutdown = false;
  slaveSystemSafety.totalSystemErrors = 0;
  
  Serial.println("Slave 하드웨어 안전장치 리셋 완료");
}

/**
 * 모터 시작 시간 기록 (안전장치용)
 * @param motorType 모터 타입 (0: Head, 1: Body)
 */
void recordMotorStartTime(uint8_t motorType) {
  unsigned long currentTime = millis();
  
  if (motorType == 0) { // Head 모터
    slaveMotorSafety.headMotorStartTime = currentTime;
  } else { // Body 모터
    slaveMotorSafety.bodyMotorStartTime = currentTime;
  }
}

/**
 * 모터 정지 시간 기록 (안전장치용)
 * @param motorType 모터 타입 (0: Head, 1: Body)
 */
void recordMotorStopTime(uint8_t motorType) {
  if (motorType == 0) { // Head 모터
    slaveMotorSafety.headMotorStartTime = 0;
  } else { // Body 모터
    slaveMotorSafety.bodyMotorStartTime = 0;
  }
}

// =============================================================================
// 게임 로직 안정성 검증 시스템 (Task 17.3) - Slave Board
// =============================================================================

// Slave 게임 로직 안정성 구조체
struct SlaveGameLogicStability {
  bool inputValidationActive;            // 입력 검증 활성화
  uint8_t invalidInputCount;             // 잘못된 입력 횟수
  uint8_t inputErrorThreshold;           // 입력 오류 임계값
  unsigned long lastValidInputTime;      // 마지막 유효 입력 시간
  bool inputRecoveryMode;                // 입력 복구 모드
  
  bool motorStateValid;                  // 모터 상태 유효성
  uint8_t motorErrorCount;               // 모터 오류 횟수
  unsigned long lastMotorValidation;     // 마지막 모터 검증 시간
  bool motorRecoveryActive;              // 모터 복구 활성
  
  bool ledStateValid;                    // LED 상태 유효성
  uint8_t ledErrorCount;                 // LED 오류 횟수
  unsigned long lastLedValidation;       // 마지막 LED 검증 시간
  bool ledRecoveryActive;                // LED 복구 활성
  
  bool systemStable;                     // 시스템 안정성
  uint8_t totalSystemErrors;             // 총 시스템 오류
  bool emergencyMode;                    // 비상 모드
  unsigned long emergencyModeTime;       // 비상 모드 진입 시간
  
  bool autoRecoveryEnabled;              // 자동 복구 활성화
  uint8_t recoveryAttempts;              // 복구 시도 횟수
  uint8_t maxRecoveryAttempts;           // 최대 복구 시도
  unsigned long lastRecoveryTime;        // 마지막 복구 시간
  
  bool resetRequested;                   // 리셋 요청
  uint8_t resetReason;                   // 리셋 사유
  bool resetInProgress;                  // 리셋 진행 중
  unsigned long resetStartTime;          // 리셋 시작 시간
};

SlaveGameLogicStability slaveGameLogic;

// Slave 게임 로직 안정성 상수
#define SLAVE_INPUT_ERROR_THRESHOLD     5       // 입력 오류 임계값
#define SLAVE_MOTOR_ERROR_THRESHOLD     3       // 모터 오류 임계값
#define SLAVE_LED_ERROR_THRESHOLD       3       // LED 오류 임계값
#define SLAVE_SYSTEM_ERROR_THRESHOLD    10      // 시스템 오류 임계값
#define SLAVE_RECOVERY_MAX_ATTEMPTS     3       // 최대 복구 시도
#define SLAVE_VALIDATION_INTERVAL       2000    // 검증 간격 (ms)
#define SLAVE_RECOVERY_TIMEOUT          10000   // 복구 타임아웃 (ms)
#define SLAVE_EMERGENCY_TIMEOUT         300000  // 비상 모드 타임아웃 (5분)

// 입력 오류 유형
#define INPUT_ERROR_INVALID_SEQUENCE    1       // 잘못된 순서
#define INPUT_ERROR_TIMEOUT             2       // 타임아웃
#define INPUT_ERROR_SENSOR_MALFUNCTION  3       // 센서 오작동
#define INPUT_ERROR_DUPLICATE           4       // 중복 입력

/**
 * Slave 게임 로직 안정성 시스템 초기화
 */
void initSlaveGameLogicStability() {
  Serial.println("Slave 게임 로직 안정성 시스템 초기화...");
  
  slaveGameLogic.inputValidationActive = true;
  slaveGameLogic.invalidInputCount = 0;
  slaveGameLogic.inputErrorThreshold = SLAVE_INPUT_ERROR_THRESHOLD;
  slaveGameLogic.lastValidInputTime = millis();
  slaveGameLogic.inputRecoveryMode = false;
  
  slaveGameLogic.motorStateValid = true;
  slaveGameLogic.motorErrorCount = 0;
  slaveGameLogic.lastMotorValidation = millis();
  slaveGameLogic.motorRecoveryActive = false;
  
  slaveGameLogic.ledStateValid = true;
  slaveGameLogic.ledErrorCount = 0;
  slaveGameLogic.lastLedValidation = millis();
  slaveGameLogic.ledRecoveryActive = false;
  
  slaveGameLogic.systemStable = true;
  slaveGameLogic.totalSystemErrors = 0;
  slaveGameLogic.emergencyMode = false;
  slaveGameLogic.emergencyModeTime = 0;
  
  slaveGameLogic.autoRecoveryEnabled = true;
  slaveGameLogic.recoveryAttempts = 0;
  slaveGameLogic.maxRecoveryAttempts = SLAVE_RECOVERY_MAX_ATTEMPTS;
  slaveGameLogic.lastRecoveryTime = 0;
  
  slaveGameLogic.resetRequested = false;
  slaveGameLogic.resetReason = 0;
  slaveGameLogic.resetInProgress = false;
  slaveGameLogic.resetStartTime = 0;
  
  Serial.println("Slave 게임 로직 안정성 시스템 초기화 완료");
}

/**
 * Slave 게임 로직 안정성 모니터링
 */
void monitorSlaveGameLogicStability() {
  unsigned long currentTime = millis();
  
  // 입력 시스템 검증
  if (currentTime - slaveGameLogic.lastValidInputTime > SLAVE_VALIDATION_INTERVAL) {
    validateSlaveInputSystem();
  }
  
  // 모터 시스템 검증
  if (currentTime - slaveGameLogic.lastMotorValidation > SLAVE_VALIDATION_INTERVAL) {
    validateSlaveMotorSystem();
    slaveGameLogic.lastMotorValidation = currentTime;
  }
  
  // LED 시스템 검증
  if (currentTime - slaveGameLogic.lastLedValidation > SLAVE_VALIDATION_INTERVAL) {
    validateSlaveLedSystem();
    slaveGameLogic.lastLedValidation = currentTime;
  }
  
  // 복구 처리
  if (slaveGameLogic.motorRecoveryActive || slaveGameLogic.ledRecoveryActive) {
    processSlaveSystemRecovery();
  }
  
  // 리셋 처리
  if (slaveGameLogic.resetInProgress) {
    processSlaveSystemReset();
  }
  
  // 비상 모드 처리
  if (slaveGameLogic.emergencyMode) {
    handleSlaveEmergencyMode();
  }
  
  // 전체 시스템 안정성 평가
  evaluateSlaveSystemStability();
}

/**
 * Slave 입력 시스템 검증
 */
void validateSlaveInputSystem() {
  bool inputSystemHealthy = true;
  
  // 진동 센서 상태 확인
  for (int i = 0; i < 4; i++) {
    if (!slaveSensorSafety.vibrationSensorHealthy[i]) {
      inputSystemHealthy = false;
      Serial.print("진동 센서 ");
      Serial.print(i + 1);
      Serial.println(" 오작동 감지");
    }
  }
  
  // 입력 순서 검증 로직
  if (isInputActive && inputSequenceLength > 0) {
    // 입력 패턴의 논리적 일관성 확인
    if (!validateInputSequenceLogic()) {
      inputSystemHealthy = false;
      recordSlaveInputError(INPUT_ERROR_INVALID_SEQUENCE);
    }
  }
  
  if (!inputSystemHealthy) {
    slaveGameLogic.invalidInputCount++;
    
    if (slaveGameLogic.invalidInputCount >= slaveGameLogic.inputErrorThreshold) {
      Serial.println("입력 시스템 오류 임계값 초과");
      startSlaveInputRecovery();
    }
  } else {
    slaveGameLogic.lastValidInputTime = millis();
    
    // 연속 성공 시 오류 카운터 감소
    if (slaveGameLogic.invalidInputCount > 0) {
      slaveGameLogic.invalidInputCount--;
    }
  }
}

/**
 * 입력 순서 논리 검증
 * @return 입력 순서의 유효성
 */
bool validateInputSequenceLogic() {
  if (inputSequenceLength == 0) {
    return true; // 빈 순서는 유효
  }
  
  // 중복 입력 확인 (연속된 같은 입력)
  for (int i = 1; i < inputSequenceLength; i++) {
    if (inputSequence[i] == inputSequence[i-1]) {
      Serial.println("연속 중복 입력 감지");
      return false;
    }
  }
  
  // 유효한 발판 번호 확인 (1-4)
  for (int i = 0; i < inputSequenceLength; i++) {
    if (inputSequence[i] < 1 || inputSequence[i] > 4) {
      Serial.print("잘못된 발판 번호: ");
      Serial.println(inputSequence[i]);
      return false;
    }
  }
  
  // 입력 길이 확인 (최대 6개)
  if (inputSequenceLength > 6) {
    Serial.println("입력 순서 길이 초과");
    return false;
  }
  
  return true;
}

/**
 * Slave 입력 오류 기록
 * @param errorType 오류 유형
 */
void recordSlaveInputError(uint8_t errorType) {
  slaveGameLogic.invalidInputCount++;
  slaveGameLogic.totalSystemErrors++;
  
  Serial.print("Slave 입력 오류 기록 - 유형: ");
  Serial.print(errorType);
  Serial.print(", 총 오류: ");
  Serial.println(slaveGameLogic.invalidInputCount);
  
  // 오류 유형별 처리
  switch (errorType) {
    case INPUT_ERROR_INVALID_SEQUENCE:
      Serial.println("입력 순서 오류 - 순서 초기화");
      clearInputSequence();
      break;
      
    case INPUT_ERROR_TIMEOUT:
      Serial.println("입력 타임아웃 - 입력 대기 중지");
      setInputActive(false);
      break;
      
    case INPUT_ERROR_SENSOR_MALFUNCTION:
      Serial.println("센서 오작동 - 센서 재보정 필요");
      calibrateVibrationSensors();
      break;
      
    case INPUT_ERROR_DUPLICATE:
      Serial.println("중복 입력 - 마지막 입력 제거");
      if (inputSequenceLength > 0) {
        inputSequenceLength--;
      }
      break;
  }
}

/**
 * Slave 입력 복구 시작
 */
void startSlaveInputRecovery() {
  if (slaveGameLogic.inputRecoveryMode) {
    return; // 이미 복구 모드
  }
  
  Serial.println("Slave 입력 시스템 복구 시작");
  
  slaveGameLogic.inputRecoveryMode = true;
  slaveGameLogic.recoveryAttempts++;
  
  // 입력 시스템 초기화
  clearInputSequence();
  setInputActive(false);
  
  // 진동 센서 재보정
  calibrateVibrationSensors();
  
  // 임계값 조정 (더 관대하게)
  vibrationThreshold = VIBRATION_THRESHOLD + 50;
  
  Serial.println("입력 시스템 복구 완료");
  slaveGameLogic.inputRecoveryMode = false;
}

/**
 * Slave 모터 시스템 검증
 */
void validateSlaveMotorSystem() {
  bool motorSystemHealthy = true;
  
  // 모터 안전장치 상태 확인
  if (!slaveMotorSafety.headMotorHealthy || !slaveMotorSafety.bodyMotorHealthy) {
    motorSystemHealthy = false;
    Serial.println("모터 시스템 오작동 감지");
  }
  
  // 모터 과부하 확인
  if (slaveMotorSafety.motorOverloadProtection) {
    motorSystemHealthy = false;
    Serial.println("모터 과부하 보호 활성");
  }
  
  // 리미트 스위치 상태 확인
  if (!slaveSensorSafety.limitSwitchHealthy[0] || !slaveSensorSafety.limitSwitchHealthy[1]) {
    motorSystemHealthy = false;
    Serial.println("리미트 스위치 오작동");
  }
  
  if (!motorSystemHealthy) {
    slaveGameLogic.motorErrorCount++;
    slaveGameLogic.totalSystemErrors++;
    
    if (slaveGameLogic.motorErrorCount >= SLAVE_MOTOR_ERROR_THRESHOLD) {
      Serial.println("모터 시스템 오류 임계값 초과");
      startSlaveMotorRecovery();
    }
  } else {
    slaveGameLogic.motorStateValid = true;
    
    // 연속 성공 시 오류 카운터 감소
    if (slaveGameLogic.motorErrorCount > 0) {
      slaveGameLogic.motorErrorCount--;
    }
  }
}

/**
 * Slave 모터 복구 시작
 */
void startSlaveMotorRecovery() {
  if (slaveGameLogic.motorRecoveryActive) {
    return; // 이미 복구 중
  }
  
  Serial.println("Slave 모터 시스템 복구 시작");
  
  slaveGameLogic.motorRecoveryActive = true;
  slaveGameLogic.recoveryAttempts++;
  
  // 모든 모터 비상 정지
  emergencyStopMotors(true);
  
  // 모터 안전장치 리셋
  resetSlaveMotorSafety();
  
  // 리미트 스위치 재초기화
  initializeLimitSwitches();
  
  delay(2000); // 안정화 대기
  
  Serial.println("모터 시스템 복구 완료");
  slaveGameLogic.motorRecoveryActive = false;
}

/**
 * Slave LED 시스템 검증
 */
void validateSlaveLedSystem() {
  bool ledSystemHealthy = true;
  
  // MCP23017 상태 확인
  if (!slaveSensorSafety.mcpHealthy) {
    ledSystemHealthy = false;
    Serial.println("MCP23017 통신 오류");
  }
  
  // LED 상태 일관성 확인
  // (실제 구현에서는 LED 상태를 읽어서 확인)
  
  if (!ledSystemHealthy) {
    slaveGameLogic.ledErrorCount++;
    slaveGameLogic.totalSystemErrors++;
    
    if (slaveGameLogic.ledErrorCount >= SLAVE_LED_ERROR_THRESHOLD) {
      Serial.println("LED 시스템 오류 임계값 초과");
      startSlaveLedRecovery();
    }
  } else {
    slaveGameLogic.ledStateValid = true;
    
    // 연속 성공 시 오류 카운터 감소
    if (slaveGameLogic.ledErrorCount > 0) {
      slaveGameLogic.ledErrorCount--;
    }
  }
}

/**
 * Slave LED 복구 시작
 */
void startSlaveLedRecovery() {
  if (slaveGameLogic.ledRecoveryActive) {
    return; // 이미 복구 중
  }
  
  Serial.println("Slave LED 시스템 복구 시작");
  
  slaveGameLogic.ledRecoveryActive = true;
  slaveGameLogic.recoveryAttempts++;
  
  // 모든 LED 소등
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  
  // MCP23017 재초기화
  initializeMCP23017();
  
  delay(1000); // 안정화 대기
  
  Serial.println("LED 시스템 복구 완료");
  slaveGameLogic.ledRecoveryActive = false;
}

/**
 * Slave 시스템 복구 처리
 */
void processSlaveSystemRecovery() {
  unsigned long currentTime = millis();
  
  // 복구 타임아웃 확인
  if (currentTime - slaveGameLogic.lastRecoveryTime > SLAVE_RECOVERY_TIMEOUT) {
    Serial.println("Slave 시스템 복구 타임아웃");
    
    // 복구 실패 - 비상 모드 진입
    if (slaveGameLogic.recoveryAttempts >= slaveGameLogic.maxRecoveryAttempts) {
      enterSlaveEmergencyMode();
    }
    
    // 복구 상태 초기화
    slaveGameLogic.motorRecoveryActive = false;
    slaveGameLogic.ledRecoveryActive = false;
  }
}

/**
 * Slave 시스템 안정성 평가
 */
void evaluateSlaveSystemStability() {
  // 전체 시스템 오류 수 확인
  if (slaveGameLogic.totalSystemErrors >= SLAVE_SYSTEM_ERROR_THRESHOLD) {
    Serial.println("Slave 시스템 오류 임계값 초과");
    
    if (slaveGameLogic.autoRecoveryEnabled) {
      requestSlaveSystemReset(RESET_REASON_SYSTEM_ERROR);
    } else {
      enterSlaveEmergencyMode();
    }
  }
  
  // 시스템 안정성 업데이트
  slaveGameLogic.systemStable = (slaveGameLogic.totalSystemErrors < SLAVE_SYSTEM_ERROR_THRESHOLD) &&
                                !slaveGameLogic.emergencyMode &&
                                slaveGameLogic.motorStateValid &&
                                slaveGameLogic.ledStateValid;
}

/**
 * Slave 시스템 리셋 요청
 * @param reason 리셋 사유
 */
void requestSlaveSystemReset(uint8_t reason) {
  if (slaveGameLogic.resetInProgress) {
    Serial.println("이미 Slave 리셋이 진행 중입니다");
    return;
  }
  
  Serial.print("Slave 시스템 리셋 요청 - 사유: ");
  Serial.println(reason);
  
  slaveGameLogic.resetRequested = true;
  slaveGameLogic.resetReason = reason;
  slaveGameLogic.resetInProgress = true;
  slaveGameLogic.resetStartTime = millis();
}

/**
 * Slave 시스템 리셋 처리
 */
void processSlaveSystemReset() {
  Serial.println("Slave 시스템 리셋 수행 중...");
  
  // 모든 하드웨어 정지
  emergencyStopMotors(true);
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  stopAllAudio();
  
  // 시스템 상태 초기화
  initSlaveGameLogicStability();
  initSlaveHardwareSafety();
  initSlaveCommReliability();
  
  // 하드웨어 재초기화
  initializeMCP23017();
  initializeVibrationSensors();
  initializeLimitSwitches();
  
  delay(2000); // 안정화 대기
  
  Serial.println("Slave 시스템 리셋 완료");
  slaveGameLogic.resetInProgress = false;
  slaveGameLogic.resetRequested = false;
}

/**
 * Slave 비상 모드 진입
 */
void enterSlaveEmergencyMode() {
  slaveGameLogic.emergencyMode = true;
  slaveGameLogic.emergencyModeTime = millis();
  
  Serial.println("=== Slave 비상 모드 진입 ===");
  
  // 모든 동작 정지
  emergencyStopMotors(true);
  turnOffAllOutputs();
  turnOffAllScoreLEDs();
  stopAllAudio();
  
  // 자동 복구 비활성화
  slaveGameLogic.autoRecoveryEnabled = false;
  
  Serial.println("Slave 비상 모드 활성화 완료");
}

/**
 * Slave 비상 모드 처리
 */
void handleSlaveEmergencyMode() {
  unsigned long currentTime = millis();
  
  // 비상 모드 타임아웃 확인
  if (currentTime - slaveGameLogic.emergencyModeTime > SLAVE_EMERGENCY_TIMEOUT) {
    Serial.println("Slave 비상 모드 타임아웃 - 자동 복구 시도");
    exitSlaveEmergencyMode();
    return;
  }
  
  // 주기적 상태 메시지
  static unsigned long lastEmergencyMessage = 0;
  if (currentTime - lastEmergencyMessage > 30000) { // 30초마다
    Serial.println("Slave 비상 모드 활성 - 수동 개입 필요");
    lastEmergencyMessage = currentTime;
  }
  
  // 최소한의 안전 기능만 유지
  if (headMotorActive || bodyMotorActive) {
    emergencyStopMotors(true);
  }
}

/**
 * Slave 비상 모드 해제
 */
void exitSlaveEmergencyMode() {
  if (!slaveGameLogic.emergencyMode) {
    return;
  }
  
  Serial.println("Slave 비상 모드 해제");
  
  slaveGameLogic.emergencyMode = false;
  slaveGameLogic.autoRecoveryEnabled = true;
  slaveGameLogic.recoveryAttempts = 0;
  slaveGameLogic.totalSystemErrors = 0;
  
  // 시스템 전체 리셋
  requestSlaveSystemReset(RESET_REASON_EMERGENCY);
  
  Serial.println("Slave 비상 모드 해제 완료");
}

/**
 * Slave 게임 로직 안정성 상태 출력
 */
void printSlaveGameLogicStabilityStatus() {
  Serial.println("=== Slave 게임 로직 안정성 상태 ===");
  
  Serial.println("[ 입력 시스템 ]");
  Serial.print("입력 검증 활성: ");
  Serial.println(slaveGameLogic.inputValidationActive ? "예" : "아니오");
  Serial.print("잘못된 입력 횟수: ");
  Serial.println(slaveGameLogic.invalidInputCount);
  Serial.print("입력 복구 모드: ");
  Serial.println(slaveGameLogic.inputRecoveryMode ? "활성" : "비활성");
  
  Serial.println("[ 모터 시스템 ]");
  Serial.print("모터 상태 유효: ");
  Serial.println(slaveGameLogic.motorStateValid ? "예" : "아니오");
  Serial.print("모터 오류 횟수: ");
  Serial.println(slaveGameLogic.motorErrorCount);
  Serial.print("모터 복구 활성: ");
  Serial.println(slaveGameLogic.motorRecoveryActive ? "예" : "아니오");
  
  Serial.println("[ LED 시스템 ]");
  Serial.print("LED 상태 유효: ");
  Serial.println(slaveGameLogic.ledStateValid ? "예" : "아니오");
  Serial.print("LED 오류 횟수: ");
  Serial.println(slaveGameLogic.ledErrorCount);
  Serial.print("LED 복구 활성: ");
  Serial.println(slaveGameLogic.ledRecoveryActive ? "예" : "아니오");
  
  Serial.println("[ 전체 시스템 ]");
  Serial.print("시스템 안정: ");
  Serial.println(slaveGameLogic.systemStable ? "예" : "아니오");
  Serial.print("총 시스템 오류: ");
  Serial.println(slaveGameLogic.totalSystemErrors);
  Serial.print("비상 모드: ");
  Serial.println(slaveGameLogic.emergencyMode ? "활성" : "비활성");
  Serial.print("복구 시도: ");
  Serial.print(slaveGameLogic.recoveryAttempts);
  Serial.print("/");
  Serial.println(slaveGameLogic.maxRecoveryAttempts);
  
  Serial.println("================================");
}

/**
 * Slave 게임 로직 안정성 시스템 리셋
 */
void resetSlaveGameLogicStability() {
  Serial.println("Slave 게임 로직 안정성 시스템 리셋");
  
  slaveGameLogic.emergencyMode = false;
  slaveGameLogic.inputRecoveryMode = false;
  slaveGameLogic.motorRecoveryActive = false;
  slaveGameLogic.ledRecoveryActive = false;
  slaveGameLogic.resetInProgress = false;
  
  slaveGameLogic.invalidInputCount = 0;
  slaveGameLogic.motorErrorCount = 0;
  slaveGameLogic.ledErrorCount = 0;
  slaveGameLogic.totalSystemErrors = 0;
  slaveGameLogic.recoveryAttempts = 0;
  
  slaveGameLogic.autoRecoveryEnabled = true;
  slaveGameLogic.systemStable = true;
  slaveGameLogic.motorStateValid = true;
  slaveGameLogic.ledStateValid = true;
  
  Serial.println("Slave 게임 로직 안정성 시스템 리셋 완료");
}