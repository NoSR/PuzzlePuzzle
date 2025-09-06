/*
 * 무궁화 게임 시스템 - Master Board (깔끔한 최적화 버전)
 * Arduino Mega 2560 기반
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// =============================================================================
// 하드웨어 핀 정의
// =============================================================================

const uint8_t PIR_PINS[4] = {23, 25, 39, 41};
const uint8_t CEILING_PINS[4] = {7, 6, 5, 4};
#define EFFECT_LED_COUNT 8
const uint8_t EFFECT_PINS[EFFECT_LED_COUNT] = {22, 24, 26, 28, 30, 32, 34, 36};

#define GREEN_BEACON_PIN 10
#define RED_BEACON_PIN 11
#define DF_RX_PIN 3
#define DF_TX_PIN 2

// =============================================================================
// 게임 상수
// =============================================================================

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

#define LIGHT_SEQUENCE_INTERVAL 500
#define MOTION_DETECT_DURATION 5000
#define INPUT_TIMEOUT 10000

// 통신 명령어
#define CMD_ROBOT_HEAD_FRONT 0x10
#define CMD_ROBOT_HEAD_BACK 0x11
#define CMD_ROBOT_BODY_FRONT 0x12
#define CMD_ROBOT_BODY_BACK 0x13
#define CMD_SHOW_PATTERN 0x20
#define CMD_UPDATE_SCORE 0x22
#define CMD_PLAY_EFFECT 0x30
#define CMD_GET_INPUT 0x41

// 응답 코드
#define RESP_ROBOT_READY 0x10
#define RESP_PATTERN_DONE 0x20
#define RESP_INPUT_CORRECT 0x31
#define RESP_INPUT_WRONG 0x32
#define RESP_INPUT_TIMEOUT 0x33
#define RESP_START_BUTTON_PRESSED 0x40

// =============================================================================
// 게임 상태 구조체
// =============================================================================

struct {
  GamePhase currentPhase;
  uint8_t currentLevel;
  uint8_t currentScore;
  bool isGameActive;
  unsigned long phaseStartTime;
  bool motionDetected;
} gameState;

struct {
  bool sensorActive[4];
  unsigned long lastTriggerTime[4];
  bool motionDetectionActive;
  unsigned long motionStartTime;
} pirState;

struct {
  bool sequenceActive;
  uint8_t currentStep;
  unsigned long lastStepTime;
} ceilingState;

struct {
  bool greenOn;
  bool redOn;
} beaconState;

// =============================================================================
// 전역 변수
// =============================================================================

SoftwareSerial dfSerial(DF_RX_PIN, DF_TX_PIN);

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

// 천정 조명 함수들
void startCeilingLightSequence();
void updateCeilingLights();

// 경광등 함수들
void setBeaconLight(bool green, bool red);
void updateBeaconLights();

// 게임 상태 함수들
void updateGameState();
void transitionToPhase(GamePhase newPhase);
void handleGameStartPhase();
void handleRobotTurnFrontPhase();
void handleNarrationPhase();
void handleRobotTurnBackPhase();
void handleShowPatternPhase();
void handlePlayerInputPhase();
void handleMotionDetectPhase();
void handleScoreUpdatePhase();
void handleGameClearPhase();

// DFPlayer 함수들
void sendDFCommand(uint8_t cmd, uint8_t param1, uint8_t param2);
void playAudio(uint8_t trackNumber);

// 이펙트 함수들
void playErrorEffect();
void playSuccessEffect();

// 통신 함수들
void sendCommandToSlave(uint8_t cmd, uint8_t data1, uint8_t data2);
void processSlaveComm();
void handleSlaveResponse(uint8_t response, uint8_t data1, uint8_t data2);

// 유틸리티 함수들
uint8_t getPatternLengthForLevel(uint8_t level);
void updateScoreLEDs();

// 시리얼 명령 함수들
void processSerialCommands();
void printGameStatus();

// =============================================================================
// Arduino 메인 함수들
// =============================================================================

void setup() {
  Serial.begin(9600);
  dfSerial.begin(9600);

  Serial.println("=== 무궁화 게임 시스템 시작 ===");

  initializePins();
  initializeGameState();
  initializeDFPlayer();

  Serial.println("Master Board 초기화 완료");
}

void loop() {
  readPIRSensors();
  updateGameState();
  updateCeilingLights();
  updateBeaconLights();
  processSlaveComm();
  processSerialCommands();

  delay(10);
}

// =============================================================================
// 초기화 함수들
// =============================================================================

void initializePins() {
  // PIR 센서 핀 설정
  for (int i = 0; i < 4; i++) {
    pinMode(PIR_PINS[i], INPUT);
  }

  // 천정 조명 핀 설정
  for (int i = 0; i < 4; i++) {
    pinMode(CEILING_PINS[i], OUTPUT);
    digitalWrite(CEILING_PINS[i], LOW);
  }

  // 경광등 핀 설정
  pinMode(GREEN_BEACON_PIN, OUTPUT);
  pinMode(RED_BEACON_PIN, OUTPUT);
  digitalWrite(GREEN_BEACON_PIN, LOW);
  digitalWrite(RED_BEACON_PIN, LOW);

  // 이펙트 LED 핀 설정
  for (int i = 0; i < EFFECT_LED_COUNT; i++) {
    pinMode(EFFECT_PINS[i], OUTPUT);
    digitalWrite(EFFECT_PINS[i], LOW);
  }

  Serial.println("핀 초기화 완료");
}

void initializeGameState() {
  gameState.currentPhase = PHASE_IDLE;
  gameState.currentLevel = 1;
  gameState.currentScore = 0;
  gameState.isGameActive = false;
  gameState.phaseStartTime = millis();
  gameState.motionDetected = false;

  // PIR 상태 초기화
  for (int i = 0; i < 4; i++) {
    pirState.sensorActive[i] = false;
    pirState.lastTriggerTime[i] = 0;
  }
  pirState.motionDetectionActive = false;
  pirState.motionStartTime = 0;

  // 천정 조명 상태 초기화
  ceilingState.sequenceActive = false;
  ceilingState.currentStep = 0;
  ceilingState.lastStepTime = 0;

  // 경광등 상태 초기화
  beaconState.greenOn = false;
  beaconState.redOn = false;

  Serial.println("게임 상태 초기화 완료");
}

void initializeDFPlayer() {
  delay(1000);
  sendDFCommand(0x06, 0, 20); // 볼륨 설정
  delay(100);
  Serial.println("DFPlayer 초기화 완료");
}

// =============================================================================
// PIR 센서 관련 함수들
// =============================================================================

void readPIRSensors() {
  unsigned long currentTime = millis();

  for (int i = 0; i < 4; i++) {
    bool currentReading = digitalRead(PIR_PINS[i]);

    if (currentReading && !pirState.sensorActive[i]) {
      pirState.sensorActive[i] = true;
      pirState.lastTriggerTime[i] = currentTime;

      if (pirState.motionDetectionActive) {
        gameState.motionDetected = true;
        Serial.print("PIR 센서 ");
        Serial.print(i);
        Serial.println(" 동작 감지!");
      }
    } else if (!currentReading && pirState.sensorActive[i]) {
      pirState.sensorActive[i] = false;
    }
  }
}

void startMotionDetection() {
  Serial.println("=== 동작 감지 시작 ===");
  pirState.motionDetectionActive = true;
  pirState.motionStartTime = millis();
  gameState.motionDetected = false;
  Serial.println("5초간 움직이지 마세요!");
}

void stopMotionDetection() {
  pirState.motionDetectionActive = false;

  Serial.print("동작 감지 결과: ");
  Serial.println(gameState.motionDetected ? "감지됨" : "안전");

  if (gameState.motionDetected) {
    playAudio(6); // 총소리
    playErrorEffect();

    if (gameState.currentScore > 0) {
      gameState.currentScore--;
      updateScoreLEDs();
    }
  }
}

// =============================================================================
// 천정 조명 관련 함수들
// =============================================================================

void startCeilingLightSequence() {
  Serial.println("=== 천정 조명 순차 점등 시작 ===");

  for (int i = 0; i < 4; i++) {
    digitalWrite(CEILING_PINS[i], LOW);
  }

  ceilingState.sequenceActive = true;
  ceilingState.currentStep = 0;
  ceilingState.lastStepTime = millis();
}

void updateCeilingLights() {
  if (!ceilingState.sequenceActive)
    return;

  unsigned long currentTime = millis();

  if (currentTime - ceilingState.lastStepTime >= LIGHT_SEQUENCE_INTERVAL) {
    if (ceilingState.currentStep < 4) {
      digitalWrite(CEILING_PINS[ceilingState.currentStep], HIGH);
      playAudio(ceilingState.currentStep + 1);

      Serial.print("천정 조명 ");
      Serial.print(ceilingState.currentStep + 1);
      Serial.println("번 점등");

      ceilingState.currentStep++;
      ceilingState.lastStepTime = currentTime;

      if (ceilingState.currentStep >= 4) {
        ceilingState.sequenceActive = false;
        Serial.println("천정 조명 순차 점등 완료");
        transitionToPhase(PHASE_ROBOT_TURN_FRONT);
      }
    }
  }
}

// =============================================================================
// 경광등 관련 함수들
// =============================================================================

void setBeaconLight(bool green, bool red) {
  beaconState.greenOn = green;
  beaconState.redOn = red;

  digitalWrite(GREEN_BEACON_PIN, green ? HIGH : LOW);
  digitalWrite(RED_BEACON_PIN, red ? HIGH : LOW);
}

void updateBeaconLights() {
  switch (gameState.currentPhase) {
  case PHASE_ROBOT_TURN_BACK:
  case PHASE_SHOW_PATTERN:
  case PHASE_PLAYER_INPUT:
    setBeaconLight(true, false); // 초록색
    break;

  case PHASE_MOTION_DETECT:
    setBeaconLight(false, true); // 빨간색
    break;

  default:
    setBeaconLight(false, false);
    break;
  }
}

// =============================================================================
// 게임 상태 관리 함수들
// =============================================================================

void updateGameState() {
  switch (gameState.currentPhase) {
  case PHASE_IDLE:
    // 게임 시작 대기
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
    Serial.println("알 수 없는 게임 단계");
    transitionToPhase(PHASE_IDLE);
    break;
  }
}

void transitionToPhase(GamePhase newPhase) {
  Serial.print("게임 단계 변경: ");
  Serial.print(gameState.currentPhase);
  Serial.print(" -> ");
  Serial.println(newPhase);

  gameState.currentPhase = newPhase;
  gameState.phaseStartTime = millis();
}

// =============================================================================
// 게임 단계별 처리 함수들
// =============================================================================

void handleGameStartPhase() {
  Serial.println("=== 게임 시작 ===");
  playAudio(10);
  startCeilingLightSequence();
  transitionToPhase(PHASE_GAME_START);
}

void handleRobotTurnFrontPhase() {
  sendCommandToSlave(CMD_ROBOT_BODY_FRONT, 0, 0);
  delay(3000);
  transitionToPhase(PHASE_NARRATION);
}

void handleNarrationPhase() {
  playAudio(11);
  delay(5000);
  transitionToPhase(PHASE_ROBOT_TURN_BACK);
}

void handleRobotTurnBackPhase() {
  sendCommandToSlave(CMD_ROBOT_HEAD_BACK, 0, 0);
  delay(3000);
  transitionToPhase(PHASE_SHOW_PATTERN);
}

void handleShowPatternPhase() {
  uint8_t patternLength = getPatternLengthForLevel(gameState.currentLevel);
  sendCommandToSlave(CMD_SHOW_PATTERN, patternLength, 0);
  delay(3000);
  transitionToPhase(PHASE_PLAYER_INPUT);
}

void handlePlayerInputPhase() {
  sendCommandToSlave(CMD_GET_INPUT, 0, 0);

  unsigned long startTime = millis();
  while (millis() - startTime < INPUT_TIMEOUT) {
    processSlaveComm();
    delay(10);
  }

  transitionToPhase(PHASE_MOTION_DETECT);
}

void handleMotionDetectPhase() {
  sendCommandToSlave(CMD_ROBOT_HEAD_FRONT, 0, 0);
  delay(1000);

  startMotionDetection();

  unsigned long startTime = millis();
  while (millis() - startTime < MOTION_DETECT_DURATION) {
    readPIRSensors();
    delay(10);
  }

  stopMotionDetection();
  transitionToPhase(PHASE_SCORE_UPDATE);
}

void handleScoreUpdatePhase() {
  updateScoreLEDs();

  if (gameState.currentScore >= 7) {
    transitionToPhase(PHASE_GAME_CLEAR);
  } else {
    gameState.currentLevel++;
    transitionToPhase(PHASE_ROBOT_TURN_BACK);
  }
}

void handleGameClearPhase() {
  Serial.println("=== 게임 클리어! ===");
  playAudio(12);
  playSuccessEffect();
  delay(5000);
  transitionToPhase(PHASE_IDLE);
}

// =============================================================================
// 유틸리티 함수들
// =============================================================================

uint8_t getPatternLengthForLevel(uint8_t level) {
  if (level <= 4)
    return level + 1;
  else if (level <= 10)
    return 5;
  else
    return 6;
}

void updateScoreLEDs() {
  sendCommandToSlave(CMD_UPDATE_SCORE, gameState.currentScore, 0);
}

void playErrorEffect() {
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < EFFECT_LED_COUNT; j++) {
      digitalWrite(EFFECT_PINS[j], HIGH);
    }
    delay(200);
    for (int j = 0; j < EFFECT_LED_COUNT; j++) {
      digitalWrite(EFFECT_PINS[j], LOW);
    }
    delay(200);
  }
}

void playSuccessEffect() {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < EFFECT_LED_COUNT; j++) {
      digitalWrite(EFFECT_PINS[j], HIGH);
      delay(100);
    }
    for (int j = 0; j < EFFECT_LED_COUNT; j++) {
      digitalWrite(EFFECT_PINS[j], LOW);
      delay(100);
    }
  }
}

// =============================================================================
// DFPlayer 관련 함수들
// =============================================================================

void sendDFCommand(uint8_t cmd, uint8_t param1, uint8_t param2) {
  uint8_t buffer[10] = {0x7E,   0xFF,   0x06, cmd,  0x00,
                        param1, param2, 0x00, 0x00, 0xEF};

  uint16_t checksum = 0;
  for (int i = 1; i < 7; i++) {
    checksum += buffer[i];
  }
  checksum = 0 - checksum;

  buffer[7] = (checksum >> 8) & 0xFF;
  buffer[8] = checksum & 0xFF;

  for (int i = 0; i < 10; i++) {
    dfSerial.write(buffer[i]);
  }
}

void playAudio(uint8_t trackNumber) {
  sendDFCommand(0x03, 0, trackNumber);
  Serial.print("오디오 재생: 트랙 ");
  Serial.println(trackNumber);
}

// =============================================================================
// 통신 관련 함수들
// =============================================================================

void sendCommandToSlave(uint8_t cmd, uint8_t data1, uint8_t data2) {
  Serial.write(0xAA);
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
  Serial.write(cmd ^ data1 ^ data2);

  Serial.print("Slave 명령 전송: 0x");
  Serial.println(cmd, HEX);
}

void processSlaveComm() {
  if (Serial.available() >= 5) {
    uint8_t header = Serial.read();
    if (header == 0xAA) {
      uint8_t response = Serial.read();
      uint8_t data1 = Serial.read();
      uint8_t data2 = Serial.read();
      uint8_t checksum = Serial.read();

      if (checksum == (response ^ data1 ^ data2)) {
        handleSlaveResponse(response, data1, data2);
      }
    }
  }
}

void handleSlaveResponse(uint8_t response, uint8_t data1, uint8_t data2) {
  // 사용하지 않는 매개변수 경고 억제
  (void)data1;
  (void)data2;

  switch (response) {
  case RESP_ROBOT_READY:
    Serial.println("로봇 회전 완료");
    break;

  case RESP_PATTERN_DONE:
    Serial.println("패턴 표시 완료");
    break;

  case RESP_INPUT_CORRECT:
    Serial.println("정답 입력!");
    gameState.currentScore++;
    break;

  case RESP_INPUT_WRONG:
    Serial.println("오답 입력!");
    break;

  case RESP_INPUT_TIMEOUT:
    Serial.println("입력 시간 초과");
    break;

  case RESP_START_BUTTON_PRESSED:
    Serial.println("=== 시작 버튼 눌림! 게임 시작 ===");
    if (gameState.currentPhase == PHASE_IDLE) {
      transitionToPhase(PHASE_GAME_START);
    } else {
      Serial.println("게임이 이미 진행 중입니다.");
    }
    break;

  default:
    Serial.println("알 수 없는 응답");
    break;
  }
}

// =============================================================================
// 시리얼 명령 처리
// =============================================================================

void processSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "start") {
      // 시리얼 명령으로도 게임 시작 가능 (테스트/디버깅용)
      Serial.println("=== 시리얼 명령으로 게임 시작 ===");
      if (gameState.currentPhase == PHASE_IDLE) {
        transitionToPhase(PHASE_GAME_START);
      } else {
        Serial.println("게임이 이미 진행 중입니다.");
      }
    } else if (command == "stop") {
      transitionToPhase(PHASE_IDLE);
    } else if (command == "status") {
      printGameStatus();
    } else if (command == "reset") {
      initializeGameState();
    }
  }
}

void printGameStatus() {
  Serial.println("=== 게임 상태 ===");
  Serial.print("단계: ");
  Serial.println(gameState.currentPhase);
  Serial.print("레벨: ");
  Serial.println(gameState.currentLevel);
  Serial.print("점수: ");
  Serial.println(gameState.currentScore);
  Serial.println("================");
}