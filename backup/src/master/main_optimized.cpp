/*
 * 무궁화 게임 시스템 - Master Board (최적화 버전)
 * Arduino Mega 2560 기반
 * 
 * 핵심 기능:
 * - 게임 상태 관리 및 메인 로직
 * - PIR 센서를 통한 동작 감지
 * - 천정 조명 순차 점등
 * - 경광등 제어
 * - Slave Board와의 시리얼 통신
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// =============================================================================
// 하드웨어 핀 정의
// =============================================================================

// PIR 센서 핀 (4개)
const uint8_t PIR_PINS[4] = {2, 3, 4, 5};

// 천정 조명 핀 (4개)
const uint8_t CEILING_PINS[4] = {6, 7, 8, 9};

// 경광등 핀
#define GREEN_BEACON_PIN 10
#define RED_BEACON_PIN 11

// 이펙트 LED 핀 (6개)
const uint8_t EFFECT_PINS[6] = {22, 24, 26, 28, 30, 32};

// DFPlayer 핀
#define DF_RX_PIN 12
#define DF_TX_PIN 13

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

// 타이밍 상수
#define LIGHT_SEQUENCE_INTERVAL 500    // 천정 조명 점등 간격 (ms)
#define MOTION_DETECT_DURATION 5000    // 동작 감지 시간 (ms)
#define PATTERN_DISPLAY_TIME 3000      // 패턴 표시 시간 (ms)
#define INPUT_TIMEOUT 10000            // 입력 대기 시간 (ms)

// 통신 상수
#define SERIAL_BAUD_RATE 9600
#define COMM_TIMEOUT 1000

// 명령어 코드
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

// =============================================================================
// 게임 상태 구조체
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

struct PIRState {
  bool sensorActive[4];
  unsigned long lastTriggerTime[4];
  bool motionDetectionActive;
  unsigned long motionStartTime;
};

struct CeilingLightState {
  bool sequenceActive;
  uint8_t currentStep;
  unsigned long lastStepTime;
};

struct BeaconState {
  bool greenOn;
  bool redOn;
  bool blinking;
  unsigned long lastBlinkTime;
};

// =============================================================================
// 전역 변수
// =============================================================================

GameState gameState;
PIRState pirState;
CeilingLightState ceilingState;
BeaconState beaconState;

SoftwareSerial dfSerial(DF_RX_PIN, DF_TX_PIN);

// =============================================================================
// 전역 변수 선언
// =============================================================================

// 전역 변수 정의
extern GameState gameState;
extern PIRState pirState;
extern CeilingLightState ceilingState;
extern BeaconState beaconState;

// 전역 변수 실제 정의
GameState gameState;
PIRState pirState;
CeilingLightState ceilingState;
BeaconState beaconState;

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
void updateCeilingLights();

// 경광등 함수들
void updateBeaconLights();

// 게임 상태 함수들
void updateGameState();
void transitionToPhase(GamePhase newPhase);
void handleIdlePhase();
void handleGameStartPhase();
void handleRobotTurnFrontPhase();
void handleNarrationPhase();
void handleRobotTurnBackPhase();
void handleShowPatternPhase();
void handlePlayerInputPhase();
void handleMotionDetectPhase();
void handleScoreUpdatePhase();
void handleGameClearPhase();

// 오디오 함수들
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
  Serial.begin(SERIAL_BAUD_RATE);
  dfSerial.begin(9600);
  
  Serial.println("=== 무궁화 게임 시스템 시작 ===");
  
  initializePins();
  initializeGameState();
  initializeDFPlayer();
  
  Serial.println("Master Board 초기화 완료");
}

void loop() {
  // PIR 센서 읽기
  readPIRSensors();
  
  // 게임 상태 업데이트
  updateGameState();
  
  // 천정 조명 업데이트
  updateCeilingLights();
  
  // 경광등 업데이트
  updateBeaconLights();
  
  // Slave 통신 처리
  processSlaveComm();
  
  // 시리얼 명령 처리
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
  for (int i = 0; i < 6; i++) {
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
  gameState.isRobotFacing = false;
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
  beaconState.blinking = false;
  beaconState.lastBlinkTime = 0;
  
  Serial.println("게임 상태 초기화 완료");
}

void initializeDFPlayer() {
  delay(1000); // DFPlayer 안정화 대기
  
  // 볼륨 설정 (0-30)
  sendDFCommand(0x06, 0, 20);
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
      // 센서 활성화
      pirState.sensorActive[i] = true;
      pirState.lastTriggerTime[i] = currentTime;
      
      // 동작 감지 중이면 결과 기록
      if (pirState.motionDetectionActive) {
        gameState.motionDetected = true;
        Serial.print("PIR 센서 ");
        Serial.print(i);
        Serial.println(" 동작 감지!");
      }
    } else if (!currentReading && pirState.sensorActive[i]) {
      // 센서 비활성화
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
    // 총소리 효과음 재생
    playAudio(6);
    
    // 이펙트 LED 점멸
    playErrorEffect();
    
    // 점수 감점
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
  
  // 모든 조명 끄기
  for (int i = 0; i < 4; i++) {
    digitalWrite(CEILING_PINS[i], LOW);
  }
  
  ceilingState.sequenceActive = true;
  ceilingState.currentStep = 0;
  ceilingState.lastStepTime = millis();
}

void updateCeilingLights() {
  if (!ceilingState.sequenceActive) return;
  
  unsigned long currentTime = millis();
  
  if (currentTime - ceilingState.lastStepTime >= LIGHT_SEQUENCE_INTERVAL) {
    if (ceilingState.currentStep < 4) {
      // 현재 단계 조명 점등
      digitalWrite(CEILING_PINS[ceilingState.currentStep], HIGH);
      
      // 조명별 효과음 재생
      playAudio(ceilingState.currentStep + 1);
      
      Serial.print("천정 조명 ");
      Serial.print(ceilingState.currentStep + 1);
      Serial.println("번 점등");
      
      ceilingState.currentStep++;
      ceilingState.lastStepTime = currentTime;
      
      // 모든 조명 점등 완료
      if (ceilingState.currentStep >= 4) {
        ceilingState.sequenceActive = false;
        Serial.println("천정 조명 순차 점등 완료");
        
        // 다음 단계로 진행
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
  
  Serial.print("경광등: 초록=");
  Serial.print(green ? "ON" : "OFF");
  Serial.print(", 빨강=");
  Serial.println(red ? "ON" : "OFF");
}

void updateBeaconLights() {
  // 게임 상태에 따른 경광등 제어
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
      setBeaconLight(false, false); // 모두 끄기
      break;
  }
}

// =============================================================================
// 게임 상태 관리 함수들
// =============================================================================

void updateGameState() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - gameState.phaseStartTime;
  
  switch (gameState.currentPhase) {
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

void handleIdlePhase() {
  // 게임 시작 대기 상태
  // 시리얼 명령으로 게임 시작 가능
}

void handleGameStartPhase() {
  Serial.println("=== 게임 시작 ===");
  
  // 게임 시작 효과음
  playAudio(10);
  
  // 천정 조명 순차 점등 시작
  startCeilingLightSequence();
  
  transitionToPhase(PHASE_GAME_START);
}

void handleRobotTurnFrontPhase() {
  // Slave에게 로봇 정면 회전 명령
  sendCommandToSlave(CMD_ROBOT_BODY_FRONT, 0, 0);
  
  // 회전 완료 대기 (실제로는 Slave 응답 대기)
  delay(3000);
  
  transitionToPhase(PHASE_NARRATION);
}

void handleNarrationPhase() {
  // 환영 내레이션 재생
  playAudio(11);
  
  // 내레이션 재생 시간 대기
  delay(5000);
  
  transitionToPhase(PHASE_ROBOT_TURN_BACK);
}

void handleRobotTurnBackPhase() {
  // Slave에게 로봇 후면 회전 명령
  sendCommandToSlave(CMD_ROBOT_HEAD_BACK, 0, 0);
  
  // 회전 완료 대기
  delay(3000);
  
  transitionToPhase(PHASE_SHOW_PATTERN);
}

void handleShowPatternPhase() {
  // Slave에게 패턴 표시 명령
  uint8_t patternLength = getPatternLengthForLevel(gameState.currentLevel);
  sendCommandToSlave(CMD_SHOW_PATTERN, patternLength, 0);
  
  // 패턴 표시 시간 대기
  delay(PATTERN_DISPLAY_TIME);
  
  transitionToPhase(PHASE_PLAYER_INPUT);
}

void handlePlayerInputPhase() {
  // Slave에게 입력 요청
  sendCommandToSlave(CMD_GET_INPUT, 0, 0);
  
  // 입력 결과 대기 (실제로는 Slave 응답 처리)
  unsigned long startTime = millis();
  while (millis() - startTime < INPUT_TIMEOUT) {
    processSlaveComm();
    delay(10);
  }
  
  // 타임아웃 시 오답 처리
  transitionToPhase(PHASE_MOTION_DETECT);
}

void handleMotionDetectPhase() {
  // Slave에게 로봇 정면 회전 명령
  sendCommandToSlave(CMD_ROBOT_HEAD_FRONT, 0, 0);
  delay(1000);
  
  // 동작 감지 시작
  startMotionDetection();
  
  // 5초간 동작 감지
  unsigned long startTime = millis();
  while (millis() - startTime < MOTION_DETECT_DURATION) {
    readPIRSensors();
    delay(10);
  }
  
  // 동작 감지 종료
  stopMotionDetection();
  
  transitionToPhase(PHASE_SCORE_UPDATE);
}

void handleScoreUpdatePhase() {
  // 점수 업데이트
  updateScoreLEDs();
  
  // 레벨 체크
  if (gameState.currentScore >= 7) {
    transitionToPhase(PHASE_GAME_CLEAR);
  } else {
    // 다음 라운드
    gameState.currentLevel++;
    transitionToPhase(PHASE_ROBOT_TURN_BACK);
  }
}

void handleGameClearPhase() {
  Serial.println("=== 게임 클리어! ===");
  
  // 게임 클리어 효과음
  playAudio(12);
  
  // 모든 LED 점멸
  playSuccessEffect();
  
  // 게임 종료
  delay(5000);
  transitionToPhase(PHASE_IDLE);
}

// =============================================================================
// 유틸리티 함수들
// =============================================================================

uint8_t getPatternLengthForLevel(uint8_t level) {
  if (level <= 4) return level + 1;
  else if (level <= 10) return 5;
  else return 6;
}

void updateScoreLEDs() {
  // Slave에게 점수 업데이트 명령
  sendCommandToSlave(CMD_UPDATE_SCORE, gameState.currentScore, 0);
}

void playErrorEffect() {
  // 이펙트 LED 5회 점멸
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], HIGH);
    }
    delay(200);
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], LOW);
    }
    delay(200);
  }
}

void playSuccessEffect() {
  // 성공 효과 LED 패턴
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], HIGH);
      delay(100);
    }
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], LOW);
      delay(100);
    }
  }
}

// =============================================================================
// DFPlayer 관련 함수들
// =============================================================================

void sendDFCommand(uint8_t cmd, uint8_t param1, uint8_t param2) {
  uint8_t buffer[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, param1, param2, 0x00, 0x00, 0xEF};
  
  // 체크섬 계산
  uint16_t checksum = 0;
  for (int i = 1; i < 7; i++) {
    checksum += buffer[i];
  }
  checksum = 0 - checksum;
  
  buffer[7] = (checksum >> 8) & 0xFF;
  buffer[8] = checksum & 0xFF;
  
  // 명령 전송
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
  Serial.write(0xAA); // 헤더
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
  Serial.write(cmd ^ data1 ^ data2); // 체크섬
  
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
      
      // 체크섬 검증
      if (checksum == (response ^ data1 ^ data2)) {
        handleSlaveResponse(response, data1, data2);
      }
    }
  }
}

void handleSlaveResponse(uint8_t response, uint8_t data1, uint8_t data2) {
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
      transitionToPhase(PHASE_GAME_START);
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
  Serial.print("활성: ");
  Serial.println(gameState.isGameActive ? "예" : "아니오");
  Serial.println("================");
}
/
/ =============================================================================
// 누락된 함수 구현들
// =============================================================================

void sendDFCommand(uint8_t cmd, uint8_t param1, uint8_t param2) {
  uint8_t buffer[10] = {0x7E, 0xFF, 0x06, cmd, 0x00, param1, param2, 0x00, 0x00, 0xEF};
  
  // 체크섬 계산
  uint16_t checksum = 0;
  for (int i = 1; i < 7; i++) {
    checksum += buffer[i];
  }
  checksum = 0 - checksum;
  
  buffer[7] = (checksum >> 8) & 0xFF;
  buffer[8] = checksum & 0xFF;
  
  // 명령 전송
  for (int i = 0; i < 10; i++) {
    dfSerial.write(buffer[i]);
  }
}

void playAudio(uint8_t trackNumber) {
  sendDFCommand(0x03, 0, trackNumber);
  Serial.print("오디오 재생: 트랙 ");
  Serial.println(trackNumber);
}

void playErrorEffect() {
  // 이펙트 LED 5회 점멸
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], HIGH);
    }
    delay(200);
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], LOW);
    }
    delay(200);
  }
}

void playSuccessEffect() {
  // 성공 효과 LED 패턴
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], HIGH);
      delay(100);
    }
    for (int j = 0; j < 6; j++) {
      digitalWrite(EFFECT_PINS[j], LOW);
      delay(100);
    }
  }
}

void sendCommandToSlave(uint8_t cmd, uint8_t data1, uint8_t data2) {
  Serial.write(0xAA); // 헤더
  Serial.write(cmd);
  Serial.write(data1);
  Serial.write(data2);
  Serial.write(cmd ^ data1 ^ data2); // 체크섬
  
  Serial.print("Slave 명령 전송: 0x");
  Serial.println(cmd, HEX);
}

void handleSlaveResponse(uint8_t response, uint8_t data1, uint8_t data2) {
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
  }
}

uint8_t getPatternLengthForLevel(uint8_t level) {
  if (level <= 4) return level + 1;
  else if (level <= 10) return 5;
  else return 6;
}

void updateScoreLEDs() {
  // Slave에게 점수 업데이트 명령
  sendCommandToSlave(CMD_UPDATE_SCORE, gameState.currentScore, 0);
}

void printGameStatus() {
  Serial.println("=== 게임 상태 ===");
  Serial.print("단계: ");
  Serial.println(gameState.currentPhase);
  Serial.print("레벨: ");
  Serial.println(gameState.currentLevel);
  Serial.print("점수: ");
  Serial.println(gameState.currentScore);
  Serial.print("활성: ");
  Serial.println(gameState.isGameActive ? "예" : "아니오");
  Serial.println("================");
}