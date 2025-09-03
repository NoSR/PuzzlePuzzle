/*
 * 무궁화 게임 시스템 - Slave Board (최적화 버전)
 * Arduino Nano 기반
 *
 * 핵심 기능:
 * - 로봇 Head/Body 모터 제어
 * - 바닥 발판 입력 처리 (진동 센서)
 * - 문제 제시 LED 및 점수 LED 제어 (MCP23017)
 * - 효과음 재생
 * - Master Board와의 시리얼 통신
 */

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// =============================================================================
// 하드웨어 핀 정의
// =============================================================================

// 모터 제어 핀 (L298N)
#define HEAD_PWM_PIN 3
#define HEAD_DIR1_PIN 4
#define HEAD_DIR2_PIN 5
#define BODY_PWM_PIN 6
#define BODY_DIR1_PIN 7
#define BODY_DIR2_PIN 8

// 진동 센서 핀 (아날로그)
const uint8_t VIBRATION_PINS[4] = {A0, A1, A2, A3};

// DFPlayer 핀
#define DF_RX_PIN 10
#define DF_TX_PIN 11

// 리미트 스위치 핀
#define HEAD_LIMIT_PIN 12
#define BODY_LIMIT_PIN 13

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
#define MOTOR_SPEED_NORMAL 150
#define MOTOR_180_DURATION 3000
#define MOTOR_BRAKE_DURATION 5000

// 진동 센서 상수
#define VIBRATION_THRESHOLD 512
#define VIBRATION_DEBOUNCE 200

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

// 효과음 ID
#define SFX_LIGHT_ON 1
#define SFX_PATTERN_BEEP 2
#define SFX_PAD_PRESS 3
#define SFX_CORRECT 4
#define SFX_WRONG 5

// =============================================================================
// 상태 구조체
// =============================================================================

struct MotorState {
  bool headActive;
  bool bodyActive;
  unsigned long stopTime;
  bool brakeActive;
};

struct VibrationState {
  uint16_t readings[4];
  unsigned long lastTriggerTime[4];
  bool padPressed[4];
  uint8_t inputSequence[6];
  uint8_t inputLength;
  bool inputActive;
};

struct LEDPatternState {
  bool patternActive;
  uint8_t currentStep;
  uint8_t patternLength;
  unsigned long stepTimer;
  uint16_t stepInterval;
};

struct ScoreLEDState {
  uint8_t currentScore;
  bool blinking;
  unsigned long blinkTimer;
  uint8_t blinkCount;
};

// =============================================================================
// 전역 변수
// =============================================================================

MotorState motorState;
VibrationState vibrationState;
LEDPatternState ledPatternState;
ScoreLEDState scoreLEDState;

SoftwareSerial dfSerial(DF_RX_PIN, DF_TX_PIN);

// 레벨별 패턴 정의
const uint8_t LEVEL_PATTERNS[][6] = {
  {1, 2, 0, 0, 0, 0},           // 레벨 1: 2개
  {1, 3, 2, 0, 0, 0},           // 레벨 2: 3개
  {2, 1, 4, 3, 0, 0},           // 레벨 3: 4개
  {3, 1, 4, 2, 1, 0},           // 레벨 4: 5개
  {2, 4, 1, 3, 2, 0},           // 레벨 5: 5개
  {1, 3, 2, 4, 1, 3}            // 레벨 6+: 6개
};

// =============================================================================
// Arduino 메인 함수들
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  dfSerial.begin(9600);
  Wire.begin();
  
  Serial.println("=== Slave Board 시작 ===");
  
  initializePins();
  initializeMCP23017();
  initializeDFPlayer();
  initializeStates();
  
  Serial.println("Slave Board 초기화 완료");
}

void loop() {
  // Master 명령 처리
  processMasterComm();
  
  // 진동 센서 모니터링
  monitorVibrationSensors();
  
  // LED 패턴 업데이트
  updateLEDPattern();
  
  // 점수 LED 점멸 업데이트
  updateScoreLEDBlinking();
  
  // 모터 브레이크 처리
  handleMotorBrake();
  
  // 시리얼 명령 처리
  processSerialCommands();
  
  delay(10);
}

// =============================================================================
// 초기화 함수들
// =============================================================================

void initializePins() {
  // 모터 제어 핀 설정
  pinMode(HEAD_PWM_PIN, OUTPUT);
  pinMode(HEAD_DIR1_PIN, OUTPUT);
  pinMode(HEAD_DIR2_PIN, OUTPUT);
  pinMode(BODY_PWM_PIN, OUTPUT);
  pinMode(BODY_DIR1_PIN, OUTPUT);
  pinMode(BODY_DIR2_PIN, OUTPUT);
  
  // 리미트 스위치 핀 설정
  pinMode(HEAD_LIMIT_PIN, INPUT_PULLUP);
  pinMode(BODY_LIMIT_PIN, INPUT_PULLUP);
  
  // 모터 초기 상태
  stopAllMotors();
  
  Serial.println("핀 초기화 완료");
}

void initializeMCP23017() {
  // MCP23017 #1 초기화 (문제 제시 LED용)
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_IODIRA);
  Wire.write(0x00); // 모든 핀을 출력으로 설정
  Wire.endTransmission();
  
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_IODIRB);
  Wire.write(0x00); // 모든 핀을 출력으로 설정
  Wire.endTransmission();
  
  // MCP23017 #2 초기화 (점수 LED용)
  Wire.beginTransmission(MCP23017_2_ADDR);
  Wire.write(MCP_IODIRA);
  Wire.write(0x00); // 모든 핀을 출력으로 설정
  Wire.endTransmission();
  
  // 초기 상태 (모든 LED 끄기)
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOB, 0x00);
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
  
  Serial.println("MCP23017 초기화 완료");
}

void initializeDFPlayer() {
  delay(1000); // DFPlayer 안정화 대기
  
  // 볼륨 설정
  sendDFCommand(0x06, 0, 20);
  delay(100);
  
  Serial.println("DFPlayer 초기화 완료");
}

void initializeStates() {
  // 모터 상태 초기화
  motorState.headActive = false;
  motorState.bodyActive = false;
  motorState.stopTime = 0;
  motorState.brakeActive = false;
  
  // 진동 센서 상태 초기화
  for (int i = 0; i < 4; i++) {
    vibrationState.readings[i] = 0;
    vibrationState.lastTriggerTime[i] = 0;
    vibrationState.padPressed[i] = false;
  }
  vibrationState.inputLength = 0;
  vibrationState.inputActive = false;
  
  // LED 패턴 상태 초기화
  ledPatternState.patternActive = false;
  ledPatternState.currentStep = 0;
  ledPatternState.patternLength = 0;
  ledPatternState.stepTimer = 0;
  ledPatternState.stepInterval = 500;
  
  // 점수 LED 상태 초기화
  scoreLEDState.currentScore = 0;
  scoreLEDState.blinking = false;
  scoreLEDState.blinkTimer = 0;
  scoreLEDState.blinkCount = 0;
  
  Serial.println("상태 초기화 완료");
}

// =============================================================================
// 모터 제어 함수들
// =============================================================================

void rotateHeadForward() {
  Serial.println("Head 모터 정방향 회전");
  
  digitalWrite(HEAD_DIR1_PIN, HIGH);
  digitalWrite(HEAD_DIR2_PIN, LOW);
  analogWrite(HEAD_PWM_PIN, MOTOR_SPEED_NORMAL);
  
  motorState.headActive = true;
  
  // 180도 회전 시간 대기
  delay(MOTOR_180_DURATION);
  
  // 모터 정지
  analogWrite(HEAD_PWM_PIN, 0);
  motorState.headActive = false;
  
  Serial.println("Head 모터 회전 완료");
}

void rotateHeadBackward() {
  Serial.println("Head 모터 역방향 회전");
  
  digitalWrite(HEAD_DIR1_PIN, LOW);
  digitalWrite(HEAD_DIR2_PIN, HIGH);
  analogWrite(HEAD_PWM_PIN, MOTOR_SPEED_NORMAL);
  
  motorState.headActive = true;
  
  // 180도 회전 시간 대기
  delay(MOTOR_180_DURATION);
  
  // 모터 정지
  analogWrite(HEAD_PWM_PIN, 0);
  motorState.headActive = false;
  
  Serial.println("Head 모터 회전 완료");
}

void rotateBodyForward() {
  Serial.println("Body 모터 정방향 회전");
  
  digitalWrite(BODY_DIR1_PIN, HIGH);
  digitalWrite(BODY_DIR2_PIN, LOW);
  analogWrite(BODY_PWM_PIN, MOTOR_SPEED_NORMAL);
  
  motorState.bodyActive = true;
  
  // 180도 회전 시간 대기
  delay(MOTOR_180_DURATION);
  
  // 모터 정지
  analogWrite(BODY_PWM_PIN, 0);
  motorState.bodyActive = false;
  
  Serial.println("Body 모터 회전 완료");
}

void rotateBodyBackward() {
  Serial.println("Body 모터 역방향 회전");
  
  digitalWrite(BODY_DIR1_PIN, LOW);
  digitalWrite(BODY_DIR2_PIN, HIGH);
  analogWrite(BODY_PWM_PIN, MOTOR_SPEED_NORMAL);
  
  motorState.bodyActive = true;
  
  // 180도 회전 시간 대기
  delay(MOTOR_180_DURATION);
  
  // 모터 정지
  analogWrite(BODY_PWM_PIN, 0);
  motorState.bodyActive = false;
  
  Serial.println("Body 모터 회전 완료");
}

void stopAllMotors() {
  analogWrite(HEAD_PWM_PIN, 0);
  analogWrite(BODY_PWM_PIN, 0);
  
  // 브레이크 적용
  digitalWrite(HEAD_DIR1_PIN, HIGH);
  digitalWrite(HEAD_DIR2_PIN, HIGH);
  digitalWrite(BODY_DIR1_PIN, HIGH);
  digitalWrite(BODY_DIR2_PIN, HIGH);
  
  motorState.headActive = false;
  motorState.bodyActive = false;
  motorState.stopTime = millis() + MOTOR_BRAKE_DURATION;
  motorState.brakeActive = true;
  
  Serial.println("모든 모터 정지 및 브레이크 적용");
}

void handleMotorBrake() {
  if (motorState.brakeActive && millis() >= motorState.stopTime) {
    // 브레이크 해제
    digitalWrite(HEAD_DIR1_PIN, LOW);
    digitalWrite(HEAD_DIR2_PIN, LOW);
    digitalWrite(BODY_DIR1_PIN, LOW);
    digitalWrite(BODY_DIR2_PIN, LOW);
    
    motorState.brakeActive = false;
    Serial.println("모터 브레이크 해제");
  }
}

// =============================================================================
// 진동 센서 관련 함수들
// =============================================================================

void monitorVibrationSensors() {
  if (!vibrationState.inputActive) return;
  
  unsigned long currentTime = millis();
  
  for (int i = 0; i < 4; i++) {
    uint16_t reading = analogRead(VIBRATION_PINS[i]);
    
    if (reading > VIBRATION_THRESHOLD && !vibrationState.padPressed[i]) {
      // 디바운싱 체크
      if (currentTime - vibrationState.lastTriggerTime[i] > VIBRATION_DEBOUNCE) {
        vibrationState.padPressed[i] = true;
        vibrationState.lastTriggerTime[i] = currentTime;
        
        // 입력 순서에 추가
        if (vibrationState.inputLength < 6) {
          vibrationState.inputSequence[vibrationState.inputLength] = i + 1;
          vibrationState.inputLength++;
          
          Serial.print("발판 ");
          Serial.print(i + 1);
          Serial.println(" 입력");
          
          // 효과음 재생
          playEffectSound(SFX_PAD_PRESS);
        }
      }
    } else if (reading <= VIBRATION_THRESHOLD && vibrationState.padPressed[i]) {
      vibrationState.padPressed[i] = false;
    }
  }
}

void startInputCapture() {
  vibrationState.inputActive = true;
  vibrationState.inputLength = 0;
  
  // 입력 순서 초기화
  for (int i = 0; i < 6; i++) {
    vibrationState.inputSequence[i] = 0;
  }
  
  Serial.println("입력 캡처 시작");
}

void stopInputCapture() {
  vibrationState.inputActive = false;
  Serial.println("입력 캡처 종료");
}

bool checkInputSequence(uint8_t level) {
  uint8_t expectedLength = getPatternLengthForLevel(level);
  
  if (vibrationState.inputLength != expectedLength) {
    return false;
  }
  
  // 패턴 비교
  for (int i = 0; i < expectedLength; i++) {
    if (level < 1 || level > 6 || i >= 6) return false; // 배열 경계 검사
    if (vibrationState.inputSequence[i] != LEVEL_PATTERNS[level - 1][i]) {
      return false;
    }
  }
  
  return true;
}

// =============================================================================
// LED 제어 함수들
// =============================================================================

void setMCP23017Output(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void showLEDPattern(uint8_t level) {
  uint8_t patternLength = getPatternLengthForLevel(level);
  
  Serial.print("LED 패턴 표시 - 레벨 ");
  Serial.print(level);
  Serial.print(", 길이 ");
  Serial.println(patternLength);
  
  ledPatternState.patternActive = true;
  ledPatternState.currentStep = 0;
  ledPatternState.patternLength = patternLength;
  ledPatternState.stepTimer = millis();
  
  // 레벨에 따른 속도 조정
  if (level <= 4) {
    ledPatternState.stepInterval = 500;
  } else {
    ledPatternState.stepInterval = 300;
  }
}

void updateLEDPattern() {
  if (!ledPatternState.patternActive) return;
  
  unsigned long currentTime = millis();
  
  if (currentTime - ledPatternState.stepTimer >= ledPatternState.stepInterval) {
    if (ledPatternState.currentStep < ledPatternState.patternLength) {
      // 현재 단계 LED 점등
      uint8_t ledNumber = LEVEL_PATTERNS[0][ledPatternState.currentStep]; // 임시로 레벨 1 패턴 사용
      
      // 모든 LED 끄기
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
      
      // 해당 LED 점등
      uint8_t ledMask = 1 << (ledNumber - 1);
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, ledMask);
      
      // 비프음 재생
      playEffectSound(SFX_PATTERN_BEEP);
      
      ledPatternState.currentStep++;
      ledPatternState.stepTimer = currentTime;
      
      Serial.print("LED ");
      Serial.print(ledNumber);
      Serial.println(" 점등");
    } else {
      // 패턴 표시 완료
      ledPatternState.patternActive = false;
      
      // 모든 LED 끄기
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
      
      Serial.println("LED 패턴 표시 완료");
      
      // Master에게 완료 응답
      sendResponseToMaster(RESP_PATTERN_DONE);
    }
  }
}

void displayScore(uint8_t score) {
  if (score > 7) score = 7;
  
  scoreLEDState.currentScore = score;
  
  // 점수에 따른 LED 패턴 (7개 LED)
  uint8_t ledMask = 0;
  for (int i = 0; i < score; i++) {
    ledMask |= (1 << i);
  }
  
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, ledMask);
  
  Serial.print("점수 LED 표시: ");
  Serial.println(score);
}

void startScoreBlinking(uint8_t score) {
  scoreLEDState.currentScore = score;
  scoreLEDState.blinking = true;
  scoreLEDState.blinkTimer = millis();
  scoreLEDState.blinkCount = 0;
  
  Serial.print("점수 LED 점멸 시작: ");
  Serial.println(score);
}

void updateScoreLEDBlinking() {
  if (!scoreLEDState.blinking) return;
  
  unsigned long currentTime = millis();
  
  if (currentTime - scoreLEDState.blinkTimer >= 500) {
    if (scoreLEDState.blinkCount < 7) {
      // 점멸 토글
      if (scoreLEDState.blinkCount % 2 == 0) {
        displayScore(scoreLEDState.currentScore);
      } else {
        setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
      }
      
      scoreLEDState.blinkCount++;
      scoreLEDState.blinkTimer = currentTime;
    } else {
      // 점멸 완료
      scoreLEDState.blinking = false;
      displayScore(scoreLEDState.currentScore);
      
      Serial.println("점수 LED 점멸 완료");
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

void playEffectSound(uint8_t soundId) {
  sendDFCommand(0x03, 0, soundId);
  Serial.print("효과음 재생: ");
  Serial.println(soundId);
}

// =============================================================================
// 통신 관련 함수들
// =============================================================================

void processMasterComm() {
  if (Serial.available() >= 5) {
    uint8_t header = Serial.read();
    if (header == 0xAA) {
      uint8_t command = Serial.read();
      uint8_t data1 = Serial.read();
      uint8_t data2 = Serial.read();
      uint8_t checksum = Serial.read();
      
      // 체크섬 검증
      if (checksum == (command ^ data1 ^ data2)) {
        handleMasterCommand(command, data1, data2);
      }
    }
  }
}

void handleMasterCommand(uint8_t command, uint8_t data1, uint8_t data2) {
  switch (command) {
    case CMD_ROBOT_HEAD_FRONT:
      rotateHeadForward();
      sendResponseToMaster(RESP_ROBOT_READY);
      break;
      
    case CMD_ROBOT_HEAD_BACK:
      rotateHeadBackward();
      sendResponseToMaster(RESP_ROBOT_READY);
      break;
      
    case CMD_ROBOT_BODY_FRONT:
      rotateBodyForward();
      sendResponseToMaster(RESP_ROBOT_READY);
      break;
      
    case CMD_ROBOT_BODY_BACK:
      rotateBodyBackward();
      sendResponseToMaster(RESP_ROBOT_READY);
      break;
      
    case CMD_SHOW_PATTERN:
      showLEDPattern(data1);
      break;
      
    case CMD_UPDATE_SCORE:
      if (data2 == 1) {
        startScoreBlinking(data1);
      } else {
        displayScore(data1);
      }
      break;
      
    case CMD_PLAY_EFFECT:
      playEffectSound(data1);
      break;
      
    case CMD_GET_INPUT:
      handleInputRequest();
      break;
      
    default:
      Serial.println("알 수 없는 명령어");
      break;
  }
}

void handleInputRequest() {
  Serial.println("입력 요청 수신");
  
  startInputCapture();
  
  // 10초간 입력 대기
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    monitorVibrationSensors();
    
    // 입력이 완료되었는지 체크 (예: 3초간 추가 입력 없음)
    if (vibrationState.inputLength > 0) {
      bool inputComplete = true;
      for (int i = 0; i < 4; i++) {
        if (millis() - vibrationState.lastTriggerTime[i] < 3000) {
          inputComplete = false;
          break;
        }
      }
      
      if (inputComplete) {
        break;
      }
    }
    
    delay(10);
  }
  
  stopInputCapture();
  
  // 입력 결과 검증 (임시로 레벨 1로 고정)
  if (vibrationState.inputLength == 0) {
    sendResponseToMaster(RESP_INPUT_TIMEOUT);
  } else if (checkInputSequence(1)) {
    sendResponseToMaster(RESP_INPUT_CORRECT);
    playEffectSound(SFX_CORRECT);
  } else {
    sendResponseToMaster(RESP_INPUT_WRONG);
    playEffectSound(SFX_WRONG);
  }
}

void sendResponseToMaster(uint8_t response) {
  Serial.write(0xAA); // 헤더
  Serial.write(response);
  Serial.write(0x00); // data1
  Serial.write(0x00); // data2
  Serial.write(response); // 체크섬 (간단화)
  
  Serial.print("Master 응답 전송: 0x");
  Serial.println(response, HEX);
}

// =============================================================================
// 유틸리티 함수들
// =============================================================================

uint8_t getPatternLengthForLevel(uint8_t level) {
  if (level <= 4) return level + 1;
  else if (level <= 10) return 5;
  else return 6;
}

// =============================================================================
// 시리얼 명령 처리
// =============================================================================

void processSerialCommands() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "status") {
      printSystemStatus();
    } else if (command == "motor_test") {
      testMotorSystem();
    } else if (command == "led_test") {
      testLEDSystem();
    } else if (command == "stop") {
      stopAllMotors();
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
      setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
    }
  }
}

void printSystemStatus() {
  Serial.println("=== Slave 시스템 상태 ===");
  Serial.print("Head 모터: ");
  Serial.println(motorState.headActive ? "활성" : "비활성");
  Serial.print("Body 모터: ");
  Serial.println(motorState.bodyActive ? "활성" : "비활성");
  Serial.print("입력 활성: ");
  Serial.println(vibrationState.inputActive ? "예" : "아니오");
  Serial.print("입력 길이: ");
  Serial.println(vibrationState.inputLength);
  Serial.print("현재 점수: ");
  Serial.println(scoreLEDState.currentScore);
  Serial.println("========================");
}

void testMotorSystem() {
  Serial.println("=== 모터 시스템 테스트 ===");
  
  Serial.println("Head 모터 정방향...");
  rotateHeadForward();
  delay(1000);
  
  Serial.println("Head 모터 역방향...");
  rotateHeadBackward();
  delay(1000);
  
  Serial.println("Body 모터 정방향...");
  rotateBodyForward();
  delay(1000);
  
  Serial.println("Body 모터 역방향...");
  rotateBodyBackward();
  delay(1000);
  
  Serial.println("모터 테스트 완료");
}

void testLEDSystem() {
  Serial.println("=== LED 시스템 테스트 ===");
  
  // 문제 제시 LED 테스트
  for (int i = 0; i < 8; i++) {
    setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 1 << i);
    delay(300);
  }
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
  
  // 점수 LED 테스트
  for (int i = 0; i <= 7; i++) {
    displayScore(i);
    delay(500);
  }
  displayScore(0);
  
  Serial.println("LED 테스트 완료");
}