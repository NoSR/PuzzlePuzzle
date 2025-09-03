/*
 * 무궁화 게임 시스템 - Slave Board (깔끔한 최적화 버전)
 * Arduino Nano 기반
 */

#include "slave_config.h"
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// =============================================================================
// 하드웨어 핀 정의
// =============================================================================

#define HEAD_PWM_PIN 5
#define HEAD_DIR1_PIN 6
#define HEAD_DIR2_PIN 7
#define BODY_PWM_PIN 10
#define BODY_DIR1_PIN 8
#define BODY_DIR2_PIN 9

const uint8_t VIBRATION_PINS[4] = {A0, A1, A2, A3};

#define DF_RX_PIN 3
#define DF_TX_PIN 2

// =============================================================================
// 통신 명령어 (헤더에 없는 것들만)
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
// 전역 상태 변수들
// =============================================================================

MotorState motorState;
VibrationState vibrationState;
LEDPatternState ledPatternState;
ScoreLEDState scoreLEDState;
DigitalInputState digitalInputState;

// =============================================================================
// 전역 변수
// =============================================================================

SoftwareSerial dfSerial(DF_RX_PIN, DF_TX_PIN);

// 레벨별 패턴 정의
const uint8_t LEVEL_PATTERNS[][6] = {
    {1, 2, 0, 0, 0, 0}, // 레벨 1: 2개
    {1, 3, 2, 0, 0, 0}, // 레벨 2: 3개
    {2, 1, 4, 3, 0, 0}, // 레벨 3: 4개
    {3, 1, 4, 2, 1, 0}, // 레벨 4: 5개
    {2, 4, 1, 3, 2, 0}, // 레벨 5: 5개
    {1, 3, 2, 4, 1, 3}  // 레벨 6+: 6개
};

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
void showLEDPattern(uint8_t level);
void updateLEDPattern();
void displayScore(uint8_t score);
void startScoreBlinking(uint8_t score);
void updateScoreLEDBlinking();

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
void testDigitalInputs();

// =============================================================================
// Arduino 메인 함수들
// =============================================================================

void setup() {
  Serial.begin(9600);
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
  processMasterComm();
  monitorVibrationSensors();
  monitorDigitalInputs();
  updateLEDPattern();
  updateScoreLEDBlinking();
  updateClearDominoEffect();
  handleMotorBrake();
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

  stopAllMotors();
  Serial.println(F("핀 초기화 완료"));
}

void initializeMCP23017() {
  // MCP23017 #1 초기화
  // GPIOA: 출력 (문제 제시 LED용)
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_IODIRA);
  Wire.write(0x00); // 모든 비트 출력
  Wire.endTransmission();

  // GPIOB: 입력 (리미트 스위치 + 디지털 입력용)
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(MCP_IODIRB);
  Wire.write(0xFF); // 모든 비트 입력
  Wire.endTransmission();

  // GPIOB 풀업 활성화 (0x0D = GPPUB 레지스터)
  Wire.beginTransmission(MCP23017_1_ADDR);
  Wire.write(0x0D); // GPPUB 레지스터
  Wire.write(0xFF); // 모든 비트 풀업 활성화
  Wire.endTransmission();

  // MCP23017 #2 초기화 (점수 LED용)
  Wire.beginTransmission(MCP23017_2_ADDR);
  Wire.write(MCP_IODIRA);
  Wire.write(0x00); // 모든 비트 출력
  Wire.endTransmission();

  // 초기 상태 (모든 LED 끄기)
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);

  Serial.println(F("MCP23017 초기화 완료"));
}

void initializeDFPlayer() {
  delay(1000);
  sendDFCommand(0x06, 0, 20); // 볼륨 설정
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
    vibrationState.lastTriggerTime[i] = 0;
  }
  vibrationState.inputLength = 0;
  vibrationState.inputActive = false;
  vibrationState.padPressed = 0;

  // LED 패턴 상태 초기화
  ledPatternState.patternActive = false;
  ledPatternState.currentStep = 0;
  ledPatternState.patternLength = 0;
  ledPatternState.stepTimer = 0;
  ledPatternState.stepInterval = 500;

  // 점수 LED 상태 초기화
  scoreLEDState.currentScore = 0;
  scoreLEDState.timer = 0;
  scoreLEDState.blinkCount = 0;
  scoreLEDState.dominoStep = 0;
  scoreLEDState.dominoCycle = 0;
  scoreLEDState.flags = 0x04; // dominoDirection = true (bit2)

  // 디지털 입력 상태 초기화
  digitalInputState.currentState = 0;
  digitalInputState.previousState = 0;
  digitalInputState.lastReadTime = 0;
  for (int i = 0; i < DIGITAL_INPUT_COUNT; i++) {
    digitalInputState.debounceTime[i] = 0;
  }

  Serial.println(F("상태 초기화 완료"));
}

// =============================================================================
// 모터 제어 함수들
// =============================================================================

void rotateHeadForward() {
  digitalWrite(HEAD_DIR1_PIN, HIGH);
  digitalWrite(HEAD_DIR2_PIN, LOW);
  analogWrite(HEAD_PWM_PIN, MOTOR_SPEED_NORMAL);

  motorState.headActive = true;
  delay(MOTOR_180_DURATION);

  analogWrite(HEAD_PWM_PIN, 0);
  motorState.headActive = false;
}

void rotateHeadBackward() {
  digitalWrite(HEAD_DIR1_PIN, LOW);
  digitalWrite(HEAD_DIR2_PIN, HIGH);
  analogWrite(HEAD_PWM_PIN, MOTOR_SPEED_NORMAL);

  motorState.headActive = true;
  delay(MOTOR_180_DURATION);

  analogWrite(HEAD_PWM_PIN, 0);
  motorState.headActive = false;
}

void rotateBodyForward() {
  digitalWrite(BODY_DIR1_PIN, HIGH);
  digitalWrite(BODY_DIR2_PIN, LOW);
  analogWrite(BODY_PWM_PIN, MOTOR_SPEED_NORMAL);

  motorState.bodyActive = true;
  delay(MOTOR_180_DURATION);

  analogWrite(BODY_PWM_PIN, 0);
  motorState.bodyActive = false;
}

void rotateBodyBackward() {
  digitalWrite(BODY_DIR1_PIN, LOW);
  digitalWrite(BODY_DIR2_PIN, HIGH);
  analogWrite(BODY_PWM_PIN, MOTOR_SPEED_NORMAL);

  motorState.bodyActive = true;
  delay(MOTOR_180_DURATION);

  analogWrite(BODY_PWM_PIN, 0);
  motorState.bodyActive = false;
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
  if (!vibrationState.inputActive)
    return;

  unsigned long currentTime = millis();

  for (int i = 0; i < 4; i++) {
    uint16_t reading = analogRead(VIBRATION_PINS[i]);
    uint8_t padMask = 1 << i;

    if (reading > VIBRATION_THRESHOLD &&
        !(vibrationState.padPressed & padMask)) {
      if (currentTime - vibrationState.lastTriggerTime[i] >
          VIBRATION_DEBOUNCE) {
        vibrationState.padPressed |= padMask;
        vibrationState.lastTriggerTime[i] = currentTime;

        if (vibrationState.inputLength < 6) {
          vibrationState.inputSequence[vibrationState.inputLength] = i + 1;
          vibrationState.inputLength++;
          playEffectSound(SFX_PAD_PRESS);
        }
      }
    } else if (reading <= VIBRATION_THRESHOLD &&
               (vibrationState.padPressed & padMask)) {
      vibrationState.padPressed &= ~padMask;
    }
  }
}

void startInputCapture() {
  vibrationState.inputActive = true;
  vibrationState.inputLength = 0;

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

  for (int i = 0; i < expectedLength; i++) {
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

uint8_t readMCP23017Input(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0xFF; // 오류 시 모든 비트 HIGH 반환
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

  if (level <= 4) {
    ledPatternState.stepInterval = 500;
  } else {
    ledPatternState.stepInterval = 300;
  }
}

void updateLEDPattern() {
  if (!ledPatternState.patternActive)
    return;

  unsigned long currentTime = millis();

  if (currentTime - ledPatternState.stepTimer >= ledPatternState.stepInterval) {
    if (ledPatternState.currentStep < ledPatternState.patternLength) {
      uint8_t ledNumber = LEVEL_PATTERNS[0][ledPatternState.currentStep];

      // 모든 문제 LED 끄기 (비트 4,5,6,7만 클리어)
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);

      // 해당 LED 점등 (GPIOA 4,5,6,7번 비트 사용)
      // ledNumber 1~4를 비트 4~7로 매핑
      uint8_t ledMask = 1 << (PROBLEM_LED_START_BIT + ledNumber - 1);
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, ledMask);

      playEffectSound(SFX_PATTERN_BEEP);

      ledPatternState.currentStep++;
      ledPatternState.stepTimer = currentTime;

      Serial.print("문제 LED ");
      Serial.print(ledNumber);
      Serial.print(" 점등 (비트 ");
      Serial.print(PROBLEM_LED_START_BIT + ledNumber - 1);
      Serial.println(")");
    } else {
      ledPatternState.patternActive = false;
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);

      Serial.println("LED 패턴 표시 완료");
      sendResponseToMaster(RESP_PATTERN_DONE);
    }
  }
}

void displayScore(uint8_t score) {
  if (score >= 7) {
    startClearDominoEffect();
    return;
  }

  scoreLEDState.currentScore = score;

  uint8_t ledMask = 0;
  for (int i = 0; i < score && i < SCORE_LED_COUNT; i++) {
    ledMask |= (1 << (SCORE_LED_START_BIT + i));
  }

  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, ledMask);
}

void startScoreBlinking(uint8_t score) {
  scoreLEDState.currentScore = score;
  scoreLEDState.flags |= 0x01; // blinking = true
  scoreLEDState.timer = millis();
  scoreLEDState.blinkCount = 0;
}

void updateScoreLEDBlinking() {
  if (!(scoreLEDState.flags & 0x01)) // blinking check
    return;

  unsigned long currentTime = millis();

  if (currentTime - scoreLEDState.timer >= 500) {
    if (scoreLEDState.blinkCount < 7) {
      if (scoreLEDState.blinkCount % 2 == 0) {
        displayScore(scoreLEDState.currentScore);
      } else {
        setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
      }
      scoreLEDState.blinkCount++;
      scoreLEDState.timer = currentTime;
    } else {
      scoreLEDState.flags &= ~0x01; // blinking = false
      displayScore(scoreLEDState.currentScore);
    }
  }
}

void startClearDominoEffect() {
  scoreLEDState.flags |= 0x02; // dominoActive = true
  scoreLEDState.flags |= 0x04; // dominoDirection = true (오름차순)
  scoreLEDState.dominoStep = 0;
  scoreLEDState.dominoCycle = 0;
  scoreLEDState.timer = millis();
  setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
}

void updateClearDominoEffect() {
  if (!(scoreLEDState.flags & 0x02)) // dominoActive check
    return;

  unsigned long currentTime = millis();

  if (currentTime - scoreLEDState.timer >= CLEAR_DOMINO_INTERVAL) {
    uint8_t currentLED = (scoreLEDState.flags & 0x04)
                             ? scoreLEDState.dominoStep
                             : (SCORE_LED_COUNT - 1) - scoreLEDState.dominoStep;

    uint8_t ledMask = 1 << (SCORE_LED_START_BIT + currentLED);
    setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, ledMask);

    scoreLEDState.dominoStep++;
    scoreLEDState.timer = currentTime;

    if (scoreLEDState.dominoStep >= SCORE_LED_COUNT) {
      scoreLEDState.dominoStep = 0;

      if (scoreLEDState.flags & 0x04) {
        scoreLEDState.flags &= ~0x04; // dominoDirection = false
      } else {
        scoreLEDState.flags |= 0x04; // dominoDirection = true
        scoreLEDState.dominoCycle++;

        if (scoreLEDState.dominoCycle >= CLEAR_DOMINO_CYCLES) {
          scoreLEDState.flags &= ~0x02; // dominoActive = false
          setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, SCORE_LED_MASK);
          return;
        }
      }
      setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
      delay(100);
    }
  }
}

// =============================================================================
// 디지털 입력 관련 함수들
// =============================================================================

void monitorDigitalInputs() {
  unsigned long currentTime = millis();

  // 10ms마다 입력 상태 읽기
  if (currentTime - digitalInputState.lastReadTime >= 10) {
    uint8_t rawInput = readMCP23017Input(MCP23017_1_ADDR, MCP_GPIOB);

    // 디바운싱 처리
    for (int i = 0; i < DIGITAL_INPUT_COUNT + 2;
         i++) { // +2는 리미트 스위치 포함
      uint8_t bitMask = 1 << i;
      bool currentBit = !(rawInput & bitMask); // 풀업이므로 반전
      bool previousBit = digitalInputState.currentState & bitMask;

      if (currentBit != previousBit) {
        if (i < DIGITAL_INPUT_COUNT + 2) {
          digitalInputState.debounceTime[i] = currentTime + 50; // 50ms 디바운스
        }
      } else if (currentTime >= digitalInputState.debounceTime[i]) {
        if (currentBit) {
          digitalInputState.currentState |= bitMask;
        } else {
          digitalInputState.currentState &= ~bitMask;
        }
      }
    }

    digitalInputState.lastReadTime = currentTime;
  }
}

bool readHeadLimitSwitch() {
  return digitalInputState.currentState & (1 << HEAD_LIMIT_BIT);
}

bool readBodyLimitSwitch() {
  return digitalInputState.currentState & (1 << BODY_LIMIT_BIT);
}

bool readDigitalInput(uint8_t inputNumber) {
  if (inputNumber >= DIGITAL_INPUT_COUNT)
    return false;

  uint8_t bitPosition = DIGITAL_INPUT_START_BIT + inputNumber;
  return digitalInputState.currentState & (1 << bitPosition);
}

uint8_t readAllDigitalInputs() {
  // GPIOB 2~6번 비트만 추출하여 0~4번 비트로 시프트
  return (digitalInputState.currentState & DIGITAL_INPUT_MASK) >>
         DIGITAL_INPUT_START_BIT;
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
  }
}

void handleInputRequest() {
  Serial.println("입력 요청 수신");

  startInputCapture();

  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {
    monitorVibrationSensors();

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
  Serial.write(0xAA);
  Serial.write(response);
  Serial.write(0x00);
  Serial.write(0x00);
  Serial.write(response);

  Serial.print("Master 응답 전송: 0x");
  Serial.println(response, HEX);
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
    } else if (command == "input_test") {
      testDigitalInputs();
    } else if (command == "stop") {
      stopAllMotors();
      setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);
      setMCP23017Output(MCP23017_2_ADDR, MCP_GPIOA, 0x00);
    }
  }
}

void printSystemStatus() {
  Serial.print(F("H:"));
  Serial.print(motorState.headActive);
  Serial.print(F(" B:"));
  Serial.print(motorState.bodyActive);
  Serial.print(F(" I:"));
  Serial.print(vibrationState.inputActive);
  Serial.print(F(" L:"));
  Serial.print(vibrationState.inputLength);
  Serial.print(F(" S:"));
  Serial.print(scoreLEDState.currentScore);
  Serial.print(F(" HL:"));
  Serial.print(readHeadLimitSwitch());
  Serial.print(F(" BL:"));
  Serial.print(readBodyLimitSwitch());
  Serial.print(F(" DI:0x"));
  Serial.println(readAllDigitalInputs(), HEX);
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

  // 문제 제시 LED 테스트 (GPIOA 4,5,6,7번 비트)
  Serial.println("문제 LED 테스트 시작 (4개)");
  for (int i = 0; i < PROBLEM_LED_COUNT; i++) {
    uint8_t ledMask = 1 << (PROBLEM_LED_START_BIT + i);
    setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, ledMask);
    Serial.print("문제 LED ");
    Serial.print(i + 1);
    Serial.print(" 점등 (비트 ");
    Serial.print(PROBLEM_LED_START_BIT + i);
    Serial.println(")");
    delay(300);
  }
  setMCP23017Output(MCP23017_1_ADDR, MCP_GPIOA, 0x00);

  // 점수 LED 테스트 (GPIOA 0,1,2,3,4,5번 비트)
  Serial.println("점수 LED 테스트 시작 (6개)");
  for (int i = 1; i <= SCORE_LED_COUNT; i++) {
    displayScore(i);
    delay(500);
  }

  // 7점 클리어 도미노 효과 테스트
  Serial.println("7점 클리어 도미노 효과 테스트");
  displayScore(7);

  // 도미노 효과가 완료될 때까지 대기
  while (scoreLEDState.flags & 0x02) { // dominoActive check
    updateClearDominoEffect();
    delay(10);
  }

  delay(2000);     // 2초 대기
  displayScore(0); // 모든 LED 끄기

  Serial.println("LED 테스트 완료");
}

void testDigitalInputs() {
  Serial.println(F("=== 디지털 입력 테스트 ==="));
  Serial.println(F("리미트 스위치와 디지털 입력을 테스트합니다."));
  Serial.println(
      F("입력을 변경하고 상태를 확인하세요. 'q'를 입력하면 종료합니다."));

  while (true) {
    // 입력 상태 업데이트
    monitorDigitalInputs();

    // 상태 출력
    Serial.print(F("Head Limit: "));
    Serial.print(readHeadLimitSwitch() ? F("ON") : F("OFF"));
    Serial.print(F(" | Body Limit: "));
    Serial.print(readBodyLimitSwitch() ? F("ON") : F("OFF"));
    Serial.print(F(" | Digital Inputs: "));

    for (int i = 0; i < DIGITAL_INPUT_COUNT; i++) {
      Serial.print(F("D"));
      Serial.print(i);
      Serial.print(F(":"));
      Serial.print(readDigitalInput(i) ? F("1") : F("0"));
      if (i < DIGITAL_INPUT_COUNT - 1)
        Serial.print(F(" "));
    }

    Serial.print(F(" | Raw: 0x"));
    Serial.println(digitalInputState.currentState, HEX);

    // 종료 체크
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n');
      if (input.indexOf('q') >= 0) {
        break;
      }
    }

    delay(200);
  }

  Serial.println(F("디지털 입력 테스트 완료"));
}