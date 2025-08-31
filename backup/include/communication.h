/*
 * 공통 통신 프로토콜 헤더 파일
 * Master-Slave 간 시리얼 통신 프로토콜 정의
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>

// =============================================================================
// 통신 설정
// =============================================================================

#define SERIAL_BAUD_RATE      9600    // 시리얼 통신 속도
#define COMM_TIMEOUT          1000    // 통신 타임아웃 (ms)
#define MAX_RETRY_COUNT       3       // 최대 재시도 횟수
#define MESSAGE_HEADER        0xAA    // 메시지 헤더

// =============================================================================
// 명령어 정의
// =============================================================================

// Master → Slave 명령어
#define CMD_ROBOT_HEAD_FRONT    0x10  // 로봇 HEAD 정면 회전
#define CMD_ROBOT_HEAD_BACK     0x11  // 로봇 HEAD 후면 회전
#define CMD_ROBOT_BODY_FRONT    0x12  // 로봇 BODY 정면 회전
#define CMD_ROBOT_BODY_BACK     0x13  // 로봇 BODY 후면 회전
#define CMD_ROBOT_STOP          0x14  // 모든 모터 정지

#define CMD_SHOW_PATTERN        0x20  // LED 패턴 표시
#define CMD_CLEAR_PATTERN       0x21  // LED 패턴 지우기
#define CMD_UPDATE_SCORE        0x22  // 점수 LED 업데이트

#define CMD_PLAY_EFFECT         0x30  // 효과음 재생
#define CMD_STOP_AUDIO          0x31  // 오디오 정지

#define CMD_GET_STATUS          0x40  // 상태 요청
#define CMD_GET_INPUT           0x41  // 입력 상태 요청
#define CMD_GET_LIMIT           0x42  // 리미트 스위치 상태 요청

#define CMD_RESET               0xF0  // 시스템 리셋
#define CMD_STOP_ALL            0xFF  // 모든 동작 정지

// =============================================================================
// 응답 코드 정의
// =============================================================================

// Slave → Master 응답
#define RESP_ACK                0x01  // 명령 수신 확인
#define RESP_NACK               0x02  // 명령 수신 실패
#define RESP_BUSY               0x03  // 시스템 사용 중

#define RESP_ROBOT_READY        0x10  // 로봇 회전 완료
#define RESP_ROBOT_TIMEOUT      0x11  // 로봇 회전 타임아웃
#define RESP_ROBOT_ERROR        0x12  // 로봇 제어 오류
#define RESP_ROBOT_FRONT_COMPLETE 0x13  // 로봇 정면 회전 완료
#define RESP_ROBOT_BACK_COMPLETE  0x14  // 로봇 후면 회전 완료

#define RESP_PATTERN_DONE       0x20  // 패턴 표시 완료
#define RESP_PATTERN_ERROR      0x21  // 패턴 표시 오류

#define RESP_INPUT_DETECTED     0x30  // 입력 감지됨
#define RESP_INPUT_CORRECT      0x31  // 올바른 입력
#define RESP_INPUT_WRONG        0x32  // 잘못된 입력
#define RESP_INPUT_TIMEOUT      0x33  // 입력 타임아웃

#define RESP_LIMIT_HEAD_ON      0x40  // HEAD 리미트 스위치 ON
#define RESP_LIMIT_HEAD_OFF     0x41  // HEAD 리미트 스위치 OFF
#define RESP_LIMIT_BODY_ON      0x42  // BODY 리미트 스위치 ON
#define RESP_LIMIT_BODY_OFF     0x43  // BODY 리미트 스위치 OFF

#define RESP_AUDIO_DONE         0x50  // 오디오 재생 완료
#define RESP_AUDIO_ERROR        0x51  // 오디오 재생 오류

#define RESP_ERROR              0xFE  // 일반 오류
#define RESP_UNKNOWN            0xFF  // 알 수 없는 명령

// =============================================================================
// 데이터 코드 정의
// =============================================================================

// LED 패턴 데이터
#define PATTERN_LED_1           0x01
#define PATTERN_LED_2           0x02
#define PATTERN_LED_3           0x04
#define PATTERN_LED_4           0x08
#define PATTERN_ALL_OFF         0x00
#define PATTERN_ALL_ON          0x0F

// 효과음 데이터 (slave_config.h에서 정의됨 - 중복 방지)
#ifndef SFX_LIGHT_ON
#define SFX_LIGHT_ON            0x01
#define SFX_PATTERN_BEEP        0x02
#define SFX_PAD_PRESS           0x03
#define SFX_CORRECT             0x04
#define SFX_WRONG               0x05
#define SFX_GUNSHOT             0x06
#define SFX_SUCCESS             0x07
#define SFX_LEVEL_UP            0x08
#endif

// 점수 데이터 (0-7)
#define SCORE_0                 0x00
#define SCORE_1                 0x01
#define SCORE_2                 0x02
#define SCORE_3                 0x03
#define SCORE_4                 0x04
#define SCORE_5                 0x05
#define SCORE_6                 0x06
#define SCORE_7                 0x07

// 입력 패드 데이터
#define PAD_1                   0x01
#define PAD_2                   0x02
#define PAD_3                   0x04
#define PAD_4                   0x08
#define PAD_NONE                0x00

// =============================================================================
// 통신 메시지 구조체
// =============================================================================

struct SerialMessage {
  uint8_t header;            // 메시지 헤더 (0xAA)
  uint8_t command;           // 명령어 코드
  uint8_t data1;             // 데이터 1
  uint8_t data2;             // 데이터 2
  uint8_t checksum;          // 체크섬 (XOR)
};

// =============================================================================
// 통신 상태 구조체
// =============================================================================

struct CommState {
  bool isConnected;          // 연결 상태
  unsigned long lastCommTime; // 마지막 통신 시간
  uint8_t retryCount;        // 재시도 횟수
  uint8_t lastError;         // 마지막 오류 코드
};

// =============================================================================
// 통신 함수 프로토타입
// =============================================================================

// 메시지 생성 및 검증
uint8_t calculateChecksum(const SerialMessage* msg);
bool validateMessage(const SerialMessage* msg);
void createMessage(SerialMessage* msg, uint8_t cmd, uint8_t data1, uint8_t data2);

// 메시지 송수신
bool sendMessage(const SerialMessage* msg);
bool receiveMessage(SerialMessage* msg, unsigned long timeout = COMM_TIMEOUT);
bool sendCommand(uint8_t cmd, uint8_t data1 = 0, uint8_t data2 = 0);
bool waitForResponse(uint8_t expectedResponse, unsigned long timeout = COMM_TIMEOUT);

// 통신 상태 관리
void initComm();
bool isCommActive();
void resetComm();
void updateCommState();

// =============================================================================
// 통신 유틸리티 매크로
// =============================================================================

// 메시지 크기
#define MESSAGE_SIZE            sizeof(SerialMessage)

// 체크섬 계산 매크로
#define CALC_CHECKSUM(cmd, d1, d2) ((MESSAGE_HEADER) ^ (cmd) ^ (d1) ^ (d2))

// 타임아웃 체크 매크로
#define IS_TIMEOUT(start, timeout) ((millis() - (start)) > (timeout))

// 응답 대기 매크로
#define WAIT_FOR_RESPONSE(resp, timeout) waitForResponse((resp), (timeout))

// =============================================================================
// 디버그 및 로깅
// =============================================================================

#ifdef DEBUG_COMM
  #define COMM_DEBUG(x) Serial.print("[COMM] "); Serial.println(x)
  #define COMM_DEBUG_HEX(x) Serial.print("[COMM] 0x"); Serial.println(x, HEX)
#else
  #define COMM_DEBUG(x)
  #define COMM_DEBUG_HEX(x)
#endif

// =============================================================================
// 에러 코드 정의
// =============================================================================

#define COMM_ERROR_NONE         0x00  // 오류 없음
#define COMM_ERROR_TIMEOUT      0x01  // 타임아웃
#define COMM_ERROR_CHECKSUM     0x02  // 체크섬 오류
#define COMM_ERROR_INVALID      0x03  // 잘못된 메시지
#define COMM_ERROR_BUFFER_FULL  0x04  // 버퍼 오버플로우
#define COMM_ERROR_NO_RESPONSE  0x05  // 응답 없음
#define COMM_ERROR_UNKNOWN      0xFF  // 알 수 없는 오류

#endif // COMMUNICATION_H