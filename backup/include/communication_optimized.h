/*
 * 최적화된 통신 프로토콜 헤더 파일
 * Master-Slave 간 시리얼 통신 프로토콜 정의
 */

#ifndef COMMUNICATION_OPTIMIZED_H
#define COMMUNICATION_OPTIMIZED_H

#include <Arduino.h>

// =============================================================================
// 통신 설정
// =============================================================================

#define SERIAL_BAUD_RATE      9600    // 시리얼 통신 속도
#define COMM_TIMEOUT          1000    // 통신 타임아웃 (ms)
#define MESSAGE_HEADER        0xAA    // 메시지 헤더

// =============================================================================
// 명령어 정의 (Master → Slave)
// =============================================================================

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
// 응답 코드 정의 (Slave → Master)
// =============================================================================

#define RESP_ACK                0x01  // 명령 수신 확인
#define RESP_NACK               0x02  // 명령 수신 실패

#define RESP_ROBOT_READY        0x10  // 로봇 회전 완료
#define RESP_ROBOT_TIMEOUT      0x11  // 로봇 회전 타임아웃
#define RESP_ROBOT_ERROR        0x12  // 로봇 제어 오류

#define RESP_PATTERN_DONE       0x20  // 패턴 표시 완료
#define RESP_PATTERN_ERROR      0x21  // 패턴 표시 오류

#define RESP_INPUT_CORRECT      0x31  // 올바른 입력
#define RESP_INPUT_WRONG        0x32  // 잘못된 입력
#define RESP_INPUT_TIMEOUT      0x33  // 입력 타임아웃

#define RESP_AUDIO_DONE         0x50  // 오디오 재생 완료
#define RESP_AUDIO_ERROR        0x51  // 오디오 재생 오류

#define RESP_ERROR              0xFE  // 일반 오류
#define RESP_UNKNOWN            0xFF  // 알 수 없는 명령

// =============================================================================
// 데이터 코드 정의
// =============================================================================

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

// 효과음 데이터
#define SFX_LIGHT_ON            0x01
#define SFX_PATTERN_BEEP        0x02
#define SFX_PAD_PRESS           0x03
#define SFX_CORRECT             0x04
#define SFX_WRONG               0x05
#define SFX_GUNSHOT             0x06
#define SFX_SUCCESS             0x07
#define SFX_LEVEL_UP            0x08

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
// 통신 함수 프로토타입
// =============================================================================

// 메시지 생성 및 검증
uint8_t calculateChecksum(uint8_t cmd, uint8_t data1, uint8_t data2);
bool validateChecksum(uint8_t cmd, uint8_t data1, uint8_t data2, uint8_t checksum);

// 메시지 송수신
void sendCommand(uint8_t cmd, uint8_t data1, uint8_t data2);
void sendResponse(uint8_t response, uint8_t data1, uint8_t data2);

// =============================================================================
// 통신 유틸리티 매크로
// =============================================================================

// 메시지 크기
#define MESSAGE_SIZE            5

// 체크섬 계산 매크로
#define CALC_CHECKSUM(cmd, d1, d2) ((cmd) ^ (d1) ^ (d2))

// 타임아웃 체크 매크로
#define IS_TIMEOUT(start, timeout) ((millis() - (start)) > (timeout))

// =============================================================================
// 에러 코드 정의
// =============================================================================

#define COMM_ERROR_NONE         0x00  // 오류 없음
#define COMM_ERROR_TIMEOUT      0x01  // 타임아웃
#define COMM_ERROR_CHECKSUM     0x02  // 체크섬 오류
#define COMM_ERROR_INVALID      0x03  // 잘못된 메시지
#define COMM_ERROR_NO_RESPONSE  0x05  // 응답 없음

#endif // COMMUNICATION_OPTIMIZED_H