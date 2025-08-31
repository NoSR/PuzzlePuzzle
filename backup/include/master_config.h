/*
 * Master Board 설정 헤더 파일
 * Arduino Mega 2560 핀 매핑 및 상수 정의
 */

#ifndef MASTER_CONFIG_H
#define MASTER_CONFIG_H

#include <Arduino.h>

// =============================================================================
// 핀 매핑 정의
// =============================================================================

// 천정 조명 제어 핀 (OUT0~3) - 12V LED
#define CEILING_LIGHT_1_PIN    4   // OUT0
#define CEILING_LIGHT_2_PIN    5   // OUT1
#define CEILING_LIGHT_3_PIN    6   // OUT2
#define CEILING_LIGHT_4_PIN    7   // OUT3

// EM-Lock 제어 핀 (OUT4~7)
#define EMLOCK_1_PIN          8    // OUT4
#define EMLOCK_2_PIN          9    // OUT5
#define EMLOCK_3_PIN          10   // OUT6
#define EMLOCK_4_PIN          11   // OUT7

// 릴레이 신호 핀 (relSig1~8)
#define GREEN_LIGHT_RELAY_PIN  12  // relSig1 - 초록색 경광등
#define RED_LIGHT_RELAY_PIN    13  // relSig2 - 빨간색 경광등
#define EFFECT_LED_1_PIN       22  // relSig3 - 이펙트 LED 1
#define EFFECT_LED_2_PIN       23  // relSig4 - 이펙트 LED 2
#define EFFECT_LED_3_PIN       24  // relSig5 - 이펙트 LED 3
#define EFFECT_LED_4_PIN       25  // relSig6 - 이펙트 LED 4
#define EFFECT_LED_5_PIN       26  // relSig7 - 이펙트 LED 5
#define EFFECT_LED_6_PIN       27  // relSig8 - 이펙트 LED 6

// DFPlayer Mini 통신 핀
#define DFTX_PIN              2    // Master -> DFPlayer
#define DFRX_PIN              3    // DFPlayer -> Master

// PIR 센서 입력 핀 (4개)
#define PIR_A_PIN             23   // A구역 PIR 센서 (디지털 핀 23)
#define PIR_B_PIN             31   // B구역 PIR 센서 (디지털 핀 31)
#define PIR_C_PIN             8    // C구역 PIR 센서 (디지털 핀 8)
#define PIR_D_PIN             A15  // D구역 PIR 센서 (아날로그 핀 A15)

// =============================================================================
// 핀 배열 정의 (편의성을 위한)
// =============================================================================

const uint8_t CEILING_LIGHT_PINS[4] = {
  CEILING_LIGHT_1_PIN, CEILING_LIGHT_2_PIN, 
  CEILING_LIGHT_3_PIN, CEILING_LIGHT_4_PIN
};

const uint8_t EMLOCK_PINS[4] = {
  EMLOCK_1_PIN, EMLOCK_2_PIN, EMLOCK_3_PIN, EMLOCK_4_PIN
};

const uint8_t RELAY_PINS[8] = {
  GREEN_LIGHT_RELAY_PIN, RED_LIGHT_RELAY_PIN,
  EFFECT_LED_1_PIN, EFFECT_LED_2_PIN, EFFECT_LED_3_PIN,
  EFFECT_LED_4_PIN, EFFECT_LED_5_PIN, EFFECT_LED_6_PIN
};

const uint8_t EFFECT_LED_PINS[6] = {
  EFFECT_LED_1_PIN, EFFECT_LED_2_PIN, EFFECT_LED_3_PIN,
  EFFECT_LED_4_PIN, EFFECT_LED_5_PIN, EFFECT_LED_6_PIN
};

const uint8_t PIR_PINS[4] = {
  PIR_A_PIN, PIR_B_PIN, PIR_C_PIN, PIR_D_PIN
};

// =============================================================================
// 게임 상수 정의
// =============================================================================

// 타이밍 상수 (밀리초)
#define LIGHT_SEQUENCE_INTERVAL   500   // 조명 순차 점등 간격
#define PATTERN_DISPLAY_INTERVAL  500   // 레벨 1-4 패턴 간격
#define PATTERN_FAST_INTERVAL     300   // 레벨 5+ 패턴 간격
#define PLAYER_INPUT_TIMEOUT      10000 // 플레이어 입력 제한시간
#define MOTION_DETECT_DURATION    5000  // 동작감지 시간
#define MOTOR_BRAKE_DURATION      5000  // 모터 브레이크 시간

// 게임 단계별 타임아웃 상수
#define ROBOT_ROTATION_TIMEOUT    15000 // 로봇 회전 타임아웃 (15초)
#define NARRATION_TIMEOUT         30000 // 내레이션 재생 타임아웃 (30초)
#define PATTERN_DISPLAY_TIMEOUT   15000 // 패턴 표시 타임아웃 (15초)
#define SCORE_UPDATE_TIMEOUT      5000  // 점수 업데이트 타임아웃 (5초)
#define GAME_CLEAR_TIMEOUT        20000 // 게임 클리어 타임아웃 (20초)

// 이펙트 LED 패턴 타이밍
#define EFFECT_SUCCESS_SPEED      200   // 성공 패턴 속도
#define EFFECT_ERROR_SPEED        100   // 오류 패턴 속도
#define EFFECT_LEVELUP_SPEED      300   // 레벨업 패턴 속도
#define EFFECT_START_SPEED        500   // 시작 패턴 속도
#define EFFECT_CLEAR_DURATION     10000 // 클리어 패턴 지속시간
#define EFFECT_TENSION_BEAT       800   // 긴장 패턴 심장박동 간격

// 게임 설정
#define MAX_SCORE                 7     // 최대 점수 (클리어 조건)
#define MAX_LEVEL                 11    // 최대 레벨
#define MIN_SCORE                 0     // 최소 점수

// PIR 센서 설정
#define PIR_SENSOR_COUNT          4     // PIR 센서 개수
#define PIR_ZONE_COUNT            4     // PIR 구역 개수 (A, B, C, D)

// =============================================================================
// 게임 상태 열거형 및 구조체
// =============================================================================

// 게임 단계 정의
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

// 이펙트 패턴 열거형
enum EffectPattern {
  SUCCESS_PATTERN = 1,       // 정답 입력시
  ERROR_PATTERN = 2,         // 오답 입력시
  LEVEL_UP_PATTERN = 3,      // 레벨 상승시
  GAME_START_PATTERN = 4,    // 게임 시작시
  GAME_CLEAR_PATTERN = 5,    // 게임 클리어시
  TENSION_PATTERN = 6        // 동작감지 중
};

// 게임 상태 구조체
struct GameState {
  uint8_t currentLevel;      // 현재 레벨 (1-11)
  uint8_t currentScore;      // 현재 점수 (0-7)
  uint8_t gamePhase;         // 게임 단계
  bool isGameActive;         // 게임 활성 상태
  bool isRobotFacing;        // 로봇이 정면을 보고 있는지
  unsigned long phaseTimer;  // 단계별 타이머
};

// 이펙트 LED 패턴 구조체
struct EffectStep {
  uint8_t ledMask;           // LED 점등 마스크 (비트별 제어)
  uint16_t duration;         // 지속 시간 (ms)
  bool isOn;                 // LED 상태 (ON/OFF만 가능)
};

struct EffectSequence {
  EffectStep steps[10];      // 최대 10단계
  uint8_t stepCount;         // 실제 단계 수
  uint8_t repeatCount;       // 반복 횟수
  bool isLooping;            // 무한 반복 여부
};

// =============================================================================
// DFPlayer 관련 상수
// =============================================================================

// 오디오 트랙 번호 (SD카드 파일 번호와 매칭)
#define AUDIO_WELCOME           1    // 환영 내레이션
#define AUDIO_GAME_START        2    // 게임 시작 안내
#define AUDIO_GAME_CLEAR        3    // 게임 클리어 축하
#define AUDIO_BGM_MAIN          4    // 메인 BGM
#define AUDIO_BGM_TENSION       5    // 긴장감 BGM

// 천정 조명 효과음 트랙 번호
#define AUDIO_LIGHT_1           10   // 천정 조명 1 효과음
#define AUDIO_LIGHT_2           11   // 천정 조명 2 효과음
#define AUDIO_LIGHT_3           12   // 천정 조명 3 효과음
#define AUDIO_LIGHT_4           13   // 천정 조명 4 효과음

// DFPlayer 명령어
#define DF_CMD_PLAY             0x0F
#define DF_CMD_PLAY_TRACK       0x03
#define DF_CMD_VOLUME           0x06
#define DF_CMD_STOP             0x16

#endif // MASTER_CONFIG_H