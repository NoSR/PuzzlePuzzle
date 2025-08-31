# 프로젝트 목표
- 시나리오를 기반으로 제작된 장치들을 사용해 인터랙티브한 게임을 구현
- 연결된 장치의 특성을 이해하고 안정적으로 동작 할 수 있도록 프로그래밍
- 제작자의 의도가 반영되고, 플래이어가 자연스럽게 이해할 수 있는 구조로 설계

### 베이스 스토리
(도입부 생략) 첫 번째 세션에서 팀 원들이 재치있는 솜씨로 기계장치들을 조작하여 잠긴 문을 열고, 두 번째 세션에 도착했다. 눈 앞에 보이는 것은 커다란 영희인형 로봇이었다. 이전에 봤던 '무궁화 꽃이 피었습니다' 일명 Red light Green light 게임이 진행될 예정이다. 그러나 이곳은 실내이기 때문에 상황이 다르다. 잠시 후 방송이 흘러나온다. "오징어게임에 참가하신 여러분, 환영합니다. 게임 규칙을 말씀 드리겠습니다. 정면에 보이는 4개의 조명이 점등되는 순서를 기억하시고, 바닥에 놓여진 발판을 같은 순서로 누르면 1점을 얻게 됩니다. 단, 로봇이 여러분을 바라보지 않고 있을 때 움직일 수 있습니다. 로봇이 뒤돌아 스테이지를 바라볼 때 움직임이 감지되면 1점이 감점 됩니다. 최종 7점을 얻게 되면 통과입니다. 점수를 얻을 때 마다 난이도가 올라간다는 점에 유의해 주세요. 그럼, 시작합니다." 이제 게임이 시작 되었다. 과연 팀 원들은 이 게임을 통과할 수 있을까.

### 플레이 시나리오
- 모든 조명은 소등 상태. 플레이어가 시작 버튼을 누르면 메인 조명이 순차적으로 점등 되며 게임이 시작된다.
- 메인 조명은 총 4개가 있으며, 1번부터 4번까지 0.5초 간격으로 효과음과 함께 순차적으로 점등 되며 긴장도를 높힌다.
- 모든 조명이 점등 되면 대형 로봇 몸체가 플레이어 방향으로 회전하며 정면을 바라보게 된다.
- 로봇이 회전하여 정면을 바라보면(내부 리미트 스위치가 눌려짐), 환영 내레이션이 재생된다.
- 플레이어가 게임 설명을 듣고 내용을 인지하게 된다.
- 로봇의 BODY가 후면 방향으로 180도 회전하고 완전히 후면을 바라보면(내부 리미트 스위치 눌려짐), 초록색 경광등이 점등된다.
- 4개의 LED가 0.5초 간격으로 레벨에 따라 무작위 점멸(효과음 재생)하며 동시에 문제를 제시한다.
- 플레이어는 LED 점멸 순서를 기억하고 바닥의 4개의 발판을 같은 순서로 밟으면(효과음 재생) 점수를 얻게 된다.
- 플레이어가 발판을 조작할 수 있는 시간은 초록색 경광등이 점등되어 있는 10초 동안만 가능하다.
- 10초가 지난 뒤 로봇의 HEAD가 정면을 향해 회전하기 시작한다. HEAD가 180도를 회전하여 완전히 정면을 바라보면(내부 리미트 스위치 눌려짐) 초록색 경광등은 소등 되고, 빨간색 경광등이 점등 되며 4개의 PIR센서가 5초간 동작감지를 시작한다.
- 플래이어가 동작감지 시간동안 움직임으로 동작감지가 되면, '총소리' 효과음이 재생 되며, 이펙트 LED가 0.2초 간격 5회 점멸하며 감점 신호가 연출되고, 점수 1점이 감점된다.
- 동작감지 시간 5초가 지나면, 로봇의 HEAD는 다시 후면을 향해 회전하고 180도 회전하여 리미트 스위치가 작동하면 레벨 사이클이 완성된다.
- 한 사이클이 완성되면 레벨이 상승하고, 문제 난이도가 높아진다.
- 레벨은 1부터 11까지이며, 1레벨부터 4레벨까지는 0.5초 간격으로 레벨 숫자만큼 LED를 점멸하여 문제를 제시하고, 레벨 5부터는 레벨 10까지는 0.3초 간격으로 5회 LED를 점멸하여 문제를 제시, 레벨 11을 0.3초 간격 6회 LED 점멸하여 문제를 제시한다. 11레벨을 지날 때까지도 총점 7점을 획득하지 못했다면, 레벨 0부터 다시 반복한다.
- 점수는 6개의 LED로 표시되며 1점을 획득할 때마다 1개의 LED가 점등된다. 반대로 감점되면 1개의 LED가 소등된다.
- 0점에서 감점될 경우 0점을 유지하고 더 이상 감점되지 않는다. 6점을 획득해 6개의 LED가 모두 점등 된 상태에서 1점을 추가 획득하여 7점이 되면 6개의 LED가 0.5초 간격으로 7회 점멸 하며 클리어 상황을 연출한다.
- 게임을 클리어하면 이펙트 LED가 클리어 효과(이펙트 LED 로직 별도 작성)를 연출한다.
- 클리어 효과 연출이 종료되면 EM-Lock가 작동하여 플레이어는 다음 세션을 위한 퍼즐 조각을 얻게 된다.

### 장치 목록
#### MASTER BOARD
##### 회로 부품
- 아두이노 Mega 2560 1개
- DFplayer Mini 1개
- 자체 제작 PCB
- 레귤레이터, 콘덴서 등 각종 부품
##### 외부 부품
- 점수 LED : 5v LED 6개
- 천정 조명 4개 : 12v LED 각각 1개씩 장착
- 동작감지 : PIR 센서 4개
- 경광등 LED 2개 : 12v LED 2개
- 이펙트 LED : 12v LED 6개
- 8핀 5v-12v 릴레이 1개 : 경광등, 이펙트 LED 연결
- 사운드 출력 : 스피커 1개

#### SLAVE BOARD
##### 회로 부품
- 아두이노 나노 1개
- DFplayer Mini 1개
- MCP23017 2개
- L298N DC모터 드라이버 1개
- 자체 제작 PCB
- 레귤레이터, 콘덴서 등 각종 부품
##### 외부 부품
- 로봇 HEAD : 12v DC Motor, Limit 스위치
- 로봇 BODY : 12v DC Motor, Limit 스위치
- 바닥 발판 4개 : Button, 진동감지센서를 발판에 각각 1개씩 장착
- 문제 제시 LED 4개 : 12v LED 각각 1개씩 장착

### 특이사항
- 버튼 입력 로직은 반드시 바운싱을 염두하여 작성되어야 함.
- 문제 LED 점등은 랜덤으로 제공되지만, 하나의 LED가 연속하여 점멸하지 않아야 함(예: 1-2-3-1-4 Ok, 1-1-3-1-4 No)
- 로봇 회전 후 모터가 정지하면, 로봇의 무게 및 관성 때문에 로봇이 움직일 수 있으니 모터 정지 신호가 입력 될 경우 모든 모터, 모든 핀에 5초간 HIGH 신호를 인가 해야 함.
- 이펙트 LED 동작을 위한 별도 로직 필요 : 예를 들어 특정 상황에 플레이어가 긴장감을 느낄 수 있도록 점멸을 반복 하거나, 정답, 오답 시 6개의 LED가 특정 패턴으로 점멸하는 등 특정한 로직을 함수로 작성하여 상황에 맞게 호출해야 함.
- 아두이노를 2개 사용한 이유는 아두이노 특성상 멀티 테스킹이 어렵기 때문에 두 가지 동작이 동시에 동작해야 하는 경우 임무를 분담하기 위함.
- 2개의 아두이노는 시리얼 포트로 연결되어 있고, 시리얼 신호로 상호 명령을 전달.
- 플레이어가 게임을 클리어 한 후 모든 장치는 동작을 정지하고 최소 전력으로 대기 상태 유지
- mcp23017 핀 순서는 반드시 매크로 변수명의 순서를 따라야 함.
- 내레이션과 BGM은 MASTER BOARD와 연결된 DFplayer가 재생
- 단발성 효과음은 SLAVE BOARD와 연결된 DFplayer가 재생

### 매크로 설정
> 매크로 변수는 각 보드마다 헤더 파일로 별도로 작성성
#### MASTER BOARD MACRO VARIABLE
```
// Analog
#define anSig1 A8
#define anSig2 A9
#define anSig3 A10

// Relay Signal
#define relSig1 22
#define relSig2 24
#define relSig3 26
#define relSig4 28
#define relSig5 30
#define relSig6 32
#define relSig7 34
#define relSig8 36

// OUTPUT-12V
#define OUT0 4
#define OUT1 5
#define OUT2 6
#define OUT3 7

// EM-LOCK
#define OUT4 8
#define OUT5 9
#define OUT6 10
#define OUT7 11

// SERIAL PORT

// DIRECT Conection Signal PIN
#define drA_in1 29
#define drA_in2 27
#define drA_in3 25
#define drA_in4 23
#define drB_in1 37
#define drB_in2 35
#define drB_in3 33
#define drB_in4 31
#define drC_in1 45
#define drC_in2 43
#define drC_in3 41
#define drC_in4 39
#define drD_in1 A12
#define drD_in2 A13
#define drD_in3 A14
#define drD_in4 A15
#define slaveA_TX 0

// DFplayer Mini
#define DFTX 2
#define DFRX 3

// Command Macro List
#define READY 0
#define START 1
#define PLAY 2
#define RIGHT 3
#define WRONG 4
#define TIMEOUT 5
#define WATCH 6
#define LOOKOUT 7
#define OVERLOOK 8
#define SHOT 9
#define CLEAR 10
#define FORWARD 11
#define BACKWARD 12
#define STOP 13
#define GO 14
#define FINISHED 15
#define RED 16
#define GREEN 17
#define HEAD 18
#define BODY 19

// Effect Sound List
#define Button_Touch 1
#define Gun_Shot 2
#define Error_Beef 3
#define Wrong_Beef 4
#define Correct_Ding 5
#define Score_Ding 6
#define QT_tone0 7
#define Turn_Motor 8
#define Light_On 9
#define Light_Off 10
#define Game_Start 11
#define Game_Finished 12
  
// Narration & BGM Sound List
#define Mugunghwa_Wecome 1
#define Mugunghwa_Overlook 2
#define Mugunghwa_Clear 3
#define GameClear 4
#define SquidGame 5
```

#### SLAVE BOARD MACRO VARIABLE
```
// mcp1 INPUT 핀 배열
#define IN0 11
#define IN1 10
#define IN2 9
#define IN3 8
#define IN4 12
#define IN5 13
#define IN6 14
#define IN7 0


// mcp1 OUTUT 핀 배열
#define OutSign0 1
#define OutSign1 2
#define OutSign2 3
#define OutSign3 4
#define OutSign4 5
#define OutSign5 6
#define OutSign6 7
#define OutSign7 15


// mcp2 first multi 핀 배열
#define fstMulti0 0
#define fstMulti1 5
#define fstMulti2 1
#define fstMulti3 4
#define fstMulti4 2
#define fstMulti5 3

  
// mcp2 second multi 핀 배열
#define sndMulti0 8
#define sndMulti1 9
#define sndMulti2 13
#define sndMulti3 10
#define sndMulti4 12
#define sndMulti5 11

  
// 진동 센서
#define Analog0 A0
#define Analog1 A1
#define Analog2 A2
#define Analog3 A3

// 아날로그 핀
#define Analog4 A7
  

// 모터 핀 배열
#define PWM_L 5
#define MIN1 6
#define MIN2 7
#define MIN3 8
#define MIN4 9
#define PWM_R 10
 

// DFPlayer 핀 배열
#define DFR_RX 2
#define DFR_TX 3
  

// 시리얼 통신
#define TX 1
#define RX 0

// 기타 핀 배열
#define BZZ 13
#define RST RESET
#define SDA A4
#define SCL A5


// Command Macro List
#define READY 0
#define START 1
#define PLAY 2
#define RIGHT 3
#define WRONG 4
#define TIMEOUT 5
#define WATCH 6
#define LOOKOUT 7
#define OVERLOOK 8
#define SHOT 9
#define CLEAR 10
#define FORWARD 11
#define BACKWARD 12
#define STOP 13
#define GO 14
#define FINISHED 15
#define RED 16
#define GREEN 17


// Effect Sound List
#define Button_Touch 1
#define Gun_Shot 2
#define Error_Beef 3
#define Wrong_Beef 4
#define Correct_Ding 5
#define Score_Ding 6
#define QT_tone0 7
#define Turn_Motor 8
#define Light_On 9
#define Light_Off 10
#define Game_Start 11
#define Game_Finished 12
  
// Narration & BGM Sound List
#define Mugunghwa_Wecome 1
#define Mugunghwa_Overlook 2
#define Mugunghwa_Clear 3
#define GameClear 4
#define SquidGame 5
```