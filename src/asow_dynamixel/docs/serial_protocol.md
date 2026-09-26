# ASOW Raspberry Pi - Arduino Serial Protocol

## 1. 목적

Raspberry Pi에서 실행되는 ROS2 Hardware Interface와
Arduino Uno에서 실행되는 Dynamixel Bridge Firmware 사이의
통신 규격을 정의한다.

통신 경로:

Raspberry Pi
→ USB-to-TTL UART
→ Arduino D7/D8 SoftwareSerial
→ Arduino Firmware
→ DYNAMIXEL Shield
→ AX-12A / AX-18A

모든 메시지는 ASCII 문자열이며 newline(`\n`)으로 끝난다.

---

## 2. 기본 설정

Pi-Arduino Serial Baudrate:

115200

메시지 종료 문자:

\n

---

## 2.1 현재 범위와 가정

본 규격은 Axis 3 초기 Bring-up 및 진단을 위한 Serial Protocol v0이다.

현재 명령은 다음 목적에 사용한다.

- Raspberry Pi ↔ Arduino 연결 확인
- Dynamixel Bus 탐색
- 개별 Motor Ping
- 개별 Position Read / Write
- 개별 Torque 제어

`ros2_control`의 실제 주기 제어에 사용할 최종 실시간 전송 구조는
Host Serial Transport 및 Hardware Interface 구현 단계에서
실제 통신량과 주기를 측정한 뒤 확정한다.

현재 통신 속도 가정:

- Raspberry Pi ↔ Arduino SoftwareSerial: 115200 bps
- Arduino ↔ Dynamixel Bus: 1000000 bps

두 값 모두 실제 Hardware 연결 후 검증 대상이다.

숫자 Argument는 전체 문자열이 유효한 정수일 때만 허용한다.
잘못된 문자열을 0으로 암묵 변환하지 않는다.

---

## 3. 명령

### HELLO

연결 상태 확인.

Pi:

HELLO

Arduino:

OK,HELLO

---

### SCAN

Dynamixel Bus에서 응답하는 Motor ID 검색.

Pi:

SCAN

Arduino 예시:

SCAN,3,5,7,12,15

검색된 모터가 없는 경우:

SCAN

---

### PING

특정 Dynamixel ID의 응답 확인.

Pi:

PING,<id>

예:

PING,3

정상:

PING,3,OK

실패:

PING,3,FAIL

---

### READ

특정 모터의 Present Position 읽기.

Pi:

READ,<id>

예:

READ,3

Arduino:

POS,3,<raw_position>

예:

POS,3,512

---

### WRITE

특정 모터의 Goal Position 설정.

Pi:

WRITE,<id>,<raw_position>

예:

WRITE,3,530

정상:

OK,WRITE,3

실패:

ERROR,WRITE,3

---

### TORQUE

특정 모터 Torque 활성화 또는 비활성화.

Pi:

TORQUE,<id>,ON

또는:

TORQUE,<id>,OFF

예:

TORQUE,3,OFF

Arduino:

OK,TORQUE,3,OFF

---

## 4. 오류

명령을 해석하지 못한 경우:

ERROR,BAD_COMMAND

잘못된 ID:

ERROR,BAD_ID

잘못된 Position:

ERROR,BAD_POSITION

Dynamixel 통신 실패:

ERROR,DXL_COMM

---

## 5. 설계 원칙

Arduino는 ROS Joint 이름을 알지 않는다.

Arduino가 다루는 것은 다음뿐이다.

- Dynamixel ID
- Torque
- Goal Position
- Present Position

다음 정보는 Raspberry Pi의 ROS2 Hardware Interface가 관리한다.

- left_joint1 ~ left_joint5
- right_joint1 ~ right_joint5
- ROS Joint ↔ Dynamixel ID Mapping
- Direction
- Zero Offset
- ROS rad ↔ Dynamixel Position 변환

따라서 실제 모터 배치가 바뀌어도 Arduino Firmware는 수정하지 않고
ROS2 설정 파일만 변경하는 구조를 사용한다.
