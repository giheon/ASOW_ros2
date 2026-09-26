#include <SoftwareSerial.h>
#include <Dynamixel2Arduino.h>

#include <stdlib.h>
#include <string.h>

// ============================================================
// ASOW Axis 3 - Arduino ↔ Dynamixel Bridge
//
// Raspberry Pi ↔ Arduino
//   D7 = RX
//   D8 = TX
//   SoftwareSerial
//
// Arduino ↔ DYNAMIXEL Shield
//   D0 = RX
//   D1 = TX
//   D2 = Direction
//   Hardware Serial
// ============================================================


// ------------------------------------------------------------
// Raspberry Pi ↔ Arduino
// ------------------------------------------------------------

static const uint8_t PI_RX_PIN = 7;
static const uint8_t PI_TX_PIN = 8;

static const uint32_t PI_BAUDRATE = 115200;

SoftwareSerial piSerial(PI_RX_PIN, PI_TX_PIN);


// ------------------------------------------------------------
// Arduino ↔ DYNAMIXEL
// ------------------------------------------------------------

#define DXL_SERIAL Serial

static const uint8_t DXL_DIR_PIN = 2;

// AX-12A / AX-18A
static const float DXL_PROTOCOL_VERSION = 1.0;

// 현재는 1 Mbps를 기본 가정.
// 실제 HW 연결 후 통신이 되지 않으면 가장 먼저 확인할 값.
static const uint32_t DXL_BAUDRATE = 1000000;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);


// ------------------------------------------------------------
// Command Buffer
// ------------------------------------------------------------

static const size_t COMMAND_BUFFER_SIZE = 80;

char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandIndex = 0;


// ------------------------------------------------------------
// Utility
// ------------------------------------------------------------

bool isValidId(long id)
{
  return id >= 0 && id <= 253;
}


bool isValidRawPosition(long position)
{
  // AX-12A / AX-18A position raw range
  return position >= 0 && position <= 1023;
}


bool parseLongStrict(const char *token, long &value)
{
  if (token == nullptr || *token == '\0')
  {
    return false;
  }

  char *end = nullptr;
  value = strtol(token, &end, 10);

  return end != token && *end == '\0';
}


// ------------------------------------------------------------
// HELLO
//
// Pi:
// HELLO
//
// Arduino:
// OK,HELLO
// ------------------------------------------------------------

void handleHello()
{
  piSerial.println("OK,HELLO");
}


// ------------------------------------------------------------
// SCAN
//
// Pi:
// SCAN
//
// Arduino:
// SCAN,3,5,15
// ------------------------------------------------------------

void handleScan()
{
  piSerial.print("SCAN");

  for (int id = 0; id <= 253; ++id)
  {
    if (dxl.ping((uint8_t)id))
    {
      piSerial.print(",");
      piSerial.print(id);
    }
  }

  piSerial.println();
}


// ------------------------------------------------------------
// PING
//
// Pi:
// PING,3
//
// Arduino:
// PING,3,OK
// ------------------------------------------------------------

void handlePing(char *idToken)
{
  if (idToken == nullptr)
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  long id = 0;

  if (!parseLongStrict(idToken, id) || !isValidId(id))
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  if (dxl.ping((uint8_t)id))
  {
    piSerial.print("PING,");
    piSerial.print(id);
    piSerial.println(",OK");
  }
  else
  {
    piSerial.print("PING,");
    piSerial.print(id);
    piSerial.println(",FAIL");
  }
}


// ------------------------------------------------------------
// READ
//
// Pi:
// READ,3
//
// Arduino:
// POS,3,512
// ------------------------------------------------------------

void handleRead(char *idToken)
{
  if (idToken == nullptr)
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  long id = 0;

  if (!parseLongStrict(idToken, id) || !isValidId(id))
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  if (!dxl.ping((uint8_t)id))
  {
    piSerial.println("ERROR,DXL_COMM");
    return;
  }

  int32_t position =
    (int32_t)dxl.getPresentPosition((uint8_t)id, UNIT_RAW);

  piSerial.print("POS,");
  piSerial.print(id);
  piSerial.print(",");
  piSerial.println(position);
}


// ------------------------------------------------------------
// WRITE
//
// Pi:
// WRITE,3,530
//
// Arduino:
// OK,WRITE,3
// ------------------------------------------------------------

void handleWrite(char *idToken, char *positionToken)
{
  if (idToken == nullptr)
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  if (positionToken == nullptr)
  {
    piSerial.println("ERROR,BAD_POSITION");
    return;
  }

  long id = 0;
  long position = 0;

  if (!parseLongStrict(idToken, id) || !isValidId(id))
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  if (!parseLongStrict(positionToken, position) ||
      !isValidRawPosition(position))
  {
    piSerial.println("ERROR,BAD_POSITION");
    return;
  }

  if (!dxl.setGoalPosition(
          (uint8_t)id,
          (float)position,
          UNIT_RAW))
  {
    piSerial.print("ERROR,WRITE,");
    piSerial.println(id);
    return;
  }

  piSerial.print("OK,WRITE,");
  piSerial.println(id);
}


// ------------------------------------------------------------
// TORQUE
//
// Pi:
// TORQUE,3,ON
// TORQUE,3,OFF
//
// Arduino:
// OK,TORQUE,3,ON
// ------------------------------------------------------------

void handleTorque(char *idToken, char *stateToken)
{
  if (idToken == nullptr)
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  if (stateToken == nullptr)
  {
    piSerial.println("ERROR,BAD_COMMAND");
    return;
  }

  long id = 0;

  if (!parseLongStrict(idToken, id) || !isValidId(id))
  {
    piSerial.println("ERROR,BAD_ID");
    return;
  }

  bool success = false;

  if (strcmp(stateToken, "ON") == 0)
  {
    success = dxl.torqueOn((uint8_t)id);

    if (success)
    {
      piSerial.print("OK,TORQUE,");
      piSerial.print(id);
      piSerial.println(",ON");
    }
  }
  else if (strcmp(stateToken, "OFF") == 0)
  {
    success = dxl.torqueOff((uint8_t)id);

    if (success)
    {
      piSerial.print("OK,TORQUE,");
      piSerial.print(id);
      piSerial.println(",OFF");
    }
  }
  else
  {
    piSerial.println("ERROR,BAD_COMMAND");
    return;
  }

  if (!success)
  {
    piSerial.println("ERROR,DXL_COMM");
  }
}


// ------------------------------------------------------------
// Command Parser
// ------------------------------------------------------------

void processCommand(char *command)
{
  char *commandToken = strtok(command, ",");

  if (commandToken == nullptr)
  {
    piSerial.println("ERROR,BAD_COMMAND");
    return;
  }

  if (strcmp(commandToken, "HELLO") == 0)
  {
    handleHello();
    return;
  }

  if (strcmp(commandToken, "SCAN") == 0)
  {
    handleScan();
    return;
  }

  if (strcmp(commandToken, "PING") == 0)
  {
    char *idToken = strtok(nullptr, ",");

    handlePing(idToken);
    return;
  }

  if (strcmp(commandToken, "READ") == 0)
  {
    char *idToken = strtok(nullptr, ",");

    handleRead(idToken);
    return;
  }

  if (strcmp(commandToken, "WRITE") == 0)
  {
    char *idToken = strtok(nullptr, ",");
    char *positionToken = strtok(nullptr, ",");

    handleWrite(idToken, positionToken);
    return;
  }

  if (strcmp(commandToken, "TORQUE") == 0)
  {
    char *idToken = strtok(nullptr, ",");
    char *stateToken = strtok(nullptr, ",");

    handleTorque(idToken, stateToken);
    return;
  }

  piSerial.println("ERROR,BAD_COMMAND");
}


// ------------------------------------------------------------
// Serial Receiver
// ------------------------------------------------------------

void receivePiCommand()
{
  while (piSerial.available() > 0)
  {
    char incoming = piSerial.read();

    if (incoming == '\r')
    {
      continue;
    }

    if (incoming == '\n')
    {
      commandBuffer[commandIndex] = '\0';

      if (commandIndex > 0)
      {
        processCommand(commandBuffer);
      }

      commandIndex = 0;

      continue;
    }

    if (commandIndex < COMMAND_BUFFER_SIZE - 1)
    {
      commandBuffer[commandIndex++] = incoming;
    }
    else
    {
      commandIndex = 0;

      piSerial.println("ERROR,BUFFER_OVERFLOW");
    }
  }
}


// ------------------------------------------------------------
// Arduino Setup
// ------------------------------------------------------------

void setup()
{
  // Raspberry Pi ↔ Arduino
  piSerial.begin(PI_BAUDRATE);

  // Arduino ↔ Dynamixel
  dxl.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 안전 원칙:
  // 부팅만으로 임의의 모터를 움직이거나 Torque ON 하지 않는다.
}


// ------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------

void loop()
{
  receivePiCommand();
}
