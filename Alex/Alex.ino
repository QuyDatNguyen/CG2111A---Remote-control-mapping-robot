#ifndef __INTELLISENSE__
#include <serialize.h>
#endif

#include <math.h>
#include "packet.h"
#include "constants.h"
#include <stdarg.h>

// ░█▀▀░█▀█░█▀█░█▀▀░▀█▀░█▀█░█▀█░▀█▀░█▀▀
// ░█░░░█░█░█░█░▀▀█░░█░░█▀█░█░█░░█░░▀▀█
// ░▀▀▀░▀▀▀░▀░▀░▀▀▀░░▀░░▀░▀░▀░▀░░▀░░▀▀▀

/**
 * Movements and Dimensions
 */

// Number of ticks per 1 revolution from the wheel encoder.
#define COUNTS_PER_REV 4
// Wheel circumference in cm.
#define WHEEL_CIRC 20
#define ALEX_LENGTH 13
#define ALEX_BREADTH 13
#define CIRC_PI 3.141592654
volatile double ALEX_DIAGONAL = 0.0;
volatile double ALEX_CIRC = 0.0;

/**
 * Color Sensing [Port C]
 */
#define COLOR_SENSOR_S0 (1 << 3) // PC3, Pin 34
#define COLOR_SENSOR_S1 (1 << 4) // PC4, Pin 33
#define COLOR_SENSOR_S2 (1 << 2) // PC2, Pin 35
#define COLOR_SENSOR_S3 (1 << 1) // PC1, Pin 36
#define COLOR_SENSOR_OUTPUT 37   // (PC0, Pin 37)
#define COLOR_SENSOR_WAIT 1000   // in millisecs

/**
 * Ultrasound Sensor [Port L]
 */
#define TRIG (1 << 3)        // PL3, PIN 46
#define ECHO (1 << 2)        // PL2, PIN 47
#define SPEED_OF_SOUND 0.345 // (mm/microseconds)
#define TIMEOUT 1e4          // Max microseconds to wait; choose according to max distance of wall
#define GAP_FROM_FRONT 35    // gap between sensor and front of the robot, in mm

// ░█░█░█▀█░█▀▄░▀█▀░█▀█░█▀▄░█░░░█▀▀░█▀▀
// ░▀▄▀░█▀█░█▀▄░░█░░█▀█░█▀▄░█░░░█▀▀░▀▀█
// ░░▀░░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀░░▀▀▀░▀▀▀░▀▀▀

volatile TDirection dir;

// Store the ticks from Alex's left and right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// variables to keep track whether we have moved the commanded distance
volatile unsigned long deltaDist;
volatile unsigned long newDist;
volatile unsigned long deltaTicks;
volatile unsigned long targetTicks;

// ░█▀▀░█▀█░█▄█░█▄█░█░█░█▀█░▀█▀░█▀▀░█▀█░▀█▀░▀█▀░█▀█░█▀█
// ░█░░░█░█░█░█░█░█░█░█░█░█░░█░░█░░░█▀█░░█░░░█░░█░█░█░█
// ░▀▀▀░▀▀▀░▀░▀░▀░▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀░▀░░▀░░▀▀▀░▀▀▀░▀░▀

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_RESPONSE;
  messagePacket.command = RESP_STATUS;
  messagePacket.params[0] = leftForwardTicks;
  messagePacket.params[1] = rightForwardTicks;
  messagePacket.params[2] = leftReverseTicks;
  messagePacket.params[3] = rightReverseTicks;
  messagePacket.params[4] = leftForwardTicksTurns;
  messagePacket.params[5] = rightForwardTicksTurns;
  messagePacket.params[6] = leftReverseTicksTurns;
  messagePacket.params[7] = rightReverseTicksTurns;
  messagePacket.params[8] = forwardDist;
  messagePacket.params[9] = reverseDist;
  messagePacket.params[10] = targetTicks;
  messagePacket.params[11] = deltaTicks;
  messagePacket.params[12] = deltaDist;

  sendResponse(&messagePacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  // dbprintf("SENDING...");

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

// using this usually causes bad magic number -> you CAN use it to find out if the code is reaching somewhere lmao
void dbprintf(char *format, ...)
{
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

// Set up the serial connection.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection.
void startSerial()
{
  // To replace later with bare-metal.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
int readSerial(char *buffer)
{
  // To replace later with bare-metal.
  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port.
void writeSerial(const char *buffer, int len)
{
  // To replace later with bare-metal.
  Serial.write(buffer, len);
}

// ░█▄█░█▀█░█░█░█▀▀░█▄█░█▀▀░█▀█░▀█▀
// ░█░█░█░█░▀▄▀░█▀▀░█░█░█▀▀░█░█░░█░
// ░▀░▀░▀▀▀░░▀░░▀▀▀░▀░▀░▀▀▀░▀░▀░░▀░

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long)((ang * ALEX_CIRC * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed)
{
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = leftReverseTicksTurns + deltaTicks;
  ccw(ang, speed);
}

void right(float ang, float speed)
{
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);

  targetTicks = rightReverseTicksTurns + deltaTicks;
  cw(ang, speed);
}

// Enable pull up resistors on pins 18 and 19
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if (dir == FORWARD)
  {
    leftForwardTicks++;
    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == BACKWARD)
  {
    leftReverseTicks++;
    reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  }
  else if (dir == LEFT)
  {
    leftReverseTicksTurns++;
  }
  else if (dir == RIGHT)
  {
    leftForwardTicksTurns++;
  }
}

void rightISR()
{
  // dbprintf("R ");
  if (dir == FORWARD)
  {
    rightForwardTicks++;
  }
  else if (dir == BACKWARD)
  {
    rightReverseTicks++;
  }
  else if (dir == LEFT)
  {
    rightForwardTicksTurns++;
  }
  else if (dir == RIGHT)
  {
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  EICRA |= 0b10100000;
  EIMSK |= 0b00001100;
}

// Implement INT2 and INT3 ISRs above.
ISR(INT3_vect)
{
  leftISR();
  rightISR();
}
ISR(INT2_vect) // NOT WORKING (PROPERLY)
{
  // right is moved up to the left side for now LMAO
}

// ░█▀▀░█▀▀░█▀█░█▀▀░█▀█░█▀▄░█▀▀
// ░▀▀█░█▀▀░█░█░▀▀█░█░█░█▀▄░▀▀█
// ░▀▀▀░▀▀▀░▀░▀░▀▀▀░▀▀▀░▀░▀░▀▀▀

// ultrasonic sensor setup
void setupUltrasonic()
{
  DDRL |= TRIG;  // set PB5 as trigger pin (output)
  DDRL &= ~ECHO; // set PB6 as echo pin (input)
}

// Reads the ultrasonic sensor and returns the distance in mms
int readUltrasonic()
{
  PORTL |= TRIG; // set HIGH
  delayMicroseconds(10);
  PORTL &= ~TRIG; // set LOW

  unsigned long duration = pulseIn(47, HIGH);                   // read pulse length in us
  double dist = duration / 2 * SPEED_OF_SOUND - GAP_FROM_FRONT; // convert to mms
  // return dist;
  return (int)dist;
}

void sendDistance()
{
  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_RESPONSE;
  messagePacket.command = RESP_IR_DISTANCE;

  int distance = readUltrasonic(); // convert to mm

  messagePacket.params[0] = distance; // printing value will be in mm

  sendResponse(&messagePacket);
}

// colour sensor setup

// Intialize Alex's internal states
void setupcolour()
{
  // setting S0, S1, S2 and S3 pins as output
  DDRC |= (((COLOR_SENSOR_S0) | (COLOR_SENSOR_S1)) | ((COLOR_SENSOR_S2) | (COLOR_SENSOR_S3)));
  // setting output pin as input
  DDRC &= ~(1 << 0);

  // setting freq scaling to 2%: S0 as LOw and S1 as HIGH
  PORTC &= ~COLOR_SENSOR_S0;
  PORTC |= COLOR_SENSOR_S1;

  // // ADC attempt:
  // // turn off power reduction register for adc
  // PRR0 &= ~0b00000001;

  // // turn on ADCSRA
  // // set prescaler to 32
  // ADCSRA = 0b10000100;

  // // set reference voltages to be AREF
  // ADMUX = 0b01000000;
  // ADCSRB |= 0b00001000;
}

int getAvgReading(int times)
{
  // props for when we do mapping:
  int reading = 0;
  // int total = 0;
  // for (int i = 0; i < times; i++)
  // {
  // wait until COLOR_SENSOR_OUTPUT is high
  // pulseIn(COLOR_SENSOR_OUTPUT, HIGH);
  reading = pulseIn(COLOR_SENSOR_OUTPUT, LOW);

  // ADCSRA |= 0b01000000;
  // while (ADCSRA & 0b01000000)
  //   ;
  // int low = ADCL;
  // int high = ADCH;
  // reading = (high << 8) + low;

  // reading = map(reading, high_map, low_map, 255, 0);
  //   total += reading;
  //   delay(20);
  // }
  // return total / times;
  return reading;
}

void sendColor()
{
  int colorR, colorG, colorB;

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_RESPONSE;
  messagePacket.command = RESP_COLOR;

  // setting RED filtered photodiodes to be read
  // setting S2 and S3 to LOW
  PORTC &= ~COLOR_SENSOR_S2;
  PORTC &= ~COLOR_SENSOR_S3;
  delay(COLOR_SENSOR_WAIT);
  colorR = getAvgReading(5);

  // setting GREEN filtered photodiodes to be read
  // setting S2 and S3 to HIGH
  PORTC |= (COLOR_SENSOR_S2);
  PORTC |= (COLOR_SENSOR_S3);
  delay(COLOR_SENSOR_WAIT);
  colorG = getAvgReading(5);

  // setting BLUE filtered photodiodes to be read
  // setting S2 to LOW and S3 to HIGH
  PORTC &= ~COLOR_SENSOR_S2;
  PORTC |= COLOR_SENSOR_S3;
  delay(COLOR_SENSOR_WAIT);
  colorB = getAvgReading(5);

  messagePacket.params[0] = colorR;
  messagePacket.params[1] = colorG;
  messagePacket.params[2] = colorB;

  sendResponse(&messagePacket);
}

void sendTooClose()
{
  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_RESPONSE;
  messagePacket.command = RESP_TOO_CLOSE;

  sendResponse(&messagePacket);
}

// ░█▀▀░█▀▀░▀█▀░█░█░█▀█
// ░▀▀█░█▀▀░░█░░█░█░█▀▀
// ░▀▀▀░▀▀▀░░▀░░▀▀▀░▀░░

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  forwardDist = 0;
  reverseDist = 0;
}

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
  // For movement commands, param[0] = distance, param[1] = speed.
  case COMMAND_FORWARD:
    sendOK();
    forward((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_REVERSE:
    sendOK();
    backward((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_TURN_LEFT:
    sendOK();
    left((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_TURN_RIGHT:
    sendOK();
    right((double)command->params[0], (float)command->params[1]);
    break;
  case COMMAND_STOP:
    sendOK();
    stop();
    break;
  case COMMAND_GET_STATS:
    sendOK();
    sendStatus();
    break;
  case COMMAND_CLEAR_STATS:
    sendOK();
    clearCounters();
    // clearOneCounter((int)command->params[0]);
    break;
  case COMMAND_GET_COLOUR:
    sendOK();
    sendColor();
    break;
  case COMMAND_GET_IR:
    sendOK();
    sendDistance();
    break;
  default:
    sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {

        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup()
{
  // put your setup code here, to run once:
  ALEX_DIAGONAL = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  ALEX_CIRC = CIRC_PI * ALEX_DIAGONAL;

  cli();
  setupEINT();
  setupSerial();
  setupcolour();
  startSerial();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
  case PACKET_TYPE_COMMAND:
    handleCommand(packet);
    break;

  case PACKET_TYPE_RESPONSE:
    break;

  case PACKET_TYPE_ERROR:
    break;

  case PACKET_TYPE_MESSAGE:
    break;

  case PACKET_TYPE_HELLO:
    break;
  }
}

// ░█▄█░█▀█░▀█▀░█▀█
// ░█░█░█▀█░░█░░█░█
// ░▀░▀░▀░▀░▀▀▀░▀░▀

void loop()
{
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
  // to track movement
  if (deltaDist > 0)
  {
    // // if get
    // //   too close to the object, the robot will stop and clear all deltaDist, newDist, deltaTicks, newTicks float dist = readUltrasonic();
    // if (dist > 0 && dist < 5)
    // {
    //   deltaDist = 0;
    //   newDist = 0;
    //   deltaTicks = 0;
    //   targetTicks = 0;
    //   sendTooClose();
    //   stop();
    // }
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOPPED)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0)
  {
    // dbprintf("TARGET TICKS: %d\n", targetTicks);
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == RIGHT)
    {
      if (rightReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else if (dir == STOPPED)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
