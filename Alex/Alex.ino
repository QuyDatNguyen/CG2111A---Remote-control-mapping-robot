
#include <serialize.h>
#include <math.h>
#include "packet.h"
#include "constants.h"
#include <stdarg.h>
// the code is modified for W9S1
volatile TDirection dir;

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;
// variables to keep track whether we have moved
// a command distance
// ref: w9s1 activity 4
unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;
// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC
#define RGBWait 300 // in milisecs
/*
 * Alex's configuration constants
 */

#define WHEEL_CIRC 20
#define ALEX_LENGTH 13
#define ALEX_BREADTH 13
#define PI 3.141592654
float alexDiagonal = 0.0;
float alexCirc = 0.0;
/*
 *    Alex's State Variables
 */

// Colour sensor
// #define S0 4
// #define S1 5
// #define S2 6
// #define S3 7
// #define sensorOut 8
// pins must be changed according to the arduino pins we use

// int frequency = 0;
//  #define RED_ARR = {255, 0, 0};
//  #define GRE_ARR = {0, 255, 0};
//  #define WHITE_ARR = {255, 255, 255};
//  #define NUMBCOL = 3 //number of colour to detect
//  static int allColourArray[NUMCOL][3] = {WHI_ARR, RED_ARR, GRE_ARR};
//  0-White; 1-RED; 2-GREEN
// change all these values after calibration
// int low_map[3] = {0, 0, 0);
// int high_map[3] = {0, 0, 0};
// used for mapping [0]-red, [1]-green, [2]-blue
unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
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

  targetTicks = leftReverseTicksTurns + deltaTicks;
  cw(ang, speed);
}
/*
 *
 * Alex Communication Routines.
 *
 */

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
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_RESPONSE;
  messagePacket.command = RESP_STATUS;
  // messagePacket.params = [
  //   leftForwardTicks,
  //   rightForwardTicks,
  //   leftReverseTicks,
  //   rightReverseTicks,
  //   leftForwardTicksTurns,
  //   rightForwardTicksTurns,
  //   leftReverseTicksTurns,
  //   rightReverseTicksTurns,
  //   forwardDist,
  //   reverseDist
  // ];
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

  sendResponse(&messagePacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

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

/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */
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
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.
  EICRA |= 0b10100000;
  EIMSK |= 0b00001100;
}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.

// Implement INT2 and INT3 ISRs above.
ISR(INT3_vect)
{
  leftISR();
}
ISR(INT2_vect)
{
  rightISR();
}
/*
 * Setup and start codes for serial communications
 *
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using the other UARTs
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs
}

/*
 * Alex's setup and run codes
 *
 */

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

// Clears one particular counter
void clearOneCounter(int which)
{

  switch (which)
  {
  case 0:
    clearCounters();
    break;

  case 1:
    leftForwardTicks = 0;
    break;

  case 2:
    rightForwardTicks = 0;
    break;

  case 3:
    leftReverseTicks = 0;
    break;

  case 4:
    rightReverseTicks = 0;
    break;

  case 5:
    forwardDist = 0;
    break;

  case 6:
    reverseDist = 0;
    break;
  }

  // clearCounters();
}
// colour sensor setup

// Intialize Alex's internal states
// void setupcolour() {
//  pinMode(S0, OUTPUT);
//  pinMode(S1, OUTPUT);
//  pinMode(S2, OUTPUT);
//  pinMode(S3, OUTPUT);
//  pinMode(sensorOut, INPUT);
//
//  // Setting frequency-scaling to 20%
//  digitalWrite(S0,HIGH);
//  digitalWrite(S1,LOW);
//
//  Serial.begin(9600); //for testing
//}
// int getAvgReading(int times, int low_map, int high_map) {
//   int reading = 0;
//   int total = 0;
//   for (int i = 0; i < times; i ++) {
//     reading = pulseIn(sensorOut, LOW);
//     reading = map(reading, high_map, low_map, 255, 0);
//     total += reading;
//     delay(50);
//   }
//   return total/times;
// }
/**
void on_colour(int colour) {
  //change to read Red value
  if (colour == 0) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    delay(RGBWait);
  }
  //change to read Green value
  if (colour == 1) {
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    delay(RGBWait);
  }
  if (colour == 2) {
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    delay(RGBWait);
  }
}
//function to clear filter on colour sensor, i.e to not read any value
void off_colour() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, LOW);
  delay(RGBWait);
}
int getColour() {
  for (int c = 0; c < 3; c ++) {
    if (SERIAL_MONITOR_ON) {
      Serial.print(colourStr[c]);
    }
    on_colour(c);
    colourArray[c] = getAvgReading(5, low_map[c], high_map[c]);

    off_colour();

    if (SERIAL_MONITOR_ON) {
      Serial.println(int(colourArray[c])); //show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
    }
  }
  int colour = -1;
  int min_dist = MIN_DIST;
  long long curr_dist;

  // compare the normalised colour values to the values we detected. Find the smallest difference and then determine the colour based off the smallest distance.
  for (int i = 0; i < 3; i ++) {
    curr_dist = 0;
    // Takes the sum of square values of difference between current colour detected
    // and reference values saved in allColourArray

    for (int j = 0; j < 3; j++) {
      curr_dist += (allColourArray[i][j] - colourArray[j])*(allColourArray[i][j] - colourArray[j]);
    }
    if (curr_dist < min_dist && curr_dist > 0) {
      colour = i;
      min_dist = curr_dist;
    }
  }
  Serial.println(colour);
  return colour;
}
*/
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
    clearOneCounter((int)command->params[0]);
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
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  // setupcolour();
  startSerial();
  enablePullups();
  initializeState();
  sei();
  Serial.begin(9600);
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

void loop()
{
  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  // forward(0, 100);

  // Uncomment the code below for Week 9 Studio 2

  // put your main code here, to run repeatedly:
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
  if (deltaDist > 0)
  {
    Serial.println(newDist);
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
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  if (deltaTicks > 0)
  {
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
    else if (dir == STOP)
    {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
