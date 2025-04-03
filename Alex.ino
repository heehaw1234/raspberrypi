#include "serialize.h"
#include <avr/interrupt.h>
#include <math.h>
#include <Servo.h>
#include "packet.h"
#include "constants.h"

Servo servo1; // bottom left
Servo servo2; // bottom right
volatile TDirection dir;
bool manual = true;   // if false uses non blocking keyboard inp, else as initially intended
bool pulseon = false; // if false no ultrasonic on
bool coloron = false; // if false no color sensor on
bool open = true; // servo open

#define SERVO1_PIN 24   // servo pin mapping
#define SERVO2_PIN 25   
#define TRIG_PIN 26     // ultrasonic sensor  
#define ECHO_PIN 27

#define ALEX_LENGTH 25
#define ALEX_BREADTH 10
#define PI 3.141592654

/*
const int s0 = 35;
const int s1 = 36;
const int s2 = 37;
const int s3 = 38;
const int signal = 39;
unsigned long clear;
unsigned long red;  
unsigned long green;
unsigned long blue;
*/

static int STRAIGHT_LINE_SPEED = 100; // speed in fwd n reverse 0-100
static int TRAVERSE_SPEED = 50;       // speed when rotating 0-100

float alexDiagonal = 0.0;
float alexCirc = 0.0;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      4

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.

volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;

volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;

volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;

unsigned long deltaTicks;
unsigned long targetTicks;


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

    if(len == 0)
      return PACKET_INCOMPLETE;
    else
      return deserialize(buffer, len, packet);
    
}

float pulse(){
  float duration;
  float distance;

  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo pulse
  duration = pulseIn(ECHO_PIN, HIGH);

  if (duration == 0) {
    return -1.0;  // timeout
  }

  // Convert time to distance (in cm)
  distance = (duration * 0.0343) / 2;
  return distance; 
}

void servo(){
  if (open){ // open
    servo1.write(102);
    servo2.write(180);
  }
  else{ // close
    servo1.write(180);
    servo2.write(102);
  }
}

void sensecolor(){
  // red filter
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  red = pulseIn(signal, HIGH);
  delay(100);
  // green filter
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  green = pulseIn(signal, HIGH);
  delay(100);
  // blue filter
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  blue = pulseIn(signal, HIGH);
  delay(100);
}

void setspeed(int speedfwd, int speedtrav){
  STRAIGHT_LINE_SPEED = speedfwd; // speed in fwd n reverse 0-100
  TRAVERSE_SPEED = speedtrav;       // speed when rotating 0-100
}

void sendStatus() {

  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;

  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;

  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;

  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprintf(const char *format, ...) {
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
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
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
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
  // Use bare-metal to enable the pull-up resistors on pins
  // 19 and 18. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
}

ISR(INT2_vect){
  rightISR();
}

ISR(INT3_vect){
  leftISR();
}

// Functions to be called by INT2 and INT3 ISRs.
void leftISR()
{
  if (dir == FORWARD) {
    leftForwardTicks+=1;
    forwardDist = (unsigned long)((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } 
  else if (dir == BACKWARD) {
    leftReverseTicks+=1;
    reverseDist = (unsigned long)((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } 
  else if (dir == LEFT) { // counterclockwise
    leftReverseTicksTurns+=1; 
  }
  else if (dir == RIGHT) { // clockwise
    leftForwardTicksTurns+=1;
  }
}

void rightISR()
{
  if (dir == FORWARD) {
    rightForwardTicks+=1;
  } 
  else if (dir == BACKWARD) {
    rightReverseTicks+=1;
  } 
  else if (dir == LEFT) {
    rightForwardTicksTurns+=1;
  } 
  else if (dir == RIGHT) {
    rightReverseTicksTurns+=1;
  }
}

// Set up the external interrupt pins INT2 and INT3
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  EIMSK |= (1 << 2) | (1 << 3);
  EICRA |= 0b10100000;
  // Use bare-metal to configure pins 18 and 19 to be
  // falling edge triggered. Remember to enable
  // the INT2 and INT3 interrupts.
  // Hint: Check pages 110 and 111 in the ATmega2560 Datasheet.

}

// Implement the external interrupt ISRs below.
// INT3 ISR should call leftISR while INT2 ISR
// should call rightISR.


// Implement INT2 and INT3 ISRs above.

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
/*
  UBRR0H = (UBRR_VALUE >> 8);  // Set upper byte of UBRR
  UBRR0L = UBRR_VALUE;         // Set lower byte of UBRR

  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Set frame format: 8 data bits, 1 stop bit, no parity
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
*/
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

  int count=0;

  // Change Serial to Serial2/Serial3/Serial4 in later labs when using other UARTs

  while(Serial.available())
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
  //forward / reverse
  leftForwardTicks=0;
  rightForwardTicks=0;
  
  leftReverseTicks=0;
  rightReverseTicks=0;

  //turns
  leftForwardTicksTurns=0;
  rightForwardTicksTurns=0;
  
  leftReverseTicksTurns=0;
  rightReverseTicksTurns=0;
  
  leftRevs=0;
  rightRevs=0;
  forwardDist=0;
  reverseDist=0; 
}
// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Alex's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{ 
  double p1 = 0;
  float p2 = STRAIGHT_LINE_SPEED;
  float p3 = TRAVERSE_SPEED;
  if (manual){
    p1 = (double) command -> params[0];
    p2 = (double) command -> params[1];
    p3 = (double) command -> params[1];
  }

  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward(p1, p2);
      break;
    case COMMAND_REVERSE:
        sendOK();
        backward(p1, p2);
      break;
    case COMMAND_TURN_RIGHT:
        sendOK();
        right(p1, p3);
      break;
    case COMMAND_TURN_LEFT:
        sendOK();
        left(p1, p3);
      break;
    case COMMAND_STOP:
        sendOK();
        stop();
      break;
    case COMMAND_GET_STATS:
        sendStatus();
      break;
    case COMMAND_CLEAR_STATS:
        clearOneCounter(command->params[0]);
        sendOK();
      break;
    case COMMAND_SERVO:
        sendOK();
        open = !open;
        servo();
      break;
    case COMMAND_MANUAL:
        sendOK();
        manual = !manual;
        setspeed(command->params[0], command->params[1]); // sets straightlinespeed and traverse speed
      break;
    case COMMAND_PULSE:
        float x = pulse();
        dbprintf("distance: %df\n", x);
      break;
    case COMMAND_COLOR:
        coloron = !coloron;
        sensecolor();
        dbprintf("clear: %ld, red: %ld, green: %ld, blue: %ld\n", clear, red, green, blue);
      break;
    default:
      sendBadCommand();
  }
}

void waitForHello()
{
  int exit=0;

  while(!exit)
  {
    TPacket hello;
    TResult result;
    
    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if(result == PACKET_OK)
    {
      if(hello.packetType == PACKET_TYPE_HELLO)
      {
     

        sendOK();
        exit=1;
      }
      else
        sendBadResponse();
    }
    else
      if(result == PACKET_BAD)
      {
        sendBadPacket();
      }
      else
        if(result == PACKET_CHECKSUM_BAD)
          sendBadChecksum();
  } // !exit
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  //dbprintf("PI is %3.2f\n", PI);
  
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  enablePullups();
  initializeState();
  sei();
  pinMode(TRIG_PIN, OUTPUT);  // Trig is an output
  pinMode(ECHO_PIN, INPUT);
  /*
  pinMode(signal, INPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  */
  servo1.attach(SERVO2_PIN);
  servo2.attach(SERVO1_PIN);
  servo1.write(102); // initial servo position
  servo2.write(180);
  digitalWrite(s0, HIGH); // frequency scaling LL off, LH 2%, HL 20%, HH 100%
  digitalWrite(s1, LOW);
}

void handlePacket(TPacket *packet)
{
  switch(packet->packetType)
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

unsigned long computeDeltaTicks(float ang){
  unsigned long ticks = (unsigned long)((ang*alexCirc * COUNTS_PER_REV)  / (360.0 * WHEEL_CIRC));
  return ticks;
}

void left(float ang, float speed)
{
  if(ang == 0){
    deltaTicks=99999999;
  }
  else{
    deltaTicks=computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;
  //rightForwardTicksTurns
  ccw(ang,speed);
}
  

void right(float ang, float speed)
{
  if(ang == 0){
    deltaTicks=99999999;
  }
  else{
    deltaTicks=computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  cw(ang,speed);
}

void loop() {
  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2
  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);
  
  if(result == PACKET_OK)
    handlePacket(&recvPacket);
  else
    if(result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else
      if(result == PACKET_CHECKSUM_BAD)
      {
        sendBadChecksum();
      } 
      
  if(deltaDist > 0)
  {
      if(dir==FORWARD)
      {
        if(forwardDist > newDist)
           {
            deltaDist=0;
            newDist=0;
            stop();
           }
      }
      else
          if(dir == BACKWARD)
          {
            if(reverseDist > newDist)
            {
              deltaDist=0;
              newDist=0;
              stop();
            }
          }
          else
      if(dir == STOP)
        {
              deltaDist=0;
              newDist=0;
              stop();
        }
  }

  if(deltaTicks > 0){
    if(dir == LEFT){
      if(leftReverseTicksTurns >= targetTicks){
          
          deltaTicks=0;
          targetTicks =0;
          stop();
        }
      }
      else
      if(dir == RIGHT){
      if(rightReverseTicksTurns >= targetTicks){
          deltaTicks=0;
          targetTicks =0;
          stop();
        }
      }
      else
        if(dir == STOP){
          deltaTicks =0;
          targetTicks=0;
          stop();
          }
      
    
   }
    
}