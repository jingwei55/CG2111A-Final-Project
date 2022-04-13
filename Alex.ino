//#include <buffer.h>

#include <serialize.h>
#include <stdarg.h>
#include <math.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "packet.h"
#include "constants.h"
//#include "ultrasonic.h"

//TBuffer *buf;

#define PIN_9 (1 << 9)
#define PIN_5 (1 << 5)
#define PIN_6 (1 << 6)
#define PIN_10 (1 << 10)

// mask definitions for Power management
 #define PRR_TWI_MASK            0b10000000
 #define PRR_SPI_MASK            0b00000100
 #define ADCSRA_ADC_MASK         0b01111111
 #define PRR_ADC_MASK            0b10000000
 #define PRR_TIMER2_MASK         0b01000000
 #define PRR_TIMER0_MASK         0b00100000 
 #define PRR_TIMER1_MASK         0b00001000
 #define SMCR_SLEEP_ENABLE_MASK  0b00000001
 #define SMCR_IDLE_MODE_MASK     0b11110001

void WDT_off(void) { 
    /* Global interrupt should be turned OFF here if not already done so */ 
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1<<WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional
    time-out */ 
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00; 
    /* Global interrupt should be turned ON
    here if subsequent operations after calling this function 
    DO NOT require turning off global interrupt */ 
} 

void setupPowerSaving() {   
    // Turn off the Watchdog Timer 
    WDT_off();
    // Modify PRR to shut down TWI
    PRR |= PRR_TWI_MASK;        
    // Modify PRR to shut down SPI
    PRR |= PRR_SPI_MASK;
    // Modify ADCSRA to disable ADC,
    // then modify PRR to shut down ADC 
    ADCSRA &= ADCSRA_ADC_MASK;
    PRR |= PRR_ADC_MASK;
    // Set the SMCR to choose the IDLE sleepmode
    // Do not set the Sleep Enable (SE) bit yet 
    SMCR &= SMCR_IDLE_MODE_MASK;
    // Set Port B Pin 5 as output pin, then write a logic LOW   
    // to it so that the LED tied to Arduino's Pin 13 is OFF.
    DDRB |= 0b00100000;
    PORTB &= 0b11011111;
}

void putArduinoToIdle() {
    // shut down timer 0, 1, 2
    PRR |= PRR_TIMER0_MASK;
    PRR |= PRR_TIMER1_MASK;
    PRR |= PRR_TIMER2_MASK;
    // Modify SE bit to enable sleep
    SMCR |= SMCR_SLEEP_ENABLE_MASK;
    
    sleep_cpu();

    SMCR &= ~SMCR_SLEEP_ENABLE_MASK;

    PRR &= ~PRR_TIMER0_MASK;
    PRR &= ~PRR_TIMER1_MASK;
    PRR &= ~PRR_TIMER2_MASK;
}

typedef enum
{
 STOP=0,
 FORWARD=1,
 BACKWARD=2,
 LEFT=3,
 RIGHT=4
} TDirection;
volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

 

// Number of ticks per revolution from the 
// wheel encoder.

#define COUNTS_PER_REV      181

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled 
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC          20.41

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF                  5   // Left forward pin
#define LR                  6   // Left reverse pin
#define RF                  10  // Right forward pin
#define RR                  9  // Right reverse pin

//#define PII            3.141592654
#define ALEX_LENGTH   16
#define ALEX_BREADTH  5
float AlexDiagonal;
float AlexCirc;

unsigned long deltaTicks;
unsigned long targetTicks;

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders for moving forward and backwards.
volatile unsigned long leftForwardTicks; 
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks; 
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning 
volatile unsigned long leftForwardTicksTurns; 
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns; 
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
//volatile unsigned long leftRevs=0;
//volatile unsigned long rightRevs=0;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;
//unsigned long param[10];

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

void sendStatus()
{
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
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
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

void dbprintf(char *format, ...) {
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
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs. 
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
  
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  if (dir == FORWARD){
    leftForwardTicks++;
    forwardDist = (unsigned long) ((float) leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
    //dbprint("ForwardDist: %d\n", forwardDist);
  } else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist = (unsigned long) ((float) leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
    //dbprint("ForwardDist: %d\n", reverseDist);
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
  }
  
//  Serial.print("LEFT: ");
//  Serial.println(leftTicks/(double)181*20.42);
}

void rightISR()
{
  if (dir == FORWARD){
    rightForwardTicks++;
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  }
//  Serial.print("RIGHT: ");
//  Serial.println(rightTicks/(double)181*20.42);
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  cli();
  EICRA = 0b00001010;
  EIMSK |= 0b00000011;
  sei();
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}



// Implement INT0 and INT1 ISRs above.

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
//  UBRR0L = 103;
//  UBRR0H = 0;
//  UCSR0C = 0b00000110;
//
//  UCSR0A = 0;

  
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
  //UCSR0B = 0b10111000;
//  UCSR0B = 0b00011000;
//
//  initBuffer(buf, 1000);
}

void setupLED() {
  DDRB |= 0b00110000;
}
void greenOn() {
  PORTB |= 0b00100000;
}
void greenOff() {
  PORTB &= 0b11011111;
}
void redOn() {
  PORTB |= 0b00010000;
}
void redOff() {
  PORTB &= 0b11101111;
}
// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid. 
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
//  int count=0;
//
//  while(dataAvailable(buf))
//    readBuffer(buf, buffer + count++);
//    
//  return count;

}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
//  for (int i=0; i < len; i++)
//    writeBuffer(buf, buffer + i);
}

/*
 * Alex's motor drivers.
 * 
 */

// void setupPWM() {
//  TCNT0 = 0;
//  TCCR0A = 0b10100001;
//  OCR0A = 0;
//  OCR0B = 0;
//  TCCR0B = 0b00000001;
//  TIMSK0 |= 0b110;
//  TCNT1 = 0;
//  TCCR1A = 0b10100001;
//  OCR1AH = 0;
//  OCR1AL = 0;
//  OCR1BH = 0;
//  OCR1BL = 0;
//  TCCR1B = 0b00000001;
//  TIMSK1 |= 0b110;
//}

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:  
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 9, PB1, OC1A
   *    B2In - pIN 10, PB2, OC1B
   */

   pinMode(LF, OUTPUT);
   pinMode(LR, OUTPUT);
   pinMode(RR, OUTPUT);
   pinMode(RF, OUTPUT);
//   DDRD |= 0b01100000;
//   DDRB |= 0b00000110;    
//        
//   TCNT0 = 0;
//   TCCR0A |= 0b10100001;
//   TIMSK0 |= 0b00000110;                   
//   TCNT1 = 0;
//   TCCR1A |= 0b10100001;
//   TIMSK1 |= 0b00000110;
  
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
//  OCR0A = 0;
//  OCR0B = 0;
//  TCCR0B = 0b00000001;
//  OCR1AL = 0;
//  OCR1AH = 0;
//  OCR1BL = 0;
//  OCR1BH = 0;
//  TCCR1B = 0b00000001;
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if(speed < 0.0)
    speed = 0;

  if(speed > 100.0)
    speed = 100.0;

  return (int) ((speed / 100.0) * 255.0);
}

//void analogWriteBareMetal(int port, int val) {
//    switch(port) {
//        case LR:
//            OCR0A = val;
//            break;
//        case LF:
//            OCR0B = val;
//            break;
//        case RF:
//            OCR1BH = 0;
//            OCR1BL = val;
//            break;
//        case RR:
//            OCR1AH = 0;
//            OCR1AL = val;
//            break;
//     }
//}
//void analogWrite(int portNum, int val) {
//  switch (portNum) {
//    case RR: 
//      OCR1AH = 0;
//      OCR1AL = val;
//      break;
//    case RF:
//      OCR1BH = 0;
//      OCR1BL = val;
//      break;
//    case LR:
//      OCR0B = val;
//      break;
//    case LF:
//      OCR0A = val;
//      break;  
//  }
//}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  dist = dist*3/4;
  clearCounters();
  dir = FORWARD;
  int val = pwmVal(speed);
  if(dist > 0)
    deltaDist = dist;
  else{
    deltaDist=9999999;}
    
  newDist=forwardDist + deltaDist;

  // For now we will ignore dist and move
  // forward indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  
  analogWrite(LF, val);
  analogWrite(RF, val);
  analogWrite(LR, 0);
  analogWrite(RR, 0);
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  dist = dist;
  clearCounters();
  if(dist > 0)
    deltaDist = dist;
  else{
    deltaDist=9999999;}
    
  newDist=reverseDist + deltaDist;

  dir = BACKWARD;

  int val = pwmVal(speed);

  // For now we will ignore dist and 
  // reverse indefinitely. We will fix this
  // in Week 9.

  // LF = Left forward pin, LR = Left reverse pin
  // RF = Right forward pin, RR = Right reverse pin
  // This will be replaced later with bare-metal code.
  analogWrite(LR, val);
  analogWrite(RR, 0.96*val);
  analogWrite(LF, 0);
  analogWrite(RF, 0);
}

unsigned long computeDeltaTicks(float ang)
{
  unsigned long ticks = (unsigned long) ((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
  return ticks;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  ang = ang/10*4.2;
  clearCounters();
  dir = LEFT;
  int val = pwmVal(speed);
  if(ang == 0)
    deltaTicks=99999999;
  else {
    deltaTicks=computeDeltaTicks(ang);
  }
  targetTicks = leftReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val);
  analogWrite(RF, val);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  ang = ang/10*4.4;
  clearCounters();
  dir = RIGHT;
  int val = pwmVal(speed);
  if(ang == 0)
    deltaTicks=99999999;
  else {
    deltaTicks=computeDeltaTicks(ang);
  }
  targetTicks = rightReverseTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, val);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  
  analogWrite(RF, 0);
  analogWrite(RR, 0);
  analogWrite(LF, 0);
  analogWrite(LR, 0);
}

/*
 * Alex's 
 and run codes
 * 
 */

// Clears all our counters
void clearCounters()
{
  leftForwardTicks=0;
  rightForwardTicks=0;
  leftReverseTicks=0; 
  rightReverseTicks=0;
  leftForwardTicksTurns=0; 
  rightForwardTicksTurns=0;
  leftReverseTicksTurns=0; 
  rightReverseTicksTurns=0;
  forwardDist=0;
  reverseDist=0; 
}

// Clears one particular counter
void clearOneCounter(int which)
{
  switch(which)
  {
    case 0:
      leftForwardTicks=0;
      break;

    case 1:
      rightForwardTicks=0;
      break;

    case 2:
      leftReverseTicks=0; 
      break;

    case 3:
      rightReverseTicks=0;
      break;

    case 4:
      leftForwardTicksTurns=0; 
      break;

    case 5:
      rightForwardTicksTurns=0;
      break;

    case 6:
      leftReverseTicksTurns=0; 
      break;

      case 7:
      rightReverseTicksTurns=0;
      break;

      case 8:
      forwardDist=0;
      break;

      case 9:
      reverseDist=0; 
      break;
  }
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch(command->command)
  {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        forward((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_REVERSE:
        sendOK();
        reverse((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_LEFT:
        sendOK();
        left((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_RIGHT:
        sendOK();
        right((float) command->params[0], (float) command->params[1]);
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

    /*
     * Implement code for other commands here.
     * 
     */
        
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
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * 
                      ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  //setupPWM();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupPowerSaving();
  setupLED();
  //setupUS();
  sei();
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

void loop() {

// Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

 // forward(0, 100);
//int dst[2];
//getDistUS(dst);
// Uncomment the code below for Week 9 Studio 2

//  if (dst[0] != 0 || dst[1] != 0) {
//    Serial.print("Distance1: ");
//    Serial.println(dst[0]);
//    delay(200);
//    Serial.print("Distance2: ");
//    Serial.println(dst[1]);
//  }
  
  if(deltaDist > 0)
  {
    if(dir==FORWARD)
    {
      greenOn();
      redOff();
      if(forwardDist >= newDist)
      {
        deltaDist=0;
        newDist=0;
        stop();
        greenOff();
        redOn();
      }
  }
  else
  if(dir == BACKWARD)
  {
    greenOn();
    redOff();
    if(reverseDist >= newDist)
    {
      deltaDist=0;
      newDist=0;
      stop();
      greenOff();
      redOn();
    }
  }
  else
    if(dir == STOP)
    {
      deltaDist=0;
      newDist=0;
      stop();
      greenOff();
      redOn();
    }
  }
  
  if (deltaTicks > 0) {
    if (dir == LEFT) {
     redOff();
     greenOn();
     if (leftReverseTicksTurns >= targetTicks)
     {
      deltaTicks=0;
      targetTicks=0;
      greenOff(); 
      redOn();
      stop();
      }
      
    }
    else
      if (dir == RIGHT) {
        //dbprintf("%u", rightReverseTicksTurns);
        redOff();
        greenOn();
        if (rightReverseTicksTurns >= targetTicks) {
          deltaTicks=0;
          targetTicks=0;
          redOn();
          greenOff();
          stop();
        }
      }
      else 
        if (dir == STOP)
        {
          deltaTicks=0;
          targetTicks=0;
          putArduinoToIdle();
          stop();
          redOn();
          greenOff();
      }
    }

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
    
      
}
