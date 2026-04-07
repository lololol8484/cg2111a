/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"

// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}

// =============================================================
// E-Stop state machine
// =============================================================

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */

volatile TState buttonState = STATE_RUNNING;
volatile bool stateChanged = false;

// The firstPnR variable tracks whether this is the first or second press-and-release
static bool firstPnR = true;
// The _timerTicks variable maintains a count of how many 0.1 ms ticks have passed by.
static unsigned int _timerTicks = 0;
// Our last time and current time variables in ms
static unsigned long _lastTime = 0;
static unsigned long _currentTime = 0;
// The THRESHOLD value for debouncing
#define THRESHOLD 50  // in ms
#define BUTTON_PIN PD0 // External Interrupt 0 is on Digital Pin 21 for Mega

ISR(INT0_vect) {
  // Read current pin state
  bool pressed = PIND & (1 << BUTTON_PIN);
  if (millis() - _lastTime > THRESHOLD) {
    _lastTime = millis();
    if(firstPnR){
      if (pressed){
        buttonState = STATE_STOPPED;
        stateChanged = true;
      }
      else{
        firstPnR = !firstPnR;
      }
    }
    else{
      if(!pressed){
        buttonState = STATE_RUNNING;
        stateChanged = true;
        firstPnR = !firstPnR;
      }
    }
  }
}

/*
ISR(TIMER4_COMPA_vect) {
  // triggers every 0.1 ms
  _timerTicks++;
  if (_timerTicks == 10) {
    _timerTicks = 0;
    _currentTime++; // shows the current time in ms
  }
}
*/

// =============================================================
// Color sensor (TCS3200)
// =============================================================

/*
 * TODO (Activity 2): Implement the color sensor.
 *
 * Wire the TCS3200 to the Arduino Mega and configure the output pins
 * (S0, S1, S2, S3) and the frequency output pin.
 *
 * Use 20% output frequency scaling (S0=HIGH, S1=LOW).  This is the
 * required standardised setting; it gives a convenient measurement range and
 * ensures all implementations report the same physical quantity.
 *
 * Use a timer to count rising edges on the sensor output over a fixed
 * window (e.g. 100 ms) for each color channel (red, green, blue).
 * Convert the edge count to hertz before sending:
 *   frequency_Hz = edge_count / measurement_window_s
 * For a 100 ms window: frequency_Hz = edge_count * 10.
 *
 * Implement a function that measures all three channels and stores the
 * frequency in Hz in three variables.
 *
 * Define your own command and response types in packets.h (and matching
 * constants in pi_sensor.py), then handle the command in handleCommand()
 * and send back the channel frequencies (in Hz) in a response packet.
 *
 * Example skeleton:
 *
 *   static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
 *       // Set S2/S3 for each channel, measure edge count, multiply by 10
 *       *r = measureChannel(0, 0) * 10;  // red,   in Hz
 *       *g = measureChannel(1, 1) * 10;  // green, in Hz
 *       *b = measureChannel(0, 1) * 10;  // blue,  in Hz
 *   }
 */

#define S0 PA0
#define S1 PA1
#define S2 PA2
#define S3 PA3

volatile uint32_t edgeCount = 0;

ISR(INT1_vect) {
  edgeCount++;
}

void colorSensorSetup() {
  DDRA |= (1 << S0) | (1 << S1) | (1 << S2) | (1 << S3);

  DDRD &= ~(1 << PD1);

  PORTA |= (1 << S0);
  PORTA &= ~(1 << S1);

  EICRA |= (1 << ISC11) | (1 << ISC10);
  EIMSK &= ~(1 << INT1);
}

uint32_t measureColor(uint8_t s2State, uint8_t s3State) {
  if (s2State)
        PORTA |= (1 << S2);
  else
        PORTA &= ~(1 << S2);

  if (s3State)
        PORTA |= (1 << S3);
  else
        PORTA &= ~(1 << S3);

  edgeCount = 0;

  //TCCR5A = 0;
  //TCCR5B = 0;
  //TCNT5 = 0;
  //OCR5A = 24999;

  EIMSK |= (1 << INT1);

  //TCCR5B = (1 << WGM52) | (1 << CS51) | (1 << CS50);

  //while (TCNT5<OCR5A);

  uint32_t prev_time = millis();
  while(millis() - prev_time < 100);

  //TCCR5B = 0;
  EIMSK &= ~(1 << INT1);

  return edgeCount * 10;
}

void readColor(uint32_t *r, uint32_t *g, uint32_t *b) {
  *r = measureColor(0, 0);
  *g = measureColor(1, 1);
  *b = measureColor(0, 1);
}


// =============================================================
// Movement
// =============================================================

volatile unsigned long leftEncoderTicks = 0;
volatile unsigned long rightEncoderTicks = 0;
volatile unsigned long targetTicks = 0;
uint8_t robotVelocity = 128;
#define distanceToTicks 1

ISR(INT2_vect){
  leftEncoderTicks++;
}

ISR(INT3_vect){
  rightEncoderTicks++;
}

void moveForward(int distance){
  targetTicks = distance * distanceToTicks;
  forward(robotVelocity);
  while(leftEncoderTicks + rightEncoderTicks < targetTicks and buttonState == STATE_RUNNING){
    TPacket incoming;
    if (receiveFrame(&incoming)) {
      handleCommand(&incoming);
    }
  }
  stop();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
}

void moveBackward(int distance){
  targetTicks = distance * distanceToTicks;
  backward(robotVelocity);
  while(leftEncoderTicks + rightEncoderTicks < targetTicks and buttonState == STATE_RUNNING){
    TPacket incoming;
    if (receiveFrame(&incoming)) {
      handleCommand(&incoming);
    }
  }
  stop();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
}

void moveLeft(int distance){
  targetTicks = distance * distanceToTicks;
  ccw(robotVelocity);
  while(leftEncoderTicks + rightEncoderTicks < targetTicks and buttonState == STATE_RUNNING){
    TPacket incoming;
    if (receiveFrame(&incoming)) {
      handleCommand(&incoming);
    }
  }
  stop();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
}

void moveRight(int distance){
  targetTicks = distance * distanceToTicks;
  cw(robotVelocity);
  while(leftEncoderTicks + rightEncoderTicks < targetTicks and buttonState == STATE_RUNNING){
    TPacket incoming;
    if (receiveFrame(&incoming)) {
      handleCommand(&incoming);
    }
  }
  stop();
  leftEncoderTicks = 0;
  rightEncoderTicks = 0;
}


// =============================================================
// Arm
// =============================================================

// MG90S Servo: 0° at 0.5ms, 90° at 1.5ms, 180° at 2.5ms, 20ms period
// To get servo precision to 1°, we need timer precision to about 10us (20 ticks)

volatile int basePos = 20, shoulderPos = 0, elbowPos = 80, gripperPos = 120; // degs
volatile int baseTarget = 20, shoulderTarget = 0, elbowTarget = 80, gripperTarget = 120; // degs
volatile int baseMin = 0, baseMax = 50;
volatile int shoulderMin = 0, shoulderMax = 30;
volatile int elbowMin = 70, elbowMax = 115;
volatile int gripperMin = 90, gripperMax = 120;
volatile int ticksArr[4] = { 1444, 2444, 5222, 8888 }; // no. of ticks before switching which servo to set to HIGH
volatile int servoCount = 0; // indicates which servo is currently HIGH
volatile int msPerDeg = 10;
volatile int msPerDegMin = 5, msPerDegMax = 20; // to be updated

ISR(TIMER5_COMPA_vect) { // 20ms reached
  // we only update the current position of the servos once every 20ms
  int stepSize = 20 / msPerDeg;
  if(abs(basePos - baseTarget) >= stepSize){
    basePos += (basePos < baseTarget) ? stepSize : -stepSize;
  }
  else{
    basePos = baseTarget;
  }
  if(abs(shoulderPos - shoulderTarget) >= stepSize){
    shoulderPos += (shoulderPos < shoulderTarget) ? stepSize : -stepSize;
  }
  else{
    shoulderPos = shoulderTarget;
  }
  if(abs(elbowPos - elbowTarget) >= stepSize){
    elbowPos += (elbowPos < elbowTarget) ? stepSize : -stepSize;
  }
  else{
    elbowPos = elbowTarget;
  }
  if(abs(gripperPos - gripperTarget) >= stepSize){
    gripperPos += (gripperPos < gripperTarget) ? stepSize : -stepSize;
  }
  else{
    gripperPos = gripperTarget;
  }
  // +1000 -> base 0.5ms, *200/9 -> *2*2000/180 -> *2 for 2ms range, *2000 to convert ms to ticks, /180 to convert from degs
  ticksArr[0] = 1000 + (basePos * 200) / 9;
  ticksArr[1] = ticksArr[0] + 1000 + (shoulderPos * 200) / 9;
  ticksArr[2] = ticksArr[1] + 1000 + (elbowPos * 200) / 9;
  ticksArr[3] = ticksArr[2] + 1000 + (gripperPos * 200) / 9;
  PORTC |= (1 << 0); // set D37 to HIGH
  servoCount = 0; // reset servoCount
  OCR5B = ticksArr[0];
}

ISR(TIMER5_COMPB_vect) { // time to switch which servo to set to HIGH
  PORTC &= ~(1 << servoCount);
  servoCount++;
  if(servoCount <= 3) {
    PORTC |= (1 << servoCount);
    OCR5B = ticksArr[servoCount];
  }
}

void robotArmSetup() {
  DDRC = 0b00001111; // set D34-D37 as output
  PORTC = 0b00000000; // set D34-D37 to LOW

  // COM bits all set to 0 as we are not using OC3A or OC3B
  // WGM bits set to 0b0100 as we are using CTC mode
  // CS bits set to 0b010 as we are using prescaler of 8
  // OCR3A set to 40000 to get 20ms period
  // OCR3B will be used to generate the 0.5-2.5ms pulse for each servo
  TCCR5A = 0b00000000;
  TCNT5 = 0;
  OCR5A = 40000;
  OCR5B = 3000;
  TIMSK5 = 0b00000110;
  TCCR5B = 0b00001010;
}


// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */


static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = 1 - buttonState;
            stateChanged = false;
            firstPnR = !firstPnR;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(buttonState);
            break;

    case COMMAND_COLOR_SENSOR:
      uint32_t r, g, b;
      TPacket resp;
      readColor(&r, &g, &b);
      memset(&resp, 0, sizeof(resp));
      resp.packetType = PACKET_TYPE_RESPONSE;
      resp.command = RESP_COLOR_SENSOR;
      resp.params[0] = r;
      resp.params[1] = g;
      resp.params[2] = b;
      TResponseType temp;
      sendResponse(temp,0);
      sendFrame(&resp);
      break;

    case COMMAND_MOVE_FORWARD:
      moveForward(String(cmd->data).toInt());
      break;

    case COMMAND_MOVE_BACKWARD:
      moveBackward(String(cmd->data).toInt());
      break;

    case COMMAND_MOVE_LEFT:
      moveLeft(String(cmd->data).toInt());
      break;

    case COMMAND_MOVE_RIGHT:
      moveRight(String(cmd->data).toInt());
      break;

    case COMMAND_CHANGE_VELOCITY:
      robotVelocity = String(cmd->data).toInt();
      break;

    case COMMAND_ARM_HOME:
      baseTarget = 20;
      shoulderTarget = 0;
      elbowTarget = 80;
      gripperTarget = 120;
      break;

    case COMMAND_ARM_BASE:
      baseTarget = constrain(String(cmd->data).toInt(), baseMin, baseMax);
      break;

    case COMMAND_ARM_SHOULDER:
      shoulderTarget = constrain(String(cmd->data).toInt(), shoulderMin, shoulderMax);
      break;

    case COMMAND_ARM_ELBOW:
      elbowTarget = constrain(String(cmd->data).toInt(), elbowMin, elbowMax);
      break;

    case COMMAND_ARM_GRIPPER:
      gripperTarget = constrain(String(cmd->data).toInt(), gripperMin, gripperMax);
      break;

    case COMMAND_ARM_VELOCITY:
      msPerDeg = constrain(String(cmd->data).toInt(), msPerDegMin, msPerDegMax);
      break;

            // call the color reading function;
            // send packet

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
    }
}


// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif
  // TODO (Activity 1): configure the button pin and its external interrupt,
  // then call sei() to enable global interrupts.
  DDRD &= ~(1 << BUTTON_PIN);
  // ISC00 = 1, ISC01 = 0 triggers on "Any logical change" for INT0
  // Falling edge for INT2 and INT3
  EICRA |= 0b10100001;
  // Set PD2 and PD3 as input
  DDRD &= ~(1 << DDD2);
  DDRD &= ~(1 << DDD3);
  // Enable internal pull-up on PD2 and PD3
  PORTD |= (1 << PORTD2);
  PORTD |= (1 << PORTD3);
  // Enable INT0, INT2, INT3
  EIMSK |= (1 << INT0) | (1 << INT2) | (1 << INT3);
  //TCCR4A = 0b00000000;
  //TIMSK4 = 0b00000010;
  //TCNT4 = 0;
  //OCR4A = 200;
  //TCCR4B = 0b00001010;
  colorSensorSetup();
  robotArmSetup();
  sei();
}

void loop() {
    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
