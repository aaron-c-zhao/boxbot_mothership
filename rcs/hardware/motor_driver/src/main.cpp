#include <Arduino.h>
#include <AccelStepper.h>

// Communication constant
#define BAUD_RATE 153600
#define WATCHDOG_TIMEOUT 2000

// Protocol constants
#define PING 0x00
#define SET_MOTOR 0x01
#define GET_MOTOR_STATUS 0x02
#define START_HOME 0x03
#define STOP_HOME 0x04
#define ZERO 0x05
#define STOP 0x06
#define RESET 0x07

// Lights + driver output
#define DIR D0 // High at boot
#define PULSE D1
#define EN D2

// Lights output
#define LEFT_WARN D3 // Boot fails if low at boot
#define RIGHT_WARN D5 
#define FAULT_WARN D4 // Boot fails if pulled low + connected to board LED

// Inputs
#define LEFT_LIMIT D6
#define RIGHT_LIMIT D7

// Motor constants
#define MAX_SPEED 2000
#define LEFT_POSITIVE true

/*
Error code flags
0x00: No error
0x01: Blocked by limit switches
0x02: Motor disabled
0x03: Watchdog fault
*/
#define NO_ERROR 0x00
#define BLOCKED_LIMIT 0x01
#define MOTOR_DISABLED 0x02
#define WATCHDOG_FAULT 0x03
#define HOMING 0x04

/*
States:
Blocked left =  0b00000001
Blocked right = 0b00000010
Idle =          0b00000100
Moving =        0b00001000
Disabled =      0b00010000
Fault =         0b00100000
Homing =        0b01000000
Homed =         0b10000000
*/

// General motor variables
AccelStepper stepper(AccelStepper::DRIVER, PULSE, DIR);
volatile bool disabled = false;
volatile bool fault = false;
volatile bool leftLimit = false;
volatile bool rightLimit = false;
volatile bool movingLeft = false;
volatile bool homing = false;
volatile bool homed = false;

// Communication definitions
unsigned long watchdogMillis = 0;
byte setCallback(const float & speed, const long & position, const bool & dis);

// Limit switch interrupts
void IRAM_ATTR leftTrigger() {
  if (digitalRead(LEFT_LIMIT) == LOW) {
    leftLimit = false;
  } else {
    leftLimit = true;
  }
}

void IRAM_ATTR rightTrigger() {
  if (digitalRead(RIGHT_LIMIT) == LOW) {
    rightLimit = false;
  } else {
    rightLimit = true;
  }
}

union {
  byte buf[8];
  struct {
    float speed;
    long location;
  } value;
} data; 

union {
  byte buf[4];
  struct {
    float speed;
  } value;
} homeSpeed; 

void IRAM_ATTR handleInput() {
  byte cmd = Serial.read();
  byte buf[4];

  watchdogMillis = millis();
  fault = false;

  switch (cmd)
  {
    case PING: {
      Serial.write(0x00);
      Serial.write(0x42);
      break;
    }
    case SET_MOTOR: {
      Serial.readBytes(data.buf, 8);
      Serial.readBytes(buf, 1);
      
      setCallback(data.value.speed, data.value.location, buf[0]);
      break;
    }
    case GET_MOTOR_STATUS: {
      bool idle = (fabs(stepper.speed()) < 0.0001);
      byte state =  leftLimit | 
                    (rightLimit << 1) | 
                    (idle << 2) |
                    (!idle << 3) |
                    (disabled << 4) |
                    (fault << 5) |
                    (homing << 6) |
                    (homed << 7);
      data.value.speed = stepper.speed();
      data.value.location = stepper.currentPosition();

      Serial.write(0x02);
      Serial.write(state);
      Serial.write(data.buf, 8);
      break;
    }
    case START_HOME: {
      if (!homing) {
        Serial.readBytes(homeSpeed.buf, 4);

        homing = true;
        homed = false;
        stepper.setSpeed(homeSpeed.value.speed);
        movingLeft = true;
      }
      break;
    }
    case STOP_HOME: {
      if (homing) {
        homing = false;
        homed = false;
        stepper.setSpeed(0);
      }
      break;
    }
    case ZERO: {
      stepper.setCurrentPosition(0);
      break;
    }
    case STOP: {
      stepper.setSpeed(0);
      break;
    }
    case RESET: {
      stepper.setCurrentPosition(0);
      stepper.moveTo(0);
      stepper.setSpeed(0);
      break;
    }
    default: {
      // Unknown command, ignoring it
      break;
    }
  }
}

void setup() {
  // Setup serial connection
  Serial.begin(BAUD_RATE);

  // Driver + lights
  pinMode(EN, OUTPUT);
  digitalWrite(EN, disabled);

  // Lights
  pinMode(LEFT_WARN, OUTPUT);
  pinMode(RIGHT_WARN, OUTPUT);
  pinMode(FAULT_WARN, OUTPUT);

  // Inputs
  pinMode(LEFT_LIMIT, INPUT_PULLUP);
  pinMode(RIGHT_LIMIT, INPUT_PULLUP);

  leftLimit = digitalRead(LEFT_LIMIT);
  rightLimit = digitalRead(RIGHT_LIMIT);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_LIMIT), leftTrigger, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_LIMIT), rightTrigger, CHANGE);

  // Motor setup
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setMinPulseWidth(3);
  stepper.setSpeed(0);

  movingLeft = LEFT_POSITIVE;
  watchdogMillis = 0;
}

void loop() {
  if (Serial.available()) {
    handleInput();
  }

  digitalWrite(EN, disabled | fault);
  digitalWrite(LEFT_WARN, leftLimit);
  digitalWrite(RIGHT_WARN, rightLimit);
  digitalWrite(FAULT_WARN, fault);

  if (disabled || fault || (leftLimit && movingLeft) || (rightLimit && !movingLeft)) {
    stepper.setSpeed(0);

    if (homing && !disabled && !fault) {
      homing = false;
      homed = true;
      stepper.setCurrentPosition(0);
      stepper.moveTo(0);
      stepper.setSpeed(0);
    }
  }

  if (homing) {
    stepper.runSpeed();
  } else {
    stepper.runSpeedToPosition();
  }

  if (millis() - watchdogMillis >= WATCHDOG_TIMEOUT) {
    stepper.setSpeed(0);
    fault = true;
  }
}

byte setCallback(const float & speed, const long & position, const bool & dis) {
  if (homing) {
    return HOMING;
  }

  // bool left = (LEFT_POSITIVE && speed > 0) || (!LEFT_POSITIVE && speed < 0);
  bool left = (LEFT_POSITIVE && position > stepper.currentPosition()) || 
  (!LEFT_POSITIVE && position < stepper.currentPosition());

  // Check if there is a watchdog fault
  if (fault) {
    return WATCHDOG_FAULT;
  }

  // Check if the motor is disabled (and won't be enabled by this request)
  if (dis && disabled) {
    return MOTOR_DISABLED;
  }

  // Check if the move is not allowed
  if ((left && leftLimit) || (!left && rightLimit)) {
    return BLOCKED_LIMIT;
  }

  movingLeft = left;
  stepper.moveTo(position);
  stepper.setSpeed(speed);
  disabled = dis;

  if (dis) {
    stepper.setSpeed(0);
  }

  return NO_ERROR;
}
