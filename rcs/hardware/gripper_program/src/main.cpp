#include <Arduino.h>

#define BAUD_RATE 153600

// Protocol constants (incoming)
#define PROT_CMD_COMPLETED 0x00
#define START_SEARCH 0x01
#define RESET 0x03
#define SET_GRIPPER_VALVE 0x04

// Protocol constants (outgoing)
#define GRIP_COMPLETED 0x06
#define GRIP_FAILED 0x07

#define DETECT_HIT 0x20 // v2
#define START_HIGHER_1 0x21 
#define RIGHT_EDGE_FOUND 0x22
#define HL_EDGE_FOUND 0x23

// Pins
#define OUTPUT_LED D4
#define UPPER_IR_SENSOR D5
#define LOWER_IR_SENSOR D1
#define UPPER_LIMIT_SENSOR D2
#define LOWER_LIMIT_SENSOR D6
#define CENTRAL_LIMIT_SENSOR D7
#define CLOSE_LIMIT_SWITCH D0
#define CLOSE_VALVE D3

// Constants
#define GRIPPER_CLOSE_WAIT 5000
#define GRIPPER_RETRIES 2
#define LIMIT_DEBOUNCE_TIME 100

// States
#define IDLE 0x00

#define MOVE_LOWER_1 0x03 // move down until limit switch hit

#define MOVING_HIGHER_3 0x05 // wait for gripper to move higher

#define MOVE_RIGHT_4 0x06 // move right until both IR sensors have lost vision (while sending message when it happens)

#define MOVE_MID_5 0x07 // wait for the gripper to move to the center again

#define MOVE_EDGE_7 0x0A // move down until lower IR sensor loses vision

#define MOVING_LOWER_10 0x11 // wait for the gripper to move, while checking limit switches and stop if triggered

#define CLOSE_GRIPPER_11 0x12 // close the gripper
#define WAITING_CLOSE_GRIPPER_11 0x13 // wait for the gripper to close for a constant amount

#define GRIP_DONE_12 0x14 // gripping done
#define GRIP_FAILED_13 0x15 // gripping failed

int state = IDLE;
unsigned long gripperOpenMillis = 0;
unsigned long gripperCloseMillis = 0;

// Debouncing constants
unsigned long limitUpperMillis = 0;
unsigned long limitUpperDebouncing = false;
unsigned long limitCentralMillis = 0;
unsigned long limitCentralDebouncing = false;
unsigned long limitLowerMillis = 0;
unsigned long limitLowerDebouncing = false;

bool commandCompleted = false;
bool upperSend = false;
bool lowerSend = false;
int gripTries = 0;

void IRAM_ATTR handleInput() {
  byte cmd = Serial.read();

  switch (cmd)
  {
    case PROT_CMD_COMPLETED: {
      commandCompleted = true;
      break;
    }
    case START_SEARCH: {
      state = MOVE_LOWER_1;
      break;
    }
    case RESET: {
      state = IDLE;
      break;
    }
    default: {
      // Unknown command, ignoring it
      break;
    }
  }
}

void sendSensorLocation(bool upper) {
  if (upper && !upperSend) {
    upperSend = true;
    Serial.write(DETECT_HIT);
    Serial.write(0x01);
  } else if (!upper && !lowerSend) {
    lowerSend = true;
    Serial.write(DETECT_HIT);
    Serial.write(0x00);
  }
}

void setup() {
  // IR Sensors
  pinMode(UPPER_IR_SENSOR, INPUT_PULLUP);
  pinMode(LOWER_IR_SENSOR, INPUT_PULLUP);

  // Limit switches side
  pinMode(UPPER_LIMIT_SENSOR, INPUT_PULLUP);
  pinMode(CENTRAL_LIMIT_SENSOR, INPUT_PULLUP);
  pinMode(LOWER_LIMIT_SENSOR, INPUT_PULLUP);

  // Limit switch top
  pinMode(CLOSE_LIMIT_SWITCH, INPUT_PULLDOWN_16);

  // Gripper close valve
  pinMode(CLOSE_VALVE, OUTPUT);
  digitalWrite(CLOSE_VALVE, HIGH);

  // Output LED
  pinMode(OUTPUT_LED, OUTPUT);
  digitalWrite(OUTPUT_LED, LOW);

  Serial.begin(BAUD_RATE);
}

void loop() {
  if (Serial.available()) {
    handleInput();
  }

  switch (state) {
    case IDLE:
      digitalWrite(CLOSE_VALVE, HIGH);
      commandCompleted = false;
      upperSend = false;
      lowerSend = false;
      gripTries = 0;
      break;
    case MOVE_LOWER_1: // client should move low gripper indefinately
      digitalWrite(OUTPUT_LED, LOW);
      //TODO(timanema): Remove debug

      // Check upper limit switch
      if (digitalRead(UPPER_LIMIT_SENSOR) == HIGH) {
        if (limitUpperDebouncing && millis() - limitUpperMillis >= LIMIT_DEBOUNCE_TIME) {
          state = MOVING_HIGHER_3;
          Serial.write(START_HIGHER_1);
          Serial.write(0x00);
        } else {
          limitUpperMillis = millis();
          limitUpperDebouncing = true;
        }
      } else {
        limitUpperDebouncing = false;
      }

      // Check central limit switch
      if (digitalRead(CENTRAL_LIMIT_SENSOR) == HIGH) {
        if (limitCentralDebouncing && millis() - limitCentralMillis >= LIMIT_DEBOUNCE_TIME) {
          state = MOVING_HIGHER_3;
          Serial.write(START_HIGHER_1);
          Serial.write(0x01);
        } else {
          limitCentralMillis = millis();
          limitCentralDebouncing = true;
        }
      } else {
        limitCentralDebouncing = false;
      }

      // Check lower limit switch
      if (digitalRead(UPPER_LIMIT_SENSOR) == HIGH) {
        if (limitLowerDebouncing && millis() - limitLowerMillis >= LIMIT_DEBOUNCE_TIME) {
          state = MOVING_HIGHER_3;
          Serial.write(START_HIGHER_1);
          Serial.write(0x02);
        } else {
          limitLowerMillis = millis();
          limitLowerDebouncing = true;
        }
      } else {
        limitLowerDebouncing = false;
      }
      break;
    case MOVING_HIGHER_3: // client should send this to the gripper when it finished moving higher
      if (commandCompleted) {
        state = MOVE_RIGHT_4;
        commandCompleted = false;
      }
      break;
    case MOVE_RIGHT_4: // client should be moving right indefinately
      if (digitalRead(UPPER_IR_SENSOR) == HIGH) {
        sendSensorLocation(true);
      } 
      if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
        sendSensorLocation(false);
      }

      if (upperSend && lowerSend) {
        //TODO(timanema): fix race condition better 
        delay(50);
        
        state = MOVE_MID_5;
        upperSend = false;
        lowerSend = false;

        // Results in the client moving back to the starting position
        Serial.write(RIGHT_EDGE_FOUND);
      }
      break;
    case MOVE_MID_5:
      if (commandCompleted) {
        state = MOVE_EDGE_7;
        commandCompleted = false;
      }
      break;  
    case MOVE_EDGE_7: // client should be moving up/down indefinately
      if (digitalRead(UPPER_IR_SENSOR) == HIGH) {
        sendSensorLocation(true);
      }
      if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
        sendSensorLocation(false);
      }

      if (upperSend || lowerSend) {
        state = MOVING_LOWER_10;

        // Results in the client moving to the calculated position
        Serial.write(HL_EDGE_FOUND);
      }
      break;
    case MOVING_LOWER_10: // client should be moving low for a constant amount
      // Failed if limit switches hit
      //TODO(timanema): Check if these switches also need debouncing
      if (digitalRead(UPPER_LIMIT_SENSOR) == HIGH || digitalRead(CENTRAL_LIMIT_SENSOR) == HIGH || digitalRead(LOWER_LIMIT_SENSOR) == HIGH) {
        state = GRIP_FAILED_13;
        Serial.write(GRIP_FAILED);
        break;
      }

      if (commandCompleted) {
        commandCompleted = false;
        state = CLOSE_GRIPPER_11;
      }
      break;  
    case CLOSE_GRIPPER_11:
      if (millis() - gripperOpenMillis >= GRIPPER_CLOSE_WAIT) {
        digitalWrite(CLOSE_VALVE, LOW);
        state = WAITING_CLOSE_GRIPPER_11;
        gripperCloseMillis = millis();
      }

      if (gripTries >= GRIPPER_RETRIES) {
        state = GRIP_FAILED_13;
        Serial.write(GRIP_FAILED);
      }
      break; 
    case WAITING_CLOSE_GRIPPER_11:
      if (millis() - gripperCloseMillis >= GRIPPER_CLOSE_WAIT) {
        // Check if gripper closed, retry if not
        if (digitalRead(CLOSE_LIMIT_SWITCH) == HIGH) {
          state = GRIP_DONE_12;
          Serial.write(GRIP_COMPLETED);
        } else {
          digitalWrite(CLOSE_VALVE, HIGH);
          state = CLOSE_GRIPPER_11;
          gripperOpenMillis = millis();
          gripTries += 1;
        }
      }
      break;  
    case GRIP_DONE_12:
      digitalWrite(OUTPUT_LED, HIGH);
      break;
    case GRIP_FAILED_13:
      digitalWrite(OUTPUT_LED, HIGH);
      break;  
    default:
      state = IDLE;
      break;
  }
}