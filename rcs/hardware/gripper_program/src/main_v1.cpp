// #include <Arduino.h>

// #define BAUD_RATE 153600

// // Protocol constants (incoming)
// #define PROT_CMD_COMPLETED 0x00
// #define START_SEARCH 0x01
// #define START_GRIP 0x02
// #define RESET 0x03

// // Protocol constants (outgoing)
// #define SEARCH_COMPLETED 0x04
// #define SEARCH_FAILED 0x05
// #define GRIP_COMPLETED 0x06
// #define GRIP_FAILED 0x07
// #define MOVE_DIR_CMD 0x08
// #define DETECT_HIT 0x09 // v2
// #define WAIT_ANALYSE 0x0A // v2

// // Protocol constants direction
// #define LEFT 0x00
// #define RIGHT 0x01
// #define UP 0x02
// #define DOWN 0x03
// #define HIGHER 0x04
// #define LOWER 0x05
// #define NONE 0x06
// #define CW_ROT 0x07
// #define CCW_ROT 0x08

// // Pins
// #define OUTPUT_LED D4
// #define UPPER_IR_SENSOR D5
// #define LOWER_IR_SENSOR D1
// #define UPPER_LIMIT_SENSOR D2
// #define LOWER_LIMIT_SENSOR D6
// #define CENTRAL_LIMIT_SENSOR D7
// #define CLOSE_LIMIT_SWITCH D0
// #define CLOSE_VALVE D3

// // Constants
// #define GRIPPER_CLOSE_WAIT 5000

// // States
// #define IDLE 0x00

// #define MOVE_LEFT_1 0x01 // move gripper to the left for a constant amount
// #define MOVING_LEFT_1 0x02 // wait for the gripper to move

// #define MOVE_LOWER_2 0x03 // move down until limit switch hit

// #define MOVE_HIGHER_3 0x04 // move gripper higher for a constant amount
// #define MOVING_HIGHER_3 0x05 // wait for gripper to move

// #define MOVE_RIGHT_4 0x06 // move right until first IR sensors loses vision

// #define ALIGN_VERTICAL_5 0x07 // align right sensors with vertical edge

// #define MOVE_LEFT_6 0x08 // move gripper to the left for a constant amount
// #define MOVING_LEFT_6 0x09 // wait for the gripper to move

// #define MOVE_DOWN_7 0x0A // move down until lower IR sensor loses vision

// #define MOVE_UP_8 0x0B // move gripper up for a constant amount
// #define MOVING_UP_8 0x0C // wait for the gripper to move

// #define MOVE_RIGHT_9 0x0D // move right until both IR sensors lose vision
// #define WAITING_GRIP_9 0x0F // done with search, ready to grip

// #define MOVE_LOWER_10 0x10 // move gripper lower for a constant amount
// #define MOVING_LOWER_10 0x11 // wait for the gripper to move, while checking limit switches and stop if triggered

// #define CLOSE_GRIPPER_11 0x12 // close the gripper
// #define WAITING_CLOSE_GRIPPER_11 0x13 // wait for the gripper to close for a constant amount

// #define GRIP_DONE_12 0x14 // gripping done
// #define GRIP_FAILED_13 0x15 // gripping failed

// int state = IDLE;
// unsigned long gripperOpenMillis = 0;
// unsigned long gripperCloseMillis = 0;

// bool state5HighFirst = false;

// bool commandCompleted = false;

// void IRAM_ATTR handleInput() {
//   byte cmd = Serial.read();

//   switch (cmd)
//   {
//     case PROT_CMD_COMPLETED: {
//       commandCompleted = true;
//       break;
//     }
//     case START_SEARCH: {
//       state = MOVE_LEFT_1;
//       break;
//     }
//     case START_GRIP: {
//       state = MOVE_LOWER_10;
//       break;
//     }
//     case RESET: {
//       state = IDLE;
//       break;
//     }
//     default: {
//       // Unknown command, ignoring it
//       break;
//     }
//   }
// }

// union {
//   byte asBytes[4];
//   float asFloat;
// } data;

// void moveDir(byte dir, bool indefinite, float amount) {
//   data.asFloat = amount;
  
//   Serial.write(MOVE_DIR_CMD);
//   Serial.write(dir);
//   Serial.write(indefinite);
//   Serial.write(data.asBytes, 4);
//   Serial.flush();
// }

// void setup() {
//   // IR Sensors
//   pinMode(UPPER_IR_SENSOR, INPUT_PULLUP);
//   pinMode(LOWER_IR_SENSOR, INPUT_PULLUP);

//   // Limit switches side
//   pinMode(UPPER_LIMIT_SENSOR, INPUT_PULLUP);
//   pinMode(CENTRAL_LIMIT_SENSOR, INPUT_PULLUP);
//   pinMode(LOWER_LIMIT_SENSOR, INPUT_PULLUP);

//   // Limit switch top
//   pinMode(CLOSE_LIMIT_SWITCH, INPUT_PULLDOWN_16);

//   // Gripper close valve
//   pinMode(CLOSE_VALVE, OUTPUT);
//   digitalWrite(CLOSE_VALVE, HIGH);

//   // Output LED
//   pinMode(OUTPUT_LED, OUTPUT);

//   Serial.begin(BAUD_RATE);
// }

// void loop() {
//   if (Serial.available()) {
//     handleInput();
//   }

//   digitalWrite(OUTPUT_LED, commandCompleted ? LOW : HIGH);
//   delay(5);

//   switch (state) {
//     case IDLE:
//       digitalWrite(CLOSE_VALVE, HIGH);
//       commandCompleted = false;
//       break;
//     case MOVE_LEFT_1:
//       // TODO: signal moving left for constant amount
//       moveDir(LEFT, false, 42.1);
//       state = MOVING_LEFT_1;
//       break;
//     case MOVING_LEFT_1:
//       if (commandCompleted) {
//         state = MOVE_LOWER_2;
//         commandCompleted = false;

//         // TODO: signal moving lower indef.
//         moveDir(LOWER, true, 0.1);
//       } 
//       break; 
//     case MOVE_LOWER_2:
//       if (digitalRead(CENTRAL_LIMIT_SENSOR) == HIGH) {
//         state = MOVING_HIGHER_3;
//         // TODO: signal moving higher for constant amount
//         moveDir(HIGHER, false, 42.2);
//       } 
//       break;
//     case MOVING_HIGHER_3:
//       if (commandCompleted) {
//         state = MOVE_RIGHT_4;
//         commandCompleted = false;
//         // TODO: signal moving higher for constant amount
//         moveDir(RIGHT, true, 0.3);
//       }
//       break;
//     case MOVE_RIGHT_4: 
//       if (digitalRead(UPPER_IR_SENSOR) == HIGH) {
//         state = ALIGN_VERTICAL_5;
//         state5HighFirst = true;
//       } else if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
//         state = ALIGN_VERTICAL_5;
//         state5HighFirst = false;
//       }
//       break;
//     case ALIGN_VERTICAL_5:
//       if (state5HighFirst) {
//         if (digitalRead(UPPER_IR_SENSOR) == LOW) {
//           // TODO: signal indef right
//           moveDir(RIGHT, true, 0);
//         } else {
//           // TODO: signal indef rot CCW
//           moveDir(CCW_ROT, true, 0);
//         }

//         //TODO: realign upper
//         if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
//           state = MOVE_LEFT_6;
//         }
//       } else {
//         if (digitalRead(LOWER_IR_SENSOR) == LOW) {
//           // TODO: signal indef right
//           moveDir(RIGHT, true, 0);
//         } else {
//           // TODO: signal indef rot CW
//           moveDir(CW_ROT, true, 0);
//         }

//         //TODO: realign lower
//         if (digitalRead(UPPER_IR_SENSOR) == HIGH) {
//           state = MOVE_LEFT_6;
//         }
//       }
//       break;
//     case MOVE_LEFT_6:
//       // TODO: signal left for constant amount
//       moveDir(LEFT, false, 42);
//       state = MOVING_LEFT_6;
//       break;
//     case MOVING_LEFT_6:
//       if (commandCompleted) {
//         commandCompleted = false;
//         state = MOVE_DOWN_7;
//       } 
//       break;  
//     case MOVE_DOWN_7:
//       if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
//         state = MOVE_UP_8;
//       }
//       break;
//     case MOVE_UP_8:
//       // TODO: signal up for constant amount
//       moveDir(UP, false, 42);
//       state = MOVING_UP_8;
//       break;
//     case MOVING_UP_8:
//       if (commandCompleted) {
//         commandCompleted = false;
//         state = MOVE_RIGHT_9;
//       }
//       break;  
//     case MOVE_RIGHT_9:
//       if (digitalRead(UPPER_IR_SENSOR) == HIGH && digitalRead(LOWER_IR_SENSOR) == HIGH) {
//         //TODO: signal stop movement
//         moveDir(NONE, true, 0);
//         Serial.write(SEARCH_COMPLETED);
//         state = WAITING_GRIP_9;
//       }
//       break;  
//     case WAITING_GRIP_9:
//       // busy wait
//       break;  
//     case MOVE_LOWER_10:
//       //TODO: signal constant movement lower
//       moveDir(LOWER, false, 42);
//       state = MOVING_LOWER_10;
//       break; 
//     case MOVING_LOWER_10:
//       // Failed if limit switches hit
//       if (digitalRead(UPPER_LIMIT_SENSOR) == HIGH || digitalRead(CENTRAL_LIMIT_SENSOR) == HIGH || digitalRead(LOWER_LIMIT_SENSOR) == HIGH) {
//         state = GRIP_FAILED_13;
//         Serial.write(GRIP_FAILED);
//         break;
//       }

//       if (commandCompleted) {
//         commandCompleted = false;
//         state = CLOSE_GRIPPER_11;
//       }
//       break;  
//     case CLOSE_GRIPPER_11:
//       if (millis() - gripperOpenMillis >= GRIPPER_CLOSE_WAIT) {
//         digitalWrite(CLOSE_VALVE, LOW);
//         state = WAITING_CLOSE_GRIPPER_11;
//         gripperCloseMillis = millis();
//       }
//       break; 
//     case WAITING_CLOSE_GRIPPER_11:
//       if (millis() - gripperCloseMillis >= GRIPPER_CLOSE_WAIT) {
//         // Check if gripper closed, retry if not
//         if (digitalRead(UPPER_LIMIT_SENSOR) == LOW) {
//           state = GRIP_DONE_12;
//           Serial.write(GRIP_COMPLETED);
//         } else {
//           //TODO: introduce max retries
//           digitalWrite(CLOSE_VALVE, HIGH);
//           state = CLOSE_GRIPPER_11;
//           gripperOpenMillis = millis();
//         }
//       }
//       break;  
//     case GRIP_DONE_12:
//       break;
//     case GRIP_FAILED_13:
//       break;  
//     default:
//       state = IDLE;
//       break;
//   }
// }