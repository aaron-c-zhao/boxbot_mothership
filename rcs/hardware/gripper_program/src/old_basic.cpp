// #include <Arduino.h>

// // Pins
// #define UPPER_IR_SENSOR D5
// #define LOWER_IR_SENSOR D1
// #define UPPER_LIMIT_SENSOR D2
// #define LOWER_LIMIT_SENSOR D6
// #define CENTRAL_LIMIT_SENSOR D7
// #define CLOSE_LIMIT_SWITCH D0
// #define CLOSE_VALVE D3


// // Constants
// #define LEFT_MOVE_C1 5000
// #define HIGH_MOVE_C2 1000
// #define LEFT_MOVE_C3 5000
// #define UP_MOVE_C4 2000
// #define LOWER_MOVE_C5 7000
// #define CLOSING_WAIT_C6 6000

// int state = 1;
// unsigned long state1Millis = 0;
// unsigned long state3Millis = 0;
// unsigned long state6Millis = 0;
// unsigned long state8Millis = 0;
// unsigned long state9Millis = 0;
// unsigned long state10Millis = 0;
// unsigned long state12Millis = 0;

// bool state5HighFirst = false;

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

//   state1Millis = millis();

//   Serial.begin(9600);
// }

// void loop() {
//   unsigned long currentMillis = millis();

//   switch (state) {
//     case 1:
//       // Move to left
//       // TODO check limit switches
//       Serial.println("left 1");
//       if (currentMillis - state1Millis >= LEFT_MOVE_C1) {
//         state = 2;
//       }
//       break;
//     case 2:
//       // Move low to click
//       Serial.println("low 2");
//       if (digitalRead(CENTRAL_LIMIT_SENSOR) == HIGH) {
//         state = 3;
//         state3Millis = currentMillis;
//       }
//       break;
//     case 3:
//       // Move high  
//       Serial.println("high 3");
//       if (currentMillis - state3Millis >= HIGH_MOVE_C2) {
//         state = 4;
//       }
//       break;
//     case 4:
//       // Move right until hit
//       Serial.println("right 4");
//       if (digitalRead(UPPER_IR_SENSOR) == HIGH) {
//         state = 5;
//         state5HighFirst = true;
//       } else if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
//         state = 5;
//       }
//       break;
//     case 5:
//       // Rotate until straight 
//       if (state5HighFirst) {
//         if (digitalRead(UPPER_IR_SENSOR) == LOW) {
//           Serial.println("right 5a");
//         } else {
//           Serial.println("rot CCW 5a");
//         }

//         //TODO: realign upper
//         if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
//           state = 6;
//           state6Millis = currentMillis;
//         }
//       } else {
//         if (digitalRead(LOWER_IR_SENSOR) == LOW) {
//           Serial.println("right 5b");
//         } else {
//           Serial.println("rot CW 5b");
//         }

//         //TODO: realign lower
//         if (digitalRead(UPPER_IR_SENSOR) == HIGH) {
//           state = 6;
//           state6Millis = currentMillis;
//         }
//       }
//       break;
//     case 6:
//       // Move left
//       Serial.println("left 6");
//       if (currentMillis - state6Millis >= LEFT_MOVE_C3) {
//         state = 7;
//       }
//       break;
//     case 7:
//       // Move down until lower hit loss
//       Serial.println("down 7");
//       if (digitalRead(LOWER_IR_SENSOR) == HIGH) {
//         state = 8;
//         state8Millis = currentMillis;
//       }
//       break;
//     case 8:
//       // Move up
//       Serial.println("up 8");
//       if (currentMillis - state8Millis >= UP_MOVE_C4) {
//         state = 9;
//         state9Millis = currentMillis;
//       }
//       break;
//     case 9:
//       // Move right until both ir sensors lose hit
//       Serial.println("right 9");  
      
//       if (digitalRead(UPPER_IR_SENSOR) == HIGH && digitalRead(LOWER_IR_SENSOR) == HIGH) {
//         state = 10;
//         state10Millis = currentMillis;
//       }
//       break;
//     case 10: 
//       // Done
//       Serial.println("down 10"); 
//       if (digitalRead(UPPER_LIMIT_SENSOR) == HIGH || digitalRead(CENTRAL_LIMIT_SENSOR) == HIGH || digitalRead(LOWER_LIMIT_SENSOR) == HIGH) {
//         Serial.println("rst 10");
//         state = 42;
//       }

//       if (currentMillis - state10Millis >= LOWER_MOVE_C5) {
//         state = 11;
//       }
//       break;
//     case 11: 
//       // Close gripper
//       Serial.println("close 11"); 
//       digitalWrite(CLOSE_VALVE, LOW);
//       state = 12;
//       state12Millis = currentMillis;
//       break;
//     case 12: 
//       // Closing gripper
//       Serial.println("closing 12"); 
//       //TODO: fix pull down here
//       if (currentMillis - state12Millis >= CLOSING_WAIT_C6) {
//         if (digitalRead(UPPER_LIMIT_SENSOR) == LOW) {
//           Serial.println("done");
//           state = 13;
//         } else {
//           //TODO: no delay
//           digitalWrite(CLOSE_VALVE, HIGH);
//           delay(5000);
//           state = 11;
//         }
//       }
//       break;
//     case 13:
//       // Done
//       Serial.println("done 13");  
//     default:
//       Serial.println("F");
//       break;
//   }
// }