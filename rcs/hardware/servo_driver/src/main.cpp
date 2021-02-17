#include <Arduino.h>
#include <Servo.h>

// Communication constant
#define BAUD_RATE 153600

// Protocol constants
#define SET_POS 0x00
#define DONE 0x01

// General constants
#define SERVO_PIN 9

// Servo vars
unsigned long changeMillis = 0;
unsigned int position = 1500;
unsigned int target = 1500;
unsigned int speed = 1500;
Servo servo;

union {
  byte buf[4];
  struct {
    unsigned int target;
    unsigned int speed;
  } value;
} data;

void setup() {
  servo.attach(SERVO_PIN);

  Serial.begin(BAUD_RATE);
  changeMillis = millis();
}

void loop() {
  if (Serial.available()) {
      byte cmd = Serial.read();

      switch(cmd) {
        case SET_POS: {
          Serial.readBytes(data.buf, 4);
          target = data.value.target;
          speed = data.value.speed;

          break;
        }
        case DONE: {
          if (position == target) {
            Serial.write(0x01);
          } else {
            Serial.write(0x00);
          }
        }
      }
  }

  int diff = target - position;
  int change = target > position ? speed : -speed;

  if (diff == 0 || (diff < 0 && -1 * diff < speed) || (diff > 0 && diff < speed )) {
    change = diff;
  }

  if (millis() - changeMillis > 20) {
    changeMillis = millis();
    position += change;
  }

  servo.writeMicroseconds(position);
}