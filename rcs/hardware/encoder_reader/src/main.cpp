#include <Arduino.h>
#include <Wire.h>

#define FREQUENCY    160                  // valid 80, 160
//
#include "ESP8266WiFi.h"
extern "C" {
#include "user_interface.h"
}

// # CHECKSUM (JS)
// let k1 = ~(((h >> 1) & 1) ^ ((h >> 3) & 1) ^ ((h >> 5) & 1) ^ ((l >> 1) & 1) ^ ((l >> 3) & 1) ^ ((l >> 5) & 1) ^ ((l >> 7) & 1)) & 1
// let k0 = ~(((h >> 0) & 1) ^ ((h >> 2) & 1) ^ ((h >> 4) & 1) ^ ((l >> 0) & 1) ^ ((l >> 2) & 1) ^ ((l >> 4) & 1) ^ ((l >> 6) & 1)) & 1
// let correct = (h >> 7) & 1 === k1 && (h >> 6) & 1 === k0

// #RES
// let n = (h & 0x3f) << 8 | l

// # TURN
// let turns = -(n & 0x2000) + (n & ~0x2000)

#define RECV_TIMEOUT 2
#define TURN_DELAY 10

#define RWPin D5
#define ENC_CNT 1

// Forward declarations
bool check(byte l, byte h);
void sendPosRequest(byte address);
void sendTurnRequest(byte address);

unsigned long turnWaitMillis = 0;
unsigned long recWaitMillis = 0;
unsigned long wdtMillis = 0;

byte id[ENC_CNT] = {0x54};
byte l_rsp[2][ENC_CNT];
byte h_rsp[2][ENC_CNT];

int enc_i = 0;
byte buf = 0x00;
int recv = 0;
bool pos = true;

void setup() {
  pinMode(RWPin, OUTPUT);

  // Start serial
  Serial.begin(2000000);
  while(!Serial);

  // Disable WiFi
  WiFi.forceSleepBegin();
  delay(1);
  system_update_cpu_freq(FREQUENCY);

  // Reset all encoders on start
  digitalWrite(RWPin, HIGH);
  for (int i = 0; i < ENC_CNT; i++) {
    Serial.write(id[i]);
    Serial.write(0x75);
    Serial.flush();
  }
  digitalWrite(RWPin, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  // Send request if buffer hasn't been updated in at least RECV_TIMEOUT ms
  if (currentMillis - recWaitMillis > RECV_TIMEOUT) {
    recWaitMillis = currentMillis;
    recv = 0;

    // Cycle to next encoder
    enc_i = (enc_i + 1) % ENC_CNT;

    // Check if positions were updated
    if (enc_i == 0 && !pos) {
      pos = true;
      turnWaitMillis = currentMillis;
    }

    // Check if all encoders have been read, and read turns if last check was more than TURN_DELAY ms ago
    if (enc_i == 0 && currentMillis - turnWaitMillis > TURN_DELAY) {
      turnWaitMillis = currentMillis;
      pos = false;
    }

    if (pos) {
      sendPosRequest(id[enc_i]);
    } else {
      sendTurnRequest(id[enc_i]);
    }
  }

  if (Serial.available() > 0) {
    byte in = Serial.read();

    if (recv == 0) {
      buf = in;
      recv = 1;
      recWaitMillis = currentMillis;
    } else {
      // Force encoder update next loop()
      recv = 0;
      recWaitMillis = 0;

      if (check(buf, in)) {
        l_rsp[pos][enc_i] = buf;
        h_rsp[pos][enc_i] = in & 0x3f;

        //TODO(timanema): Remove debug write
        Serial.write(pos);
        Serial.write(l_rsp[pos][enc_i]);
        Serial.write(h_rsp[pos][enc_i]);
      }
    }
  }

  // Reset WDT
  if (currentMillis - wdtMillis >= 500) {
    wdtMillis = currentMillis;
    wdt_reset();
  }
}

bool check(byte l, byte h) {
  byte k1 = ~(((h >> 1) & 1) ^ ((h >> 3) & 1) ^ ((h >> 5) & 1) ^ ((l >> 1) & 1) ^ ((l >> 3) & 1) ^ ((l >> 5) & 1) ^ ((l >> 7) & 1)) & 1;
  byte k0 = ~(((h >> 0) & 1) ^ ((h >> 2) & 1) ^ ((h >> 4) & 1) ^ ((l >> 0) & 1) ^ ((l >> 2) & 1) ^ ((l >> 4) & 1) ^ ((l >> 6) & 1)) & 1;

  return ((h >> 7) & 1) == k1 && ((h >> 6) & 1) == k0;
}

void sendPosRequest(byte address) {
  digitalWrite(RWPin, HIGH);
  Serial.write(address);
  Serial.flush();
  digitalWrite(RWPin, LOW);
}

void sendTurnRequest(byte address) {
  digitalWrite(RWPin, HIGH);
  Serial.write(address + 0x01);
  Serial.flush();
  digitalWrite(RWPin, LOW);
}