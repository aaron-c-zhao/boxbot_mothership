#include <Adafruit_NeoPixel.h>
#define PIN 3
#define NUM_LEDS 45
Adafruit_NeoPixel strip = Adafruit_NeoPixel(45, 3, NEO_GRB + NEO_KHZ800);
 //green 0x00 0x00 0xFF
 //Red 0xFF 0x00 0x00
 //Blue 0x00 0xFF 0x00
 
void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}


void loop() {
  colorWipe(0x00,0x00,0xFF, 90);
}

void colorWipe(byte red, byte green, byte blue, int SpeedDelay) {
  for(uint16_t i=0; i<NUM_LEDS; i++) {
      setPixel(i, red, green, blue);
      showStrip();
      delay(SpeedDelay);
  }

}

void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}
