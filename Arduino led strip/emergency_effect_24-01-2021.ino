#include <Adafruit_NeoPixel.h>
#define PIN 5
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
  // Slower:
  // Strobe(0xff, 0x77, 0x00, 10, 100, 1000);
  // Fast:
  Strobe(0xFF, 0x00, 0x00, 10, 500, 1000);
}

void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause){
  for(int j = 0; j < StrobeCount; j++) {
    setAll(red,green,blue);
    showStrip();
    delay(FlashDelay);
    setAll(0,0,0);
    showStrip();
    delay(FlashDelay);
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
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}
