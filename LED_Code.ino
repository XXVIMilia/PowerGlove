
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 90 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 5 // Time (in milliseconds) to pause between pixels

void setup() {
  Serial.begin(115200);
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
}

global data;
void loop() {
 if(Serial.available()>0){
  global data;
  String data =Serial.readStringUntil('\n');
  Serial.print(data);
 }
  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(200, 0, 50));
    pixels.setPixelColor(i-1, pixels.Color(100, 0, 25));
    pixels.setPixelColor(i-2, pixels.Color(50, 0, 5));
    pixels.setPixelColor(i+1, pixels.Color(100, 0, 25));
    pixels.setPixelColor(i+2, pixels.Color(50, 0, 5));
    pixels.setPixelColor(i-3, pixels.Color(0, 150, 150));
    pixels.setPixelColor(i+3, pixels.Color(0, 150, 150));

    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop
  }
  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<data; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(100-i, pixels.Color(200, 0, 50));
    pixels.setPixelColor(100-(i-1), pixels.Color(100, 0, 25));
    pixels.setPixelColor(100-(i-2), pixels.Color(50, 0, 5));
    pixels.setPixelColor(100-(i+1), pixels.Color(100, 0, 25));
    pixels.setPixelColor(100-(i+2), pixels.Color(50, 0, 5));
    pixels.setPixelColor(100-(i-3), pixels.Color(0, 150, 150));
    pixels.setPixelColor(100-(i+3), pixels.Color(0, 150, 150));

    pixels.show();   // Send the updated pixel colors to the hardware.

    delay(DELAYVAL); // Pause before next pass through loop
  }
}