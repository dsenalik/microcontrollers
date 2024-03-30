
/*
Tools -> Board -> Adafruit AVR Boards -> ProTrinket 5v/16MHz (USB)
Tools -> Port -> /dev/ttyS0
Tools -> Programmer -> USBtinyISP
Sketch -> Upload Using Programmer
*/

/* Physical Layout East< >West       Row (East no row 4)
*   9 <- 0   |       0 -> 19         0
* ---------- | --------------------
*  10 -> 19  |      39 <- 20         1
* ---------- | --------------------
*            |      40 -> 59         2
*  29 <- 20  | --------------------
* ---------- |      79 <- 60         3
*            | --------------------
*  30 -> 39  |      80 -> 99         4
* ---------- | --------------------
*  Or/W      |      Or       = +5V   (Connector Red)
*  Brn/W     |      Grn/W    = Gnd   (Connector Grn)
*  Brn       |      Grn      = Data  (Connector White)
*
* Logical Columns:
*          1 -> 30
*/

#include <Adafruit_NeoPixel.h>


// Neopixel parameters
#define PIN_W      4
#define PIN_E      8
#define NUM_SHELVES_W 5
#define NUM_SHELVES_E 4
#define STRIPLEN_W 20
#define STRIPLEN_E 10
#define NUM_LEDS_W NUM_SHELVES_W*STRIPLEN_W  // 5 shelves 20 LEDs each = 100
#define NUM_LEDS_E NUM_SHELVES_E*STRIPLEN_E  // 4 shelves 10 LEDs each = 40
int tdelay = 400;  // milliseconds between neopixel updates
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
// NEO_KHZ800 800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
// NEO_KHZ400 400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
// NEO_GRB Pixels are wired for GRB bitstream (most NeoPixel products)
// NEO_RGB Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
// Use NEO_GRB for strips, NEO_RGB for Amazon ALITOVE WS2811 12mm Diffused Digital RGB LED Pixel Light Individually Addressable Round LED Pixels Module IP68 Waterproof DC 5V 50pcs/Set
Adafruit_NeoPixel strip_w = Adafruit_NeoPixel(NUM_LEDS_W, PIN_W, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_e = Adafruit_NeoPixel(NUM_LEDS_E, PIN_E, NEO_GRB + NEO_KHZ800);

// serial port speed
#define SERIALBPS 115200

void showStrip() {
  strip_w.show();
  strip_e.show();
}
 
void lightsOff() {
  int i;
  for(i=0; i<NUM_LEDS_W; i++) {
    strip_w.setPixelColor(i, strip_w.Color(0, 0, 0));
  }  
  for(i=0; i<NUM_LEDS_E; i++) {
    strip_e.setPixelColor(i, strip_e.Color(0, 0, 0));
  }  
  showStrip();
  Serial.println("lightsOff() called");
}

int getPhysicalLoc (int row, int col) {
  // Pass row 0..4, and col 0..29
  // Will return value+1024 for east, +0 for west
  int physloc;
  // East cabinet
  if (col < STRIPLEN_E) {
    // Rows 0, 2 = right to left
    if ((row % 2) == 0) {
      physloc = (((row + 1) * STRIPLEN_E - 1) - col) + 1024;
    }
    // Rows 1, 3 = left to right
    else {
      physloc = ((row * STRIPLEN_E) + col) + 1024;
    }
  }
  // West cabinet
  else {
    col = col - STRIPLEN_E;
    // Rows 0, 2, 4 = left to right
    if ((row % 2) == 0) {
      physloc = (row * STRIPLEN_W) + col;
    }
    // Rows 1, 3 = right to left
    else {
      physloc = ((row + 1) * STRIPLEN_W - 1) - col;
    }
  }
  return physloc;
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
 
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }
  return c;
}

void rainbowCycle(int SpeedDelay) {
  byte *c;
  uint16_t i, j, row;
  int physicalLoc;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i<(STRIPLEN_E+STRIPLEN_W); i++) {
      c=Wheel(((i * 256 / (STRIPLEN_E+STRIPLEN_W)) + j) & 255);
      for (row=0; row<=4; row++) {
        physicalLoc = getPhysicalLoc(row, i);
        if (physicalLoc >= 1024) {
          // East cabinet
          strip_e.setPixelColor(physicalLoc-1024, strip_e.Color(*c, *(c+1), *(c+2)));
        }
        else {
          // West cabinet
          strip_w.setPixelColor(physicalLoc, strip_w.Color(*c, *(c+1), *(c+2)));
        }
      }
    }
    showStrip();
    delay(SpeedDelay);
  }
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS_W; i++ ) {
    strip_w.setPixelColor(i, strip_w.Color(red, green, blue));
  }
  showStrip();
}

void setup() {
  strip_w.begin();
  strip_e.begin();
  lightsOff();
}

void loop() {
  rainbowCycle(tdelay);
}
