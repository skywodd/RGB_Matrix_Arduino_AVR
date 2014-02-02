/* Dependencies */
#include <MsTimer2.h>

/* Compile time constants */
static const byte NB_HORIZONTAL_MATRIX = 2;
static const byte NB_VERTICAL_MATRIX = 1;

static const byte NB_LINES_PER_MATRIX = 32;
static const byte NB_COLUMNS_PER_MATRIX = 32;
static const byte MATRIX_SCANLINE_SIZE = 16;

static const byte NB_LINES_COUNT = NB_VERTICAL_MATRIX * NB_LINES_PER_MATRIX;
static const byte NB_COLUMNS_COUNT = NB_HORIZONTAL_MATRIX * NB_COLUMNS_PER_MATRIX;

/* Pin mapping */
#if defined(__AVR_ATmega2560__) // For Arduino Mega2560
// R1, G1, B1, R2, G2, B2 hard-wired on PA2~PA7
// A, B, C, D hard-wired on PF0~PF3
// CLK, OE, LAT hard-wired on PB1~PB3
#define DATA_PORT PORTA
#define DATA_DDR DDRA
#define ADDR_PORT PORTF
#define ADDR_DDR DDRF
#define CTRL_PORT PORTB
#define CTRL_PIN PINB
#define CTRL_DDR DDRB
#define CTRL_BITOFFSET 1
#define DEBUG_LED_SETUP() (DDRB |= 1 << 7, PORTB &= ~(1 << 7))
#define DEBUG_LED_OFF() (PORTB &= ~(1 << 7))
#define DEBUG_LED_ON() (PORTB |= 1 << 7)
#else  // For Arduino UNO
// R1, G1, B1, R2, G2, B2 hard-wired on PD2~PD7
// A, B, C, D hard-wired on PC0~PC3
// CLK, OE, LAT hard-wired on PB0~PB2
#define DATA_PORT PORTD
#define DATA_DDR DDRD
#define ADDR_PORT PORTC
#define ADDR_DDR DDRC
#define CTRL_PORT PORTB
#define CTRL_PIN PINB
#define CTRL_DDR DDRB
#define CTRL_BITOFFSET 0
#define DEBUG_LED_SETUP() (DDRB |= 1 << 5, PORTB &= ~(1 << 5))
#define DEBUG_LED_OFF() (PORTB &= ~(1 << 5))
#define DEBUG_LED_ON() (PORTB |= 1 << 5)
#endif

/** 
 * Framebuffer for RGB
 *
 * Array format : [line index] [column pixels bundle index] [color data R, G, B]
 * Data format : ... [R1 G1 B1][R2, G2, B2] ...
 */
static volatile byte framebuffer[NB_LINES_COUNT][NB_COLUMNS_COUNT / 8][3];

/**
 * Possible color enumeration
 */
enum {
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE,
  COLOR_YELLOW,
  COLOR_CYAN,
  COLOR_PINK,
  COLOR_WHITE,
  COLOR_BLACK  
};

/**
 * Set the color of a pixel in the framebuffer.
 * 
 * @param x X position of the pixel.
 * @param y Y position of the pixel.
 * @param Color Color to set.
 */
static void setPixelAt(const byte x, const byte y, const byte color) {
  volatile byte *pixel = framebuffer[y][x / 8];
  byte mask = 1 << (x & 7);
  switch(color) {
  case COLOR_RED:
    pixel[0] |= mask;
    pixel[1] &= ~mask;
    pixel[2] &= ~mask;
    break;
  case COLOR_GREEN:
    pixel[0] &= ~mask;
    pixel[1] |= mask;
    pixel[2] &= ~mask;
    break;
  case COLOR_BLUE:
    pixel[0] &= ~mask;
    pixel[1] &= ~mask;
    pixel[2] |= mask;
    break;
  case COLOR_YELLOW:
    pixel[0] |= mask;
    pixel[1] |= mask;
    pixel[2] &= ~mask;
    break;
  case COLOR_CYAN:
    pixel[0] &= ~mask;
    pixel[1] |= mask;
    pixel[2] |= mask;
    break;
  case COLOR_PINK:
    pixel[0] |= mask;
    pixel[1] &= ~mask;
    pixel[2] |= mask;
    break;
  case COLOR_WHITE:
    pixel[0] |= mask;
    pixel[1] |= mask;
    pixel[2] |= mask;
    break;
  default:
    pixel[0] &= ~mask;
    pixel[1] &= ~mask;
    pixel[2] &= ~mask;
    break;
  }
}

/**
 * Get the color of a pixel in the framebuffer.
 * 
 * @param x X position of the pixel.
 * @param y Y position of the pixel.
 * @return The color of the pixel.
 */
static byte getPixelAt(const byte x, const byte y) {
  volatile byte* pixel = framebuffer[y][x / 8];
  byte mask = 1 << (x & 7);
  byte r = !!(pixel[0] & mask);
  byte g = !!(pixel[1] & mask);
  byte b = !!(pixel[2] & mask);
  if(r && !g && !b)
    return COLOR_RED;
  if(!r && g && !b)
    return COLOR_GREEN;
  if(!r && !g && b)
    return COLOR_BLUE;
  if(r && g && !b)
    return COLOR_YELLOW;
  if(!r && g && b)
    return COLOR_CYAN;
  if(r && !g && b)
    return COLOR_PINK;
  if(r && g && b)
    return COLOR_WHITE;
  if(!r && !g && !b)
    return COLOR_BLACK;
}

/** Setup */
void setup() {

  // Setup pins
  pinMode(13, OUTPUT);
  DATA_DDR = 0b11111100; // Data port
  DATA_PORT = 0;

  ADDR_DDR = 0b1111; // Addr port
  ADDR_PORT = 0;

  CTRL_DDR = 0b111 << CTRL_BITOFFSET; // Ctrl port
  CTRL_PORT = 0b10 << CTRL_BITOFFSET;

  DEBUG_LED_SETUP();

  // Init frame buffers (all pixels black)
  memset((void*) framebuffer, 0, NB_LINES_COUNT * (NB_COLUMNS_COUNT / 8) * 3);

  // Init refresh timer
  MsTimer2::set(1, refreshDisplay);
  MsTimer2::start();
}

/** Loop */
void loop() {

  // Demo code
  static byte x = 0;
  static byte y = 0;
  static byte color = COLOR_RED;

  setPixelAt(x, y, color);

  if(++x == NB_COLUMNS_COUNT) {
    x = 0;

    if(++y == NB_LINES_COUNT) {
      y = 0;

      if(color == COLOR_BLACK)
        color = COLOR_RED;
      else
        ++color; 
    }
  }

  // No flood delay
  delay(10);
}

/** Display scaline refresh routine */
void refreshDisplay() {

  // Scan line index
  static byte scanlineIndex = 0;

  // Setup control lines and address lines
  CTRL_PORT = 0b110 << CTRL_BITOFFSET;
  ADDR_PORT = scanlineIndex;
  DEBUG_LED_ON();

  // For each vertical matrix
  for (int vMatrixIndex = NB_VERTICAL_MATRIX - 1; vMatrixIndex >= 0; --vMatrixIndex) {

    // Hardcoded 1/16 scanline with 32x32 matrix

    // Get the current lines buffers
    unsigned int lineOffset = vMatrixIndex * NB_LINES_PER_MATRIX + scanlineIndex;
    volatile byte (*lineBufferA)[3] = framebuffer[lineOffset];
    volatile byte (*lineBufferB)[3] = framebuffer[lineOffset + MATRIX_SCANLINE_SIZE];

    // For each column pixels bundle (8 columns = 1 bundle)
    for (int bundleIndex = (NB_COLUMNS_COUNT / 8) - 1; bundleIndex >= 0; --bundleIndex) {

      // Get column pixels bundle
      volatile byte (*bundleBufferA) = lineBufferA[bundleIndex];
      volatile byte (*bundleBufferB) = lineBufferB[bundleIndex];

      // Get RGB values for line N and N + 16
      byte r1 = bundleBufferA[0];
      byte g1 = bundleBufferA[1];
      byte b1 = bundleBufferA[2];
      byte r2 = bundleBufferB[0];
      byte g2 = bundleBufferB[1];
      byte b2 = bundleBufferB[2];

      // For each bits of each values
      for(int bitIndex = 7; bitIndex >= 0; --bitIndex) {

        // Shift out bits
        byte mask = 1 << bitIndex;
        DATA_PORT = (DATA_PORT & 0b11) | (!!(r1 & mask) << 2) | (!!(g1 & mask) << 3) | (!!(b1 & mask) << 4)| (!!(r2 & mask) << 5) | (!!(g2 & mask) << 6) | (!!(b2 & mask) << 7);
        CTRL_PIN = 1 << CTRL_BITOFFSET;
        CTRL_PIN = 1 << CTRL_BITOFFSET; // CLK pulse
      }
    }
  }

  // Trigger latch
  CTRL_PORT = 0;

  // Handle scan line overflow
  if (++scanlineIndex == MATRIX_SCANLINE_SIZE) {

    // Reset scan line index
    scanlineIndex = 0;
  }

  DEBUG_LED_OFF();
}