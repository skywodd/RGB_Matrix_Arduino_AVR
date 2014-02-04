/* Compile time constants */
static const byte NB_HORIZONTAL_MATRIX = 1;
static const byte NB_VERTICAL_MATRIX = 2;

static const byte NB_LINES_PER_MATRIX = 32;
static const byte NB_COLUMNS_PER_MATRIX = 32;
static const byte MATRIX_SCANLINE_SIZE = 16;

static const byte NB_LINES_COUNT = NB_VERTICAL_MATRIX * NB_LINES_PER_MATRIX;
static const byte NB_COLUMNS_COUNT = NB_HORIZONTAL_MATRIX * NB_COLUMNS_PER_MATRIX;

/* Pin mapping */
#if defined(__AVR_ATmega2560__) // Mega2560
static const byte PIN_R1 = 24;  // R1
static const byte PIN_G1 = 25;  // G1
static const byte PIN_B1 = 26;  // B1
static const byte PIN_R2 = 27;  // R2
static const byte PIN_G2 = 28;  // G2
static const byte PIN_B2 = 29;  // B2
static const byte PIN_A = A0;   // A
static const byte PIN_B = A1;   // B
static const byte PIN_C = A2;   // C
static const byte PIN_D = A3;   // D
static const byte PIN_CLK = 50; // CLK
static const byte PIN_OE = 51;  // OE
static const byte PIN_LAT = 52;	// LAT
#else // UNO
static const byte PIN_R1 = 2;   // R1
static const byte PIN_G1 = 3;   // G1
static const byte PIN_B1 = 4;   // B1
static const byte PIN_R2 = 5;   // R2
static const byte PIN_G2 = 6;   // G2
static const byte PIN_B2 = 7;   // B2
static const byte PIN_A = A0;   // A
static const byte PIN_B = A1;   // B
static const byte PIN_C = A2;   // C
static const byte PIN_D = A3;   // D
static const byte PIN_CLK = 8;  // CLK
static const byte PIN_OE = 9;   // OE
static const byte PIN_LAT = 10; // LAT
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
 * @param color Color to set.
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
 * @return The state of the pixel.
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
  pinMode(PIN_R1, OUTPUT);
  pinMode(PIN_G1, OUTPUT);
  pinMode(PIN_B1, OUTPUT);
  pinMode(PIN_R2, OUTPUT);
  pinMode(PIN_G2, OUTPUT);
  pinMode(PIN_B2, OUTPUT);
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);
  pinMode(PIN_D, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_OE, OUTPUT);
  pinMode(PIN_LAT, OUTPUT);

  // Init pins
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  digitalWrite(PIN_C, LOW);
  digitalWrite(PIN_D, LOW);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_OE, HIGH);
  digitalWrite(PIN_LAT, LOW);

  // Init frame buffers (all pixels black)
  memset((void*) framebuffer, 0, NB_LINES_COUNT * (NB_COLUMNS_COUNT / 8) * 3);
}

/** Loop */
void loop() {

  // Refresh the display as fast as possible
  refreshDisplay();
}

/** Display scaline refresh routine */
void refreshDisplay() {

  // Scan line index
  static byte scanlineIndex = 0;
  digitalWrite(13, HIGH);

  // Setup control lines and address lines
  digitalWrite(PIN_OE, HIGH);
  digitalWrite(PIN_LAT, HIGH);
  digitalWrite(PIN_A, (scanlineIndex & 1) ? HIGH : LOW);
  digitalWrite(PIN_B, (scanlineIndex & 2) ? HIGH : LOW);
  digitalWrite(PIN_C, (scanlineIndex & 4) ? HIGH : LOW);
  digitalWrite(PIN_D, (scanlineIndex & 8) ? HIGH : LOW);

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
        digitalWrite(PIN_R1, (r1 & mask) ? HIGH : LOW);
        digitalWrite(PIN_G1, (g1 & mask) ? HIGH : LOW);
        digitalWrite(PIN_B1, (b1 & mask) ? HIGH : LOW);
        digitalWrite(PIN_R2, (r2 & mask) ? HIGH : LOW);
        digitalWrite(PIN_G2, (g2 & mask) ? HIGH : LOW);
        digitalWrite(PIN_B2, (b2 & mask) ? HIGH : LOW);
        digitalWrite(PIN_CLK, HIGH);
        digitalWrite(PIN_CLK, LOW);
      }
    }
  }

  // Trigger latch
  digitalWrite(PIN_OE, LOW);
  digitalWrite(PIN_LAT, LOW);

  // Handle scan line overflow
  if (++scanlineIndex == MATRIX_SCANLINE_SIZE) {

    // Reset scan line index
    scanlineIndex = 0; 
    
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
  }

  digitalWrite(13, LOW);
}

