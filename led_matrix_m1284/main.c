/* Includes */
#include <avr/interrupt.h> /* For timer 2 */
#include <util/delay.h>    /* For delay */
#include <avr/io.h>        /* For registers and IO */
#include <stdint.h>        /* For hardcoded type */
#include <string.h>        /* For memset() */

/* Compile time constants */
static const uint8_t NB_HORIZONTAL_MATRIX = 2;
static const uint8_t NB_VERTICAL_MATRIX = 1;

static const uint8_t NB_LINES_PER_MATRIX = 32;   // MUST be 32 (hard-coded assembly)
static const uint8_t NB_COLUMNS_PER_MATRIX = 32; // MUST be 32 (hard-coded assembly)
static const uint8_t MATRIX_SCANLINE_SIZE = 16;  // MUST be 16 (hard-coded assembly)

static const uint8_t NB_MATRIX_COUNT = NB_VERTICAL_MATRIX * NB_HORIZONTAL_MATRIX;
static const uint8_t NB_LINES_COUNT = NB_VERTICAL_MATRIX * NB_LINES_PER_MATRIX;
static const uint8_t NB_COLUMNS_COUNT = NB_HORIZONTAL_MATRIX * NB_COLUMNS_PER_MATRIX;

/* Pin mapping */
#if defined(__AVR_ATmega2560__) // For Arduino Mega2560
// R1, G1, B1, R2, G2, B2 hard-wired on PA2~PA7
// A, B, C, D hard-wired on PF0~PF3
// CLK, OE, LAT hard-wired on PB3~PB1 and LED PB7
#define DATA_PORT PORTA
#define DATA_DDR DDRA
#define ADDR_PORT PORTF
#define ADDR_DDR DDRF 
#define CTRL_PORT PORTB
#define CTRL_PIN PINB
#define CTRL_DDR DDRB
#define CTRL_CLK_PIN (1 << 3)
#define CTRL_OE_PIN (1 << 2)
#define CTRL_LAT_PIN (1 << 1)
#define CTRL_LED_PIN (1 << 7)
#elif defined(__AVR_ATmega1284P__) // For DIY controller board
// R1, G1, B1, R2, G2, B2 hard-wired on PC2~PC7
// LED, CLK, LAT, OE hard-wired on PD4~PD7
// A, B, C, D hard-wired on PB0~PB3
#define DATA_DDR  DDRC
#define DATA_PORT PORTC
#define ADDR_DDR  DDRB
#define ADDR_PORT PORTB
#define CTRL_DDR  DDRD
#define CTRL_PIN  PIND
#define CTRL_PORT PORTD
#define CTRL_LED_PIN (1 << 4)
#define CTRL_CLK_PIN (1 << 5)
#define CTRL_LAT_PIN (1 << 6)
#define CTRL_OE_PIN (1 << 7)
#else  // For Arduino UNO
// R1, G1, B1, R2, G2, B2 hard-wired on PD2~PD7
// A, B, C, D hard-wired on PC0~PC3
// CLK, OE, LAT hard-wired on PB0~PB2 and LED PB5
#define DATA_PORT PORTD
#define DATA_DDR DDRD
#define ADDR_PORT PORTC
#define ADDR_DDR DDRC
#define CTRL_PORT PORTB
#define CTRL_PIN PINB
#define CTRL_DDR DDRB
#define CTRL_CLK_PIN (1 << 0)
#define CTRL_OE_PIN (1 << 1)
#define CTRL_LAT_PIN (1 << 2)
#define CTRL_LED_PIN (1 << 5)
#endif
#define CTRL_MASK (CTRL_CLK_PIN | CTRL_OE_PIN | CTRL_LAT_PIN  | CTRL_LED_PIN)

/** 
 * Framebuffer for RGB (hardcoded for 1/16 scanline with 32x32 matrix)
 *
 * Array format : [scan line index] [column data stream]
 * Data format : ... [R1 G1 B1, R2, G2, B2, x, x] ...
 */
static volatile uint8_t framebuffer[MATRIX_SCANLINE_SIZE][NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT];

/**
 * Possible color enumeration
 */
enum {
  COLOR_RED,    // R
  COLOR_GREEN,  // G
  COLOR_BLUE,   // B
  COLOR_YELLOW, // R + G
  COLOR_CYAN,   // G + B
  COLOR_PINK,   // R + B
  COLOR_WHITE,  // R + G + B
  COLOR_BLACK   // nothing
};

/**
 * Set the color of a pixel in the framebuffer.
 * 
 * @param x X position of the pixel.
 * @param y Y position of the pixel.
 * @param Color Color to set.
 */
static void setPixelAt(const uint8_t x, const uint8_t y, const uint8_t color) {
  volatile uint8_t* pixel = &framebuffer[y & (MATRIX_SCANLINE_SIZE - 1)][(NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT) - 1 - (x + (y / NB_LINES_PER_MATRIX * NB_COLUMNS_COUNT))];
  uint8_t bitsOffset = ((y & (NB_LINES_PER_MATRIX - 1)) > 15) ? 5 : 2;
  const uint8_t colorTable[] = {
    // COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_YELLOW, COLOR_CYAN, COLOR_PINK, COLOR_WHITE, COLOR_BLACK
       0b001,     0b010,       0b100,      0b011,        0b110,      0b101,      0b111,       0b000
  };
  *pixel = (*pixel & ~(0b111 << bitsOffset)) | (colorTable[color] << bitsOffset);
}

/**
 * Get the color of a pixel in the framebuffer.
 * 
 * @param x X position of the pixel.
 * @param y Y position of the pixel.
 * @return The color of the pixel.
 */
static uint8_t getPixelAt(const uint8_t x, const uint8_t y) {
  uint8_t pixel = framebuffer[y & (MATRIX_SCANLINE_SIZE - 1)][(NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT) - 1 - (x + (y / NB_LINES_PER_MATRIX * NB_COLUMNS_COUNT))];
  uint8_t bitsOffset = ((y & (NB_LINES_PER_MATRIX - 1)) > 15) ? 5 : 2;
  const uint8_t colorTable[] = {
    COLOR_BLACK, COLOR_RED, COLOR_GREEN, COLOR_YELLOW, COLOR_BLUE, COLOR_PINK, COLOR_CYAN, COLOR_WHITE
  };
  return colorTable[(pixel >> bitsOffset) & 3];
}

/**
 * Interruption routine - line refresh at 60Hz
 */
ISR(TIMER2_COMPA_vect) {

  // Scan line index
  static uint8_t scanlineIndex = 0;

  // Setup control lines and address lines
  CTRL_PORT = (CTRL_PORT & ~CTRL_MASK) | CTRL_OE_PIN | CTRL_LAT_PIN | CTRL_LED_PIN;
  ADDR_PORT = (ADDR_PORT & 0b11110000) | scanlineIndex;

  // Get line buffer
  uint8_t *lineBuffer = (uint8_t*) framebuffer[scanlineIndex];

  // Constant variable for the inline assembly
  const uint8_t clkPinMask = CTRL_CLK_PIN; // CLK pin mask
  
  // One pixel macro
#define LD_PX "ld __tmp_reg__, %a2+\n\t" \
			  "out %0, __tmp_reg__\n\t"  \
			  "out %1, %3\n\t"           \
			  "out %1, %3\n\t" // 2 + 1 + 1 + 1 = 5 ticks
#define LD_MX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX \
			  LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX \
			  LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX \
			  LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX // 32x LD_PX

  // For each pixels bundle of matrix 4/4, 3/3, 2/2 or 1/1
  asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						"I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));

  // For each pixels bundle of matrix 3/4, 2/3, 1/2
  if (NB_MATRIX_COUNT >= 2) {
    asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						  "I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));
  }

  // For each pixels bundle of matrix 2/4, 1/3
  if (NB_MATRIX_COUNT >= 3) {
    asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						  "I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));
  }

  // For each pixels bundle of matrix 1/4
  if (NB_MATRIX_COUNT >= 4) {
    asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						  "I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));
  }

  // Trigger latch
  CTRL_PORT = CTRL_PORT & ~CTRL_MASK;

  // Handle scan line overflow
  if (++scanlineIndex == MATRIX_SCANLINE_SIZE) {

    // Reset scan line index
    scanlineIndex = 0;
  }
}

/** Main */
int main(void) {

  /* Setup pins */
  DATA_DDR |= 0b11111100; // Data port
  DATA_PORT = DATA_PORT & 0b11;

  ADDR_DDR |= 0b1111; // Addr port
  ADDR_PORT = ADDR_PORT & 0b11110000;

  CTRL_DDR |= CTRL_MASK; // Ctrl port + debug led
  CTRL_PORT = (CTRL_PORT & ~CTRL_MASK) | CTRL_OE_PIN | CTRL_LAT_PIN;

  /* Init the framebuffer (all pixels black) */
  memset((void*) framebuffer, 0, MATRIX_SCANLINE_SIZE * NB_MATRIX_COUNT * NB_COLUMNS_COUNT);
  
  /* Setup refresh timer */
  cli();
  TCCR2A = _BV(WGM21);             // CTC mode
  TCCR2B = _BV(CS22) | _BV(CS21);  // Prescaler /256
  TCNT2 = 0;                       // Counter reset
  OCR2A = (F_CPU / 256 / 960) - 1; // 960Hz ISR
  TIMSK2 = _BV(OCIE2A);            // Enable timer 2's compare match A ISR
  sei();
  
  /* Main loop */
  for(;;) {
  
    // Demo code
    static uint8_t x = 0;
    static uint8_t y = 0;
    static uint8_t color = COLOR_RED;

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
    _delay_ms(10);
  }
  
  /* Compiler fix */
  return 0;
}
