/* Includes */
#include <avr/interrupt.h> /* For timer 2 */
#include <util/delay.h>    /* For delay */
#include <avr/io.h>        /* For registers and IO */
#include <stdint.h>        /* For hardcoded type */
#include <string.h>        /* For memset() */

/* Compile time constants */
static const uint8_t NB_HORIZONTAL_MATRIX = 2;
static const uint8_t NB_VERTICAL_MATRIX = 1;

#define NB_RESOLUTION_BITS 4
// 1 matrix = max 5 bits
// 2 matrix = max 4 bits
// 3 matrix = max 3 bits
// 4 matrix = max 2 bits
#include "gamma.h"

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
static volatile uint8_t framebuffer[MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS][NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT];

/**
 * Set the color of a pixel in the framebuffer.
 * 
 * @param x X position of the pixel.
 * @param y Y position of the pixel.
 * @param r Color to set (Red).
 * @param g Color to set (Green).
 * @param b Color to set (Blue).
 */
static void setPixelAt(const uint8_t x, const uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
  
  /* Gamma correction */
  r = gamma(r);
  g = gamma(g);
  b = gamma(b);

  /* Viva el offset */
  uint16_t pixelOffset = (NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT) - 1 - (x + (y / NB_LINES_PER_MATRIX * NB_COLUMNS_COUNT));
  uint8_t scanlineOffset = y & (MATRIX_SCANLINE_SIZE - 1);
  uint8_t bitsOffset = ((y & (NB_LINES_PER_MATRIX - 1)) > 15) ? 5 : 2;
  
  /* Resolution bit 0 */
  volatile uint8_t* pixel0 = &framebuffer[scanlineOffset][pixelOffset];
  *pixel0 = (*pixel0 & ~(0b111 << bitsOffset)) | (!!(r & 1) << bitsOffset) | (!!(g & 1) << (bitsOffset + 1)) | (!!(b & 1) << (bitsOffset + 2));
  
  /* Resolution bit 1 */
  if(NB_RESOLUTION_BITS > 1) {
    volatile uint8_t* pixel1 = &framebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE][pixelOffset];
    *pixel1 = (*pixel1 & ~(0b111 << bitsOffset)) | (!!(r & 2) << bitsOffset) | (!!(g & 2) << (bitsOffset + 1)) | (!!(b & 2) << (bitsOffset + 2));
  }
  
  /* Resolution bit 2 */
  if(NB_RESOLUTION_BITS > 2) {
    volatile uint8_t* pixel2 = &framebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE * 2][pixelOffset];
    *pixel2 = (*pixel2 & ~(0b111 << bitsOffset)) | (!!(r & 4) << bitsOffset) | (!!(g & 4) << (bitsOffset + 1)) | (!!(b & 4) << (bitsOffset + 2));
  }
  
  /* Resolution bit 3 */
  if(NB_RESOLUTION_BITS > 3) {
    volatile uint8_t* pixel3 = &framebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE * 3][pixelOffset];
    *pixel3 = (*pixel3 & ~(0b111 << bitsOffset)) | (!!(r & 8) << bitsOffset) | (!!(g & 8) << (bitsOffset + 1)) | (!!(b & 8) << (bitsOffset + 2));
  }
  
  /* Resolution bit 4 */
  if(NB_RESOLUTION_BITS > 4) {
    volatile uint8_t* pixel4 = &framebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE * 4][pixelOffset];
    *pixel4 = (*pixel4 & ~(0b111 << bitsOffset)) | (!!(r & 16) << bitsOffset) | (!!(g & 16) << (bitsOffset + 1)) | (!!(b & 16) << (bitsOffset + 2));
  }
}

/**
 * Interruption routine - line refresh at 60Hz
 */
ISR(TIMER1_COMPA_vect) {

  // Scan line index & resolution bit index
  static uint8_t scanlineIndex = MATRIX_SCANLINE_SIZE - 1;
  static uint8_t resolutionBitIndex = NB_RESOLUTION_BITS - 1;
  
  // Handle resolution bit index overflow
  if (++resolutionBitIndex == NB_RESOLUTION_BITS) {
	
	// Reset resolution bit index
	resolutionBitIndex = 0;
	
	// Reset timer frequency
	OCR1A = (F_CPU / 60 / 16 / ((1 << NB_RESOLUTION_BITS) - 1)) - 1;
	
    // Handle scanline index overflow
    if (++scanlineIndex == MATRIX_SCANLINE_SIZE) {

	  // Reset scanline index counter
	  scanlineIndex = 0;
    }
	
  } else {
	
    // Divide frequency by two
    OCR1A <<= 1;
  }
  
  // Setup control lines and address lines
  CTRL_PORT = (CTRL_PORT & ~CTRL_MASK) | CTRL_OE_PIN | CTRL_LAT_PIN | CTRL_LED_PIN;
  ADDR_PORT = (ADDR_PORT & 0b11110000) | scanlineIndex;

  // Get line buffer
  uint8_t *lineBuffer = (uint8_t*) framebuffer[scanlineIndex + resolutionBitIndex * MATRIX_SCANLINE_SIZE];

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
  memset((void*) framebuffer, 0, MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS * NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT);
  
  /* Setup refresh timer (16 bits) */
  cli();
  TCCR1A = 0;                      // CTC mode
  TCCR1B = _BV(WGM12) | _BV(CS10); // No prescaler
  TCCR1C = 0;
  TCNT1 = 0;                       // Counter reset
  OCR1A = (F_CPU / 60 / 16 / ((1 << NB_RESOLUTION_BITS) - 1)) - 1; // ISR
  TIMSK1 = _BV(OCIE1A);            // Enable timer 1's compare match A ISR
  sei();
  
  /* Main loop */
  for(;;) {
  
    // Demo code
    static uint8_t x = 0;
    static uint8_t y = 0;
    static uint8_t r = 0;
    static uint8_t g = 0;
    static uint8_t b = 0;

    setPixelAt(x, y, r, g, b);

    if(++x == NB_COLUMNS_COUNT) {
      x = 0;

      if(++y == NB_LINES_COUNT) {
        y = 0;
      }
    }
	
	/* Draw color pattern */
#define COLOR_STEP 15
	if((r += COLOR_STEP) >= 256 - COLOR_STEP) {
      r = 0;
		  
	  if((g += COLOR_STEP) >= 256 - COLOR_STEP) {
        g = 0;
		  
		if((b += COLOR_STEP) >= 256 - COLOR_STEP) {
          b = 0;
        }
      }
    }

    // No flood delay
    _delay_ms(1);
  }
  
  /* Compiler fix */
  return 0;
}
