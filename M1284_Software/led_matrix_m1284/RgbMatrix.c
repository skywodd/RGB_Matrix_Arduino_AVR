/* Dependencies */
#include <avr/interrupt.h> /* For timer 2 */
#include <avr/io.h>        /* For registers and IO */
#include <stdint.h>        /* For hardcoded type */
#include <string.h>        /* For memset() */
#include "RgbMatrix.h"     /* For the API declarations */
#include "pinmaps.h"       /* For the pin mapping */

/* Include gamma correction table if required */
#ifdef USE_GAMMA_CORRECTION
#include "gamma.h"
#endif

/* 
 * Framebuffer for RGB (hardcoded for 1/16 scanline with 32x32 matrix)
 *
 * Array format : [buffer][scan line index] [column data stream]
 * Data format : ... [R1 G1 B1, R2, G2, B2, x, x] ...
 */
#if defined(USE_DOUBLE_BUFFERING)
static volatile uint8_t _framebuffer[2][MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS][NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT];
#elif defined(USE_TRIPLE_BUFFERING)
static volatile uint8_t _framebuffer[3][MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS][NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT];
#else
static volatile uint8_t _framebuffer[1][MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS][NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT];
#endif

/* Framebuffer pointers */
#if defined(USE_DOUBLE_BUFFERING)
static volatile uint8_t (*drawingFramebuffer)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = _framebuffer[0];
static volatile uint8_t (*displayFramebuffer)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = _framebuffer[1];
#elif defined(USE_TRIPLE_BUFFERING)
static volatile uint8_t (*drawingFramebuffer)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = _framebuffer[0];
static volatile uint8_t (*waitingFramebuffer)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = _framebuffer[1];
static volatile uint8_t (*displayFramebuffer)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = _framebuffer[2];
#else
static volatile uint8_t (*drawingFramebuffer)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = _framebuffer[0];
static volatile uint8_t (*displayFramebuffer)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = _framebuffer[0];
#endif

/* Framebuffer rotation flag */
static volatile uint8_t framebufferRotateFlag = 0;

void setPixelAt(const uint8_t x, const uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
  
  /* Bit shifting of gamma correction */
#ifdef USE_GAMMA_CORRECTION
  r = gamma(r);
  g = gamma(g);
  b = gamma(b);
#else
  r = (r) >> (8 - NB_RESOLUTION_BITS);
  g = (g) >> (8 - NB_RESOLUTION_BITS);
  b = (b) >> (8 - NB_RESOLUTION_BITS);
#endif

  /* Viva el offset */
  uint16_t pixelOffset = x + (y / NB_LINES_PER_MATRIX * NB_COLUMNS_COUNT);
  uint8_t scanlineOffset = y & (MATRIX_SCANLINE_SIZE - 1);
  uint8_t bitsOffset = ((y & (NB_LINES_PER_MATRIX - 1)) > 15) ? 5 : 2;
  
  /* Resolution bit 0 */
  volatile uint8_t* pixel0 = &drawingFramebuffer[scanlineOffset][pixelOffset];
  *pixel0 = (*pixel0 & ~(0b111 << bitsOffset)) | (!!(r & 1) << bitsOffset) | (!!(g & 1) << (bitsOffset + 1)) | (!!(b & 1) << (bitsOffset + 2));
  
  /* Resolution bit 1 */
  if(NB_RESOLUTION_BITS > 1) {
    volatile uint8_t* pixel1 = &drawingFramebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE][pixelOffset];
    *pixel1 = (*pixel1 & ~(0b111 << bitsOffset)) | (!!(r & 2) << bitsOffset) | (!!(g & 2) << (bitsOffset + 1)) | (!!(b & 2) << (bitsOffset + 2));
  }
  
  /* Resolution bit 2 */
  if(NB_RESOLUTION_BITS > 2) {
    volatile uint8_t* pixel2 = &drawingFramebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE * 2][pixelOffset];
    *pixel2 = (*pixel2 & ~(0b111 << bitsOffset)) | (!!(r & 4) << bitsOffset) | (!!(g & 4) << (bitsOffset + 1)) | (!!(b & 4) << (bitsOffset + 2));
  }
  
  /* Resolution bit 3 */
  if(NB_RESOLUTION_BITS > 3) {
    volatile uint8_t* pixel3 = &drawingFramebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE * 3][pixelOffset];
    *pixel3 = (*pixel3 & ~(0b111 << bitsOffset)) | (!!(r & 8) << bitsOffset) | (!!(g & 8) << (bitsOffset + 1)) | (!!(b & 8) << (bitsOffset + 2));
  }
  
  /* Resolution bit 4 */
  if(NB_RESOLUTION_BITS > 4) {
    volatile uint8_t* pixel4 = &drawingFramebuffer[scanlineOffset + MATRIX_SCANLINE_SIZE * 4][pixelOffset];
    *pixel4 = (*pixel4 & ~(0b111 << bitsOffset)) | (!!(r & 16) << bitsOffset) | (!!(g & 16) << (bitsOffset + 1)) | (!!(b & 16) << (bitsOffset + 2));
  }
}

/*
 * Interruption routine - line refresh at LINE_REFRESH_FREQUENCY
 */
ISR(TIMER1_COMPA_vect) {

  /* Scan line index & resolution bit index */
  static uint8_t scanlineIndex = MATRIX_SCANLINE_SIZE - 1;
  static uint8_t resolutionBitIndex = NB_RESOLUTION_BITS - 1;
  
  /* Handle resolution bit index overflow */
  if (++resolutionBitIndex == NB_RESOLUTION_BITS) {
	
	/* Reset resolution bit index */
	resolutionBitIndex = 0;
	
	/* Reset timer frequency */
	OCR1A = (F_CPU / LINE_REFRESH_FREQUENCY / MATRIX_SCANLINE_SIZE / ((1 << NB_RESOLUTION_BITS) - 1)) - 1;
	
    /* Handle scanline index overflow */
    if (++scanlineIndex == MATRIX_SCANLINE_SIZE) {

	  /* Reset scanline index counter */
	  scanlineIndex = 0;
	  
	  /* Handle framebuffer rotation */
#if defined(USE_DOUBLE_BUFFERING) || defined(USE_TRIPLE_BUFFERING)
	  if (framebufferRotateFlag) {

        /* Rotate drawing/waiting and display framebuffer */
#if defined(USE_DOUBLE_BUFFERING)
        volatile uint8_t (*tmp)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = drawingFramebuffer;
        drawingFramebuffer = displayFramebuffer;
        displayFramebuffer = tmp;
#elif defined(USE_TRIPLE_BUFFERING)
        volatile uint8_t (*tmp)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = waitingFramebuffer;
        waitingFramebuffer = displayFramebuffer;
        displayFramebuffer = tmp;
#endif

        /* Unset the flag */
        framebufferRotateFlag = 0; 
      }
#endif
    }

  } else {
	
    /* Divide frequency by two */
    OCR1A <<= 1;
  }
  
  /* Setup control lines and address lines */
  CTRL_PORT = (CTRL_PORT & ~CTRL_MASK) | CTRL_OE_PIN | CTRL_LAT_PIN | CTRL_LED_PIN;
  ADDR_PORT = (ADDR_PORT & 0b11110000) | scanlineIndex;

  /* Get line buffer */
  uint8_t *lineBuffer = (uint8_t*) displayFramebuffer[scanlineIndex + resolutionBitIndex * MATRIX_SCANLINE_SIZE];

  /* Go to the end of the buffer + 1 */
  lineBuffer += NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT;
  
  /* Constant variable for the inline assembly */
  const uint8_t clkPinMask = CTRL_CLK_PIN; // CLK pin mask
  
  /* One pixel macro */
#define LD_PX "ld __tmp_reg__, -%a2\n\t" \
			  "out %0, __tmp_reg__\n\t"  \
			  "out %1, %3\n\t"           \
			  "out %1, %3\n\t" // 2 + 1 + 1 + 1 = 5 ticks
#define LD_MX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX \
			  LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX \
			  LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX \
			  LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX LD_PX // 32x LD_PX

  /* For each pixels bundle of matrix 4/4, 3/3, 2/2 or 1/1 */
  asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						"I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));

  /* For each pixels bundle of matrix 3/4, 2/3, 1/2 */
  if (NB_MATRIX_COUNT >= 2) {
    asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						  "I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));
  }

  /* For each pixels bundle of matrix 2/4, 1/3 */
  if (NB_MATRIX_COUNT >= 3) {
    asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						  "I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));
  }

  /* For each pixels bundle of matrix 1/4 */
  if (NB_MATRIX_COUNT >= 4) {
    asm volatile(LD_MX :: "I" (_SFR_IO_ADDR(DATA_PORT)), 
						  "I" (_SFR_IO_ADDR(CTRL_PIN)), "e" (lineBuffer), "r" (clkPinMask));
  }

  /* Trigger latch */
  CTRL_PORT = CTRL_PORT & ~CTRL_MASK;
}

void setupMatrixDriver(void) {

  /* Setup pins */
  DATA_DDR |= 0b11111100; // Data port
  DATA_PORT = DATA_PORT & 0b11;

  ADDR_DDR |= 0b1111; // Addr port
  ADDR_PORT = ADDR_PORT & 0b11110000;

  CTRL_DDR |= CTRL_MASK; // Ctrl port + debug led
  CTRL_PORT = (CTRL_PORT & ~CTRL_MASK) | CTRL_OE_PIN | CTRL_LAT_PIN;

  /* Init the framebuffer (all pixels black) */
  memset((void*) _framebuffer[0], 0, MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS * NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT);
#if defined(USE_DOUBLE_BUFFERING) || defined(USE_TRIPLE_BUFFERING)
  memset((void*) _framebuffer[1], 0, MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS * NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT);
#endif
#if defined(USE_TRIPLE_BUFFERING)
  memset((void*) _framebuffer[2], 0, MATRIX_SCANLINE_SIZE * NB_RESOLUTION_BITS * NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT);
#endif

  /* Setup refresh timer (16 bits) */
  cli();
  TCCR1A = 0;                      // CTC mode
  TCCR1B = _BV(WGM12) | _BV(CS10); // No prescaler
  TCCR1C = 0;
  TCNT1 = 0;                       // Counter reset
  OCR1A = (F_CPU / LINE_REFRESH_FREQUENCY / MATRIX_SCANLINE_SIZE / ((1 << NB_RESOLUTION_BITS) - 1)) - 1; // ISR
  TIMSK1 = _BV(OCIE1A);            // Enable timer 1's compare match A ISR
  sei();
}

#if defined(USE_DOUBLE_BUFFERING) || defined(USE_TRIPLE_BUFFERING)
void rotateFramebuffer(void) {
  
#if defined(USE_TRIPLE_BUFFERING)
  /* Avoid poorly timed framebuffer rotation */
  while (framebufferRotateFlag);

  /* Rotate drawing and waiting framebuffer */
  volatile uint8_t (*tmp)[NB_VERTICAL_MATRIX * NB_COLUMNS_COUNT] = drawingFramebuffer;
  drawingFramebuffer = waitingFramebuffer;
  waitingFramebuffer = tmp;
#endif
  
  /* Turn flag on */
  framebufferRotateFlag = 1; // Byte level operations should be atomic by themself
  
#if defined(USE_DOUBLE_BUFFERING)
  /* Wait for framebuffer rotation */
  while (framebufferRotateFlag);
#endif
}
#endif
