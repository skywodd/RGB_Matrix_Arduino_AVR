/**
 * Pinmap declaration for RGB Matrix driver code.
 */
#ifndef RGB_MATRIX_PINMAPS_H_
#define RGB_MATRIX_PINMAPS_H_

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

#endif /* RGB_MATRIX_PINMAPS_H_ */