from math import pow

#arraysize = int(raw_input("Gamma array size (power of two): "))
arraysize = 256
#width = (1 << int(raw_input("Total PWM bits (1, 2, 3, 4, 5): "))) - 1
gamma = float(raw_input("Gamma correction (standard: 2.2, maxim: 2.5): ")) # maxim uses 2.5

NUMBER_PER_LINE = 16

print """/* ----- BEGIN OF AUTO-GENERATED CODE - DO NOT EDIT ----- */

#ifndef GAMMA_H_
#define GAMMA_H_

/* Dependencies for PROGMEM */
#include <avr/pgmspace.h>

/** Gamma table in flash memory. */
static const uint8_t PROGMEM _gamma[] = {"""

for bitres in [1, 2, 3, 4, 5]:
    width = (1 << bitres) - 1

    print "#if NB_RESOLUTION_BITS == %d" % bitres
    
    for index in range(0, arraysize):
        #print hex(int(width * pow((float(width) / arraysize * (index + 1)) / width, gamma))),
        val = str(int((pow(float(index) / 255.0, gamma) * float(width) + 0.5))),
        if index != arraysize - 1:
            if (index % NUMBER_PER_LINE) == 0:
                print ' ',
            print "%s, " % val,
            if (index % NUMBER_PER_LINE) == (NUMBER_PER_LINE - 1):
                print
        else:
            print "%s" % val

    print "#endif"

print """};

/**
 * Turn a 8-bits value into a NB_RESOLUTION_BITS value with gamma correction.
 *
 * @param x The input 8-bits value.
 * @return The output NB_RESOLUTION_BITS value with gamma correction.
 */
static inline uint8_t gamma(uint8_t x) {
  return pgm_read_byte(&_gamma[x]);
}

#endif /* GAMMA_H_ */

/* ----- END OF AUTO-GENERATED CODE ----- */"""
