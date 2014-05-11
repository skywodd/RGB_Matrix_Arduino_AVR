/**
 * @file RgbMatrix.h
 * @brief RGB Matrix driver code API declaration header file.
 * @author SkyWodd
 * @version 1.0
 * @see http://skyduino.wordpress.com/
 *
 * @section intro_sec Introduction
 * This functions bundle implement everything related to the RGB matrix driver code.\n
 * You don't need to worry about how it's work, just draw pixels and the driver code will display them.\n
 * \n
 * Please report bug to <skywodd at gmail.com>
 *
 * @section licence_sec Licence
 *  This program is free software: you can redistribute it and/or modify\n
 *  it under the terms of the GNU General Public License as published by\n
 *  the Free Software Foundation, either version 3 of the License, or\n
 *  (at your option) any later version.\n
 * \n
 *  This program is distributed in the hope that it will be useful,\n
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of\n
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n
 *  GNU General Public License for more details.\n
 * \n
 *  You should have received a copy of the GNU General Public License\n
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.\n
 */
#ifndef RGB_MATRIX_H_
#define RGB_MATRIX_H_

/* Default compile time constants (can and should be makefile-defined) */
#ifndef USE_GAMMA_CORRECTION
#define USE_GAMMA_CORRECTION // Comment this line to disable gamma correction by default
#endif

#ifndef USE_DOUBLE_BUFFERING
//#define USE_DOUBLE_BUFFERING  // Uncomment this line to enable double buffering by default
#endif
#ifndef USE_TRIPLE_BUFFERING
//#define USE_TRIPLE_BUFFERING // Uncomment this line to enable triple buffering by default
#endif

#ifndef NB_HORIZONTAL_MATRIX
#define NB_HORIZONTAL_MATRIX 1 // Default matrix configuration
#endif
#ifndef NB_VERTICAL_MATRIX
#define NB_VERTICAL_MATRIX 1 // Default matrix configuration
#endif

#ifndef NB_RESOLUTION_BITS
#define NB_RESOLUTION_BITS 4 // Default colors configuration
#endif

#ifndef LINE_REFRESH_FREQUENCY
#define LINE_REFRESH_FREQUENCY 60 // Default refresh configuration
#endif

/* --------------- DO NOT EDIT AFTER THIS LINE --------------- */

/* Take down idiot */
#if defined(USE_DOUBLE_BUFFERING) && defined(USE_TRIPLE_BUFFERING)
#error You cannot use double and triple buffering at the same time!
#endif

/* Don't change this constants */
static const uint8_t NB_LINES_PER_MATRIX = 32;   // MUST be 32 (hard-coded assembly)
static const uint8_t NB_COLUMNS_PER_MATRIX = 32; // MUST be 32 (hard-coded assembly)
static const uint8_t MATRIX_SCANLINE_SIZE = 16;  // MUST be 16 (hard-coded assembly)
static const uint8_t NB_MATRIX_COUNT = NB_VERTICAL_MATRIX * NB_HORIZONTAL_MATRIX;
static const uint8_t NB_LINES_COUNT = NB_VERTICAL_MATRIX * NB_LINES_PER_MATRIX;
static const uint8_t NB_COLUMNS_COUNT = NB_HORIZONTAL_MATRIX * NB_COLUMNS_PER_MATRIX;

/**
 * Set the color of a pixel in the framebuffer.
 * 
 * @param x X position of the pixel.
 * @param y Y position of the pixel.
 * @param r Color to set (Red).
 * @param g Color to set (Green).
 * @param b Color to set (Blue).
 */
void setPixelAt(const uint8_t x, const uint8_t y, uint8_t r, uint8_t g, uint8_t b);

/**
 * Setup the hardware required by the RGB matrix driver.
 */
void setupMatrixDriver(void);

#if defined(USE_DOUBLE_BUFFERING) || defined(USE_TRIPLE_BUFFERING)
/**
 * Schedule a rotation of the drawing and display framebuffer at the next end of frame.
 * In double-buffered mode this function will also wait for the rotation to finish (and by the way, divide the maximum refresh frequency by two).
 * In triple-buffered mode this function MUST NOT be called more than LINE_REFRESH_FREQUENCY per second.
 */
void rotateFramebuffer(void);
#endif

#endif /* RGB_MATRIX_H_ */