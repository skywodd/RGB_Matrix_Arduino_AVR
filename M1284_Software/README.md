# Released code for ATmega1254p based boards
## By Fabien Batteix (alias SkyWodd)

This directory contain all released code versions for driving RGB leds matrix using an ATmega1284p based board.

Backward compatibily with Arduino Mega2560 and UNO boards is included but not fully tested.
This code is designed to run on an ATmega1284p mcu, use at your own risk.

---

M1284_8colors is an heavily optimized version of the Arduino-based demo 3 code with lot of compile-time constant.
No more runtime loops, all the job is done as fast as possible by using low-level hand-crafted assembly and GCC tricks.

---

M1284_Ncolors is an upgraded version of the basic 8 colors one.
This version use Binary Coded Modulation (BCM) to get far over the 8 colors limit.
Can go up to 32K colors with a single matrix and to 4K colors with 4 matrix.

---

M1284 is the lastest code version.