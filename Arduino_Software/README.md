# Demonstration code for Arduino boards
## By Fabien Batteix (alias SkyWodd)

This directory contain some demonstration code for driving RGB leds matrix using an Arduino boards.

---

Demo 1 is a pure Arduino-based implementation of the code. 
This version is for educational purpose only and not usable for real-world usage.
This version is really slow but compatible with any Arduino-like boards (chipkit, Maple, etc).

---

Demo 2 is an optimized version of the demo 1 using low-level AVR IO registers and some tricks.
This version is made to work on Arduino UNO and Arduino Mega2560 only.

---

Demo 3 is an really-optimized version of the demo 1 and 2. 
This version use only AVR registers, AVR interrupts, lot of tricks and some memory management tips.
This version can be used for real-world usage and can also be used as a good base for driving other type of leds matrix (with another hardware configuration).

Warning: The code itself is not loop-optimized, all computation of loop indexes are made at runtime. 
This slow-down the code and make it more "fat" but allow you to change it as you need for driving uncommon type of led matrix.
For really-really-optimized and fast version take a look at the M1284_Software directory.