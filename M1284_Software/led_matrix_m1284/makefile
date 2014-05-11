# AVR mcu target, should not be changed
TARGET ?= atmega1284p

# System clock frequency
F_CPU ?= 16000000

# Source files to compile
SOURCES = main.c RgbMatrix.c

# Output project name
OUTPUT = rgbLedMatrix

# Programmer name
AVRDUDE_PROGRAMMER ?= usbtiny

# RGB Matrix configuration
CCFLAGS += -DUSE_GAMMA_CORRECTION
CCFLAGS += -DUSE_DOUBLE_BUFFERING
#CCFLAGS += -DUSE_TRIPLE_BUFFERING
CCFLAGS += -DNB_HORIZONTAL_MATRIX=2
CCFLAGS += -DNB_VERTICAL_MATRIX=1
CCFLAGS += -DNB_RESOLUTION_BITS=4
#CCFLAGS += -DLINE_REFRESH_FREQUENCY=60

# Compilator switch
CCFLAGS += -Wall -Os -ffunction-sections
LDFLAGS += -Wl,-gc-sections

# ----- Nothing to change after this line
OBJECTS =  $(SOURCES:.c=.o)
COMPILER = avr-g++ -mmcu=$(TARGET) -DF_CPU=$(F_CPU)UL
OBJCOPY = avr-objcopy -O ihex
OBJSIZE = avr-size -C --mcu=$(TARGET)
AVRDUDE_ICSP = avrdude -p $(TARGET) -c $(AVRDUDE_PROGRAMMER)

all: elf hex size

%.o: %.c
	$(COMPILER) -c $< -o $@ $(CCFLAGS)
	
elf: $(OBJECTS)
	$(COMPILER) -o $(OUTPUT).elf $(LDFLAGS) $^
	
hex: $(OUTPUT).elf
	$(OBJCOPY) $(OUTPUT).elf $(OUTPUT).hex
	
size: $(OUTPUT).elf
	$(OBJSIZE) $(OUTPUT).elf

flash: $(OUTPUT).hex
	$(AVRDUDE_ICSP) -U flash:w:$(OUTPUT).hex
	
fuses:
	$(AVRDUDE_ICSP) -U lfuse:w:0xf7:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m  -U lock:w:0x0f:m
	
clean:
	rm -f $(OBJECTS) $(OUTPUT).elf $(OUTPUT).hex