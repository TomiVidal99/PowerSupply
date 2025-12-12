# AVR-GCC Makefile for ATmega328P

# Project name
TARGET = main

# Define the microcontroller
MCU = atmega328p

# Define the clock frequency
F_CPU = 16000000UL

PROGRAMMER = arduino
CONFIG_FILE = "D:\AVRDUDE\avrdude.conf" 
# BAUD = 115200
BAUD = 57600
AVRDUDE = "D:\AVRDUDE\avrdude.exe"
COM_PORT = COM9

# Source files
SRC = src/*.c

# Included directories
INCLUDE_DIR = include

# Compiler and linker flags
# Considerar: -Werror -Wfatal-errors -Wall -Wextra
CFLAGS = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -I${INCLUDE_DIR} -Wall -Wextra -Werror -Wfatal-errors
LDFLAGS = -mmcu=$(MCU)

# Output directory
BUILD_DIR = build

# Tools
CC = "D:\avr8-gnu-toolchain\bin\avr-gcc.exe"
OBJCOPY = avr-objcopy

# Output file names
ELF = $(BUILD_DIR)/$(TARGET).elf
HEX = $(BUILD_DIR)/$(TARGET).hex

# Default target
all: $(BUILD_DIR) $(HEX)

$(BUILD_DIR):
	if not exist $(BUILD_DIR) mkdir $(BUILD_DIR)

# Build elf from source
$(ELF): $(SRC)
	$(CC) $(CFLAGS) $^ -o $@

# Build hex from elf
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

# Flashing the microcontroller
flash: $(HEX)
	$(AVRDUDE) -c $(PROGRAMMER) -C $(CONFIG_FILE) -p $(MCU) -P$(COM_PORT) -b $(BAUD) -U flash:w:$(HEX):i

# Clean up build directory
clean:
	if exist $(BUILD_DIR) del /Q $(BUILD_DIR)\*

# Phony targets
.PHONY: all clean flash