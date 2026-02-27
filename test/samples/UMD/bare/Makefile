#to compile/run
#make        # compile
#make flash  # upload
#make monitor # see distance readings (Ctrl+A then K to exit screen)

# Project
TARGET   = main
SRC      = main.c

# MCU & Clock
MCU      = atmega328p
F_CPU    = 16000000UL

# Programmer
PROGRAMMER = arduino
BAUD       = 57600

# Mac port - run 'ls /dev/tty.*' in terminal to find yours
PORT       = /dev/tty.usbserial-1420

# Toolchain
CC       = avr-gcc
OBJCOPY  = avr-objcopy
AVRDUDE  = avrdude

# Flags
CFLAGS   = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall -Wextra

# Build
all: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Flash to board
flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) -U flash:w:$<:i

# Open serial monitor
monitor:
	screen $(PORT) 9600

# Clean build files
clean:
	rm -f $(TARGET).elf $(TARGET).hex