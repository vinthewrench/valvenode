# ---------------- Configuration ----------------
MCU        = atmega88pb
PROGRAMMER = atmelice_isp
PORT       = usb
BITCLOCK   = 100

TARGET  = valvenode_master
F_CPU   = 8000000UL
I2C_ADDR = 0x09

CC      = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

CFLAGS  = -Wall -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU) -DI2C_ADDR=$(I2C_ADDR)
LDFLAGS = -mmcu=$(MCU)

# ---------------- Default build ----------------
all: $(TARGET).hex

$(TARGET).elf: $(TARGET).c
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# ---------------- Flash ----------------
flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(MCU) \
		-B $(BITCLOCK) \
		-U flash:w:$<:i

# ---------------- Signature probe ----------------
isp:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(MCU) \
		-B $(BITCLOCK) \
		-U signature:r:-:h \
		-v

sig: isp

# ---------------- Fuse utilities ----------------
checkfuses:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(MCU) \
		-B $(BITCLOCK) \
		-U lfuse:r:-:h \
		-U hfuse:r:-:h \
		-U efuse:r:-:h

setfuses:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(MCU) \
		-B $(BITCLOCK) \
		-U lfuse:w:0xE2:m \
		-U hfuse:w:0xDF:m \
		-U efuse:w:0xFF:m

# ---------------- Cleanup ----------------
clean:
	rm -f *.o *.elf *.hex
