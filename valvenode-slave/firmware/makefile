# ============================================================
# Valve Node Firmware Makefile
# Target: ATtiny4313 @ 8 MHz internal RC
# ============================================================


# ------------------------------------------------------------
# Project Configuration
# ------------------------------------------------------------

FW_VERSION_BCD := 0x0107

PROJECT := valvenode
MCU     := attiny4313
F_CPU   := 8000000UL


# ------------------------------------------------------------
# Toolchain
# ------------------------------------------------------------

CC      := avr-gcc
OBJCOPY := avr-objcopy
OBJDUMP := avr-objdump
SIZE    := avr-size
AVRDUDE := avrdude


# ------------------------------------------------------------
# Programmer Configuration
# ------------------------------------------------------------

PROGRAMMER  := atmelice_isp
PORT        := usb
AVRDUDE_MCU := t4313

ISP_BITCLOCK ?= 5


# ------------------------------------------------------------
# Directories
# ------------------------------------------------------------

OBJ_DIR := build


# ------------------------------------------------------------
# Compiler Flags
# ------------------------------------------------------------

CFLAGS := \
	-mmcu=$(MCU) \
	-DF_CPU=$(F_CPU) \
	-DFW_VERSION=$(FW_VERSION_BCD) \
	-Wall -Wextra -Werror \
	-Os \
	-ffunction-sections \
	-fdata-sections \
	-std=gnu11

LDFLAGS := \
	-mmcu=$(MCU) \
	-Wl,--gc-sections \
	-Wl,-Map=$(PROJECT).map


# ------------------------------------------------------------
# Source Files
# ------------------------------------------------------------

SRCS := \
	valvenode.c

OBJS := $(SRCS:%.c=$(OBJ_DIR)/%.o)


# ------------------------------------------------------------
# Default Target
# ------------------------------------------------------------

all: $(PROJECT).hex


# ------------------------------------------------------------
# Compile Rule
# ------------------------------------------------------------

$(OBJ_DIR)/%.o: %.c
	@mkdir -p "$(dir $@)"
	$(CC) $(CFLAGS) -c "$<" -o "$@"


# ------------------------------------------------------------
# Link
# ------------------------------------------------------------

$(PROJECT).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o "$@"
	$(SIZE) "$@"


# ------------------------------------------------------------
# Output Formats
# ------------------------------------------------------------

$(PROJECT).hex: $(PROJECT).elf
	$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

$(PROJECT).lst: $(PROJECT).elf
	$(OBJDUMP) -h -S "$<" > "$@"


# ------------------------------------------------------------
# Flash
# ------------------------------------------------------------

flash: $(PROJECT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B $(ISP_BITCLOCK) \
		-U flash:w:"$<":i


# ------------------------------------------------------------
# Flashing, preserve EEPROM, no chip erase
# Use for iterative bring-up and debugging
# ------------------------------------------------------------

flash-part: $(PROJECT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B 50 -D \
		-U flash:w:"$<":i


# ------------------------------------------------------------
# Programmer / ISP sanity check
# ------------------------------------------------------------

isp:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B 100 \
		-v


# ------------------------------------------------------------
# EEPROM identity inspection / repair
#
# EEPROM identity record used by valvenode.c:
#
#   byte 0 = magic       0xA5
#   byte 1 = node addr   0x01..0xFE
#   byte 2 = ~node addr
#
# Examples:
#
#   node 1 = A5 01 FE
#   node 2 = A5 02 FD
#   node 3 = A5 03 FC
#   node 4 = A5 04 FB
#   node 5 = A5 05 FA
#   node 6 = A5 06 F9
#
# Fresh EEPROM:
#
#   FF FF FF
#
# Fresh blank EEPROM is treated as unassigned.
# Bad magic, bad address, or bad inverse byte is treated as corrupt and causes
# the fast address-fault LED blink.
# ------------------------------------------------------------

read-eeprom:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B $(ISP_BITCLOCK) \
		-U eeprom:r:eeprom.bin:r
	hexdump -C eeprom.bin | head
	@echo
	@echo "Identity bytes:"
	@hexdump -v -e '3/1 "%02X " "\n"' -n 3 eeprom.bin

write-eeprom-node:
	@if [ -z "$(NODE)" ]; then \
		echo "usage: make write-eeprom-node NODE=3"; \
		exit 1; \
	fi
	@if [ "$(NODE)" -lt 1 ] || [ "$(NODE)" -gt 254 ]; then \
		echo "NODE must be 1..254"; \
		exit 1; \
	fi
	@rm -f node_identity.bin
	@dd if=/dev/zero bs=256 count=1 2>/dev/null | tr '\000' '\377' > node_identity.bin
	@printf "%b%b%b" \
		"$$(printf '\\%03o' 165)" \
		"$$(printf '\\%03o' $(NODE))" \
		"$$(printf '\\%03o' $$((255 - $(NODE))))" \
		| dd of=node_identity.bin bs=1 seek=0 conv=notrunc 2>/dev/null
	hexdump -C node_identity.bin | head
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B $(ISP_BITCLOCK) \
		-U eeprom:w:node_identity.bin:r

write-eeprom-blank:
	printf '\xFF\xFF\xFF' > blank_eeprom.bin
	hexdump -C blank_eeprom.bin
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B $(ISP_BITCLOCK) \
		-U eeprom:w:blank_eeprom.bin:r


# ------------------------------------------------------------
# Fuse Configuration
# Internal 8 MHz RC, CKDIV8 disabled
# Verify before writing on your actual hardware
# ------------------------------------------------------------

LFUSE := 0xE4
HFUSE := 0xDF
EFUSE := 0xFF

set-fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) -B 50 \
		-U lfuse:w:$(LFUSE):m \
		-U hfuse:w:$(HFUSE):m \
		-U efuse:w:$(EFUSE):m

check-fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-U lfuse:r:-:h \
		-U hfuse:r:-:h \
		-U efuse:r:-:h


# ------------------------------------------------------------
# Size
# ------------------------------------------------------------

size: $(PROJECT).elf
	@echo "---- AVR Memory Usage ----"
	@$(SIZE) --format=avr --mcu=$(MCU) "$<"


# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------

clean:
	rm -rf $(OBJ_DIR) *.elf *.hex *.lst *.map
	rm -f eeprom.bin blank_eeprom.bin node_identity.bin
