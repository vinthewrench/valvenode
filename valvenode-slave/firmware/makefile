# ============================================================
# Valve Node Firmware Makefile
# Target: ATtiny4313 @ 8 MHz (internal RC)
# ============================================================


# ------------------------------------------------------------
# Project Configuration
# ------------------------------------------------------------

FW_VERSION_BCD := 0x0100

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
# Flashing (preserve EEPROM, no chip erase)
# Use for iterative bring-up and debugging
# ------------------------------------------------------------
flash-part: $(PROJECT).hex
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B 50 -D \
		-U flash:w:"$<":i

isp:
	$(AVRDUDE) -c $(PROGRAMMER) -P $(PORT) -p $(AVRDUDE_MCU) \
		-B 100 \
		-v


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
