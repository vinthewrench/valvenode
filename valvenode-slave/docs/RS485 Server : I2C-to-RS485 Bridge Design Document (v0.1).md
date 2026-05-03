# RS485 Server / I2C-to-RS485 Bridge Design Document (v0.1)

**Date:** April 9, 2026

**Project Goal**

This board acts as a simple, reliable bridge between a Raspberry Pi (5 V I2C master) and the RS-485 bus that controls multiple valve nodes. It receives high-level commands over I2C and converts them into the framed ASCII protocol used by the valve nodes.

The design follows the same principles as the powercontrol board: boring, deterministic, minimal part count, easy to debug, and fully offline.

## 1. Purpose and Scope

- Receive commands from the Raspberry Pi over I2C (5 V)
- Convert and forward them as framed ASCII protocol over RS-485 at 19200 baud
- Support the full valve node command set (PING, STATUS, OPEN, CLOSE, WHO, etc.)
- Provide visual feedback with two LEDs (status + comm activity)
- Stay simple and inspectable — no complex state machines

This is a pure protocol bridge. No autonomous irrigation logic lives here.

## 2. High-Level Operation

1. Raspberry Pi sends a command packet over I2C.
2. The ATmega88PB receives the I2C data, validates it, and builds the correct framed RS-485 message.
3. The bridge transmits the frame on the RS-485 bus (with proper DE timing).
4. Any reply from a valve node is forwarded back to the Pi over I2C.
5. LEDs provide immediate visual feedback.


The bridge is transparent — it does not interpret the valve commands, only translates and forwards them.

## 3. Major Components

- **MCU**: ATmega88PB-AU (TQFP-32)
- **Clock**: Internal 8 MHz RC oscillator
- **I2C**: 5 V I2C slave (address to be decided, e.g. 0x08 like powercontrol)
- **RS-485**: Half-duplex at 19200 baud, 8N1
- **RS-485 Transceiver**: Standard 5 V part (e.g. MAX485, SP3485, or SN65HVD72)
- **Status LEDs**: Two LEDs (general status + RS-485 activity)
- **Power**: 5 V from the Raspberry Pi or local regulator

## 4. Pin Assignment

| Pin   | Function                        | Direction     | Notes |
|-------|---------------------------------|---------------|-------|
| PC4   | SDA (I²C)                       | Bidirectional | 5 V I2C from Raspberry Pi |
| PC5   | SCL (I²C)                       | Output        | 5 V I2C from Raspberry Pi |
| PB3   | MOSI (ISP)                      | -             | ISP programming |
| PB4   | MISO (ISP)                      | -             | ISP programming |
| PB5   | SCK (ISP)                       | -             | ISP programming |
| PD0   | USART RX                        | Input         | From RS-485 transceiver RO |
| PD1   | USART TX                        | Output        | To RS-485 transceiver DI |
| PD2   | RS485_DE                        | Output        | High = transmit, Low = receive (DE and /RE tied together) |
| PB0   | Status LED                      | Output        | General status / ready indicator |
| PB1   | Comm LED                        | Output        | RS-485 activity (blink on TX or RX) |
| PD3   | Spare                           | -             | Available for future use |

All other pins remain free.

## 5. Power and Protection

- 5 V supplied directly from the Raspberry Pi (I2C is already 5 V)
- Optional local 5 V regulator if you want isolation
- Series resistors on I2C lines (like powercontrol)
- TVS or clamping diodes on RS-485 A/B lines recommended for field use
- Proper grounding between Pi and this board

## 6. Firmware Behavior

- I2C slave mode, simple command/response
- Receive command from Pi → build and send framed ASCII message on RS-485
- Receive reply from valve node → forward to Pi over I2C
- Blink Comm LED on every transmitted or received byte
- Status LED indicates ready / error state
- Watchdog enabled for robustness

The protocol on the RS-485 side remains exactly the same as the valve nodes:
 
```
@|LEN|COMMAND[|ARG1[|ARG2...]]|CHECKSUM<CR>
``` 

## 7. Recommended Connectors

- 4-pin header or screw terminal for RS-485 (A, B, +5 V, GND)
- Standard I2C header to Raspberry Pi
- 6-pin ISP header for programming
- Test points or header for the two LEDs

## 8. Bring-Up Checklist

1. Verify 5 V power and I2C communication from Pi
2. Verify ISP programming works with PB3/PB4/PB5
3. Verify USART TX/RX and DE pin toggling with a scope
4. Test basic PING command end-to-end (Pi → bridge → valve node → reply)
5. Verify LED behavior
6. Test WHO broadcast discovery

## 9. Known Limits (Rev A)

- Single RS-485 bus only
- No advanced error recovery or buffering yet
- I2C address and command format still to be finalized

## 10. Next Steps

- Finalize I2C command set (mirroring valve protocol where possible)
- Write initial firmware skeleton
- Decide on exact I2C address and packet format
- Add basic watchdog and timeout handling
- Bench test with one or more valve nodes
