# Standalone Valve Node Design Document (v0.1)

**Date:** April 7, 2026

**Sources of Truth:**
- `valvenode.c`
- Valve node protocol draft
- Locked node concept from this conversation

## 1. Purpose and Scope (Goal)

The valve node is a small remote controller for a **single 12 V latching irrigation valve**. It is intended to sit near the valve and receive commands from a master over a shared serial bus.

Core design goals:
- Fully offline, no cloud, no wireless dependency
- Simple 4-wire field connection:
  - +12 V
  - GND
  - RS-485 A
  - RS-485 B
- Small AVR with minimal part count
- Deterministic and inspectable behavior
- Local generation of the valve pulse using a VNH7100BAS H-bridge driver
- Boring firmware, easy bench debug, easy field replacement

This document describes the **single-channel node** only.

## 2. How It Works (High-Level Overview)

1. The master sends a framed serial command over RS-485.
2. The node receives the frame and checks:
   - framing
   - payload length
   - optionally checksum
   - destination address
3. If the command is for this node:
   - `OPEN` sends one polarity pulse to the valve
   - `CLOSE` sends the opposite polarity pulse
   - `STATUS` reports current logical state
   - `PING` returns an ACK
4. If the command is broadcast `WHO|255`, each node waits a delay based on node address, then replies with its ID.
5. After the pulse, the bridge is turned fully off.

This is a simple command/response node. It is not autonomous irrigation logic.

## 3. System Overview – Major Components

- **MCU**: ATtiny4313
- **Bus Interface**: 5 V RS-485 transceiver
- **Valve Driver**: VNH7100BAS / VNH7100BASTR
- **Valve Type**: 12 V latching irrigation valve
- **Power Input**: 12 V nominal
- **Logic Supply**: 5 V regulator from 12 V input
- **Optional Status LED**: one MCU-driven LED

## 4. Power Architecture

- **Input power**: 12 VDC from field cable
- **Direct high-current path**: 12 V to VNH7100BAS motor supply
- **Logic rail**: local 5 V regulator for MCU and RS-485 transceiver

Recommended basics:
- reverse polarity protection on 12 V input
- TVS diode on 12 V input
- local bulk capacitance near the VNH supply pins
- 0.1 µF decoupling at MCU and transceiver
- separate local bulk cap on 5 V rail

### Power Notes

The VNH7100BAS drives the valve pulse from the 12 V input supply. The MCU does not generate PWM for speed control. The VNH `PWM` pin is used only as an **enable gate**.

Safe default:
- `PWM = low`
- `INA = low`
- `INB = low`

That keeps the bridge off at reset and idle.

## 5. MCU Details and Pin Assignments

**MCU Part Number**: ATtiny4313  
**Package**: 20-pin DIP or equivalent package variant  
**Clock**: Internal 8 MHz RC oscillator  
**Voltage**: 5 V

### MCU Specs
- Flash: 4 KB
- SRAM: 256 B
- EEPROM: 256 B
- One hardware UART
- Enough GPIO for one valve node

### Locked Design Decisions
- UART used for command and reply traffic
- RS-485 is half duplex
- One GPIO controls RS-485 DE and /RE together
- VNH7100BAS `PWM` pin is enable only
- Single valve channel only
- Node address compiled in at build time for now

### Power Pins
Wire the MCU power pins in the schematic explicitly.

Recommended:
- VCC to +5 V
- GND to common ground
- 0.1 µF ceramic at the MCU supply pins
- 4.7–10 µF local bulk cap near MCU if convenient

### Reset / Programming
- Standard AVR ISP header recommended
- Keep RESET available
- Do not get cute with fuse settings unless you actually need to

## 6. ATtiny4313 Full Physical Pin Assignment for PDIP-20 and SOIC-20

This section lists the **complete physical pinout** for the ATtiny4313 in **PDIP-20** and **SOIC-20**. For these two packages, the **pin numbers and signal names are the same**. Only the package body changes.  [oai_citation:1‡Microchip](https://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf)

### Package-wide pin map

| Physical Pin | AVR Signal | Alternate Functions | Valve Node Use |
|--------------|------------|---------------------|----------------|
| 1  | PA2 | PCINT10 / RESET / dW | RESET, ISP programming |
| 2  | PD0 | PCINT11 / RXD | UART_RX from RS-485 transceiver RO |
| 3  | PD1 | PCINT12 / TXD | UART_TX to RS-485 transceiver DI |
| 4  | PA1 | PCINT9 / XTAL2 | Spare |
| 5  | PA0 | PCINT8 / CLKI / XTAL1 | Spare |
| 6  | PD2 | PCINT13 / CKOUT / XCK / INT0 | Spare |
| 7  | PD3 | PCINT14 / INT1 | Spare |
| 8  | PD4 | PCINT15 / T0 | Spare |
| 9  | PD5 | PCINT16 / OC0B / T1 | Spare |
| 10 | GND | Ground | Ground |
| 11 | PD6 | PCINT17 / ICP1 | Spare |
| 12 | PB0 | AIN0 / PCINT0 | RS485_DE_RE |
| 13 | PB1 | AIN1 / PCINT1 | VNH_INA |
| 14 | PB2 | OC0A / PCINT2 | VNH_INB |
| 15 | PB3 | OC1A / PCINT3 | VNH_PWM_EN |
| 16 | PB4 | OC1B / PCINT4 | STATUS_LED |
| 17 | PB5 | MOSI / DI / SDA / PCINT5 | Spare, possible ISP/data use |
| 18 | PB6 | MISO / DO / PCINT6 | Spare, possible ISP/data use |
| 19 | PB7 | USCK / SCL / SCK / PCINT7 | Spare, possible ISP/clock use |
| 20 | VCC | Supply | +5 V logic supply |

### Locked valve node assignments

These are the pins currently assigned by firmware and should stay aligned with `valvenode.c`.

| Function     | AVR Signal | Physical Pin | Direction | Notes |
|--------------|------------|--------------|-----------|-------|
| UART_RX      | PD0        | 2  | Input  | From RS-485 transceiver RO |
| UART_TX      | PD1        | 3  | Output | To RS-485 transceiver DI |
| RS485_DE_RE  | PB0        | 12 | Output | 1 = transmit, 0 = receive |
| VNH_INA      | PB1        | 13 | Output | H-bridge direction input A |
| VNH_INB      | PB2        | 14 | Output | H-bridge direction input B |
| VNH_PWM_EN   | PB3        | 15 | Output | H-bridge enable only, not real PWM |
| STATUS_LED   | PB4        | 16 | Output | Optional status LED |
| RESET        | PA2        | 1  | Input  | Keep for ISP programming |
| VCC          | VCC        | 20 | Power  | +5 V |
| GND          | GND        | 10 | Power  | Ground |

### Pins reserved but currently unused in Rev A

These are available on the package but are **not assigned in the current valve node firmware**.

| Physical Pin | AVR Signal | Notes |
|--------------|------------|-------|
| 4  | PA1 | Unused |
| 5  | PA0 | Unused |
| 6  | PD2 | Unused |
| 7  | PD3 | Unused |
| 8  | PD4 | Unused |
| 9  | PD5 | Unused |
| 11 | PD6 | Unused |
| 17 | PB5 | Unused, overlaps common ISP/data functions |
| 18 | PB6 | Unused, overlaps common ISP/data functions |
| 19 | PB7 | Unused, overlaps common ISP/clock functions |

### Board design notes

- Put the **0.1 µF decoupler** close to pins **20 (VCC)** and **10 (GND)**.
- Keep **pin 1 RESET** accessible for AVR ISP.
- Keep the **RS-485 traces** away from the high-current valve output path.
- Keep the **VNH output current loop** physically separated from the MCU and UART area.
- Do not leave `VNH_PWM_EN` floating at reset. The firmware assumes it comes up off.
- If you use PB5/PB6/PB7 later, remember those overlap common serial-programming functions and can make bring-up more annoying if you get sloppy.

### Package note

This section applies to:
- **ATtiny4313-PU** in PDIP-20
- **ATtiny4313** in SOIC-20

The pin mapping above is taken from the official ATtiny2313A/4313 pinout figure.  [oai_citation:2‡Microchip](https://ww1.microchip.com/downloads/en/DeviceDoc/doc8246.pdf)

## 7. RS-485 Interface Wiring

Use a standard half-duplex RS-485 transceiver.

### Required signals

| RS-485 Transceiver Pin | Connect To | Notes |
|------------------------|------------|-------|
| DI                     | MCU TX (PD1) | UART transmit data |
| RO                     | MCU RX (PD0) | UART receive data |
| DE                     | MCU PB0 | Tie to /RE for one-pin direction control |
| /RE                    | MCU PB0 | Active low receive enable |
| A                      | Bus A | Field bus |
| B                      | Bus B | Field bus |
| VCC                    | +5 V | Logic supply |
| GND                    | GND | Common logic ground |

### RS-485 Notes
- Tie `DE` and `/RE` together for one-pin control
- Default to receive mode at boot
- Put termination only where appropriate for the bus layout
- Do not scatter termination all over the property like confetti
- Biasing should usually live at the master, not every node

## 8. VNH7100BAS Wiring

The VNH7100BAS is used as a bidirectional pulse driver for the latching valve.

### Control Signals

| VNH Pin | Connect To | Notes |
|---------|------------|-------|
| INA     | MCU PB1    | Direction control |
| INB     | MCU PB2    | Direction control |
| PWM     | MCU PB3    | Enable only |
| SEL0    | Fixed logic level | Leave simple for rev A |
| MultiSense | Optional test point or NC | Not used in rev A logic |

### Power / Output

| VNH Pin Group | Connect To | Notes |
|---------------|------------|-------|
| Motor supply  | +12 V      | Bulk decoupling required nearby |
| Power ground  | GND        | Heavy return path |
| OUTA / OUTB   | Valve coil | Two-wire latching valve |

### Recommended control sequence

**OPEN pulse**
1. Set `INA = 1`, `INB = 0`
2. Wait about 20 µs
3. Set `PWM = 1`
4. Hold for pulse duration
5. Set `PWM = 0`
6. Return inputs to safe state if desired

**CLOSE pulse**
1. Set `INA = 0`, `INB = 1`
2. Wait about 20 µs
3. Set `PWM = 1`
4. Hold for pulse duration
5. Set `PWM = 0`
6. Return inputs to safe state

Current firmware pulse width:
- **50 ms**

That value is only a starting point. The real valve should be tested.

## 9. Valve Wiring

The valve is a simple two-wire latching coil.

| Valve Lead | Connect To |
|------------|------------|
| Lead 1     | VNH output A |
| Lead 2     | VNH output B |

The node reverses polarity to open vs close the valve.

This is the entire point of using the H-bridge.

## 10. Recommended Connectors

### Field bus / power input
4-position terminal block:
1. +12 V
2. GND
3. RS-485 A
4. RS-485 B

### Valve output
2-position terminal block:
1. Valve A
2. Valve B

### Programming
6-pin AVR ISP header

That is enough. No beauty contest hardware needed.

## 11. Firmware Behavior

### Supported commands
- `PING`
- `STATUS`
- `OPEN`
- `CLOSE`
- `WHO`

### Replies
- `ACK|addr`
- `STATE|addr|OPEN`
- `STATE|addr|CLOSED`
- `ID|addr`
- `ERR|addr|BADCMD`

### Current protocol format
ASCII framed protocol:

~~~text
@|LEN|COMMAND[|ARG1[|ARG2...]]|CHECKSUM<CR>
~~~

Examples:
~~~text
@|6|OPEN|1|E0
@|7|CLOSE|1|4F
@|8|STATUS|1|57
@|7|WHO|255|0D
~~~

Current firmware option:
- `IGNORE_RX_CHECKSUM = 1` may be used during bring-up

## 12. Boot and Safe State Requirements

At reset and startup:
- RS-485 should default to receive mode
- VNH `PWM` must be low
- VNH `INA` and `INB` must be low
- Valve must not receive any accidental pulse

This matters more than clever protocol junk.

## 13. Bring-Up Checklist

1. Verify +12 V input polarity
2. Verify regulator output is +5 V
3. Verify MCU clock and UART alive
4. Verify RS-485 receive path
5. Verify `PING` reply
6. Verify VNH control pins with a scope
7. Verify `OPEN` pulse width
8. Verify `CLOSE` pulse width
9. Verify valve actuates reliably
10. Verify no idle current path through the valve

## 14. Known Limits of Rev A

- Single valve channel only
- No current sensing
- No supply voltage telemetry
- No capacitor-bank telemetry
- No EEPROM address provisioning yet
- No watchdog/recovery logic documented here
- No hardware fault reporting from MultiSense in use yet

That is fine for a first board.

## 15. Recommended Next Steps

- Add watchdog
- Add simple brownout sanity checks
- Decide whether node ID stays compile-time or moves to EEPROM
- Decide whether MultiSense is worth reading
- Bench test real valve pulse current and required pulse width
- Confirm idle current in the full node

## 16. Summary

This node is intentionally plain:

- ATtiny4313
- RS-485 UART
- VNH7100BAS as a simple polarity-reversing pulse driver
- one valve
- 12 V power input
- 5 V logic
- boring framed ASCII protocol

That is the right level of complexity for rev A.
