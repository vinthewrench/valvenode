# Valve Master Design Document (v0.3)

**Date:** 2026-04-22

## 1. Purpose and Scope

The Valve Master is the controller for the RS-485 valve network. It powers the field bus, sends commands to valve-node slaves, receives replies, monitors a fault input, and provides local visual status.

This document covers the current master board as drawn in the uploaded schematic and BOM.

Design goals:
- Offline operation
- Deterministic command/response behavior
- Simple firmware and easy field debugging
- RS-485 half-duplex bus control
- Master-side switched 12 V field power
- Separate COMM and FAULT indicators
- Clean service access for reset and AVR-ISP
- Dual-use assembly option depending on host system

## 2. Source of Truth

This document is based on:
- Uploaded master schematic
- Uploaded BOM
- Locked ATmega88PB-AU pin assignments from this conversation
- Locked design rule that the master is not terminated
- User-defined dual-use assembly mode described in this conversation
- GardenIrrigation repository reference for the garden-system mode  [oai_citation:0‡GitHub](https://github.com/vinthewrench/GardenIrrigation)

## 3. High-Level Function

The master board performs these functions:

1. Accept raw +12 V input power when used in standalone NodeLync mode.
2. Generate local +5 V logic power when the onboard regulator is used.
3. Switch field +12 V to the downstream valve-node network through a relay.
4. Drive an RS-485 half-duplex network through a MAX485.
5. Monitor a FAULT input.
6. Indicate power, communication activity, fault state, and switched 12 V state with LEDs.
7. Expose AVR-ISP programming and reset access.
8. Provide nodeLync / isolated-side headers and I2C signals.
9. Support two distinct assembly/use modes.

## 4. Two Supported Assembly / Use Modes

This board can be built and used in two different ways.

### Mode A: Standalone NodeLync wiring mode

In this mode the board is wired using the NodeLync IN and OUT connectors.

Requirements:
- Raw +12 V supply is provided to the board.
- The onboard regulator **D45V5F5** is populated.
- The NodeLync connectors are populated.
- Jumper **J6 POWER_SELECT** is set to **REG**.

In this mode:
- the board makes its own local +5 V from the onboard regulator
- NodeLync IN and OUT are used as the system interconnect
- this is the self-powered standalone master configuration

### Mode B: GardenIrrigation system mode

In this mode the board is plugged into the user's GardenIrrigation system as defined by the referenced repository. The repository is the hardware for the garden irrigation project and includes the garden PCB and DRV103 plugin board context.  [oai_citation:1‡GitHub](https://github.com/vinthewrench/GardenIrrigation)

Requirements:
- Jumper **J6 POWER_SELECT** is set to **ISO**
- NodeLync IN and OUT connectors are **not populated**
- The onboard regulator **D45V5F5** is **not populated**

In this mode:
- the board is powered from the external GardenIrrigation system side
- the isolated / external +5 V path is used instead of the local regulator
- NodeLync connectors are omitted from assembly

## 5. Major Components

### Core Logic
- **U2**: ATMEGA88PB-AU

### RS-485 Interface
- **U4**: MAX485EPA
- **D2**: SM712_SOT23, RS-485 line protection on A/B

### Power
- **J1**: raw +12 V power input
- **U1**: D45V5F5, 5 V regulator
- **TVS1**: P6SMB18A, 12 V line clamp
- **J6**: POWER_SELECT header, selects REG or ISO source

### Relay / Field Power Switching
- **U3**: ULN2003A
- **K1**: DSP1-L2-DC12V relay
- **F1**: Keystone 3568 fuse holder
- **Output**: +12 SWITCHED

### User / Service Interface
- **SW1**: reset switch
- **J2**: AVR-ISP-6 programming header

### Indicators
- **LED1**: ISO +5 V indicator, green
- **LED2**: +5 V indicator, green
- **LED3**: +12 V switched indicator, green
- **LED4**: COMM indicator, blue
- **LED5**: FAULT indicator, red

## 6. Power Architecture

### Inputs
- Raw supply enters on **J1** as **+12V** and **GND1** when used in standalone mode.

### Protection
- **TVS1 = P6SMB18A** clamps the 12 V input line.
- **D2 = SM712** protects the RS-485 A/B lines.

### Regulation / Source Selection
The board supports two possible +5 V sourcing paths.

#### REG position
- selected when onboard regulator is used
- requires **U1 = D45V5F5** to be populated
- used in standalone NodeLync mode

#### ISO position
- selected when +5 V is supplied externally by the host system
- used in GardenIrrigation system mode
- in this mode **U1** is not populated

### Switched Field Power
- Relay **K1** switches field power and produces **+12 SWITCHED**
- Relay is driven by **ULN2003A**
- Fuse holder **F1** is in the switched power path

## 7. Board-Level Signal Assignments

## 7.1 Locked MCU Pin Assignment

| AVR Pin | Function | Direction | Notes |
|---|---|---|---|
| PB3 | MOSI | I/O | SPI / AVR-ISP |
| PB4 | MISO | I/O | SPI / AVR-ISP |
| PB5 | SCK | I/O | SPI / AVR-ISP |
| PC0 | FAULT | Input | Fault sense input |
| PC4 | ISO SDA | I/O | I2C SDA through R9 |
| PC5 | ISO SCL | I/O | I2C SCL through R11 |
| PD0 | 485RX | Input | UART RX from MAX485 RO |
| PD1 | 485TX | Output | UART TX to MAX485 DI |
| PD2 | 485DE | Output | MAX485 RE/DE control |
| PD3 | RELAY SET / drive out | Output | ULN2003A input |
| PD4 | RELAY RESET / drive out | Output | ULN2003A input |
| PD5 | BLUE LED COMM | Output | Communication activity LED |
| PD6 | RED LED FAULT | Output | Fault LED |
| PC6 | RESET | Input | Reset pull-up and reset switch |

## 8. Connector Table

| Ref | Description | Function |
|---|---|---|
| J1 | 2-pin 5.08 mm terminal | Raw +12 V power input |
| J2 | AVR-ISP-6 | Programming header |
| J3 | nodeLync OUT, 3-pin | NodeLync output, populate only in standalone mode |
| J5 | nodeLync IN, 3-pin | NodeLync input, populate only in standalone mode |
| J6 | 1x03 header | POWER_SELECT, REG or ISO |
| J7 | 2x08 socket | ISO / bus-side support signals |
| J8 | 4-pin 5.08 mm terminal | RS-485 and field power out: B, A, GND1, +12V |

## 9. RS-485 Interface Details

### Transceiver
- **MAX485EPA**

### Connections
- `RO` -> `485RX` -> U2 PD0
- `DI` <- `485TX` <- U2 PD1
- `RE` and `DE` tied together -> `485DE` -> U2 PD2
- `A` -> `485_A`
- `B` -> `485_B`

### Bus Connector J8
| J8 Pin | Signal |
|---|---|
| 1 | B |
| 2 | A |
| 3 | GND1 |
| 4 | +12V |

### Protection
- **SM712** is placed on the A/B pair

### Termination Policy
The master does not include RS-485 termination.

Termination is provided only at selected remote slave nodes that serve as the two physical end points of the installed network.

## 10. I2C / External Support Signals

The schematic shows support signals brought through J7 and through series resistors into the MCU:

- **ISO SDA** through **R9 = 220 Ω** to **PC4**
- **ISO SCL** through **R11 = 220 Ω** to **PC5**
- **FAULT** brought to **PC0**
- **ISO +5V**
- **+12V**
- **GND1**

**I2C address is 0x09**

## 11. Reset and Programming

### Reset
- **R6 = 10K** pull-up on RESET
- **SW1** reset switch to ground

### AVR-ISP
- **J2** AVR-ISP-6
- Series resistors on programming-related lines:
  - **R3 = 470 Ω**
  - **R4 = 470 Ω**
  - **R5 = 470 Ω**

## 12. LED Functions

### LED1
- Green
- ISO +5 V present

### LED2
- Green
- Local +5 V present

### LED3
- Green
- +12 V switched present

### LED4
- Blue
- COMM activity

### LED5
- Red
- FAULT indication

## 13. Assembly Variants

## 13.1 Standalone NodeLync build
Populate:
- J3
- J5
- U1
- associated regulator support parts
- J6 set to REG

Requires:
- local +12 V input

## 13.2 GardenIrrigation build
Do not populate:
- J3
- J5
- U1

Set:
- J6 to ISO

This build relies on the external GardenIrrigation system for the selected supply path and interface context. The referenced repository describes that garden hardware project.  [oai_citation:2‡GitHub](https://github.com/vinthewrench/GardenIrrigation)

## 14. Firmware Behavior Assumptions

The master firmware should follow strict request/reply rules:

- one request
- one reply or timeout
- then next request

Do not pipeline commands.

Suggested responsibilities:
- drive `485DE` only during transmit
- wait for reply or timeout before sending next command
- validate reply structure and checksum
- control relay drive outputs safely
- monitor `FAULT`
- flash COMM LED on send and/or receive
- assert FAULT LED when fault is present

## Master Functional Role

The master board acts as an I2C-controlled RS-485 valve network controller.

It is commanded by an upstream host over I2C. The host does not talk directly to valve slaves.

The master performs these operations on behalf of the host:

- power the RS-485 valve bus on through the relay
- wait for field-side power stabilization
- transmit RS-485 valve-node commands
- wait for reply or timeout
- optionally query node status
- power the RS-485 bus back off after command completion

This architecture is intended for latching-solenoid valve nodes, where continuous field power is not required.

## Planned I2C Command Model

The I2C interface will be used to command the master to perform valve-network actions.

Planned command categories:
- valve ON / OPEN
- valve OFF / CLOSE
- status request
- slave address configuration / enrollment
- optional discovery / identify operations

The exact I2C register map and command encoding are still to be defined.


## 15. Open Items / Review Notes

1. Confirm exact relay semantics of PD3 and PD4 in final firmware.
2. Confirm fuse sizing for the switched 12 V field feed.
3. Confirm external bus bias location, since the master is not terminated.
4. Mechanical review still needed for final enclosure fit and connector access.
5. Final assembly documentation should explicitly separate the standalone REG build from the GardenIrrigation ISO build.

## 16. Summary

The master board supports two valid build modes:

- a standalone NodeLync-based self-powered build using the onboard regulator with J6 set to REG
- a GardenIrrigation-hosted build using the external system path with J6 set to ISO, with NodeLync connectors and U1 not populated

In both cases the board remains the RS-485 command master, provides relay-switched 12 V field power, uses MAX485 for the bus, uses SM712 for A/B protection, and does not provide RS-485 termination on the master.