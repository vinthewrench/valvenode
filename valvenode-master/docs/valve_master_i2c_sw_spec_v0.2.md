# Valve Master I2C Software Specification

Version: 0.2  
Target board: Valve Master  
MCU: ATmega88PB-AU  
I2C role: slave device  
Default I2C address: 0x09

## 1. Purpose

The Valve Master board provides an I2C interface between the host controller and an RS485 valve-node network.

The Valve Master is responsible for:

~~~text
receiving simple I2C commands
controlling the RS485 transceiver
sending valve commands to slave valve nodes
reporting command status and result
optionally scanning the RS485 bus for responding nodes
controlling relay power to the slave valve-node bus
~~~

The host/client driver is responsible for higher-level policy:

~~~text
power sequencing
wake delay after slave power-on
command queueing
relay anti-chatter timing
completion callbacks
configuration of expected valve nodes
~~~

The firmware does not manage command queues or wake delays.

## 2. I2C Address

The Valve Master uses this default 7-bit I2C address:

~~~c
#define VM_I2C_ADDR  0x09
~~~

Address 0x08 is reserved for the existing power-control board.

Reserved future Valve Master alternate addresses:

~~~text
0x0A
0x0B
0x0C
~~~

## 3. I2C Transaction Model

The Valve Master uses a register-style I2C interface.

Typical write:

~~~text
START
I2C_ADDR + WRITE
REGISTER
DATA...
STOP
~~~

Typical read:

~~~text
START
I2C_ADDR + WRITE
REGISTER
REPEATED START
I2C_ADDR + READ
DATA...
STOP
~~~

Long operations are asynchronous.

An I2C write to COMMAND means:

~~~text
command accepted or rejected by the Valve Master
~~~

It does not mean:

~~~text
the RS485 slave completed the operation
~~~

For commands involving RS485, the client must:

~~~text
write command
poll STATUS until BUSY clears
read LAST_RESULT
~~~

## 4. Register Map

### 4.1 Core Registers

~~~text
0x00  COMMAND
0x01  STATUS
0x02  CURRENT_NODE
0x03  NODE_COUNT
0x04  ARG0
0x05  ARG1
0x06  POWER_STATE
0x07  LAST_RESULT
~~~

### 4.2 Master Version Registers

~~~text
0x08  MASTER_VERSION_HI
0x09  MASTER_VERSION_LO
0x0A  MASTER_PROTOCOL
~~~

The master firmware version is stored as a packed 16-bit BCD value, high byte first.

Example:

~~~text
0x0100 = 1.00
0x0215 = 2.15
~~~

### 4.3 Last Queried Node Info Registers

~~~text
0x10  NODE_INFO_ID
0x11  NODE_VERSION_HI
0x12  NODE_VERSION_LO
0x13  NODE_PROTOCOL
0x14  NODE_VALVE_COUNT
0x15  NODE_BOARD_TYPE
~~~

These registers contain the cached result of the most recent GET_NODE_INFO command.

### 4.4 Node Bitmap

~~~text
0x20-0x3F  NODE_MAP[32]
~~~

This is a 32-byte bitmap showing which RS485 node addresses responded to the most recent WHO_SCAN.

### 4.5 Reserved

~~~text
0x0B-0x0F  reserved
0x16-0x1F  reserved
0x40-0xFF  reserved for future use
~~~

## 5. Register Details

### 5.1 COMMAND, 0x00

Write-only command register.

Writing a command value starts the requested operation.

~~~c
#define VM_REG_COMMAND  0x00
~~~

Supported commands:

~~~c
#define VM_CMD_NONE       0x00
#define VM_CMD_WHO_SCAN   0x01
#define VM_CMD_SET_VALVE  0x02
#define VM_CMD_POWER_ON   0x03
#define VM_CMD_POWER_OFF  0x04
#define VM_CMD_GET_NODE_INFO  0x05
~~~

### 5.2 STATUS, 0x01

Read-only status register.

~~~c
#define VM_REG_STATUS  0x01
~~~

Status bits:

~~~c
#define VM_STATUS_READY      0x01
#define VM_STATUS_BUSY       0x02
#define VM_STATUS_MAP_VALID  0x04
#define VM_STATUS_ERROR      0x08
#define VM_STATUS_POWER_ON   0x10
~~~

| Bit | Name | Meaning |
|---:|---|---|
| 0x01 | READY | Firmware initialized |
| 0x02 | BUSY | Command in progress |
| 0x04 | MAP_VALID | NODE_MAP contains valid scan data |
| 0x08 | ERROR | Last completed command failed |
| 0x10 | POWER_ON | Slave-node power relay is on |

Normal boot state:

~~~text
READY = 1
BUSY = 0
MAP_VALID = 0
ERROR = 0
POWER_ON = 0
~~~

### 5.3 CURRENT_NODE, 0x02

Read-only.

~~~c
#define VM_REG_CURRENT_NODE  0x02
~~~

During WHO_SCAN, this contains the RS485 node address currently being tested.

During SET_VALVE, this contains the target node address.

When idle, this should be:

~~~text
0x00
~~~

### 5.4 NODE_COUNT, 0x03

Read-only.

~~~c
#define VM_REG_NODE_COUNT  0x03
~~~

Number of nodes discovered during the most recent WHO_SCAN.

Valid range:

~~~text
0-254
~~~

### 5.5 ARG0, 0x04

Read/write argument register.

~~~c
#define VM_REG_ARG0  0x04
~~~

For SET_VALVE:

~~~text
ARG0 = RS485 node ID
~~~

Valid node IDs:

~~~text
1-254
~~~

Reserved:

~~~text
0   invalid / master / unused
255 broadcast / reserved
~~~

### 5.6 ARG1, 0x05

Read/write argument register.

~~~c
#define VM_REG_ARG1  0x05
~~~

For SET_VALVE, this is the packed valve/state byte.

~~~text
bit 7     state
          0 = OFF / CLOSE
          1 = ON / OPEN

bits 2-0  valve index
          0 = valve 1
          1 = valve 2
          ...
          7 = valve 8

bits 6-3  reserved, must be 0
~~~

Definitions:

~~~c
#define VM_VALVE_STATE_BIT        0x80
#define VM_VALVE_INDEX_MASK       0x07
#define VM_VALVE_RESERVED_MASK    0x78
~~~

| Valve | State | ARG1 |
|---:|---|---:|
| 1 | OFF | 0x00 |
| 1 | ON | 0x80 |
| 2 | OFF | 0x01 |
| 2 | ON | 0x81 |
| 8 | OFF | 0x07 |
| 8 | ON | 0x87 |

### 5.7 POWER_STATE, 0x06

Read-only.

~~~c
#define VM_REG_POWER_STATE  0x06
~~~

Values:

~~~c
#define VM_POWER_OFF  0x00
#define VM_POWER_ON   0x01
~~~

This mirrors the relay output state.

### 5.8 LAST_RESULT, 0x07

Read-only.

~~~c
#define VM_REG_LAST_RESULT  0x07
~~~

Result of the most recently completed or rejected command.

~~~c
#define VM_RESULT_OK              0x00
#define VM_RESULT_BUSY            0x01
#define VM_RESULT_BAD_NODE        0x02
#define VM_RESULT_BAD_ARGUMENT    0x03
#define VM_RESULT_NODE_NOT_FOUND  0x04
#define VM_RESULT_RS485_TIMEOUT   0x05
#define VM_RESULT_RS485_ERROR     0x06
#define VM_RESULT_POWER_OFF       0x07
#define VM_RESULT_BAD_COMMAND     0x08
~~~

### 5.9 MASTER_VERSION_HI / MASTER_VERSION_LO, 0x08-0x09

Read-only.

~~~c
#define VM_REG_MASTER_VERSION_HI  0x08
#define VM_REG_MASTER_VERSION_LO  0x09
~~~

The Valve Master firmware version as a packed 16-bit BCD value, high byte first.

~~~text
0x0100 = 1.00
0x0215 = 2.15
~~~

### 5.10 MASTER_PROTOCOL, 0x0A

Read-only.

~~~c
#define VM_REG_MASTER_PROTOCOL  0x0A
~~~

Protocol version implemented by the Valve Master firmware.

### 5.11 NODE_INFO_ID, 0x10

Read-only.

~~~c
#define VM_REG_NODE_INFO_ID  0x10
~~~

RS485 node ID for the most recently cached node-info response. This register is valid only after GET_NODE_INFO completes with VM_RESULT_OK.

### 5.12 NODE_VERSION_HI / NODE_VERSION_LO, 0x11-0x12

Read-only.

~~~c
#define VM_REG_NODE_VERSION_HI  0x11
#define VM_REG_NODE_VERSION_LO  0x12
~~~

Firmware version of the most recently queried slave node as a packed 16-bit BCD value, high byte first.

### 5.13 NODE_PROTOCOL, 0x13

Read-only.

~~~c
#define VM_REG_NODE_PROTOCOL  0x13
~~~

RS485 protocol version reported by the most recently queried slave node.

### 5.14 NODE_VALVE_COUNT, 0x14

Read-only.

~~~c
#define VM_REG_NODE_VALVE_COUNT  0x14
~~~

Number of valve channels reported by the most recently queried slave node. Present dual-valve boards should report 2. Future boards may report up to 8.

### 5.15 NODE_BOARD_TYPE, 0x15

Read-only.

~~~c
#define VM_REG_NODE_BOARD_TYPE  0x15
~~~

Board-type identifier reported by the most recently queried slave node. Exact board-type values are reserved for the slave-node protocol definition.

## 6. Node Bitmap

The node bitmap is located at:

~~~c
#define VM_REG_NODE_MAP  0x20
~~~

Register range:

~~~text
0x20-0x3F
~~~

Length:

~~~text
32 bytes
~~~

The bitmap represents RS485 node IDs 1-254.

Mapping:

~~~c
byte_index = node_id >> 3;
bit_index  = node_id & 0x07;
~~~

Node examples:

~~~text
node 1   -> NODE_MAP[0], bit 1
node 2   -> NODE_MAP[0], bit 2
node 7   -> NODE_MAP[0], bit 7
node 8   -> NODE_MAP[1], bit 0
node 17  -> NODE_MAP[2], bit 1
node 254 -> NODE_MAP[31], bit 6
~~~

Node 0 is unused. Node 255 is reserved.

To read the full map:

~~~bash
i2ctransfer -y 1 w1@0x09 0x20 r32
~~~

## 7. Commands

### 7.1 VM_CMD_WHO_SCAN

~~~c
#define VM_CMD_WHO_SCAN  0x01
~~~

Purpose:

~~~text
Scan RS485 node addresses 1-254 and record which nodes respond.
~~~

Arguments:

~~~text
none
~~~

Behavior:

~~~text
1. If BUSY is already set:
     reject command
     LAST_RESULT = VM_RESULT_BUSY
     ERROR = 1

2. Otherwise:
     clear NODE_MAP
     NODE_COUNT = 0
     MAP_VALID = 0
     ERROR = 0
     BUSY = 1

3. For node addresses 1-254:
     CURRENT_NODE = node
     send RS485 WHO request
     wait for valid reply
     if reply valid:
         set node bit in NODE_MAP
         increment NODE_COUNT

4. When scan completes:
     CURRENT_NODE = 0
     BUSY = 0
     MAP_VALID = 1
     LAST_RESULT = VM_RESULT_OK
~~~

The firmware does not automatically power on the slave-node bus before WHO_SCAN.

If the client wants to scan powered nodes, it must do:

~~~text
POWER_ON
wait client-defined wake delay
WHO_SCAN
poll BUSY
read NODE_MAP
~~~

### 7.2 VM_CMD_SET_VALVE

~~~c
#define VM_CMD_SET_VALVE  0x02
~~~

Purpose:

~~~text
Send an ON/OFF command to one valve on one RS485 valve-node board.
~~~

Arguments:

~~~text
ARG0 = node ID, 1-254
ARG1 = packed valve/state byte
~~~

Packed ARG1:

~~~text
bit 7     state, 0=OFF, 1=ON
bits 2-0  valve index, 0-7
bits 6-3  reserved, must be 0
~~~

Behavior:

~~~text
1. If BUSY is already set:
     reject command
     LAST_RESULT = VM_RESULT_BUSY
     ERROR = 1

2. Validate ARG0:
     valid node ID is 1-254
     otherwise LAST_RESULT = VM_RESULT_BAD_NODE

3. Validate ARG1:
     bits 6-3 must be 0
     otherwise LAST_RESULT = VM_RESULT_BAD_ARGUMENT

4. Check power:
     if POWER_ON is not set:
         LAST_RESULT = VM_RESULT_POWER_OFF
         ERROR = 1
         do not send RS485 command

5. If valid:
     BUSY = 1
     ERROR = 0
     CURRENT_NODE = ARG0

6. Send RS485 SET_VALVE command.

7. Wait for valid slave reply.

8. On success:
     LAST_RESULT = VM_RESULT_OK

9. On timeout:
     LAST_RESULT = VM_RESULT_RS485_TIMEOUT
     ERROR = 1

10. On malformed/error reply:
     LAST_RESULT = VM_RESULT_RS485_ERROR
     ERROR = 1

11. Clear:
     CURRENT_NODE = 0
     BUSY = 0
~~~

The firmware does not perform slave wake delay.

The client must do:

~~~text
POWER_ON
wait wake delay
SET_VALVE
poll BUSY
read LAST_RESULT
POWER_OFF when appropriate
~~~

### 7.3 VM_CMD_POWER_ON

~~~c
#define VM_CMD_POWER_ON  0x03
~~~

Purpose:

~~~text
Turn on relay power to slave valve-node boards.
~~~

Arguments:

~~~text
none
~~~

Behavior:

~~~text
1. Turn relay on.
2. POWER_STATE = VM_POWER_ON.
3. STATUS_POWER_ON = 1.
4. LAST_RESULT = VM_RESULT_OK.
~~~

The firmware does not wait for slaves to boot.

Wake delay is a client-driver responsibility.

### 7.4 VM_CMD_POWER_OFF

~~~c
#define VM_CMD_POWER_OFF  0x04
#define VM_CMD_GET_NODE_INFO  0x05
~~~

Purpose:

~~~text
Turn off relay power to slave valve-node boards.
~~~

Arguments:

~~~text
none
~~~

Behavior:

~~~text
1. Turn relay off.
2. POWER_STATE = VM_POWER_OFF.
3. STATUS_POWER_ON = 0.
4. MAP_VALID = 0.
5. LAST_RESULT = VM_RESULT_OK.
~~~

MAP_VALID is cleared because the discovered map is stale once slave-node power is removed.

### 7.5 VM_CMD_GET_NODE_INFO

~~~c
#define VM_CMD_GET_NODE_INFO  0x05
~~~

Purpose:

~~~text
Query one RS485 slave node and cache its version/capability information in the NODE_INFO registers.
~~~

Arguments:

~~~text
ARG0 = node ID, 1-254
~~~

Behavior:

~~~text
1. If BUSY is already set:
     reject command
     LAST_RESULT = VM_RESULT_BUSY
     ERROR = 1

2. Validate ARG0:
     valid node ID is 1-254
     otherwise LAST_RESULT = VM_RESULT_BAD_NODE

3. Check power:
     if POWER_ON is not set:
         LAST_RESULT = VM_RESULT_POWER_OFF
         ERROR = 1
         do not send RS485 command

4. If valid:
     BUSY = 1
     ERROR = 0
     CURRENT_NODE = ARG0

5. Send RS485 GET_NODE_INFO request.

6. Wait for valid slave reply.

7. On success:
     cache NODE_INFO_ID
     cache NODE_VERSION_HI / NODE_VERSION_LO
     cache NODE_PROTOCOL
     cache NODE_VALVE_COUNT
     cache NODE_BOARD_TYPE
     LAST_RESULT = VM_RESULT_OK

8. On timeout:
     LAST_RESULT = VM_RESULT_RS485_TIMEOUT
     ERROR = 1

9. On malformed/error reply:
     LAST_RESULT = VM_RESULT_RS485_ERROR
     ERROR = 1

10. Clear:
     CURRENT_NODE = 0
     BUSY = 0
~~~

The firmware does not perform slave wake delay. The client must explicitly power the slave bus and wait before issuing GET_NODE_INFO.

This command is intended for diagnostics, commissioning, and support for future slave boards with different valve counts or driver hardware.

## 8. Client Driver Flow

### 8.1 Normal SET_VALVE Flow

The client driver should perform:

~~~text
1. Queue valve request.
2. If slave power is off:
     send POWER_ON
     wait client-defined wake delay

3. Write ARG0 = node ID.
4. Write ARG1 = packed valve/state.
5. Write COMMAND = SET_VALVE.

6. Poll STATUS until BUSY clears.

7. Read LAST_RESULT.

8. Invoke completion callback.

9. If more queued valve requests exist:
     process next request.

10. If queue is idle:
     wait client-defined hold interval.
     send POWER_OFF.
~~~

Suggested client timing defaults:

~~~text
wake delay after POWER_ON:    500 ms
inter-command delay:          50-100 ms
idle power-off delay:         1-2 seconds
~~~

These are client policy values, not firmware constants.

### 8.2 Example: Node 7, Valve 2 ON

Valve 2 means valve index 1.

State ON means bit 7 set.

~~~text
ARG0 = 0x07
ARG1 = 0x81
COMMAND = 0x02
~~~

Linux command example:

~~~bash
i2cset -y 1 0x09 0x04 0x07
i2cset -y 1 0x09 0x05 0x81
i2cset -y 1 0x09 0x00 0x02
~~~

Poll status:

~~~bash
i2cget -y 1 0x09 0x01
~~~

When STATUS & 0x02 is zero, read result:

~~~bash
i2cget -y 1 0x09 0x07
~~~

Expected success:

~~~text
LAST_RESULT = 0x00
~~~

### 8.3 Example: Node 7, Valve 2 OFF

~~~text
ARG0 = 0x07
ARG1 = 0x01
COMMAND = 0x02
~~~

Linux:

~~~bash
i2cset -y 1 0x09 0x04 0x07
i2cset -y 1 0x09 0x05 0x01
i2cset -y 1 0x09 0x00 0x02
~~~

### 8.4 Example: Read Master Version

Read high and low bytes and combine them as a packed 16-bit BCD value.

~~~bash
i2cget -y 1 0x09 0x08
i2cget -y 1 0x09 0x09
~~~

Example result:

~~~text
MASTER_VERSION_HI = 0x01
MASTER_VERSION_LO = 0x00
version = 0x0100 = 1.00
~~~

### 8.5 Example: Read Node Info for Node 7

The client must power the slave bus and wait for node wakeup before issuing GET_NODE_INFO.

~~~bash
i2cset -y 1 0x09 0x00 0x03      # POWER_ON
sleep 1
i2cset -y 1 0x09 0x04 0x07      # ARG0 = node 7
i2cset -y 1 0x09 0x00 0x05      # GET_NODE_INFO
~~~

Poll status until BUSY clears:

~~~bash
i2cget -y 1 0x09 0x01
~~~

Then read LAST_RESULT:

~~~bash
i2cget -y 1 0x09 0x07
~~~

If LAST_RESULT is 0x00, read cached node information:

~~~bash
i2cget -y 1 0x09 0x10      # NODE_INFO_ID
i2cget -y 1 0x09 0x11      # NODE_VERSION_HI
i2cget -y 1 0x09 0x12      # NODE_VERSION_LO
i2cget -y 1 0x09 0x13      # NODE_PROTOCOL
i2cget -y 1 0x09 0x14      # NODE_VALVE_COUNT
i2cget -y 1 0x09 0x15      # NODE_BOARD_TYPE
~~~

## 9. Firmware Rules

### 9.1 I2C ISR Rule

The I2C interrupt handler must not perform RS485 work.

It may:

~~~text
store written register values
capture COMMAND
set command-pending flag
return quickly
~~~

The main loop performs:

~~~text
RS485 transmit
RS485 receive wait
timeouts
WHO scan
SET_VALVE transaction
~~~

### 9.2 Busy Rule

If a command is received while BUSY is set:

~~~text
reject command
LAST_RESULT = VM_RESULT_BUSY
ERROR = 1
~~~

No command queue is implemented in firmware.

The only exception may be added later for an ABORT command, but no abort command exists in v0.1.

### 9.3 Power Rule

SET_VALVE does not turn power on.

If field power is off:

~~~text
SET_VALVE fails with VM_RESULT_POWER_OFF
~~~

Client code must explicitly issue:

~~~text
POWER_ON
wait wake delay
SET_VALVE
~~~

### 9.4 Discovery Rule

Discovery is diagnostic/commissioning behavior.

It is not required for normal operation.

The client configuration is the normal source of truth for expected nodes and valves.

## 10. Recommended C Constants

~~~c
#ifndef VALVE_MASTER_I2C_H
#define VALVE_MASTER_I2C_H

#define VM_I2C_ADDR              0x09

#define VM_REG_COMMAND           0x00
#define VM_REG_STATUS            0x01
#define VM_REG_CURRENT_NODE      0x02
#define VM_REG_NODE_COUNT        0x03
#define VM_REG_ARG0              0x04
#define VM_REG_ARG1              0x05
#define VM_REG_POWER_STATE       0x06
#define VM_REG_LAST_RESULT       0x07
#define VM_REG_MASTER_VERSION_HI 0x08
#define VM_REG_MASTER_VERSION_LO 0x09
#define VM_REG_MASTER_PROTOCOL   0x0A
#define VM_REG_NODE_INFO_ID      0x10
#define VM_REG_NODE_VERSION_HI   0x11
#define VM_REG_NODE_VERSION_LO   0x12
#define VM_REG_NODE_PROTOCOL     0x13
#define VM_REG_NODE_VALVE_COUNT  0x14
#define VM_REG_NODE_BOARD_TYPE   0x15
#define VM_REG_NODE_MAP          0x20
#define VM_NODE_MAP_BYTES        32

#define VM_CMD_NONE              0x00
#define VM_CMD_WHO_SCAN          0x01
#define VM_CMD_SET_VALVE         0x02
#define VM_CMD_POWER_ON          0x03
#define VM_CMD_POWER_OFF         0x04
#define VM_CMD_GET_NODE_INFO     0x05

#define VM_STATUS_READY          0x01
#define VM_STATUS_BUSY           0x02
#define VM_STATUS_MAP_VALID      0x04
#define VM_STATUS_ERROR          0x08
#define VM_STATUS_POWER_ON       0x10

#define VM_POWER_OFF             0x00
#define VM_POWER_ON              0x01

#define VM_RESULT_OK             0x00
#define VM_RESULT_BUSY           0x01
#define VM_RESULT_BAD_NODE       0x02
#define VM_RESULT_BAD_ARGUMENT   0x03
#define VM_RESULT_NODE_NOT_FOUND 0x04
#define VM_RESULT_RS485_TIMEOUT  0x05
#define VM_RESULT_RS485_ERROR    0x06
#define VM_RESULT_POWER_OFF      0x07
#define VM_RESULT_BAD_COMMAND    0x08

#define VM_VALVE_STATE_BIT       0x80
#define VM_VALVE_INDEX_MASK      0x07
#define VM_VALVE_RESERVED_MASK   0x78

#endif
~~~

## 11. First Firmware Implementation Target

Implement first:

~~~text
POWER_ON
POWER_OFF
SET_VALVE
WHO_SCAN
GET_NODE_INFO
STATUS reads
LAST_RESULT reads
MASTER_VERSION reads
NODE_INFO reads
NODE_MAP reads
~~~

Do not implement yet:

~~~text
command queueing
automatic relay holdoff
automatic slave wake delay
automatic discovery on boot
automatic node capability negotiation
address assignment
abort command
~~~

This keeps the firmware target clean and testable while allowing diagnostics and future slave-node capability reporting.
