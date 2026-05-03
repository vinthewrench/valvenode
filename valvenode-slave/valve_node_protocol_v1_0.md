# Valve Node Serial Protocol v1.0

## Purpose

This protocol defines simple ASCII-framed communication between a master and one or more dual-valve slave nodes on a shared half-duplex serial bus such as RS-485.

The goals are:

- easy to read in a terminal
- easy to debug with a logic analyzer
- small enough for AVR firmware
- deterministic parsing and timing
- explicit node addressing
- simple broadcast discovery and configuration

This is not a binary protocol. Frames are ASCII text.

## Frame Format

Each request or reply has this structure:

~~~text
:DDC[A][B][KK]<EOL>
~~~

Where:

- `:` is the start-of-frame marker
- `DD` is a 2 hex digit node address
- `C` is a 1 character command or reply code
- `A` is an optional 1 character argument
- `B` is an optional 1 character second argument, used in some replies
- `KK` is an optional 2 hex digit checksum on requests, and a required 2 hex digit checksum on replies
- `<EOL>` may be `CR`, `LF`, or `CRLF`

Examples:

~~~text
:04P
:04O1
:04S2
:04V
:04PB4
:04S1B5
~~~

## Addressing

Address rules:

- `01` to `FE` are valid assigned node addresses
- `FF` is broadcast
- `00` is invalid for normal operation and represents an unassigned node

Notes:

- An unassigned node does not answer normal directed commands
- An unassigned node may accept `:00M` to enter config mode
- The master is not a node on this protocol and does not have a node address

## Checksum

Checksum is the 8-bit unsigned sum of the ASCII body bytes modulo 256.

Do not include:

- the leading `:`
- the final end-of-line

The checksum is transmitted as two ASCII hexadecimal digits.

### Example checksum calculation

Body:

~~~text
04P
~~~

ASCII bytes:

~~~text
'0' = 0x30
'4' = 0x34
'P' = 0x50
~~~

Sum:

~~~text
0x30 + 0x34 + 0x50 = 0xB4
~~~

So the request with checksum is:

~~~text
:04PB4
~~~

Replies always include checksum.

Requests may omit checksum.

## Command Set

### Directed commands

- `:DDP` ping node `DD`
- `:DDO1` open valve 1 on node `DD`
- `:DDC1` close valve 1 on node `DD`
- `:DDO2` open valve 2 on node `DD`
- `:DDC2` close valve 2 on node `DD`
- `:DDS1` read valve 1 state on node `DD`
- `:DDS2` read valve 2 state on node `DD`
- `:DDV` read firmware version from node `DD`
- `:DDI` put node `DD` into identify mode
- `:DDM` put node `DD` into config mode

### Special directed command for unassigned node

- `:00M` put an unassigned node into config mode

### Config-mode address assignment

- `:DDN` assign new address `DD`

Rules:

- `N` is only valid while the target node is already in config mode
- `00` and `FF` are not valid assigned addresses
- after successful assignment the node stores the new address in EEPROM, updates runtime address, replies using the new address, and exits config mode

### Broadcast commands

- `:FFW` broadcast WHO
- `:FFX` broadcast cancel special modes

`X` silently cancels:

- config mode
- identify mode

## Replies

The node uses these reply codes:

- `A` acknowledge
- `E` error
- `W` WHO response
- `R` valve state response
- `V` firmware version response
- `B` boot banner

### Examples

Ping reply from node 04:

~~~text
:04AA5
~~~

Valve 1 open reply from node 04:

~~~text
:04R1O42
~~~

Valve 1 closed reply from node 04:

~~~text
:04R1C36
~~~

WHO reply from node 04:

~~~text
:04WBB
~~~

Firmware version reply, version 1.00:

~~~text
:04V01004B
~~~

Boot banner from node 04:

~~~text
:04BA6
~~~

## Firmware Version Encoding

Firmware version is returned as 16-bit packed BCD, transmitted as 4 hexadecimal characters.

Examples:

- `0x0100` means version `1.00`
- `0x0215` means version `2.15`

Example reply:

~~~text
:04V01004B
~~~

## Valve State Replies

Status replies use command `R`.

Format:

~~~text
:DDRxyKK
~~~

Where:

- `x` is valve channel, `1` or `2`
- `y` is valve state:
  - `O` = open
  - `C` = closed

Examples:

~~~text
:04R1O42
:04R1C36
:04R2O43
:04R2C37
~~~

## Config Mode

Config mode is used for address management.

Entry methods:

- directed command `:DDM`
- boot-time local config button hold
- special directed command `:00M` for an unassigned node

Behavior:

- node blinks the LED in config pattern while in config mode
- config mode times out automatically if not completed
- successful address assignment exits config mode
- `:FFX` cancels config mode silently

## Identify Mode

Identify mode is used to physically locate a node in the field.

Entry method:

- directed command `:DDI`

Behavior:

- node continues to function normally while identifying
- LED blinks at the configured identify rate
- `:FFX` cancels identify mode silently

## Broadcast WHO Discovery

Broadcast WHO is used to discover assigned nodes on the bus.

Request:

~~~text
:FFW
~~~

Behavior for each assigned node:

1. verify the frame is a valid broadcast WHO
2. ignore the request if the node is still unassigned `00`
3. compute a deterministic reply delay based on node address
4. wait its slot time
5. send `W` reply containing its own address in the frame header

Slot timing rule:

~~~text
delay_ms = node_addr * WHO_SLOT_MS
~~~

Example with `WHO_SLOT_MS = 20 ms`:

- node `01` replies after about `20 ms`
- node `02` replies after about `40 ms`
- node `04` replies after about `80 ms`

Important:

- WHO replies must not add extra pre-reply LED delays, or slot timing will collide
- the master must wait long enough for all expected reply slots to expire

## Receiver Behavior

A receiver should:

1. wait for `:`
2. collect body characters until `CR` or `LF`
3. accept `CR`, `LF`, or `CRLF`
4. restart frame capture if a new `:` appears mid-frame
5. validate the frame length against the allowed command shapes
6. parse address and command
7. if checksum is present, verify it
8. dispatch only valid frames

Malformed frames should be discarded silently.

## Boot Behavior

At boot the node:

1. initializes GPIO and UART
2. loads node address from EEPROM into RAM
3. optionally checks the config button hold window
4. emits startup blink pattern
5. sends boot banner if the node has a valid assigned address

Unassigned node `00` stays quiet at boot.

## Example Commands

### Basic directed commands

~~~text
:04P
:04S1
:04S2
:04O1
:04O2
:04C1
:04C2
:04V
:04I
:04M
~~~

### Unassigned node enrollment

~~~text
:00M
:04N
:04P
:04V
~~~

### Broadcast commands

~~~text
:FFW
:FFX
~~~

### Commands with real checksums

~~~text
:04PB4
:04S1B5
:04S2B6
:04O1B1
:04O2B2
:04C1A5
:04C2A6
:04VB6
:04IB0
:04MB4
:00M9D
:04NB5
:FFW1C
:FFX1D
~~~

## Suggested Bench Test Order

Assigned node:

~~~text
:04P
:04V
:04S1
:04O1
:04S1
:04C1
:04S1
:04O2
:04S2
:04C2
:04S2
:04I
:FFX
:FFW
~~~

Unassigned node:

~~~text
:00M
:04N
:04P
:04V
:FFW
~~~

## Terminal Notes

- The parser accepts `CR`, `LF`, or `CRLF`
- A normal serial terminal is sufficient
- Do not paste multiple half-duplex request frames back-to-back without waiting for reply or timeout
