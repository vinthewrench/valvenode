# Valve Node Serial Protocol v0.1

## Purpose

This protocol is for simple ASCII-framed communication between a master and one or more valve nodes on a shared serial bus such as RS-485.

The goals are:

- easy to read on a terminal
- easy to debug with a logic analyzer
- simple enough for small AVR firmware
- deterministic parsing
- room to grow later

This is not a binary protocol. All fields are ASCII text.

## Frame Format

Each frame has this structure:

~~~text
@|LEN|COMMAND[|ARG1[|ARG2...]]|CHECKSUM<CR>
~~~

Where:

- `@` is the start-of-frame header
- `|` is the field separator
- `LEN` is the ASCII decimal payload length
- `COMMAND` is the command name
- `ARG1`, `ARG2`, and so on are optional ASCII arguments
- `CHECKSUM` is a two-digit ASCII hexadecimal checksum
- `<CR>` is carriage return, `0x0D`

No line feed is required.

## Basic Rules

1. Every frame starts with `@`.
2. Every frame ends with `<CR>`.
3. Fields are separated by the `|` character.
4. `LEN` is the character count of the text between the second `|` and the `|` immediately before `CHECKSUM`.
5. `CHECKSUM` is computed over the payload text only, not including:
   - the `@`
   - the `LEN`
   - the separators before the payload
   - the separator before `CHECKSUM`
   - the final `<CR>`
6. All command and argument text is ASCII.
7. Command names should be uppercase.
8. Nodes should ignore frames with bad checksum or malformed length.

## Payload Definition

The payload is everything from `COMMAND` through the last argument, joined by `|`.

Example payload:

~~~text
OPEN|7
~~~

In this example:

- `COMMAND = OPEN`
- `ARG1 = 7`
- payload length = 6 characters

Character count:

- `O` `P` `E` `N` = 4
- `|` = 1
- `7` = 1

Total = 6

## Length Field

`LEN` is ASCII decimal.

Examples:

- `4`
- `6`
- `12`

Leading zeroes should not be required, but a receiver may accept them.

Examples:

~~~text
@|4|PING|2A<CR>
@|6|OPEN|7|5F<CR>
@|13|STATUS|NODE1|91<CR>
~~~

## Checksum

Checksum is the 8-bit unsigned sum of all payload characters, modulo 256.

Transmit it as two ASCII hexadecimal digits, uppercase preferred.

### Example checksum calculation

Payload:

~~~text
OPEN|7
~~~

ASCII bytes:

~~~text
O  = 0x4F
P  = 0x50
E  = 0x45
N  = 0x4E
|  = 0x7C
7  = 0x37
~~~

Sum:

~~~text
0x4F + 0x50 + 0x45 + 0x4E + 0x7C + 0x37 = 0x1E5
~~~

Modulo 256:

~~~text
0xE5
~~~

Frame:

~~~text
@|6|OPEN|7|E5<CR>
~~~

## Addressing

The first argument after the command should normally be the destination node address unless the command is a broadcast command.

Recommended address rules:

- `1` to `254` are valid node addresses
- `255` is reserved for broadcast
- `0` is invalid or reserved

Examples:

~~~text
@|6|OPEN|7|E5<CR>
@|8|CLOSE|12|0D<CR>
@|10|STATUS|255|76<CR>
~~~

## Recommended Command Set

### Master to node

- `PING|addr`
- `STATUS|addr`
- `OPEN|addr`
- `CLOSE|addr`
- `WHO|255`

### Node to master replies

- `ACK|addr`
- `ERR|addr|code`
- `STATE|addr|OPEN`
- `STATE|addr|CLOSED`
- `ID|addr`

## Broadcast Discovery

To ask all nodes to identify themselves:

~~~text
@|7|WHO|255|CS<CR>
~~~

Each node that hears this valid broadcast should:

1. verify checksum
2. verify destination is `255`
3. compute a reply delay from its own node ID
4. wait its slot time
5. send an `ID|addr` reply

### Slot timing

Use a deterministic slot based on node address.

Example:

~~~text
delay_ms = node_addr * slot_ms
~~~

For a small system:

- baud rate: 19200
- slot time: 10 ms

Examples:

- node 1 replies at 10 ms
- node 2 replies at 20 ms
- node 7 replies at 70 ms

This avoids reply collisions as long as node IDs are unique.

## Receiver Behavior

A receiver should:

1. wait for `@`
2. collect bytes until `<CR>`
3. split fields on `|`
4. validate field count
5. parse `LEN`
6. rebuild payload text
7. verify payload length
8. compute checksum
9. dispatch command only if all checks pass

On failure, discard the frame.

## Error Handling

Recommended error codes:

- `BADLEN`
- `BADCS`
- `BADCMD`
- `BADADDR`
- `BUSY`

Example:

~~~text
@|14|ERR|7|BADCS|A8<CR>
~~~

## Safe Parsing Limits

For small AVR firmware, define fixed limits.

Recommended first cut:

- max full frame length: 64 bytes
- max payload length: 48 bytes
- max command length: 12 bytes
- max argument count: 4

If a frame exceeds limits, discard it.

## Examples

### Ping node 3

~~~text
@|6|PING|3|D4<CR>
~~~

Reply:

~~~text
@|5|ACK|3|C2<CR>
~~~

### Open node 7

~~~text
@|6|OPEN|7|E5<CR>
~~~

Reply:

~~~text
@|5|ACK|7|C6<CR>
~~~

### Ask node 7 for status

~~~text
@|8|STATUS|7|9E<CR>
~~~

Reply if open:

~~~text
@|12|STATE|7|OPEN|69<CR>
~~~

### Broadcast discovery

~~~text
@|7|WHO|255|1F<CR>
~~~

Possible replies over time:

~~~text
@|4|ID|1|16<CR>
@|4|ID|7|1C<CR>
@|5|ID|12|50<CR>
~~~

## Implementation Notes

- Keep command matching case-sensitive and uppercase.
- Do not allow nodes to reply to malformed broadcast frames.
- Do not allow a node to act on a command until length and checksum are both valid.
- Default idle state for the valve driver must be disabled.
- Replies should be short.

## Versioning

Protocol version should be tracked in firmware documentation, not necessarily inside every frame in rev A.

If needed later, add:

~~~text
PROTO|1
~~~

as part of a capability or identification response.

## Summary

This protocol is intentionally plain and boring:

- ASCII framing
- fixed header
- decimal length
- text commands
- 8-bit checksum
- `<CR>` terminator

That makes it easy to debug and easy to implement on a small AVR.
