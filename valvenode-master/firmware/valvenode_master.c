/**
 * @file valvenode_master.c
 * @brief Real Valve Master firmware for ATmega88PB.
 *
 * This firmware implements the production valve-master architecture:
 *
 *   Host over I2C
 *     -> ATmega88PB Valve Master
 *       -> switched 12 V field bus
 *       -> half-duplex RS-485
 *       -> valve-node slaves
 *
 * The I2C register map, command values, result values, reply registers, and
 * command deferral model intentionally match valvenode_master_sim.c. The sim
 * firmware is the bench model. This file is the real hardware backend.
 *
 * Target:
 *   ATmega88PB-AU @ 8 MHz
 *
 * I2C:
 *   7-bit address passed by Makefile using -DI2C_ADDR=0x09.
 *   Defaults to 0x09 if not supplied.
 *
 * RS-485 valve-node protocol:
 *   Requests: :DDC[A]\r
 *   Replies:  :DDC[A...][KK]\r or :DDC[A...][KK]\n
 *
 * Request checksums are omitted because the current valve-node slave accepts
 * checksumless requests. Reply checksums are verified.
 *
 * Locked product MCU pin assignments:
 *   PB3  MOSI
 *   PB4  MISO
 *   PB5  SCK
 *   PC0  FAULT_OUT, open-drain style: normal = high-Z, fault = driven low
 *   PC4  SDA
 *   PC5  SCL
 *   PD0  485RX
 *   PD1  485TX
 *   PD2  485DE
 *   PD3  RELAY_SET
 *   PD4  RELAY_RESET
 *   PD5  BLUE_LED_COMM
 *   PD6  RED_LED_FAULT
 */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Build configuration
 * ========================================================================== */

/** @brief 7-bit I2C slave address. Usually supplied by Makefile. */
#ifndef I2C_ADDR
#define I2C_ADDR                    0x09u
#endif

/** @brief Master firmware version high byte. */
#define MASTER_VERSION_HI           0x01u

/** @brief Master firmware version low byte. */
#define MASTER_VERSION_LO           0x03u

/** @brief Highest valid assigned valve-node address. */
#define MAX_NODE_ADDR               254u

/** @brief Highest channel number accepted by the master API. */
#define MAX_CHANNEL_API             16u

/** @brief Maximum node-map entries exposed in the register file. */
#define NODE_MAP_MAX                32u

/* ============================================================================
 * Product hardware pin map
 * ========================================================================== */

/**
 * @brief Fault output.
 *
 * Open-drain style:
 *   normal / no fault = high-Z
 *   fault             = driven low
 */
#define FAULT_OUT_PIN               PC0

/** @brief RS-485 driver-enable pin. High = transmit, low = receive. */
#define RS485_DE_PIN                PD2

/** @brief Latching relay SET coil driver output. */
#define RELAY_SET_PIN               PD3

/** @brief Latching relay RESET coil driver output. */
#define RELAY_RESET_PIN             PD4

/** @brief Communications activity LED output. Active-low. */
#define COMM_LED_PIN                PD5

/** @brief Fault LED output. Active-low. */
#define FAULT_LED_PIN               PD6

/* ============================================================================
 * Timing
 * ========================================================================== */

/** @brief Relay coil pulse width in milliseconds. */
#define RELAY_PULSE_MS              30u

/** @brief Delay after enabling field power so downstream nodes can wake. */
#define BUS_POWER_UP_DELAY_MS       250u

/** @brief UART baud rate for valve-node RS-485 bus. */
#define BAUD                        9600UL

/** @brief UART baud register value for F_CPU and BAUD. */
#define UBRR_VALUE                  ((F_CPU / (16UL * BAUD)) - 1UL)

/** @brief Delay before switching from RX to TX on the RS-485 transceiver. */
#define RS485_RX_TO_TX_TURNAROUND_MS 2u

/** @brief Normal command reply timeout in milliseconds. */
#define RS485_NORMAL_TIMEOUT_MS       250u

/** @brief WHO discovery timeout in milliseconds. */
#define RS485_WHO_TIMEOUT_MS          6000u

/** @brief Grace delay after broadcast cancel before powering down. */
#define RS485_CANCEL_GRACE_MS         20u

/** @brief Grace delay after broadcast close-all before host may power down. */
#define RS485_CLOSE_ALL_GRACE_MS      100u

/** @brief Maximum received RS-485 line length including ':' and NUL. */
#define RX_LINE_MAX                 24u

/* ============================================================================
 * I2C register map
 * ========================================================================== */

/** @brief Command register. Writing this queues a command. */
#define REG_COMMAND                 0x00u

/** @brief Status flags register. */
#define REG_STATUS                  0x01u

/** @brief Command argument 0, usually node address. */
#define REG_ARG0                    0x02u

/** @brief Command argument 1, usually channel. */
#define REG_ARG1                    0x03u

/** @brief Command argument 2, usually state. */
#define REG_ARG2                    0x04u

/** @brief Last command result. */
#define REG_RESULT                  0x05u

/** @brief Field power state, 0=off, 1=on. */
#define REG_POWER_STATE             0x06u

/** @brief Number of valid node entries in node map. */
#define REG_NODE_COUNT              0x07u

/** @brief Last reply source node. */
#define REG_REPLY_NODE              0x08u

/** @brief Last reply command byte. */
#define REG_REPLY_CMD               0x09u

/** @brief Last reply argument 0. */
#define REG_REPLY_ARG0              0x0Au

/** @brief Last reply argument 1. */
#define REG_REPLY_ARG1              0x0Bu

/** @brief Master firmware version high byte. */
#define REG_VERSION_HI              0x10u

/** @brief Master firmware version low byte. */
#define REG_VERSION_LO              0x11u

/** @brief Base address of node-list map. */
#define REG_NODE_MAP_BASE           0x20u

/* ============================================================================
 * Command values
 * ========================================================================== */

/** @brief No operation. */
#define CMD_NONE                    0x00u

/** @brief Turn field power on. */
#define CMD_POWER_ON                0x01u

/** @brief Turn field power off. */
#define CMD_POWER_OFF               0x02u

/** @brief Broadcast WHO and rebuild node map. */
#define CMD_WHO                     0x03u

/** @brief Ping one node. ARG0=node. */
#define CMD_PING                    0x04u

/** @brief Set channel state. ARG0=node, ARG1=channel, ARG2=state. */
#define CMD_SET_CHANNEL             0x05u

/** @brief Query channel state. ARG0=node, ARG1=channel. */
#define CMD_GET_CHANNEL_STATUS      0x06u

/** @brief Query node firmware version. ARG0=node. */
#define CMD_GET_NODE_VERSION        0x07u

/** @brief Put one node into identify mode. ARG0=node. */
#define CMD_IDENTIFY                0x08u

/** @brief Broadcast cancel for identify/config modes. */
#define CMD_CANCEL                  0x09u

/** @brief Put a node into config mode. ARG0=node, 0=unassigned node. */
#define CMD_CONFIG                  0x0Au

/** @brief Assign new address to node currently in config mode. ARG0=new node. */
#define CMD_ASSIGN                  0x0Bu

/** @brief Clear latched local error state. */
#define CMD_CLEAR_ERROR             0x0Cu

/** @brief Deliberately set local error state for bench testing. */
#define CMD_SET_ERROR               0x0Du

/** @brief Broadcast close-all command to all valve nodes. */
#define CMD_CLOSE_ALL               0x0Fu

/* ============================================================================
 * Status and result values
 * ========================================================================== */

/** @brief Set while a command is executing in the main loop. */
#define STATUS_BUSY                 (1u << 0)

/** @brief Set when last command or local condition produced an error. */
#define STATUS_ERROR                (1u << 1)

/** @brief Set when switched field power is believed to be on. */
#define STATUS_POWER_ON             (1u << 2)

/** @brief Command completed successfully. */
#define RESULT_OK                   0x00u

/** @brief Unknown command value. */
#define RESULT_BAD_COMMAND          0x01u

/** @brief Invalid node address argument. */
#define RESULT_BAD_NODE             0x02u

/** @brief Invalid channel argument. */
#define RESULT_BAD_CHANNEL          0x03u

/** @brief Node did not respond or is not known. */
#define RESULT_NODE_NOT_FOUND       0x04u

/** @brief Node rejected the requested channel. */
#define RESULT_UNSUPPORTED_CHANNEL  0x05u

/** @brief Assign requested but no node is in config mode. */
#define RESULT_CONFIG_REQUIRED      0x06u

/** @brief Requested address is already in use. */
#define RESULT_ADDRESS_IN_USE       0x07u

/** @brief Firmware was already busy when a command was submitted. */
#define RESULT_BUSY                 0x08u

/** @brief RS-485 reply was not received before timeout. */
#define RESULT_RS485_TIMEOUT        0x09u

/** @brief RS-485 reply checksum did not match. */
#define RESULT_RS485_BAD_CHECKSUM   0x0Au

/** @brief RS-485 reply was malformed or unexpected. */
#define RESULT_RS485_BAD_REPLY      0x0Bu

/** @brief Reserved legacy result slot. */
#define RESULT_RESERVED_0C          0x0Cu

/** @brief RS-485 command requested while field power is off. */
#define RESULT_POWER_OFF            0x0Eu

/* ============================================================================
 * Global state
 * ========================================================================== */

/** @brief Register file exposed over I2C. */
static volatile uint8_t g_regs[256];

/** @brief I2C register pointer. */
static volatile uint8_t g_reg_ptr = 0u;

/** @brief True after first write byte selected the register pointer. */
static volatile bool g_have_reg_ptr = false;

/** @brief Pending command copied by TWI ISR. */
static volatile uint8_t g_pending_cmd = CMD_NONE;

/** @brief True when main loop has a command to execute. */
static volatile bool g_cmd_pending = false;

/** @brief RX line buffer for RS-485 replies. */
static char g_rx_line[RX_LINE_MAX];

/* ============================================================================
 * GPIO helpers
 * ========================================================================== */

/** @brief Turn the communications LED on. LED is active-low. */
static inline void comm_led_on(void)
{
    PORTD &= (uint8_t)~(1u << COMM_LED_PIN);
}

/** @brief Turn the communications LED off. LED is active-low. */
static inline void comm_led_off(void)
{
    PORTD |= (1u << COMM_LED_PIN);
}

/** @brief Turn the fault LED on. LED is active-low. */
static inline void fault_led_on(void)
{
    PORTD &= (uint8_t)~(1u << FAULT_LED_PIN);
}

/** @brief Turn the fault LED off. LED is active-low. */
static inline void fault_led_off(void)
{
    PORTD |= (1u << FAULT_LED_PIN);
}

/** @brief Release fault output, high-Z / no fault. */
static inline void fault_out_release(void)
{
    PORTC &= (uint8_t)~(1u << FAULT_OUT_PIN);  /* no internal pull-up */
    DDRC  &= (uint8_t)~(1u << FAULT_OUT_PIN);  /* input mode = high-Z */
}

/** @brief Assert fault output low. */
static inline void fault_out_assert(void)
{
    PORTC &= (uint8_t)~(1u << FAULT_OUT_PIN);  /* output low */
    DDRC  |= (1u << FAULT_OUT_PIN);            /* drive low */
}

/** @brief Update local red LED and external open-drain-style fault output. */
static void update_fault_outputs(void)
{
    if (g_regs[REG_STATUS] & STATUS_ERROR) {
        fault_led_on();
        fault_out_assert();
    } else {
        fault_led_off();
        fault_out_release();
    }
}

/** @brief Set RS-485 transceiver to receive mode. */
static inline void rs485_set_rx(void)
{
    PORTD &= (uint8_t)~(1u << RS485_DE_PIN);
}

/** @brief Set RS-485 transceiver to transmit mode. */
static inline void rs485_set_tx(void)
{
    PORTD |= (1u << RS485_DE_PIN);
}

/** @brief Initialize GPIO direction and safe idle states. */
static void gpio_init(void)
{
    /*
     * FAULT output is open-drain style:
     *   normal = high-Z
     *   fault  = driven low
     */
    fault_out_release();

    /*
     * Preload output latches before enabling outputs.
     *
     * Relay outputs idle low.
     * RS-485 DE idle low = receive.
     * LEDs are active-low, so idle high = off.
     */
    PORTD &= (uint8_t)~(1u << RS485_DE_PIN);
    PORTD &= (uint8_t)~(1u << RELAY_SET_PIN);
    PORTD &= (uint8_t)~(1u << RELAY_RESET_PIN);
    PORTD |= (1u << COMM_LED_PIN);
    PORTD |= (1u << FAULT_LED_PIN);

    DDRD |= (1u << RS485_DE_PIN) |
            (1u << RELAY_SET_PIN) |
            (1u << RELAY_RESET_PIN) |
            (1u << COMM_LED_PIN) |
            (1u << FAULT_LED_PIN);

    rs485_set_rx();
    comm_led_off();
    fault_led_off();
}

/* ============================================================================
 * Result / register helpers
 * ========================================================================== */

/**
 * @brief Store command result and update STATUS_ERROR.
 *
 * @param result One of RESULT_*.
 */
static void set_result(uint8_t result)
{
    g_regs[REG_RESULT] = result;

    if (result == RESULT_OK) {
        g_regs[REG_STATUS] &= (uint8_t)~STATUS_ERROR;
    } else {
        g_regs[REG_STATUS] |= STATUS_ERROR;
    }

    update_fault_outputs();
}

/**
 * @brief Check whether switched field power is believed to be on.
 *
 * @return true if field power is on.
 */
static bool field_power_is_on(void)
{
    return (g_regs[REG_STATUS] & STATUS_POWER_ON) != 0u;
}

/**
 * @brief Require field power before sending RS-485 commands.
 *
 * @retval true Field power is on.
 * @retval false Field power is off and RESULT_POWER_OFF was set.
 */
static bool require_field_power(void)
{
    if (!field_power_is_on()) {
        set_result(RESULT_POWER_OFF);
        return false;
    }

    return true;
}

/** @brief Clear last reply registers. */
static void clear_reply_regs(void)
{
    g_regs[REG_REPLY_NODE] = 0u;
    g_regs[REG_REPLY_CMD] = 0u;
    g_regs[REG_REPLY_ARG0] = 0u;
    g_regs[REG_REPLY_ARG1] = 0u;
}

/** @brief Clear node-map registers and node count. */
static void clear_node_map(void)
{
    for (uint8_t i = 0u; i < NODE_MAP_MAX; i++) {
        g_regs[REG_NODE_MAP_BASE + i] = 0u;
    }

    g_regs[REG_NODE_COUNT] = 0u;
}

/**
 * @brief Validate assigned node address.
 *
 * @param node Candidate node address.
 * @return true if node is in the valid assigned range.
 */
static bool valid_node_addr(uint8_t node)
{
    return (node >= 1u) && (node <= MAX_NODE_ADDR);
}

/**
 * @brief Validate master API channel number.
 *
 * @param channel Candidate channel number.
 * @return true if channel is valid.
 */
static bool valid_channel(uint8_t channel)
{
    return (channel >= 1u) && (channel <= MAX_CHANNEL_API);
}

/* ============================================================================
 * Relay / field power
 * ========================================================================== */

/** @brief De-energize both latching relay coil driver outputs. */
static void relay_drive_off(void)
{
    PORTD &= (uint8_t)~(1u << RELAY_SET_PIN);
    PORTD &= (uint8_t)~(1u << RELAY_RESET_PIN);
}

/** @brief Pulse relay SET coil. */
static void relay_pulse_set(void)
{
    PORTD |= (1u << RELAY_SET_PIN);

    for (uint8_t i = 0u; i < RELAY_PULSE_MS; i++) {
        _delay_ms(1);
    }

    PORTD &= (uint8_t)~(1u << RELAY_SET_PIN);
}

/** @brief Pulse relay RESET coil. */
static void relay_pulse_reset(void)
{
    PORTD |= (1u << RELAY_RESET_PIN);

    for (uint8_t i = 0u; i < RELAY_PULSE_MS; i++) {
        _delay_ms(1);
    }

    PORTD &= (uint8_t)~(1u << RELAY_RESET_PIN);
}

/** @brief Turn on switched field power and wait for downstream nodes. */
static void bus_power_on(void)
{
    relay_pulse_set();

    g_regs[REG_POWER_STATE] = 1u;
    g_regs[REG_STATUS] |= STATUS_POWER_ON;

    for (uint16_t i = 0u; i < BUS_POWER_UP_DELAY_MS; i++) {
        _delay_ms(1);
    }
}

/** @brief Turn off switched field power. */
static void bus_power_off(void)
{
    relay_pulse_reset();

    g_regs[REG_POWER_STATE] = 0u;
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_POWER_ON;
}

/* ============================================================================
 * UART / RS-485 low level
 * ========================================================================== */

/** @brief Initialize UART0 for 9600 8N1 and leave RS-485 in receive mode. */
static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFFu);

    UCSR0A = 0u;
    UCSR0B = (1u << RXEN0) | (1u << TXEN0);
    UCSR0C = (1u << UCSZ01) | (1u << UCSZ00);

    rs485_set_rx();
}

/**
 * @brief Write one character to UART0.
 *
 * @param c Character to transmit.
 */
static void uart_putc(char c)
{
    while ((UCSR0A & (1u << UDRE0)) == 0u) {
    }

    UDR0 = (uint8_t)c;
}

/**
 * @brief Non-blocking UART read.
 *
 * @param[out] out Receives character if available.
 * @return true if one character was read.
 */
static bool uart_getc_nonblocking(char *out)
{
    if (UCSR0A & (1u << RXC0)) {
        *out = (char)UDR0;
        return true;
    }

    return false;
}

/** @brief Drain any stale bytes from UART receive buffer. */
static void uart_flush_rx(void)
{
    char c;

    while (uart_getc_nonblocking(&c)) {
    }
}

/** @brief Wait before enabling RS-485 transmit mode. */
static void rs485_wait_rx_to_tx_turnaround(void)
{
    for (uint8_t i = 0u; i < RS485_RX_TO_TX_TURNAROUND_MS; i++) {
        _delay_ms(1);
    }
}

/**
 * @brief Write a complete RS-485 frame.
 *
 * This function flushes stale RX bytes, enables transmit mode, writes the
 * NUL-terminated frame, waits for transmission complete, and returns to RX.
 *
 * @param s NUL-terminated frame string.
 */
static void uart_write_frame(const char *s)
{
    rs485_wait_rx_to_tx_turnaround();
    uart_flush_rx();

    rs485_set_tx();
    comm_led_on();

    UCSR0A |= (1u << TXC0);

    while (*s != '\0') {
        uart_putc(*s++);
    }

    while ((UCSR0A & (1u << TXC0)) == 0u) {
    }

    rs485_set_rx();
    comm_led_off();
}

/* ============================================================================
 * Protocol utilities
 * ========================================================================== */

/**
 * @brief Convert a nibble to uppercase ASCII hex.
 *
 * @param v Value whose low nibble is converted.
 * @return ASCII hex digit.
 */
static char nibble_to_hex(uint8_t v)
{
    v &= 0x0Fu;

    if (v < 10u) {
        return (char)('0' + v);
    }

    return (char)('A' + (v - 10u));
}

/**
 * @brief Convert ASCII hex digit to integer.
 *
 * @param c ASCII character.
 * @return 0..15 for valid hex, -1 otherwise.
 */
static int8_t hex_value(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return (int8_t)(c - '0');
    }

    if ((c >= 'A') && (c <= 'F')) {
        return (int8_t)(10 + (c - 'A'));
    }

    if ((c >= 'a') && (c <= 'f')) {
        return (int8_t)(10 + (c - 'a'));
    }

    return -1;
}

/**
 * @brief Parse two ASCII hex characters into one byte.
 *
 * @param s Pointer to two hex characters.
 * @param[out] out Parsed byte.
 * @return true on success.
 */
static bool parse_hex_byte(const char *s, uint8_t *out)
{
    int8_t hi = hex_value(s[0]);
    int8_t lo = hex_value(s[1]);

    if ((hi < 0) || (lo < 0)) {
        return false;
    }

    *out = (uint8_t)(((uint8_t)hi << 4) | (uint8_t)lo);
    return true;
}

/**
 * @brief Compute 8-bit ASCII sum checksum.
 *
 * @param body Pointer to bytes to sum.
 * @param len Number of bytes to sum.
 * @return 8-bit checksum.
 */
static uint8_t ascii_sum_checksum(const char *body, uint8_t len)
{
    uint8_t sum = 0u;

    for (uint8_t i = 0u; i < len; i++) {
        sum = (uint8_t)(sum + (uint8_t)body[i]);
    }

    return sum;
}

/**
 * @brief Convert channel number to one-character wire encoding.
 *
 * Mapping:
 *   1..9   -> '1'..'9'
 *   10..15 -> 'A'..'F'
 *   16     -> '0'
 *
 * @param channel Channel number.
 * @param[out] out Encoded character.
 * @return true if channel was valid.
 */
static bool channel_to_wire_char(uint8_t channel, char *out)
{
    if (!valid_channel(channel)) {
        return false;
    }

    if (channel <= 9u) {
        *out = (char)('0' + channel);
    } else if (channel <= 15u) {
        *out = (char)('A' + (channel - 10u));
    } else {
        *out = '0';
    }

    return true;
}

/**
 * @brief Convert one-character wire channel encoding to channel number.
 *
 * @param c Wire channel character.
 * @param[out] channel Decoded channel number.
 * @return true if character was valid.
 */
static bool wire_char_to_channel(char c, uint8_t *channel)
{
    if ((c >= '1') && (c <= '9')) {
        *channel = (uint8_t)(c - '0');
        return true;
    }

    if ((c >= 'A') && (c <= 'F')) {
        *channel = (uint8_t)(10u + (uint8_t)(c - 'A'));
        return true;
    }

    if ((c >= 'a') && (c <= 'f')) {
        *channel = (uint8_t)(10u + (uint8_t)(c - 'a'));
        return true;
    }

    if (c == '0') {
        *channel = 16u;
        return true;
    }

    return false;
}

/**
 * @brief Build a checksumless valve-node request frame.
 *
 * @param node Destination node address.
 * @param cmd Command character.
 * @param arg Optional one-character argument, or 0.
 * @param[out] frame Output buffer.
 * @param frame_len Output buffer length.
 */
static void make_request(uint8_t node, char cmd, char arg, char *frame, uint8_t frame_len)
{
    if (arg != 0) {
        if (frame_len >= 7u) {
            frame[0] = ':';
            frame[1] = nibble_to_hex((uint8_t)(node >> 4));
            frame[2] = nibble_to_hex(node);
            frame[3] = cmd;
            frame[4] = arg;
            frame[5] = '\r';
            frame[6] = '\0';
        }
    } else {
        if (frame_len >= 6u) {
            frame[0] = ':';
            frame[1] = nibble_to_hex((uint8_t)(node >> 4));
            frame[2] = nibble_to_hex(node);
            frame[3] = cmd;
            frame[4] = '\r';
            frame[5] = '\0';
        }
    }
}

/* ============================================================================
 * RS-485 receive / parse
 * ========================================================================== */

/**
 * @brief Read one line-oriented RS-485 reply with timeout.
 *
 * The returned line is stored in g_rx_line without CR/LF.
 *
 * @param timeout_ms Timeout in milliseconds.
 * @return true if a complete line was received.
 */
static bool read_line_timeout(uint16_t timeout_ms)
{
    bool active = false;
    uint8_t len = 0u;

    for (uint16_t elapsed = 0u; elapsed < timeout_ms; elapsed++) {
        char c;

        while (uart_getc_nonblocking(&c)) {
            if (!active) {
                if (c == ':') {
                    active = true;
                    len = 0u;
                    g_rx_line[len++] = ':';
                }
                continue;
            }

            if ((c == '\r') || (c == '\n')) {
                if (len > 0u) {
                    g_rx_line[len] = '\0';
                    return true;
                }

                active = false;
                len = 0u;
                continue;
            }

            if (c == ':') {
                active = true;
                len = 0u;
                g_rx_line[len++] = ':';
                continue;
            }

            if (len < (RX_LINE_MAX - 1u)) {
                g_rx_line[len++] = c;
            } else {
                active = false;
                len = 0u;
            }
        }

        _delay_ms(1);
    }

    return false;
}

/**
 * @brief Parse and checksum-verify the current g_rx_line.
 *
 * @param[out] node Reply source node.
 * @param[out] cmd Reply command.
 * @param[out] arg0 Optional reply arg0, or 0.
 * @param[out] arg1 Optional reply arg1, or 0.
 * @return true if reply syntax and checksum are valid.
 */
static bool parse_reply_line(uint8_t *node, char *cmd, char *arg0, char *arg1)
{
    uint8_t line_len = 0u;
    uint8_t got_cs = 0u;
    uint8_t calc_cs = 0u;
    uint8_t body_len = 0u;

    while ((line_len < RX_LINE_MAX) && (g_rx_line[line_len] != '\0')) {
        line_len++;
    }

    if (line_len < 5u) {
        return false;
    }

    if (g_rx_line[0] != ':') {
        return false;
    }

    if (!parse_hex_byte(&g_rx_line[1], node)) {
        return false;
    }

    if (!parse_hex_byte(&g_rx_line[line_len - 2u], &got_cs)) {
        return false;
    }

    body_len = (uint8_t)(line_len - 3u);
    calc_cs = ascii_sum_checksum(&g_rx_line[1], body_len);

    if (calc_cs != got_cs) {
        set_result(RESULT_RS485_BAD_CHECKSUM);
        return false;
    }

    *cmd = g_rx_line[3];
    *arg0 = 0;
    *arg1 = 0;

    if (line_len > 6u) {
        *arg0 = g_rx_line[4];
    }

    if (line_len > 7u) {
        *arg1 = g_rx_line[5];
    }

    return true;
}

/**
 * @brief Wait for a reply matching optional node and command filters.
 *
 * @param timeout_ms Timeout in milliseconds.
 * @param expected_node Expected node, or 0 for any node.
 * @param expected_cmd Expected command, or 0 for any command.
 * @param[out] reply_node Reply source node.
 * @param[out] reply_cmd Reply command.
 * @param[out] reply_arg0 Reply argument 0.
 * @param[out] reply_arg1 Reply argument 1.
 * @return true if a matching reply was received.
 */
static bool wait_for_reply(uint16_t timeout_ms,
                           uint8_t expected_node,
                           char expected_cmd,
                           uint8_t *reply_node,
                           char *reply_cmd,
                           char *reply_arg0,
                           char *reply_arg1)
{
    uint8_t node = 0u;
    char cmd = 0;
    char arg0 = 0;
    char arg1 = 0;

    while (read_line_timeout(timeout_ms)) {
        if (!parse_reply_line(&node, &cmd, &arg0, &arg1)) {
            return false;
        }

        if ((expected_node != 0u) && (node != expected_node)) {
            continue;
        }

        if ((expected_cmd != 0) && (cmd != expected_cmd)) {
            continue;
        }

        *reply_node = node;
        *reply_cmd = cmd;
        *reply_arg0 = arg0;
        *reply_arg1 = arg1;

        return true;
    }

    set_result(RESULT_RS485_TIMEOUT);
    return false;
}

/**
 * @brief Send a request and wait for one reply from that node.
 *
 * @param node Destination and expected reply node.
 * @param cmd Command character.
 * @param arg Optional argument character, or 0.
 * @param timeout_ms Reply timeout.
 * @param[out] reply_node Reply source node.
 * @param[out] reply_cmd Reply command.
 * @param[out] reply_arg0 Reply argument 0.
 * @param[out] reply_arg1 Reply argument 1.
 * @return true if a reply from the requested node was received.
 */
static bool send_request_wait_reply(uint8_t node,
                                    char cmd,
                                    char arg,
                                    uint16_t timeout_ms,
                                    uint8_t *reply_node,
                                    char *reply_cmd,
                                    char *reply_arg0,
                                    char *reply_arg1)
{
    char frame[8];

    make_request(node, cmd, arg, frame, sizeof(frame));
    uart_write_frame(frame);

    return wait_for_reply(timeout_ms, node, 0, reply_node, reply_cmd, reply_arg0, reply_arg1);
}

/* ============================================================================
 * Command backend
 * ========================================================================== */

/** @brief Execute CMD_POWER_ON. */
static void cmd_power_on(void)
{
    bus_power_on();
    set_result(RESULT_OK);
}

/** @brief Execute CMD_POWER_OFF. */
static void cmd_power_off(void)
{
    /*
     * The host owns power policy, but the master still gives the bus a short
     * grace period before cutting field power.
     */
    for (uint8_t i = 0u; i < RS485_CANCEL_GRACE_MS; i++) {
        _delay_ms(1);
    }

    bus_power_off();
    set_result(RESULT_OK);
}

/** @brief Execute CMD_WHO and rebuild node map from broadcast replies. */
static void cmd_who(void)
{
    char frame[8];
    uint8_t count = 0u;

    if (!require_field_power()) {
        return;
    }

    clear_node_map();

    make_request(0xFFu, 'W', 0, frame, sizeof(frame));
    uart_write_frame(frame);

    for (;;) {
        uint8_t node = 0u;
        char reply_cmd = 0;
        char arg0 = 0;
        char arg1 = 0;

        if (!read_line_timeout(RS485_WHO_TIMEOUT_MS)) {
            break;
        }

        if (!parse_reply_line(&node, &reply_cmd, &arg0, &arg1)) {
            return;
        }

        if ((reply_cmd == 'W') && valid_node_addr(node)) {
            if (count < NODE_MAP_MAX) {
                g_regs[REG_NODE_MAP_BASE + count] = node;
            }

            count++;
        }

        if (count >= NODE_MAP_MAX) {
            break;
        }
    }

    g_regs[REG_NODE_COUNT] = count;
    g_regs[REG_REPLY_CMD] = 'W';

    set_result(RESULT_OK);
}

/** @brief Execute CMD_PING. */
static void cmd_ping(void)
{
    uint8_t node = g_regs[REG_ARG0];
    uint8_t reply_node = 0u;
    char reply_cmd = 0;
    char arg0 = 0;
    char arg1 = 0;

    if (!valid_node_addr(node)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (!require_field_power()) {
        return;
    }

    if (!send_request_wait_reply(node,
                                 'P',
                                 0,
                                 RS485_NORMAL_TIMEOUT_MS,
                                 &reply_node,
                                 &reply_cmd,
                                 &arg0,
                                 &arg1)) {
        return;
    }

    if (reply_cmd != 'A') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    g_regs[REG_REPLY_NODE] = reply_node;
    g_regs[REG_REPLY_CMD] = (uint8_t)reply_cmd;
    set_result(RESULT_OK);
}

/** @brief Execute CMD_SET_CHANNEL. */
static void cmd_set_channel(void)
{
    uint8_t node = g_regs[REG_ARG0];
    uint8_t channel = g_regs[REG_ARG1];
    uint8_t state = g_regs[REG_ARG2];
    uint8_t reply_node = 0u;
    char reply_cmd = 0;
    char arg0 = 0;
    char arg1 = 0;
    char channel_char = 0;
    char cmd = state ? 'O' : 'C';

    if (!valid_node_addr(node)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (!channel_to_wire_char(channel, &channel_char)) {
        set_result(RESULT_BAD_CHANNEL);
        return;
    }

    if (!require_field_power()) {
        return;
    }

    if (!send_request_wait_reply(node,
                                 cmd,
                                 channel_char,
                                 RS485_NORMAL_TIMEOUT_MS,
                                 &reply_node,
                                 &reply_cmd,
                                 &arg0,
                                 &arg1)) {
        return;
    }

    g_regs[REG_REPLY_NODE] = reply_node;
    g_regs[REG_REPLY_CMD] = (uint8_t)reply_cmd;

    if (reply_cmd == 'A') {
        set_result(RESULT_OK);
    } else if (reply_cmd == 'E') {
        set_result(RESULT_UNSUPPORTED_CHANNEL);
    } else {
        set_result(RESULT_RS485_BAD_REPLY);
    }
}

/** @brief Execute CMD_GET_CHANNEL_STATUS. */
static void cmd_get_channel_status(void)
{
    uint8_t node = g_regs[REG_ARG0];
    uint8_t channel = g_regs[REG_ARG1];
    uint8_t reply_node = 0u;
    char reply_cmd = 0;
    char arg0 = 0;
    char arg1 = 0;
    char channel_char = 0;
    uint8_t reply_channel = 0u;

    if (!valid_node_addr(node)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (!channel_to_wire_char(channel, &channel_char)) {
        set_result(RESULT_BAD_CHANNEL);
        return;
    }

    if (!require_field_power()) {
        return;
    }

    if (!send_request_wait_reply(node,
                                 'S',
                                 channel_char,
                                 RS485_NORMAL_TIMEOUT_MS,
                                 &reply_node,
                                 &reply_cmd,
                                 &arg0,
                                 &arg1)) {
        return;
    }

    g_regs[REG_REPLY_NODE] = reply_node;
    g_regs[REG_REPLY_CMD] = (uint8_t)reply_cmd;

    if (reply_cmd == 'R') {
        if (!wire_char_to_channel(arg0, &reply_channel)) {
            set_result(RESULT_RS485_BAD_REPLY);
            return;
        }

        g_regs[REG_REPLY_ARG0] = reply_channel;
        g_regs[REG_REPLY_ARG1] = (uint8_t)arg1;
        set_result(RESULT_OK);
        return;
    }

    if (reply_cmd == 'E') {
        set_result(RESULT_UNSUPPORTED_CHANNEL);
        return;
    }

    set_result(RESULT_RS485_BAD_REPLY);
}

/** @brief Execute CMD_GET_NODE_VERSION. */
static void cmd_get_node_version(void)
{
    uint8_t node = g_regs[REG_ARG0];
    uint8_t reply_node = 0u;
    char reply_cmd = 0;
    char arg0 = 0;
    char arg1 = 0;

    if (!valid_node_addr(node)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (!require_field_power()) {
        return;
    }

    if (!send_request_wait_reply(node,
                                 'V',
                                 0,
                                 RS485_NORMAL_TIMEOUT_MS,
                                 &reply_node,
                                 &reply_cmd,
                                 &arg0,
                                 &arg1)) {
        return;
    }

    if (reply_cmd != 'V') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    /*
     * Slave replies as :DDVvvvvKK, four hex version digits.
     * parse_reply_line only returns arg0 and arg1, so read directly from line.
     */
    if ((g_rx_line[4] == 0) || (g_rx_line[5] == 0) ||
        (g_rx_line[6] == 0) || (g_rx_line[7] == 0)) {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    {
        int8_t v0 = hex_value(g_rx_line[4]);
        int8_t v1 = hex_value(g_rx_line[5]);
        int8_t v2 = hex_value(g_rx_line[6]);
        int8_t v3 = hex_value(g_rx_line[7]);

        if ((v0 < 0) || (v1 < 0) || (v2 < 0) || (v3 < 0)) {
            set_result(RESULT_RS485_BAD_REPLY);
            return;
        }

        g_regs[REG_REPLY_NODE] = reply_node;
        g_regs[REG_REPLY_CMD] = (uint8_t)'V';
        g_regs[REG_REPLY_ARG0] = (uint8_t)(((uint8_t)v0 << 4) | (uint8_t)v1);
        g_regs[REG_REPLY_ARG1] = (uint8_t)(((uint8_t)v2 << 4) | (uint8_t)v3);
    }

    set_result(RESULT_OK);
}

/** @brief Execute CMD_IDENTIFY. */
static void cmd_identify(void)
{
    uint8_t node = g_regs[REG_ARG0];
    uint8_t reply_node = 0u;
    char reply_cmd = 0;
    char arg0 = 0;
    char arg1 = 0;

    if (!valid_node_addr(node)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (!require_field_power()) {
        return;
    }

    if (!send_request_wait_reply(node,
                                 'I',
                                 0,
                                 RS485_NORMAL_TIMEOUT_MS,
                                 &reply_node,
                                 &reply_cmd,
                                 &arg0,
                                 &arg1)) {
        return;
    }

    if (reply_cmd != 'A') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    g_regs[REG_REPLY_NODE] = reply_node;
    g_regs[REG_REPLY_CMD] = (uint8_t)reply_cmd;
    set_result(RESULT_OK);
}

/** @brief Execute CMD_CANCEL. */
static void cmd_cancel(void)
{
    char frame[8];

    if (!require_field_power()) {
        return;
    }

    make_request(0xFFu, 'X', 0, frame, sizeof(frame));
    uart_write_frame(frame);

    for (uint8_t i = 0u; i < RS485_CANCEL_GRACE_MS; i++) {
        _delay_ms(1);
    }

    g_regs[REG_REPLY_CMD] = (uint8_t)'X';
    set_result(RESULT_OK);
}

/** @brief Execute CMD_CONFIG. */
static void cmd_config(void)
{
    uint8_t node = g_regs[REG_ARG0];
    uint8_t reply_node = 0u;
    char reply_cmd = 0;
    char arg0 = 0;
    char arg1 = 0;

    if (node != 0u && !valid_node_addr(node)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (!require_field_power()) {
        return;
    }

    if (!send_request_wait_reply(node,
                                 'M',
                                 0,
                                 RS485_NORMAL_TIMEOUT_MS,
                                 &reply_node,
                                 &reply_cmd,
                                 &arg0,
                                 &arg1)) {
        return;
    }

    if (reply_cmd != 'A') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    g_regs[REG_REPLY_NODE] = reply_node;
    g_regs[REG_REPLY_CMD] = (uint8_t)reply_cmd;
    set_result(RESULT_OK);
}

/** @brief Execute CMD_ASSIGN. */
static void cmd_assign(void)
{
    uint8_t new_node = g_regs[REG_ARG0];
    uint8_t reply_node = 0u;
    char reply_cmd = 0;
    char arg0 = 0;
    char arg1 = 0;

    if (!valid_node_addr(new_node)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (!require_field_power()) {
        return;
    }

    if (!send_request_wait_reply(new_node,
                                 'N',
                                 0,
                                 RS485_NORMAL_TIMEOUT_MS,
                                 &reply_node,
                                 &reply_cmd,
                                 &arg0,
                                 &arg1)) {
        return;
    }

    if (reply_cmd != 'A') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    g_regs[REG_REPLY_NODE] = reply_node;
    g_regs[REG_REPLY_CMD] = (uint8_t)reply_cmd;
    set_result(RESULT_OK);
}

/** @brief Execute CMD_CLOSE_ALL. */
static void cmd_close_all(void)
{
    char frame[8];

    if (!require_field_power()) {
        return;
    }

    /*
     * Broadcast close-all. No replies are expected.
     * Valve-node firmware must treat broadcast C with no channel argument as
     * close every local valve channel.
     */
    make_request(0xFFu, 'C', 0, frame, sizeof(frame));
    uart_write_frame(frame);

    /*
     * Hold field power long enough for nodes to receive the frame and pulse
     * their local latching solenoids before the host is likely to power down.
     */
    for (uint8_t i = 0u; i < RS485_CLOSE_ALL_GRACE_MS; i++) {
        _delay_ms(1);
    }

    g_regs[REG_REPLY_CMD] = (uint8_t)'C';
    set_result(RESULT_OK);
}

/** @brief Execute CMD_CLEAR_ERROR. */
static void cmd_clear_error(void)
{
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_ERROR;
    g_regs[REG_RESULT] = RESULT_OK;
    update_fault_outputs();
}

/** @brief Execute CMD_SET_ERROR. */
static void cmd_set_error(void)
{
    g_regs[REG_STATUS] |= STATUS_ERROR;
    g_regs[REG_RESULT] = RESULT_OK;
    update_fault_outputs();
}

/**
 * @brief Execute one queued command outside the TWI ISR.
 *
 * @param cmd Command value.
 */
static void execute_command(uint8_t cmd)
{
    g_regs[REG_STATUS] |= STATUS_BUSY;
    clear_reply_regs();

    switch (cmd) {
    case CMD_NONE:
        set_result(RESULT_OK);
        break;

    case CMD_POWER_ON:
        cmd_power_on();
        break;

    case CMD_POWER_OFF:
        cmd_power_off();
        break;

    case CMD_WHO:
        cmd_who();
        break;

    case CMD_PING:
        cmd_ping();
        break;

    case CMD_SET_CHANNEL:
        cmd_set_channel();
        break;

    case CMD_GET_CHANNEL_STATUS:
        cmd_get_channel_status();
        break;

    case CMD_GET_NODE_VERSION:
        cmd_get_node_version();
        break;

    case CMD_IDENTIFY:
        cmd_identify();
        break;

    case CMD_CANCEL:
        cmd_cancel();
        break;

    case CMD_CONFIG:
        cmd_config();
        break;

    case CMD_ASSIGN:
        cmd_assign();
        break;

    case CMD_CLEAR_ERROR:
        cmd_clear_error();
        break;

    case CMD_SET_ERROR:
        cmd_set_error();
        break;

    case CMD_CLOSE_ALL:
        cmd_close_all();
        break;

    default:
        set_result(RESULT_BAD_COMMAND);
        break;
    }

    g_regs[REG_STATUS] &= (uint8_t)~STATUS_BUSY;
}

/* ============================================================================
 * I2C slave
 * ========================================================================== */

/** @brief Initialize ATmega88PB TWI as an I2C slave. */
static void i2c_init(void)
{
    TWAR = (uint8_t)(I2C_ADDR << 1);
    TWAMR = 0x00u;
    TWCR = (1u << TWEA) | (1u << TWEN) | (1u << TWIE);
}

/**
 * @brief TWI interrupt handler.
 *
 * Write protocol:
 *   First received byte sets the register pointer.
 *   Subsequent bytes write sequential registers and auto-increment pointer.
 *   Writing REG_COMMAND queues command execution for the main loop.
 *
 * Read protocol:
 *   Current register pointer is read and auto-incremented.
 */
ISR(TWI_vect)
{
    switch (TW_STATUS) {

    case TW_SR_SLA_ACK:
    case TW_SR_GCALL_ACK:
        /*
         * A new slave-receive transaction is starting.
         * The first data byte received will be the register pointer.
         */
        g_have_reg_ptr = false;

        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;

    case TW_SR_DATA_ACK:
    {
        uint8_t data = TWDR;

        if (!g_have_reg_ptr) {
            /*
             * First byte after SLA+W is the register pointer.
             */
            g_reg_ptr = data;
            g_have_reg_ptr = true;
        } else {
            /*
             * Additional bytes write sequential registers.
             */
            g_regs[g_reg_ptr] = data;

            if (g_reg_ptr == REG_COMMAND) {
                if (!g_cmd_pending) {
                    g_pending_cmd = data;
                    g_cmd_pending = true;
                } else {
                    g_regs[REG_RESULT] = RESULT_BUSY;
                    g_regs[REG_STATUS] |= STATUS_ERROR;
                    update_fault_outputs();
                }
            }

            g_reg_ptr++;
        }

        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;
    }

    case TW_SR_STOP:
        /*
         * End of write transaction.
         *
         * Do not clear g_reg_ptr here. A following read transaction may depend
         * on the register pointer just written by the host.
         */
        g_have_reg_ptr = false;

        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;

    case TW_ST_SLA_ACK:
        /*
         * SLA+R received. Load the first byte for this read transaction.
         */
        if (g_reg_ptr == REG_VERSION_HI) {
            TWDR = MASTER_VERSION_HI;
        } else if (g_reg_ptr == REG_VERSION_LO) {
            TWDR = MASTER_VERSION_LO;
        } else {
            TWDR = g_regs[g_reg_ptr];
        }

        g_reg_ptr++;

        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;

    case TW_ST_DATA_ACK:
        /*
         * Previous byte was ACKed by the master. Load the next byte.
         */
        if (g_reg_ptr == REG_VERSION_HI) {
            TWDR = MASTER_VERSION_HI;
        } else if (g_reg_ptr == REG_VERSION_LO) {
            TWDR = MASTER_VERSION_LO;
        } else {
            TWDR = g_regs[g_reg_ptr];
        }

        g_reg_ptr++;

        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;

    case TW_ST_DATA_NACK:
    case TW_ST_LAST_DATA:
        /*
         * Master is done reading.
         */
        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;

    case TW_BUS_ERROR:
    default:
        /*
         * Recover from illegal START/STOP condition.
         */
        TWCR = (1u << TWSTO) |
               (1u << TWINT) |
               (1u << TWEN)  |
               (1u << TWEA)  |
               (1u << TWIE);
        break;
    }
}

/* ============================================================================
 * Init
 * ========================================================================== */

/** @brief Initialize register file to safe defaults. */
static void regs_init(void)
{
    for (uint16_t i = 0u; i < 256u; i++) {
        g_regs[i] = 0u;
    }

    g_regs[REG_STATUS] = 0u;
    g_regs[REG_RESULT] = RESULT_OK;
    g_regs[REG_POWER_STATE] = 0u;

    /*
     * Also store version in g_regs for ordinary register inspection.
     * The TWI ISR returns these two registers directly from constants too.
     */
    g_regs[REG_VERSION_HI] = MASTER_VERSION_HI;
    g_regs[REG_VERSION_LO] = MASTER_VERSION_LO;

    g_reg_ptr = 0u;
    g_have_reg_ptr = false;
    g_pending_cmd = CMD_NONE;
    g_cmd_pending = false;

    clear_reply_regs();
    clear_node_map();
}

/* ============================================================================
 * Main
 * ========================================================================== */

/**
 * @brief Firmware entry point.
 *
 * @return Never returns.
 */
int main(void)
{
    gpio_init();
    regs_init();
    uart_init();
    i2c_init();

    /*
     * Safe idle outputs.
     *
     * Do not call bus_power_off() here. That function pulses the latching
     * relay RESET coil. During firmware/debug cycles, reset should not
     * repeatedly fire relay coils.
     */
    relay_drive_off();

    g_regs[REG_POWER_STATE] = 0u;
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_POWER_ON;
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_BUSY;
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_ERROR;
    g_regs[REG_RESULT] = RESULT_OK;
    update_fault_outputs();

    sei();

    for (;;) {
        if (g_cmd_pending) {
            uint8_t cmd;

            cli();
            cmd = g_pending_cmd;
            g_pending_cmd = CMD_NONE;
            g_cmd_pending = false;
            sei();

            execute_command(cmd);
        }

        update_fault_outputs();
    }
}
