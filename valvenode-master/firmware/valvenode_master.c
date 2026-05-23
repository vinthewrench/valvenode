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
 *   Requests: :DDC[A]KK\r
 *   Replies:  :DDC[A...][KK]\r or :DDC[A...][KK]\n
 *
 * Request checksums are now always sent. Reply checksums are verified.
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
#define MASTER_VERSION_LO           0x05u

/** @brief Highest valid assigned valve-node address. */
#define MAX_NODE_ADDR               254u

/** @brief Highest channel number accepted by the master API. */
#define MAX_CHANNEL_API             16u

/** @brief Maximum node-map entries exposed in the register file. */
#define NODE_MAP_MAX                32u

/* ============================================================================
 * Product hardware pin map
 * ========================================================================== */

#define FAULT_OUT_PIN               PC0
#define RS485_DE_PIN                PD2
#define RELAY_SET_PIN               PD3
#define RELAY_RESET_PIN             PD4
#define COMM_LED_PIN                PD5
#define FAULT_LED_PIN               PD6

/* ============================================================================
 * Timing
 * ========================================================================== */

#define RELAY_PULSE_MS              30u
#define BUS_POWER_UP_DELAY_MS       250u
#define BAUD                        9600UL
#define UBRR_VALUE                  ((F_CPU / (16UL * BAUD)) - 1UL)
#define RS485_RX_TO_TX_TURNAROUND_MS 2u
#define RS485_NORMAL_TIMEOUT_MS       1000u
#define RS485_WHO_TIMEOUT_MS          6000u
#define RS485_CANCEL_GRACE_MS         20u
#define RS485_CLOSE_ALL_GRACE_MS      100u
#define RX_LINE_MAX                 24u
#define RS485_RETRY_COUNT             1u
#define RS485_RETRY_DELAY_MS          100u

/* ============================================================================
 * I2C register map
 * ========================================================================== */

#define REG_COMMAND                 0x00u
#define REG_STATUS                  0x01u
#define REG_ARG0                    0x02u
#define REG_ARG1                    0x03u
#define REG_ARG2                    0x04u
#define REG_RESULT                  0x05u
#define REG_POWER_STATE             0x06u
#define REG_NODE_COUNT              0x07u
#define REG_REPLY_NODE              0x08u
#define REG_REPLY_CMD               0x09u
#define REG_REPLY_ARG0              0x0Au
#define REG_REPLY_ARG1              0x0Bu
#define REG_VERSION_HI              0x10u
#define REG_VERSION_LO              0x11u
#define REG_NODE_MAP_BASE           0x20u

/* ============================================================================
 * Command values
 * ========================================================================== */

#define CMD_NONE                    0x00u
#define CMD_POWER_ON                0x01u
#define CMD_POWER_OFF               0x02u
#define CMD_WHO                     0x03u
#define CMD_PING                    0x04u
#define CMD_SET_CHANNEL             0x05u
#define CMD_GET_CHANNEL_STATUS      0x06u
#define CMD_GET_NODE_VERSION        0x07u
#define CMD_IDENTIFY                0x08u
#define CMD_CANCEL                  0x09u
#define CMD_CONFIG                  0x0Au
#define CMD_ASSIGN                  0x0Bu
#define CMD_CLEAR_ERROR             0x0Cu
#define CMD_SET_ERROR               0x0Du
#define CMD_CLOSE_ALL               0x0Fu

/* ============================================================================
 * Status and result values
 * ========================================================================== */

#define STATUS_BUSY                 (1u << 0)
#define STATUS_ERROR                (1u << 1)
#define STATUS_POWER_ON             (1u << 2)

#define RESULT_OK                   0x00u
#define RESULT_BAD_COMMAND          0x01u
#define RESULT_BAD_NODE             0x02u
#define RESULT_BAD_CHANNEL          0x03u
#define RESULT_NODE_NOT_FOUND       0x04u
#define RESULT_UNSUPPORTED_CHANNEL  0x05u
#define RESULT_CONFIG_REQUIRED      0x06u
#define RESULT_ADDRESS_IN_USE       0x07u
#define RESULT_BUSY                 0x08u
#define RESULT_RS485_TIMEOUT        0x09u
#define RESULT_RS485_BAD_CHECKSUM   0x0Au
#define RESULT_RS485_BAD_REPLY      0x0Bu
#define RESULT_RESERVED_0C          0x0Cu
#define RESULT_POWER_OFF            0x0Eu

/* ============================================================================
 * Global state
 * ========================================================================== */

static volatile uint8_t g_regs[256];
static volatile uint8_t g_reg_ptr = 0u;
static volatile bool g_have_reg_ptr = false;
static volatile uint8_t g_pending_cmd = CMD_NONE;
static volatile bool g_cmd_pending = false;

static char g_rx_line[RX_LINE_MAX];

/* ============================================================================
 * GPIO helpers
 * ========================================================================== */

static inline void comm_led_on(void)
{
    PORTD &= (uint8_t)~(1u << COMM_LED_PIN);
}

static inline void comm_led_off(void)
{
    PORTD |= (1u << COMM_LED_PIN);
}

static inline void fault_led_on(void)
{
    PORTD &= (uint8_t)~(1u << FAULT_LED_PIN);
}

static inline void fault_led_off(void)
{
    PORTD |= (1u << FAULT_LED_PIN);
}

static inline void fault_out_release(void)
{
    PORTC &= (uint8_t)~(1u << FAULT_OUT_PIN);
    DDRC  &= (uint8_t)~(1u << FAULT_OUT_PIN);
}

static inline void fault_out_assert(void)
{
    PORTC &= (uint8_t)~(1u << FAULT_OUT_PIN);
    DDRC  |= (1u << FAULT_OUT_PIN);
}

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

static inline void rs485_set_rx(void)
{
    PORTD &= (uint8_t)~(1u << RS485_DE_PIN);
}

static inline void rs485_set_tx(void)
{
    PORTD |= (1u << RS485_DE_PIN);
}

static void gpio_init(void)
{
    fault_out_release();

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

static bool field_power_is_on(void)
{
    return (g_regs[REG_STATUS] & STATUS_POWER_ON) != 0u;
}

static bool require_field_power(void)
{
    if (!field_power_is_on()) {
        set_result(RESULT_POWER_OFF);
        return false;
    }

    return true;
}

static void clear_reply_regs(void)
{
    g_regs[REG_REPLY_NODE] = 0u;
    g_regs[REG_REPLY_CMD] = 0u;
    g_regs[REG_REPLY_ARG0] = 0u;
    g_regs[REG_REPLY_ARG1] = 0u;
}

static void clear_node_map(void)
{
    for (uint8_t i = 0u; i < NODE_MAP_MAX; i++) {
        g_regs[REG_NODE_MAP_BASE + i] = 0u;
    }

    g_regs[REG_NODE_COUNT] = 0u;
}

static bool valid_node_addr(uint8_t node)
{
    return (node >= 1u) && (node <= MAX_NODE_ADDR);
}

static bool valid_channel(uint8_t channel)
{
    return (channel >= 1u) && (channel <= MAX_CHANNEL_API);
}

/* ============================================================================
 * Relay / field power
 * ========================================================================== */

static void relay_drive_off(void)
{
    PORTD &= (uint8_t)~(1u << RELAY_SET_PIN);
    PORTD &= (uint8_t)~(1u << RELAY_RESET_PIN);
}

static void relay_pulse_set(void)
{
    PORTD |= (1u << RELAY_SET_PIN);

    for (uint8_t i = 0u; i < RELAY_PULSE_MS; i++) {
        _delay_ms(1);
    }

    PORTD &= (uint8_t)~(1u << RELAY_SET_PIN);
}

static void relay_pulse_reset(void)
{
    PORTD |= (1u << RELAY_RESET_PIN);

    for (uint8_t i = 0u; i < RELAY_PULSE_MS; i++) {
        _delay_ms(1);
    }

    PORTD &= (uint8_t)~(1u << RELAY_RESET_PIN);
}

static void bus_power_on(void)
{
    relay_pulse_set();

    g_regs[REG_POWER_STATE] = 1u;
    g_regs[REG_STATUS] |= STATUS_POWER_ON;

    for (uint16_t i = 0u; i < BUS_POWER_UP_DELAY_MS; i++) {
        _delay_ms(1);
    }
}

static void bus_power_off(void)
{
    relay_pulse_reset();

    g_regs[REG_POWER_STATE] = 0u;
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_POWER_ON;
}

/* ============================================================================
 * UART / RS-485 low level
 * ========================================================================== */

static void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFFu);

    UCSR0A = 0u;
    UCSR0B = (1u << RXEN0) | (1u << TXEN0);
    UCSR0C = (1u << UCSZ01) | (1u << UCSZ00);

    rs485_set_rx();
}

static void uart_putc(char c)
{
    while ((UCSR0A & (1u << UDRE0)) == 0u) {
    }

    UDR0 = (uint8_t)c;
}

static bool uart_getc_nonblocking(char *out)
{
    if (UCSR0A & (1u << RXC0)) {
        *out = (char)UDR0;
        return true;
    }

    return false;
}

static void uart_flush_rx(void)
{
    char c;

    while (uart_getc_nonblocking(&c)) {
    }
}

static void rs485_wait_rx_to_tx_turnaround(void)
{
    for (uint8_t i = 0u; i < RS485_RX_TO_TX_TURNAROUND_MS; i++) {
        _delay_ms(1);
    }
}

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

static char nibble_to_hex(uint8_t v)
{
    v &= 0x0Fu;

    if (v < 10u) {
        return (char)('0' + v);
    }

    return (char)('A' + (v - 10u));
}

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

static uint8_t ascii_sum_checksum(const char *body, uint8_t len)
{
    uint8_t sum = 0u;

    for (uint8_t i = 0u; i < len; i++) {
        sum = (uint8_t)(sum + (uint8_t)body[i]);
    }

    return sum;
}

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
 * @brief Build a checksummed valve-node request frame.
 *
 * Request body is:
 *   DD C [A]
 *
 * Checksum is the 8-bit ASCII sum of the body bytes only. The leading ':' and
 * trailing CR are not included.
 *
 * No-argument frame:
 *   :DDCKK\r
 *
 * Argument frame:
 *   :DDCAKK\r
 */
static void make_request(uint8_t node, char cmd, char arg, char *frame, uint8_t frame_len)
{
    char body[5];
    uint8_t body_len = 0u;
    uint8_t cs = 0u;
    uint8_t pos = 0u;

    if (frame_len == 0u) {
        return;
    }

    frame[0] = '\0';

    body[body_len++] = nibble_to_hex((uint8_t)(node >> 4));
    body[body_len++] = nibble_to_hex(node);
    body[body_len++] = cmd;

    if (arg != 0) {
        body[body_len++] = arg;
    }

    /*
     * Need:
     *   ':' + body + 2 checksum chars + '\r' + '\0'
     *
     * body_len is 3 or 4, so required length is 8 or 9.
     */
    if (frame_len < (uint8_t)(body_len + 5u)) {
        return;
    }

    cs = ascii_sum_checksum(body, body_len);

    frame[pos++] = ':';

    for (uint8_t i = 0u; i < body_len; i++) {
        frame[pos++] = body[i];
    }

    frame[pos++] = nibble_to_hex((uint8_t)(cs >> 4));
    frame[pos++] = nibble_to_hex(cs);
    frame[pos++] = '\r';
    frame[pos] = '\0';
}

/* ============================================================================
 * RS-485 receive / parse
 * ========================================================================== */

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
            if (g_regs[REG_RESULT] != RESULT_RS485_BAD_CHECKSUM) {
                set_result(RESULT_RS485_BAD_REPLY);
            }

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

static bool send_request_wait_reply(uint8_t node,
                                    char cmd,
                                    char arg,
                                    uint16_t timeout_ms,
                                    uint8_t *reply_node,
                                    char *reply_cmd,
                                    char *reply_arg0,
                                    char *reply_arg1)
{
    char frame[12];

    make_request(node, cmd, arg, frame, sizeof(frame));

    if (frame[0] == '\0') {
        set_result(RESULT_RS485_BAD_REPLY);
        return false;
    }

    for (uint8_t attempt = 0u; attempt <= RS485_RETRY_COUNT; attempt++) {
        uart_flush_rx();
        clear_reply_regs();

        uart_write_frame(frame);

        if (wait_for_reply(timeout_ms,
                           node,
                           0,
                           reply_node,
                           reply_cmd,
                           reply_arg0,
                           reply_arg1)) {
            return true;
        }

        if (g_regs[REG_RESULT] != RESULT_RS485_TIMEOUT) {
            return false;
        }

        if (attempt < RS485_RETRY_COUNT) {
            for (uint8_t i = 0u; i < RS485_RETRY_DELAY_MS; i++) {
                _delay_ms(1);
            }
        }
    }

    return false;
}

/* ============================================================================
 * Command backend
 * ========================================================================== */

static void cmd_power_on(void)
{
    bus_power_on();
    set_result(RESULT_OK);
}

static void cmd_power_off(void)
{
    for (uint8_t i = 0u; i < RS485_CANCEL_GRACE_MS; i++) {
        _delay_ms(1);
    }

    bus_power_off();
    set_result(RESULT_OK);
}

static void cmd_who(void)
{
    char frame[12];
    uint8_t count = 0u;

    if (!require_field_power()) {
        return;
    }

    clear_node_map();

    make_request(0xFFu, 'W', 0, frame, sizeof(frame));

    if (frame[0] == '\0') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

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

static void cmd_cancel(void)
{
    char frame[12];

    if (!require_field_power()) {
        return;
    }

    make_request(0xFFu, 'X', 0, frame, sizeof(frame));

    if (frame[0] == '\0') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    uart_write_frame(frame);

    for (uint8_t i = 0u; i < RS485_CANCEL_GRACE_MS; i++) {
        _delay_ms(1);
    }

    g_regs[REG_REPLY_CMD] = (uint8_t)'X';
    set_result(RESULT_OK);
}

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

static void cmd_close_all(void)
{
    char frame[12];

    if (!require_field_power()) {
        return;
    }

    make_request(0xFFu, 'C', 0, frame, sizeof(frame));

    if (frame[0] == '\0') {
        set_result(RESULT_RS485_BAD_REPLY);
        return;
    }

    uart_write_frame(frame);

    for (uint8_t i = 0u; i < RS485_CLOSE_ALL_GRACE_MS; i++) {
        _delay_ms(1);
    }

    g_regs[REG_REPLY_CMD] = (uint8_t)'C';
    set_result(RESULT_OK);
}

static void cmd_clear_error(void)
{
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_ERROR;
    g_regs[REG_RESULT] = RESULT_OK;
    update_fault_outputs();
}

static void cmd_set_error(void)
{
    g_regs[REG_STATUS] |= STATUS_ERROR;
    g_regs[REG_RESULT] = RESULT_OK;
    update_fault_outputs();
}

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

static void i2c_init(void)
{
    TWAR = (uint8_t)(I2C_ADDR << 1);
    TWAMR = 0x00u;
    TWCR = (1u << TWEA) | (1u << TWEN) | (1u << TWIE);
}

ISR(TWI_vect)
{
    switch (TW_STATUS) {

    case TW_SR_SLA_ACK:
    case TW_SR_GCALL_ACK:
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
            g_reg_ptr = data;
            g_have_reg_ptr = true;
        } else {
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
        g_have_reg_ptr = false;

        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;

    case TW_ST_SLA_ACK:
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
        TWCR = (1u << TWINT) |
               (1u << TWEA)  |
               (1u << TWEN)  |
               (1u << TWIE);
        break;

    case TW_BUS_ERROR:
    default:
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

static void regs_init(void)
{
    for (uint16_t i = 0u; i < 256u; i++) {
        g_regs[i] = 0u;
    }

    g_regs[REG_STATUS] = 0u;
    g_regs[REG_RESULT] = RESULT_OK;
    g_regs[REG_POWER_STATE] = 0u;

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
