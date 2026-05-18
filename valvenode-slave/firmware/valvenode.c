/**
 * @file valvenode.c
 * @brief Dual-valve RS-485 slave node firmware.
 *
 * This firmware implements a compact command/response protocol for a two-channel
 * valve node. The node receives framed ASCII commands over a half-duplex
 * RS-485 link, executes valve operations, reports status, supports address
 * assignment, firmware version query, identify mode, and config mode, and
 * stores its assigned node address in EEPROM.
 *
 * Protocol summary
 * ----------------
 *
 * Request format:
 *   :DDC[A][KK]<EOL>
 *
 * Reply format:
 *   :DDC[A][B]KK<EOL>
 *
 * Field meanings:
 *   - ':'  start of frame
 *   - DD   2 hex digit node address
 *   - C    1 character command
 *   - A    optional 1 character argument
 *   - B    optional 1 character second argument in replies
 *   - KK   optional 2 hex digit checksum on requests
 *          required 2 hex digit checksum on replies
 *   - EOL  CR, LF, or CRLF
 *
 * Addressing:
 *   - 01..FE valid node addresses
 *   - FF     broadcast
 *   - 00     invalid / unassigned
 *
 * Supported requests:
 *   - :04P   ping node 04
 *   - :04O1  open valve 1 on node 04
 *   - :04C1  close valve 1 on node 04
 *   - :04O2  open valve 2 on node 04
 *   - :04C2  close valve 2 on node 04
 *   - :04S1  status valve 1 on node 04
 *   - :04S2  status valve 2 on node 04
 *   - :04V   get firmware version from node 04
 *   - :04I   enable identify mode on node 04
 *   - :04M   put node 04 into config mode
 *   - :00M   put an unassigned node into config mode
 *   - :DDN   assign new address DD while node is in config mode
 *   - :FFW   broadcast who
 *   - :FFC   broadcast close all local valves
 *   - :FFX   broadcast cancel config / identify mode
 *
 * Example replies:
 *   - :04AKK       ACK from node 04
 *   - :04EKK       error from node 04
 *   - :04WKK       node 04 present
 *   - :04VvvvvKK   firmware version from node 04, vvvv is packed BCD
 *   - :04R1OKK     node 04 valve 1 open
 *   - :04R1CKK     node 04 valve 1 closed
 *   - :04R2OKK     node 04 valve 2 open
 *   - :04R2CKK     node 04 valve 2 closed
 *
 * Example version reply:
 *   - :04V010580   node 04 firmware version 1.05
 *
 * Checksum:
 *   8-bit sum of ASCII body bytes modulo 256.
 *   ':' and EOL are not included in the checksum.
 *
 * Behavioral notes:
 *   - Active-low status LED on PB0
 *   - RS-485 DE and /RE are tied together on PD2
 *   - Replies always include checksum
 *   - Requests may omit checksum
 *   - Node address is stored in EEPROM and copied into RAM at boot
 *   - EEPROM identity uses one packed magic/address/inverse-address record
 *   - Fresh blank EEPROM is treated as unassigned, not corrupt
 *   - Corrupt EEPROM identity causes fast address-fault LED blink
 *   - Protocol is strict command/response, one request then one reply
 *   - Valve pulse width is 20 ms
 *   - Production valve output uses a 2.2 ohm series resistor
 *   - Watchdog is enabled to recover from firmware hangs during valve pulses
 */

#define F_CPU 8000000UL

#include <avr/eeprom.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/delay.h>

/* ============================================================================
 * BUILD CONFIGURATION
 * ========================================================================== */

/**
 * @brief Firmware version as packed 16-bit BCD.
 *
 * Example:
 *   0x0100 = 1.00
 *   0x0104 = 1.04
 *   0x0105 = 1.05
 *   0x0215 = 2.15
 */
#ifndef FW_VERSION
#define FW_VERSION 0x0106
#endif

/** @brief Default runtime address for an unassigned node. */
#define DEFAULT_NODE_ADDR              0x00

/** @brief EEPROM identity record magic byte. */
#define EEPROM_ADDR_MAGIC              0xA5u

/** @brief UART baud rate. */
#define BAUD                           9600UL

/** @brief UART divisor for F_CPU and BAUD. */
#define UBRR_VALUE                     ((F_CPU / (16UL * BAUD)) - 1UL)

/**
 * @brief Valve drive pulse width in milliseconds.
 *
 * Production valve output uses a 2.2 ohm series resistor with the latching
 * solenoid. The shorter pulse limits solenoid heating while still providing
 * enough energy to latch the valve.
 */
#define VALVE_PULSE_MS                 20

/** @brief RX-to-TX turnaround delay before transmitting reply. */
#define RS485_RX_TO_TX_TURNAROUND_MS   2

/** @brief Slot size for broadcast WHO reply staggering. */
#define WHO_SLOT_MS                    20

/** @brief Broadcast node address. */
#define NODE_BROADCAST_ADDR            0xFF

/** @brief Invalid / unassigned node address. */
#define NODE_INVALID_ADDR              0x00

/** @brief Maximum body length excluding ':' and EOL. */
#define RX_BODY_MAX_CHARS              8

/** @brief Config mode timeout in milliseconds. */
#define CONFIG_MODE_TIMEOUT_MS         30000u

/** @brief Config mode LED half-period, 1 s on and 1 s off. */
#define CONFIG_BLINK_HALF_MS           1000u

/** @brief Address-fault LED half-period in milliseconds. */
#define ADDR_FAULT_BLINK_HALF_MS       100u

/** @brief Main loop poll tick in milliseconds. */
#define POLL_TICK_MS                   1u

/** @brief Identify mode LED half-period in milliseconds. */
#define IDENTIFY_BLINK_HALF_MS         500u

/** @brief Boot-time config button hold threshold in milliseconds. */
#define CONFIG_BOOT_HOLD_MS            3000u

/** @brief Boot-time config button sample period in milliseconds. */
#define CONFIG_BOOT_SAMPLE_MS          10u

/* ============================================================================
 * MCU PIN MAP
 * ========================================================================== */

/** @name RS-485 direction control
 *  @{
 */
#define RS485_DIR_PORT                 PORTD
#define RS485_DIR_DDR                  DDRD
#define RS485_DIR_BIT                  PD2
/** @} */

/** @name Valve 1 driver pins
 *  @{
 */
#define VNH1_PORT                      PORTB
#define VNH1_DDR                       DDRB
#define VNH1_INA_BIT                   PB3
#define VNH1_INB_BIT                   PB2
#define VNH1_PWM_PORT                  PORTD
#define VNH1_PWM_DDR                   DDRD
#define VNH1_PWM_BIT                   PD4
/** @} */

/** @name Valve 2 driver pins
 *  @{
 */
#define VNH2_PORT                      PORTB
#define VNH2_DDR                       DDRB
#define VNH2_INA_BIT                   PB1
#define VNH2_INB_PORT                  PORTD
#define VNH2_INB_DDR                   DDRD
#define VNH2_INB_BIT                   PD5
#define VNH2_PWM_PORT                  PORTD
#define VNH2_PWM_DDR                   DDRD
#define VNH2_PWM_BIT                   PD6
/** @} */

/** @name Status LED
 *  @{
 */
#define LED_PORT                       PORTB
#define LED_DDR                        DDRB
#define LED_BIT                        PB0
/** @} */

/** @name Config button input
 *  @{
 */
#define CONFIG_BTN_PORT                PORTD
#define CONFIG_BTN_DDR                 DDRD
#define CONFIG_BTN_PINR                PIND
#define CONFIG_BTN_BIT                 PD3
/** @} */

/* ============================================================================
 * TYPES AND PERSISTENT STATE
 * ========================================================================== */

/**
 * @brief Logical valve state tracked by firmware.
 */
typedef enum
{
    VALVE_UNKNOWN = 0, /**< State not known. */
    VALVE_OPEN,        /**< Valve was last commanded open. */
    VALVE_CLOSED       /**< Valve was last commanded closed. */
} valve_state_t;

/**
 * @brief EEPROM node-address load result.
 */
typedef enum
{
    ADDR_LOAD_VALID = 0,       /**< EEPROM contained a valid assigned address. */
    ADDR_LOAD_UNASSIGNED,      /**< EEPROM is blank/new/unassigned. */
    ADDR_LOAD_CORRUPT          /**< EEPROM identity record is malformed. */
} addr_load_status_t;

/**
 * @brief EEPROM identity record.
 *
 * Keep this as one record so byte ordering and layout are explicit:
 *   byte 0 = magic
 *   byte 1 = address
 *   byte 2 = bitwise inverse of address
 */
typedef struct
{
    uint8_t magic;
    uint8_t addr;
    uint8_t addr_inv;
} node_identity_t;

/** @brief Runtime state for valve channel 1. */
static volatile valve_state_t g_valve1_state = VALVE_CLOSED;

/** @brief Runtime state for valve channel 2. */
static volatile valve_state_t g_valve2_state = VALVE_CLOSED;

/** @brief True while node is in config mode. */
static bool g_config_mode = false;

/** @brief Elapsed config mode time in milliseconds. */
static uint16_t g_config_mode_ms = 0u;

/** @brief Config mode blink accumulator in milliseconds. */
static uint16_t g_config_blink_ms = 0u;

/** @brief Current config mode LED state. */
static bool g_config_led_on = false;

/** @brief True while node is in identify mode. */
static bool g_identify_mode = false;

/** @brief Identify mode blink accumulator in milliseconds. */
static uint16_t g_identify_blink_ms = 0u;

/** @brief Current identify mode LED state. */
static bool g_identify_led_on = false;

/** @brief Address-fault blink accumulator in milliseconds. */
static uint16_t g_addr_fault_blink_ms = 0u;

/** @brief Current address-fault LED state. */
static bool g_addr_fault_led_on = false;

/** @brief Active node address loaded from EEPROM at boot. */
static uint8_t g_node_addr = DEFAULT_NODE_ADDR;

/** @brief Result of loading node address from EEPROM. */
static addr_load_status_t g_addr_load_status = ADDR_LOAD_UNASSIGNED;

/** @brief True when EEPROM address identity is corrupt. */
static bool g_addr_fault = false;

/** @brief Node identity persisted in EEPROM. */
static node_identity_t EEMEM ee_node_identity;

/* ============================================================================
 * EEPROM HELPERS
 * ========================================================================== */

/**
 * @brief Validate a new address for assignment.
 *
 * @param addr Candidate node address.
 * @retval true  Address is valid assignable address.
 * @retval false Address is invalid or broadcast.
 */
static bool valid_assign_addr(uint8_t addr)
{
    return (addr != NODE_INVALID_ADDR) && (addr != NODE_BROADCAST_ADDR);
}

/**
 * @brief Load node address from EEPROM.
 *
 * Fresh/unassigned EEPROM is not treated as a fault. A malformed identity
 * record is treated as a fault and should be indicated.
 *
 * @param[out] status Address load status.
 * @return Runtime node address to use after boot.
 */
static uint8_t load_node_addr(addr_load_status_t *status)
{
    node_identity_t id;

    eeprom_read_block(&id, &ee_node_identity, sizeof(id));

    if ((id.magic == 0xFFu) &&
        (id.addr == 0xFFu) &&
        (id.addr_inv == 0xFFu)) {
        *status = ADDR_LOAD_UNASSIGNED;
        return DEFAULT_NODE_ADDR;
    }

    if ((id.magic == EEPROM_ADDR_MAGIC) &&
        (id.addr == NODE_INVALID_ADDR) &&
        (id.addr_inv == (uint8_t)~NODE_INVALID_ADDR)) {
        *status = ADDR_LOAD_UNASSIGNED;
        return DEFAULT_NODE_ADDR;
    }

    if (id.magic != EEPROM_ADDR_MAGIC) {
        *status = ADDR_LOAD_CORRUPT;
        return DEFAULT_NODE_ADDR;
    }

    if (!valid_assign_addr(id.addr)) {
        *status = ADDR_LOAD_CORRUPT;
        return DEFAULT_NODE_ADDR;
    }

    if (id.addr_inv != (uint8_t)~id.addr) {
        *status = ADDR_LOAD_CORRUPT;
        return DEFAULT_NODE_ADDR;
    }

    *status = ADDR_LOAD_VALID;
    return id.addr;
}

/**
 * @brief Persist node address to EEPROM and verify it.
 *
 * @param addr New node address to store.
 * @return true if EEPROM verified after write.
 */
static bool save_node_addr(uint8_t addr)
{
    node_identity_t id;
    node_identity_t verify;

    if (!valid_assign_addr(addr)) {
        return false;
    }

    id.magic = EEPROM_ADDR_MAGIC;
    id.addr = addr;
    id.addr_inv = (uint8_t)~addr;

    eeprom_update_block(&id, &ee_node_identity, sizeof(id));
    eeprom_read_block(&verify, &ee_node_identity, sizeof(verify));

    if (verify.magic != EEPROM_ADDR_MAGIC) {
        return false;
    }

    if (verify.addr != addr) {
        return false;
    }

    if (verify.addr_inv != (uint8_t)~addr) {
        return false;
    }

    return true;
}

/* ============================================================================
 * WATCHDOG / SAFE DELAY HELPERS
 * ========================================================================== */

/**
 * @brief Enable watchdog reset protection.
 *
 * If firmware ever wedges while a valve driver output is active, the watchdog
 * resets the MCU. Startup GPIO initialization must leave all VNH inputs and
 * enable/PWM pins inactive.
 */
static void watchdog_init(void)
{
    MCUSR &= (uint8_t)~(1u << WDRF);
    wdt_enable(WDTO_500MS);
}

/**
 * @brief Delay in milliseconds while servicing the watchdog.
 *
 * @param ms Delay time in milliseconds.
 */
static void delay_ms_safe(uint16_t ms)
{
    while (ms-- > 0u) {
        wdt_reset();
        _delay_ms(1);
    }
}

/* ============================================================================
 * UART / RS-485 LOW LEVEL
 * ========================================================================== */

/**
 * @brief Put the RS-485 transceiver into receive mode.
 */
static inline void rs485_set_rx(void)
{
    RS485_DIR_PORT &= (uint8_t)~(1u << RS485_DIR_BIT);
}

/**
 * @brief Put the RS-485 transceiver into transmit mode.
 */
static inline void rs485_set_tx(void)
{
    RS485_DIR_PORT |= (uint8_t)(1u << RS485_DIR_BIT);
}

/**
 * @brief Wait the programmed RX-to-TX turnaround delay.
 */
static void rs485_wait_rx_to_tx_turnaround(void)
{
    delay_ms_safe(RS485_RX_TO_TX_TURNAROUND_MS);
}

/**
 * @brief Initialize UART for 9600 8N1 and leave RS-485 in RX mode.
 */
static void uart_init(void)
{
    UBRRH = (uint8_t)(UBRR_VALUE >> 8);
    UBRRL = (uint8_t)(UBRR_VALUE & 0xFF);

    UCSRA = 0;
    UCSRB = (1u << RXEN) | (1u << TXEN);
    UCSRC = (1u << UCSZ1) | (1u << UCSZ0);

    rs485_set_rx();
}

/**
 * @brief Transmit one character over UART.
 *
 * @param c Character to send.
 */
static void uart_putc(char c)
{
    while ((UCSRA & (1u << UDRE)) == 0u) {
        wdt_reset();
    }

    UDR = c;
}

/**
 * @brief Read one character from UART if available.
 *
 * @param[out] out Pointer receiving the character.
 * @return true if a character was available, false otherwise.
 */
static bool uart_getc_nonblocking(char *out)
{
    uint8_t status;

    if ((UCSRA & (1u << RXC)) == 0u) {
        return false;
    }

    status = UCSRA;
    *out = UDR;

    if (status & ((1u << FE) | (1u << DOR) | (1u << UPE))) {
        return false;
    }

    return true;
}

/**
 * @brief Write a NUL-terminated string on RS-485.
 *
 * This function handles turnaround timing, drives DE, transmits the full
 * string, waits for the final byte to clear the UART, and returns the bus
 * to receive mode.
 *
 * @param s NUL-terminated string to send.
 */
static void uart_write(const char *s)
{
    rs485_wait_rx_to_tx_turnaround();
    rs485_set_tx();

    UCSRA |= (1u << TXC);

    while (*s != '\0') {
        uart_putc(*s++);
    }

    while ((UCSRA & (1u << TXC)) == 0u) {
        wdt_reset();
    }

    _delay_us(100);
    rs485_set_rx();
}

/* ============================================================================
 * GPIO / INDICATORS / BUTTON
 * ========================================================================== */

/**
 * @brief Turn the active-low status LED on.
 */
static inline void led_on(void)
{
    LED_PORT &= (uint8_t)~(1u << LED_BIT);
}

/**
 * @brief Turn the active-low status LED off.
 */
static inline void led_off(void)
{
    LED_PORT |= (uint8_t)(1u << LED_BIT);
}

/**
 * @brief Read the config button.
 *
 * @retval true  Button is pressed.
 * @retval false Button is released.
 */
static inline bool config_button_pressed(void)
{
    return (CONFIG_BTN_PINR & (1u << CONFIG_BTN_BIT)) == 0u;
}

/**
 * @brief Initialize config button input with internal pull-up.
 */
static void config_button_init(void)
{
    CONFIG_BTN_DDR  &= (uint8_t)~(1u << CONFIG_BTN_BIT);
    CONFIG_BTN_PORT |= (uint8_t)(1u << CONFIG_BTN_BIT);
}

/**
 * @brief Flash the LED for a short visible indication.
 */
static void led_flash_short(void)
{
    led_on();
    delay_ms_safe(60u);
    led_off();
}

/**
 * @brief Enter identify mode.
 */
static void enter_identify_mode(void)
{
    g_identify_mode = true;
    g_identify_blink_ms = 0u;
    g_identify_led_on = false;
    led_off();
}

/**
 * @brief Exit identify mode.
 */
static void exit_identify_mode(void)
{
    g_identify_mode = false;
    g_identify_blink_ms = 0u;
    g_identify_led_on = false;
    led_off();
}

/**
 * @brief Service address-fault LED blinking.
 *
 * This is intentionally much faster than normal config blink.
 */
static void address_fault_blink_poll(void)
{
    if (!g_addr_fault) {
        return;
    }

    g_addr_fault_blink_ms += POLL_TICK_MS;
    if (g_addr_fault_blink_ms >= ADDR_FAULT_BLINK_HALF_MS) {
        g_addr_fault_blink_ms = 0u;
        g_addr_fault_led_on = !g_addr_fault_led_on;

        if (g_addr_fault_led_on) {
            led_on();
        } else {
            led_off();
        }
    }
}

/**
 * @brief Service identify mode LED blinking.
 */
static void identify_mode_poll(void)
{
    if (!g_identify_mode) {
        return;
    }

    g_identify_blink_ms += POLL_TICK_MS;
    if (g_identify_blink_ms >= IDENTIFY_BLINK_HALF_MS) {
        g_identify_blink_ms = 0u;
        g_identify_led_on = !g_identify_led_on;

        if (g_identify_led_on) {
            led_on();
        } else {
            led_off();
        }
    }
}

/**
 * @brief Initialize GPIO directions and safe idle states.
 */
static void gpio_init(void)
{
    RS485_DIR_PORT &= (uint8_t)~(1u << RS485_DIR_BIT);

    VNH1_PORT &= (uint8_t)~(1u << VNH1_INA_BIT);
    VNH1_PORT &= (uint8_t)~(1u << VNH1_INB_BIT);
    VNH1_PWM_PORT &= (uint8_t)~(1u << VNH1_PWM_BIT);

    VNH2_PORT &= (uint8_t)~(1u << VNH2_INA_BIT);
    VNH2_INB_PORT &= (uint8_t)~(1u << VNH2_INB_BIT);
    VNH2_PWM_PORT &= (uint8_t)~(1u << VNH2_PWM_BIT);

    LED_PORT |= (uint8_t)(1u << LED_BIT);

    RS485_DIR_DDR |= (1u << RS485_DIR_BIT);

    VNH1_DDR |= (1u << VNH1_INA_BIT) | (1u << VNH1_INB_BIT);
    VNH1_PWM_DDR |= (1u << VNH1_PWM_BIT);

    VNH2_DDR |= (1u << VNH2_INA_BIT);
    VNH2_INB_DDR |= (1u << VNH2_INB_BIT);
    VNH2_PWM_DDR |= (1u << VNH2_PWM_BIT);

    LED_DDR |= (1u << LED_BIT);

    config_button_init();

    rs485_set_rx();
    led_off();
}

/* ============================================================================
 * VALVE DRIVER CONTROL
 * ========================================================================== */

static void valve_driver_off(uint8_t channel)
{
    if (channel == 1u) {
        VNH1_PWM_PORT &= (uint8_t)~(1u << VNH1_PWM_BIT);
        VNH1_PORT &= (uint8_t)~(1u << VNH1_INA_BIT);
        VNH1_PORT &= (uint8_t)~(1u << VNH1_INB_BIT);
    } else {
        VNH2_PWM_PORT &= (uint8_t)~(1u << VNH2_PWM_BIT);
        VNH2_PORT &= (uint8_t)~(1u << VNH2_INA_BIT);
        VNH2_INB_PORT &= (uint8_t)~(1u << VNH2_INB_BIT);
    }
}

static void valve_pulse_open(uint8_t channel)
{
    valve_driver_off(channel);

    if (channel == 1u) {
        VNH1_PORT &= (uint8_t)~(1u << VNH1_INA_BIT);
        VNH1_PORT |= (1u << VNH1_INB_BIT);
        _delay_us(20);
        VNH1_PWM_PORT |= (1u << VNH1_PWM_BIT);
    } else {
        VNH2_PORT &= (uint8_t)~(1u << VNH2_INA_BIT);
        VNH2_INB_PORT |= (1u << VNH2_INB_BIT);
        _delay_us(20);
        VNH2_PWM_PORT |= (1u << VNH2_PWM_BIT);
    }

    led_on();
    delay_ms_safe(VALVE_PULSE_MS);
    led_off();

    valve_driver_off(channel);

    if (channel == 1u) {
        g_valve1_state = VALVE_OPEN;
    } else {
        g_valve2_state = VALVE_OPEN;
    }
}

static void valve_pulse_close(uint8_t channel)
{
    valve_driver_off(channel);

    if (channel == 1u) {
        VNH1_PORT |= (1u << VNH1_INA_BIT);
        VNH1_PORT &= (uint8_t)~(1u << VNH1_INB_BIT);
        _delay_us(20);
        VNH1_PWM_PORT |= (1u << VNH1_PWM_BIT);
    } else {
        VNH2_PORT |= (1u << VNH2_INA_BIT);
        VNH2_INB_PORT &= (uint8_t)~(1u << VNH2_INB_BIT);
        _delay_us(20);
        VNH2_PWM_PORT |= (1u << VNH2_PWM_BIT);
    }

    led_on();
    delay_ms_safe(VALVE_PULSE_MS);
    led_off();

    valve_driver_off(channel);

    if (channel == 1u) {
        g_valve1_state = VALVE_CLOSED;
    } else {
        g_valve2_state = VALVE_CLOSED;
    }
}

static void valve_close_all(void)
{
    valve_pulse_close(1u);
    delay_ms_safe(20u);
    valve_pulse_close(2u);
}

/* ============================================================================
 * CONFIG MODE CONTROL
 * ========================================================================== */

static void enter_config_mode(void)
{
    g_config_mode = true;
    g_config_mode_ms = 0u;
    g_config_blink_ms = 0u;
    g_config_led_on = false;
    led_off();
}

static void exit_config_mode(void)
{
    g_config_mode = false;
    g_config_mode_ms = 0u;
    g_config_blink_ms = 0u;
    g_config_led_on = false;
    led_off();
}

static void config_mode_poll(void)
{
    if (!g_config_mode) {
        return;
    }

    if (g_config_mode_ms < CONFIG_MODE_TIMEOUT_MS) {
        g_config_mode_ms += POLL_TICK_MS;
    } else {
        exit_config_mode();
        return;
    }

    g_config_blink_ms += POLL_TICK_MS;
    if (g_config_blink_ms >= CONFIG_BLINK_HALF_MS) {
        g_config_blink_ms = 0u;
        g_config_led_on = !g_config_led_on;

        if (g_config_led_on) {
            led_on();
        } else {
            led_off();
        }
    }
}

static void check_config_button_at_boot(void)
{
    uint16_t held_ms = 0u;

    if (!config_button_pressed()) {
        return;
    }

    while (held_ms < CONFIG_BOOT_HOLD_MS) {
        wdt_reset();

        if (!config_button_pressed()) {
            return;
        }

        delay_ms_safe(CONFIG_BOOT_SAMPLE_MS);
        held_ms += CONFIG_BOOT_SAMPLE_MS;
    }

    enter_config_mode();
}

/* ============================================================================
 * PROTOCOL UTILITIES
 * ========================================================================== */

static char nibble_to_hex(uint8_t v)
{
    v &= 0x0Fu;
    if (v < 10u) {
        return (char)('0' + v);
    }
    return (char)('A' + (v - 10u));
}

static bool hex_char_to_nibble(char c, uint8_t *out)
{
    if ((c >= '0') && (c <= '9')) {
        *out = (uint8_t)(c - '0');
        return true;
    }
    if ((c >= 'A') && (c <= 'F')) {
        *out = (uint8_t)(c - 'A' + 10);
        return true;
    }
    if ((c >= 'a') && (c <= 'f')) {
        *out = (uint8_t)(c - 'a' + 10);
        return true;
    }
    return false;
}

static bool hex_pair_to_u8(char hi_c, char lo_c, uint8_t *out)
{
    uint8_t hi;
    uint8_t lo;

    if (!hex_char_to_nibble(hi_c, &hi)) {
        return false;
    }
    if (!hex_char_to_nibble(lo_c, &lo)) {
        return false;
    }

    *out = (uint8_t)((hi << 4) | lo);
    return true;
}

static uint8_t ascii_sum_checksum(const char *body, uint8_t body_len_without_checksum)
{
    uint16_t sum = 0u;

    for (uint8_t i = 0u; i < body_len_without_checksum; i++) {
        sum += (uint8_t)body[i];
    }

    return (uint8_t)(sum & 0xFFu);
}

static bool node_addr_matches(uint8_t addr)
{
    if (g_node_addr == NODE_INVALID_ADDR) {
        return (addr == NODE_BROADCAST_ADDR);
    }

    return (addr == g_node_addr) || (addr == NODE_BROADCAST_ADDR);
}

static bool config_entry_addr_matches(uint8_t addr)
{
    if (addr == g_node_addr) {
        return true;
    }

    if ((g_node_addr == NODE_INVALID_ADDR) && (addr == NODE_INVALID_ADDR)) {
        return true;
    }

    return false;
}

static void send_reply(uint8_t addr, char reply_cmd, char arg0, char arg1)
{
    char body[8];
    char frame[12];
    uint8_t body_len = 0u;
    uint8_t pos = 0u;
    uint8_t cs;

    body[body_len++] = nibble_to_hex((uint8_t)(addr >> 4));
    body[body_len++] = nibble_to_hex(addr);
    body[body_len++] = reply_cmd;

    if (arg0 != 0) {
        body[body_len++] = arg0;
    }
    if (arg1 != 0) {
        body[body_len++] = arg1;
    }

    cs = ascii_sum_checksum(body, body_len);

    frame[pos++] = ':';
    for (uint8_t i = 0u; i < body_len; i++) {
        frame[pos++] = body[i];
    }
    frame[pos++] = nibble_to_hex((uint8_t)(cs >> 4));
    frame[pos++] = nibble_to_hex(cs);
    frame[pos++] = '\r';
    frame[pos++] = '\n';
    frame[pos] = '\0';

    uart_write(frame);
}

static void send_version_reply(void)
{
    char body[10];
    char frame[14];
    uint8_t body_len = 0u;
    uint8_t pos = 0u;
    uint8_t cs;

    body[body_len++] = nibble_to_hex((uint8_t)(g_node_addr >> 4));
    body[body_len++] = nibble_to_hex(g_node_addr);
    body[body_len++] = 'V';
    body[body_len++] = nibble_to_hex((uint8_t)(FW_VERSION >> 12));
    body[body_len++] = nibble_to_hex((uint8_t)(FW_VERSION >> 8));
    body[body_len++] = nibble_to_hex((uint8_t)(FW_VERSION >> 4));
    body[body_len++] = nibble_to_hex((uint8_t)(FW_VERSION >> 0));

    cs = ascii_sum_checksum(body, body_len);

    frame[pos++] = ':';
    for (uint8_t i = 0u; i < body_len; i++) {
        frame[pos++] = body[i];
    }
    frame[pos++] = nibble_to_hex((uint8_t)(cs >> 4));
    frame[pos++] = nibble_to_hex(cs);
    frame[pos++] = '\r';
    frame[pos++] = '\n';
    frame[pos] = '\0';

    uart_write(frame);
}

static void delay_ms_block(uint16_t ms)
{
    delay_ms_safe(ms);
}

static void startup_blink(void)
{
    for (uint8_t i = 0; i < 3u; i++) {
        led_on();
        delay_ms_block(120u);
        led_off();
        delay_ms_block(120u);
    }
}

/* ============================================================================
 * RECEIVE PARSER AND COMMAND EXECUTION
 * ========================================================================== */

static char g_rx_body[RX_BODY_MAX_CHARS + 1];
static uint8_t g_rx_len = 0u;
static bool g_rx_active = false;

static void handle_command(uint8_t addr, char cmd, char arg)
{
    bool handled = false;
    bool stay_in_config = false;

    switch (cmd) {

    case 'N':
    case 'n':
        if (!g_config_mode) {
            return;
        }

        if (!valid_assign_addr(addr)) {
            send_reply(g_node_addr, 'E', 0, 0);
            return;
        }

        if (!save_node_addr(addr)) {
            g_addr_fault = true;
            g_addr_load_status = ADDR_LOAD_CORRUPT;
            send_reply(g_node_addr, 'E', 0, 0);
            return;
        }

        g_addr_fault = false;
        g_addr_fault_blink_ms = 0u;
        g_addr_fault_led_on = false;
        g_addr_load_status = ADDR_LOAD_VALID;
        g_node_addr = addr;

        send_reply(g_node_addr, 'A', 0, 0);
        handled = true;
        break;

    case 'M':
    case 'm':
        if (!config_entry_addr_matches(addr)) {
            return;
        }

        enter_config_mode();
        send_reply(g_node_addr, 'A', 0, 0);
        handled = true;
        stay_in_config = true;
        break;

    case 'X':
    case 'x':
        if (addr != NODE_BROADCAST_ADDR) {
            return;
        }

        if (g_config_mode) {
            exit_config_mode();
            handled = true;
        }

        if (g_identify_mode) {
            exit_identify_mode();
            handled = true;
        }
        break;

    default:
        if (!node_addr_matches(addr)) {
            return;
        }

        switch (cmd) {

        case 'I':
        case 'i':
            if (addr == NODE_BROADCAST_ADDR) {
                return;
            }

            enter_identify_mode();
            send_reply(g_node_addr, 'A', 0, 0);
            handled = true;
            break;

        case 'P':
        case 'p':
            if (addr == NODE_BROADCAST_ADDR) {
                return;
            }

            send_reply(g_node_addr, 'A', 0, 0);
            led_flash_short();
            handled = true;
            break;

        case 'O':
        case 'o':
            if (addr == NODE_BROADCAST_ADDR) {
                return;
            }

            if (arg == '1') {
                valve_pulse_open(1u);
                send_reply(g_node_addr, 'A', 0, 0);
                handled = true;
            } else if (arg == '2') {
                valve_pulse_open(2u);
                send_reply(g_node_addr, 'A', 0, 0);
                handled = true;
            } else {
                send_reply(g_node_addr, 'E', 0, 0);
            }
            break;

        case 'C':
        case 'c':
            if (addr == NODE_BROADCAST_ADDR) {
                if (g_node_addr == NODE_INVALID_ADDR) {
                    return;
                }

                if (arg != 0) {
                    return;
                }

                for (uint8_t i = 0u; i < g_node_addr; i++) {
                    delay_ms_safe(WHO_SLOT_MS);
                }

                valve_close_all();
                handled = true;
                break;
            }

            if (arg == '1') {
                valve_pulse_close(1u);
                send_reply(g_node_addr, 'A', 0, 0);
                handled = true;
            } else if (arg == '2') {
                valve_pulse_close(2u);
                send_reply(g_node_addr, 'A', 0, 0);
                handled = true;
            } else {
                send_reply(g_node_addr, 'E', 0, 0);
            }
            break;

        case 'S':
        case 's':
            if (addr == NODE_BROADCAST_ADDR) {
                return;
            }

            if (arg == '1') {
                send_reply(g_node_addr,
                           'R',
                           '1',
                           (g_valve1_state == VALVE_OPEN) ? 'O' : 'C');
                handled = true;
            } else if (arg == '2') {
                send_reply(g_node_addr,
                           'R',
                           '2',
                           (g_valve2_state == VALVE_OPEN) ? 'O' : 'C');
                handled = true;
            } else {
                send_reply(g_node_addr, 'E', 0, 0);
            }
            break;

        case 'W':
        case 'w':
            if (addr != NODE_BROADCAST_ADDR) {
                return;
            }

            if (g_node_addr == NODE_INVALID_ADDR) {
                return;
            }

            for (uint8_t i = 0u; i < g_node_addr; i++) {
                delay_ms_safe(WHO_SLOT_MS);
            }

            send_reply(g_node_addr, 'W', 0, 0);
            handled = true;
            break;

        case 'V':
        case 'v':
            if (addr == NODE_BROADCAST_ADDR) {
                return;
            }

            send_version_reply();
            handled = true;
            break;

        default:
            if (addr != NODE_BROADCAST_ADDR) {
                send_reply(g_node_addr, 'E', 0, 0);
            }
            break;
        }
        break;
    }

    if (handled && g_config_mode && !stay_in_config) {
        exit_config_mode();
    }
}

static void process_body(char *body, uint8_t len)
{
    uint8_t addr;
    char cmd;
    char arg = 0;
    uint8_t rx_cs;
    uint8_t calc_cs;
    uint8_t cs_start_index;

    if ((len != 3u) && (len != 4u) && (len != 5u) && (len != 6u)) {
        return;
    }

    if (!hex_pair_to_u8(body[0], body[1], &addr)) {
        return;
    }

    cmd = body[2];

    if ((len == 4u) || (len == 6u)) {
        arg = body[3];
    }

    if ((len == 5u) || (len == 6u)) {
        cs_start_index = len - 2u;

        if (!hex_pair_to_u8(body[cs_start_index], body[cs_start_index + 1u], &rx_cs)) {
            return;
        }

        calc_cs = ascii_sum_checksum(body, cs_start_index);
        if (calc_cs != rx_cs) {
            return;
        }
    }

    handle_command(addr, cmd, arg);
}

static void serial_poll(void)
{
    char c;

    while (uart_getc_nonblocking(&c)) {

        if (!g_rx_active) {
            if (c == ':') {
                g_rx_active = true;
                g_rx_len = 0u;
            }
            continue;
        }

        if ((c == '\r') || (c == '\n')) {
            if (g_rx_len > 0u) {
                g_rx_body[g_rx_len] = '\0';
                process_body(g_rx_body, g_rx_len);
            }
            g_rx_active = false;
            g_rx_len = 0u;
            continue;
        }

        if (c == ':') {
            g_rx_len = 0u;
            continue;
        }

        if (g_rx_len >= RX_BODY_MAX_CHARS) {
            g_rx_active = false;
            g_rx_len = 0u;
            continue;
        }

        g_rx_body[g_rx_len++] = c;
    }
}

/* ============================================================================
 * MAIN
 * ========================================================================== */

int main(void)
{
    gpio_init();
    uart_init();

    valve_driver_off(1u);
    valve_driver_off(2u);

    watchdog_init();

    g_node_addr = load_node_addr(&g_addr_load_status);
    g_addr_fault = (g_addr_load_status == ADDR_LOAD_CORRUPT);

    delay_ms_safe(20u);

    check_config_button_at_boot();

    startup_blink();

    for (;;) {
        wdt_reset();

        serial_poll();

        if (g_addr_fault) {
            address_fault_blink_poll();
        } else if (g_config_mode) {
            config_mode_poll();
        } else {
            identify_mode_poll();
        }

        delay_ms_safe(POLL_TICK_MS);
    }
}
