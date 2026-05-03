/**
 * @file valvenode_master_sim.c
 * @brief Simulated valve-node master firmware for ATmega88PB bench testing.
 *
 * This firmware runs on an ATmega88PB test board and behaves like the future
 * valve-node master from the host side. It exposes the planned I2C register
 * interface at address 0x09, accepts master commands, and returns simulated
 * node replies without requiring the real valve-master PCB or RS-485 hardware.
 *
 * This file is intended to become the architectural basis for the real product
 * firmware. The I2C register map, command values, result values, command
 * deferral model, and reply registers should remain stable. The simulator
 * backend can later be replaced with the real relay + RS-485 transaction
 * backend.
 *
 * Target:
 *   ATmega88PB @ 8 MHz
 *
 * I2C:
 *   Address 0x09
 *
 * Simulated network:
 *   - Node 1 present
 *   - Node 2 present
 *   - Optional temporary unassigned/config node
 *   - Nodes report firmware version 0x0100
 *   - Nodes support 2 channels
 *   - Master API accepts channels 1..16
 *   - Unsupported channels return RESULT_UNSUPPORTED_CHANNEL
 *
 * SRAM note:
 *   ATmega88PB has limited SRAM. This simulator intentionally keeps only a
 *   few fake node slots instead of a 254-node RAM table.
 */

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * Build configuration
 * ========================================================================== */

#ifndef I2C_ADDR
#define I2C_ADDR                    0x09u
#endif

#define MASTER_VERSION_HI           0x01u
#define MASTER_VERSION_LO           0x00u

#define MAX_NODE_ADDR               254u
#define MAX_CHANNEL_API             16u
#define FAKE_NODE_CHANNEL_COUNT     2u

/** @brief Number of fake node records kept in SRAM. */
#define SIM_NODE_SLOTS              4u

/** @brief Number of node-map bytes exposed in the register file. */
#define NODE_MAP_MAX                32u

/* ============================================================================
 * Spare board hardware profile
 * ========================================================================== */

/*
 * Spare power-control style board assumptions:
 *   PC4 = SDA
 *   PC5 = SCL
 *   PD4 = green LED, active-low
 *   PD5 = red LED, active-low
 *   PD6 = relay SET test output
 *   PD7 = relay RESET test output
 */

#define SIM_GREEN_LED_PIN           PD4
#define SIM_RED_LED_PIN             PD5
#define SIM_RELAY_SET_PIN           PD6
#define SIM_RELAY_RESET_PIN         PD7

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

/* ============================================================================
 * Simulated node model
 * ========================================================================== */

typedef struct
{
    bool present;
    bool config_mode;
    bool identify_mode;
    uint8_t addr;
    uint8_t supported_channels;
    uint16_t version;
    uint16_t open_mask;
} fake_node_t;

/* ============================================================================
 * Global state
 * ========================================================================== */

static volatile uint8_t g_regs[256];

static volatile uint8_t g_reg_ptr = 0u;
static volatile bool g_have_reg_ptr = false;

static volatile uint8_t g_pending_cmd = CMD_NONE;
static volatile bool g_cmd_pending = false;

static fake_node_t g_nodes[SIM_NODE_SLOTS];

/* ============================================================================
 * GPIO helpers
 * ========================================================================== */

static inline void green_led_on(void)
{
    PORTD &= (uint8_t)~(1u << SIM_GREEN_LED_PIN);
}

static inline void green_led_off(void)
{
    PORTD |= (uint8_t)(1u << SIM_GREEN_LED_PIN);
}

static inline void red_led_on(void)
{
    PORTD &= (uint8_t)~(1u << SIM_RED_LED_PIN);
}

static inline void red_led_off(void)
{
    PORTD |= (uint8_t)(1u << SIM_RED_LED_PIN);
}

static void update_leds(void)
{
    if (g_regs[REG_STATUS] & STATUS_POWER_ON) {
        green_led_on();
    } else {
        green_led_off();
    }

    if (g_regs[REG_STATUS] & STATUS_ERROR) {
        red_led_on();
    } else {
        red_led_off();
    }
}

static void gpio_init(void)
{
    DDRD |= (1u << SIM_GREEN_LED_PIN) |
            (1u << SIM_RED_LED_PIN) |
            (1u << SIM_RELAY_SET_PIN) |
            (1u << SIM_RELAY_RESET_PIN);

    PORTD &= (uint8_t)~(1u << SIM_RELAY_SET_PIN);
    PORTD &= (uint8_t)~(1u << SIM_RELAY_RESET_PIN);

    green_led_off();
    red_led_off();
}

/* ============================================================================
 * Simulator helpers
 * ========================================================================== */

static bool valid_node_addr(uint8_t node)
{
    return (node >= 1u) && (node <= MAX_NODE_ADDR);
}

static bool valid_channel(uint8_t channel)
{
    return (channel >= 1u) && (channel <= MAX_CHANNEL_API);
}

static void set_result(uint8_t result)
{
    g_regs[REG_RESULT] = result;

    if (result == RESULT_OK) {
        g_regs[REG_STATUS] &= (uint8_t)~STATUS_ERROR;
    } else {
        g_regs[REG_STATUS] |= STATUS_ERROR;
    }

    update_leds();
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
}

static fake_node_t *find_node(uint8_t addr)
{
    for (uint8_t i = 0u; i < SIM_NODE_SLOTS; i++) {
        if (g_nodes[i].present && g_nodes[i].addr == addr) {
            return &g_nodes[i];
        }
    }

    return 0;
}

static bool address_in_use(uint8_t addr)
{
    return find_node(addr) != 0;
}

static fake_node_t *find_config_node(void)
{
    for (uint8_t i = 0u; i < SIM_NODE_SLOTS; i++) {
        if (g_nodes[i].present && g_nodes[i].config_mode) {
            return &g_nodes[i];
        }
    }

    return 0;
}

static fake_node_t *find_free_slot(void)
{
    for (uint8_t i = 0u; i < SIM_NODE_SLOTS; i++) {
        if (!g_nodes[i].present) {
            return &g_nodes[i];
        }
    }

    return 0;
}

static void rebuild_node_map(void)
{
    uint8_t count = 0u;

    clear_node_map();

    for (uint8_t i = 0u; i < SIM_NODE_SLOTS; i++) {
        if (g_nodes[i].present && g_nodes[i].addr != 0u) {
            if (count < NODE_MAP_MAX) {
                g_regs[REG_NODE_MAP_BASE + count] = g_nodes[i].addr;
            }

            count++;
        }
    }

    g_regs[REG_NODE_COUNT] = count;
}

static void init_node_slot(fake_node_t *node, uint8_t addr)
{
    if (node == 0) {
        return;
    }

    node->present = true;
    node->config_mode = false;
    node->identify_mode = false;
    node->addr = addr;
    node->supported_channels = FAKE_NODE_CHANNEL_COUNT;
    node->version = 0x0100u;
    node->open_mask = 0u;
}

static void clear_node_slot(fake_node_t *node)
{
    if (node == 0) {
        return;
    }

    node->present = false;
    node->config_mode = false;
    node->identify_mode = false;
    node->addr = 0u;
    node->supported_channels = 0u;
    node->version = 0u;
    node->open_mask = 0u;
}

static void sim_init(void)
{
    for (uint16_t i = 0u; i < 256u; i++) {
        g_regs[i] = 0u;
    }

    for (uint8_t i = 0u; i < SIM_NODE_SLOTS; i++) {
        clear_node_slot(&g_nodes[i]);
    }

    init_node_slot(&g_nodes[0], 1u);
    init_node_slot(&g_nodes[1], 2u);

    g_regs[REG_STATUS] = 0u;
    g_regs[REG_RESULT] = RESULT_OK;
    g_regs[REG_POWER_STATE] = 0u;
    g_regs[REG_VERSION_HI] = MASTER_VERSION_HI;
    g_regs[REG_VERSION_LO] = MASTER_VERSION_LO;

    rebuild_node_map();
    update_leds();
}

/* ============================================================================
 * Command backend
 * ========================================================================== */

static void cmd_power_on(void)
{
    PORTD |= (1u << SIM_RELAY_SET_PIN);
    PORTD &= (uint8_t)~(1u << SIM_RELAY_SET_PIN);

    g_regs[REG_POWER_STATE] = 1u;
    g_regs[REG_STATUS] |= STATUS_POWER_ON;
    set_result(RESULT_OK);
}

static void cmd_power_off(void)
{
    PORTD |= (1u << SIM_RELAY_RESET_PIN);
    PORTD &= (uint8_t)~(1u << SIM_RELAY_RESET_PIN);

    g_regs[REG_POWER_STATE] = 0u;
    g_regs[REG_STATUS] &= (uint8_t)~STATUS_POWER_ON;
    set_result(RESULT_OK);
}

static void cmd_who(void)
{
    rebuild_node_map();

    g_regs[REG_REPLY_CMD] = 'W';
    set_result(RESULT_OK);
}

static void cmd_ping(void)
{
    uint8_t addr = g_regs[REG_ARG0];
    fake_node_t *node = find_node(addr);

    if (node == 0) {
        set_result(RESULT_NODE_NOT_FOUND);
        return;
    }

    g_regs[REG_REPLY_NODE] = node->addr;
    g_regs[REG_REPLY_CMD] = 'A';
    set_result(RESULT_OK);
}

static void cmd_set_channel(void)
{
    uint8_t addr = g_regs[REG_ARG0];
    uint8_t channel = g_regs[REG_ARG1];
    uint8_t state = g_regs[REG_ARG2];
    fake_node_t *node = find_node(addr);

    if (!valid_channel(channel)) {
        set_result(RESULT_BAD_CHANNEL);
        return;
    }

    if (node == 0) {
        set_result(RESULT_NODE_NOT_FOUND);
        return;
    }

    if (channel > node->supported_channels) {
        g_regs[REG_REPLY_NODE] = addr;
        g_regs[REG_REPLY_CMD] = 'E';
        set_result(RESULT_UNSUPPORTED_CHANNEL);
        return;
    }

    if (state) {
        node->open_mask |= (uint16_t)(1u << (channel - 1u));
    } else {
        node->open_mask &= (uint16_t)~(1u << (channel - 1u));
    }

    g_regs[REG_REPLY_NODE] = addr;
    g_regs[REG_REPLY_CMD] = 'A';
    set_result(RESULT_OK);
}

static void cmd_get_channel_status(void)
{
    uint8_t addr = g_regs[REG_ARG0];
    uint8_t channel = g_regs[REG_ARG1];
    fake_node_t *node = find_node(addr);

    if (!valid_channel(channel)) {
        set_result(RESULT_BAD_CHANNEL);
        return;
    }

    if (node == 0) {
        set_result(RESULT_NODE_NOT_FOUND);
        return;
    }

    if (channel > node->supported_channels) {
        g_regs[REG_REPLY_NODE] = addr;
        g_regs[REG_REPLY_CMD] = 'E';
        set_result(RESULT_UNSUPPORTED_CHANNEL);
        return;
    }

    g_regs[REG_REPLY_NODE] = addr;
    g_regs[REG_REPLY_CMD] = 'R';
    g_regs[REG_REPLY_ARG0] = channel;

    if (node->open_mask & (uint16_t)(1u << (channel - 1u))) {
        g_regs[REG_REPLY_ARG1] = 'O';
    } else {
        g_regs[REG_REPLY_ARG1] = 'C';
    }

    set_result(RESULT_OK);
}

static void cmd_get_node_version(void)
{
    uint8_t addr = g_regs[REG_ARG0];
    fake_node_t *node = find_node(addr);

    if (node == 0) {
        set_result(RESULT_NODE_NOT_FOUND);
        return;
    }

    g_regs[REG_REPLY_NODE] = addr;
    g_regs[REG_REPLY_CMD] = 'V';
    g_regs[REG_REPLY_ARG0] = (uint8_t)(node->version >> 8);
    g_regs[REG_REPLY_ARG1] = (uint8_t)(node->version & 0xFFu);

    set_result(RESULT_OK);
}

static void cmd_identify(void)
{
    uint8_t addr = g_regs[REG_ARG0];
    fake_node_t *node = find_node(addr);

    if (node == 0) {
        set_result(RESULT_NODE_NOT_FOUND);
        return;
    }

    node->identify_mode = true;

    g_regs[REG_REPLY_NODE] = addr;
    g_regs[REG_REPLY_CMD] = 'A';
    set_result(RESULT_OK);
}

static void cmd_cancel(void)
{
    for (uint8_t i = 0u; i < SIM_NODE_SLOTS; i++) {
        g_nodes[i].identify_mode = false;
        g_nodes[i].config_mode = false;
    }

    g_regs[REG_REPLY_CMD] = 'X';
    set_result(RESULT_OK);
}

static void cmd_config(void)
{
    uint8_t addr = g_regs[REG_ARG0];
    fake_node_t *node;

    if (addr == 0u) {
        node = find_node(0u);

        if (node == 0) {
            node = find_free_slot();

            if (node == 0) {
                set_result(RESULT_ADDRESS_IN_USE);
                return;
            }

            init_node_slot(node, 0u);
        }

        node->config_mode = true;
        g_regs[REG_REPLY_NODE] = 0u;
        g_regs[REG_REPLY_CMD] = 'A';
        set_result(RESULT_OK);
        return;
    }

    node = find_node(addr);

    if (node == 0) {
        set_result(RESULT_NODE_NOT_FOUND);
        return;
    }

    node->config_mode = true;
    g_regs[REG_REPLY_NODE] = addr;
    g_regs[REG_REPLY_CMD] = 'A';
    set_result(RESULT_OK);
}

static void cmd_assign(void)
{
    uint8_t new_addr = g_regs[REG_ARG0];
    fake_node_t *active = find_config_node();

    if (!valid_node_addr(new_addr)) {
        set_result(RESULT_BAD_NODE);
        return;
    }

    if (active == 0) {
        set_result(RESULT_CONFIG_REQUIRED);
        return;
    }

    if ((active->addr != new_addr) && address_in_use(new_addr)) {
        set_result(RESULT_ADDRESS_IN_USE);
        return;
    }

    active->addr = new_addr;
    active->config_mode = false;
    active->identify_mode = false;

    rebuild_node_map();

    g_regs[REG_REPLY_NODE] = new_addr;
    g_regs[REG_REPLY_CMD] = 'A';
    set_result(RESULT_OK);
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
        g_regs[REG_STATUS] &= (uint8_t)~STATUS_ERROR;
        set_result(RESULT_OK);
        break;

    default:
        set_result(RESULT_BAD_COMMAND);
        break;
    }

    g_regs[REG_STATUS] &= (uint8_t)~STATUS_BUSY;
    update_leds();
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
        TWCR = (1u << TWINT) | (1u << TWEA) | (1u << TWEN) | (1u << TWIE);
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
                }
            }

            g_reg_ptr++;
        }

        TWCR = (1u << TWINT) | (1u << TWEA) | (1u << TWEN) | (1u << TWIE);
        break;
    }

    case TW_SR_STOP:
        g_have_reg_ptr = false;
        TWCR = (1u << TWINT) | (1u << TWEA) | (1u << TWEN) | (1u << TWIE);
        break;

    case TW_ST_SLA_ACK:
    case TW_ST_DATA_ACK:
        TWDR = g_regs[g_reg_ptr];
        g_reg_ptr++;
        TWCR = (1u << TWINT) | (1u << TWEA) | (1u << TWEN) | (1u << TWIE);
        break;

    case TW_ST_DATA_NACK:
    case TW_ST_LAST_DATA:
        TWCR = (1u << TWINT) | (1u << TWEA) | (1u << TWEN) | (1u << TWIE);
        break;

    case TW_BUS_ERROR:
    default:
        TWCR = (1u << TWSTO) |
               (1u << TWINT) |
               (1u << TWEN) |
               (1u << TWEA) |
               (1u << TWIE);
        break;
    }
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(void)
{
    gpio_init();
    sim_init();
    i2c_init();

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

        update_leds();
    }
}
