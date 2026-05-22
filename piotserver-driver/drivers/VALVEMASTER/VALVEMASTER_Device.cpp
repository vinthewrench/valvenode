#include "VALVEMASTER_Device.hpp"

#include "LogMgr.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <thread>

using namespace std;

/* ============================================================================
 * Valve Master I2C register map
 * ============================================================================
 *
 * These values must match the register map implemented by valvenode_master.c
 * running on the ATmega88PB Valve Master board.
 *
 * The pIoTServer plugin talks only to the Valve Master over I2C. The Valve
 * Master then handles field-bus power, RS-485 direction control, and the slave
 * node command protocol.
 */

static constexpr uint8_t REG_COMMAND      = 0x00;
static constexpr uint8_t REG_STATUS       = 0x01;
static constexpr uint8_t REG_ARG0         = 0x02;
static constexpr uint8_t REG_ARG1         = 0x03;
static constexpr uint8_t REG_ARG2         = 0x04;
static constexpr uint8_t REG_RESULT       = 0x05;
static constexpr uint8_t REG_POWER_STATE  = 0x06;
static constexpr uint8_t REG_NODE_COUNT   = 0x07;
static constexpr uint8_t REG_REPLY_NODE   = 0x08;
static constexpr uint8_t REG_REPLY_CMD    = 0x09;
static constexpr uint8_t REG_REPLY_ARG0   = 0x0A;
static constexpr uint8_t REG_REPLY_ARG1   = 0x0B;
static constexpr uint8_t REG_VERSION_HI   = 0x10;
static constexpr uint8_t REG_VERSION_LO   = 0x11;
static constexpr uint8_t REG_NODE_MAP     = 0x20;

/* ============================================================================
 * Valve Master command values
 * ============================================================================
 *
 * These command bytes are written to REG_COMMAND. The firmware executes the
 * command from its main loop, not inside the TWI ISR. The host must then poll
 * STATUS_BUSY and check REG_RESULT.
 */

static constexpr uint8_t CMD_POWER_ON         = 0x01;
static constexpr uint8_t CMD_POWER_OFF        = 0x02;
static constexpr uint8_t CMD_WHO              = 0x03;
static constexpr uint8_t CMD_PING             = 0x04;
static constexpr uint8_t CMD_SET_CHANNEL      = 0x05;
static constexpr uint8_t CMD_GET_NODE_VERSION = 0x07;
static constexpr uint8_t CMD_CLOSE_ALL        = 0x0F;

/* ============================================================================
 * Status and result values
 * ============================================================================
 *
 * STATUS_BUSY means the Valve Master is still executing a command. Do not issue
 * another command until it clears.
 *
 * STATUS_ERROR is tracked by firmware and mirrored in REG_RESULT.
 *
 * STATUS_POWER_ON reflects the Valve Master's current belief about switched
 * field-bus power.
 */

static constexpr uint8_t STATUS_BUSY          = (1u << 0);
static constexpr uint8_t STATUS_ERROR         = (1u << 1);
static constexpr uint8_t STATUS_POWER_ON      = (1u << 2);

static constexpr uint8_t RESULT_OK            = 0x00;

/* ============================================================================
 * Node map / timing constants
 * ============================================================================
 *
 * NODE_MAP_BYTES is the maximum number of node entries exposed by the Valve
 * Master firmware.
 *
 * Timeouts here are host-side waits for the Valve Master BUSY bit to clear.
 * The Valve Master firmware has its own RS-485 timeouts internally.
 */

static constexpr uint8_t NODE_MAP_BYTES       = 32;

static constexpr uint32_t WHO_SCAN_TIMEOUT_MS     = 10000;
static constexpr uint32_t PING_TIMEOUT_MS         = 2000;
static constexpr uint32_t VERSION_TIMEOUT_MS      = 2000;
static constexpr uint32_t SET_CHANNEL_TIMEOUT_MS  = 3000;
static constexpr uint32_t CLOSE_ALL_TIMEOUT_MS    = 3000;

/* ============================================================================
 * Small parsing helpers
 * ========================================================================== */

/**
 * @brief Parse common bool-like strings used by pIoTServer schemas.
 *
 * Accepted true values:
 *   1, true, on, yes
 *
 * Accepted false values:
 *   0, false, off, no
 *
 * @param value Input string.
 * @param[out] out Parsed boolean value.
 * @return true if the string was recognized.
 */
static bool parse_bool_string(const string& value, bool& out)
{
    string s = value;

    transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(tolower(c));
    });

    if(s == "1" || s == "true" || s == "on" || s == "yes") {
        out = true;
        return true;
    }

    if(s == "0" || s == "false" || s == "off" || s == "no") {
        out = false;
        return true;
    }

    return false;
}

/**
 * @brief Parse an I2C 7-bit address from decimal or C-style hex text.
 *
 * Examples:
 *   "9"
 *   "0x09"
 *
 * @param text Address string.
 * @param[out] out Parsed 7-bit address.
 * @return true if valid.
 */
static bool parse_u8_address(const string& text, uint8_t& out)
{
    if(text.empty()) {
        return false;
    }

    char* end = nullptr;
    errno = 0;

    unsigned long value = strtoul(text.c_str(), &end, 0);

    if(errno != 0 || end == text.c_str() || *end != '\0') {
        return false;
    }

    if(value > 0x7f) {
        return false;
    }

    out = static_cast<uint8_t>(value);
    return true;
}

/**
 * @brief Parse an unsigned 8-bit value from decimal or C-style hex text.
 *
 * Used for schema fields that may be JSON strings, such as "1" or "0x01".
 *
 * @param text Value string.
 * @param[out] out Parsed byte.
 * @return true if valid.
 */
static bool parse_u8_value(const string& text, uint8_t& out)
{
    if(text.empty()) {
        return false;
    }

    char* end = nullptr;
    errno = 0;

    unsigned long value = strtoul(text.c_str(), &end, 0);

    if(errno != 0 || end == text.c_str() || *end != '\0') {
        return false;
    }

    if(value > 255u) {
        return false;
    }

    out = static_cast<uint8_t>(value);
    return true;
}

/**
 * @brief Extract a uint8_t from JSON.
 *
 * The schema's otherProps may contain node/valve values as unsigned numbers,
 * signed numbers, or strings. This helper accepts all three forms but rejects
 * values outside 0..255.
 *
 * @param j JSON object.
 * @param key Key name to extract.
 * @param[out] out Parsed byte value.
 * @return true if key exists and value is valid.
 */
static bool json_get_u8(const json& j, const char* key, uint8_t& out)
{
    if(!j.contains(key)) {
        return false;
    }

    if(j[key].is_number_unsigned()) {
        unsigned int value = j[key].get<unsigned int>();

        if(value > 255u) {
            return false;
        }

        out = static_cast<uint8_t>(value);
        return true;
    }

    if(j[key].is_number_integer()) {
        int value = j[key].get<int>();

        if(value < 0 || value > 255) {
            return false;
        }

        out = static_cast<uint8_t>(value);
        return true;
    }

    if(j[key].is_string()) {
        return parse_u8_value(j[key].get<string>(), out);
    }

    return false;
}

/**
 * @brief Validate a real assigned RS-485 node address.
 *
 * Address 0 is reserved for unassigned nodes. Address 255 is broadcast.
 */
static bool valid_node(uint8_t node)
{
    return node >= 1u && node <= 254u;
}

/**
 * @brief Validate a public valve/channel number.
 *
 * Current slave firmware supports only 1 and 2, but the Valve Master API is
 * designed for up to 16 channels. A future capability query should report what
 * each node actually supports.
 */
static bool valid_channel(uint8_t channel)
{
    return channel >= 1u && channel <= 16u;
}

/**
 * @brief Convert Valve Master result code to readable text.
 *
 * These names mirror valvenode_master.c.
 */
static const char* result_name(uint8_t result)
{
    switch(result) {
    case 0x00: return "OK";
    case 0x01: return "BAD_COMMAND";
    case 0x02: return "BAD_NODE";
    case 0x03: return "BAD_CHANNEL";
    case 0x04: return "NODE_NOT_FOUND";
    case 0x05: return "UNSUPPORTED_CHANNEL";
    case 0x06: return "CONFIG_REQUIRED";
    case 0x07: return "ADDRESS_IN_USE";
    case 0x08: return "BUSY";
    case 0x09: return "RS485_TIMEOUT";
    case 0x0A: return "RS485_BAD_CHECKSUM";
    case 0x0B: return "RS485_BAD_REPLY";
    case 0x0C: return "RESERVED_0C";
    case 0x0E: return "POWER_OFF";
    default:   return "UNKNOWN";
    }
}

/* ============================================================================
 * Object lifetime / schema setup
 * ========================================================================== */

VALVEMASTER_Device::VALVEMASTER_Device(string devID, string driverName)
{
    setDeviceID(devID, driverName);
    _deviceState = DEVICE_STATE_UNKNOWN;
}

/**
 * @brief Initialize driver state from the pIoTServer schema.
 *
 * The schema tells us which pIoTServer keys are controllable and how they map
 * to physical valve-node channels.
 *
 * Required per controllable key:
 *
 *   otherProps.node
 *   otherProps.valve
 *
 * Example:
 *
 *   SPRK_1:
 *     node  = 1
 *     valve = 1
 *
 *   SPRK_2:
 *     node  = 1
 *     valve = 2
 */
bool VALVEMASTER_Device::initWithSchema(deviceSchemaMap_t deviceSchema)
{
    _schema = deviceSchema;
    _state.clear();
    _bindings.clear();

    /*
     * Build initial cached state only for writable bool/actuator-like schema
     * entries. Everything starts "off" until the server asks otherwise.
     */
    for(const auto& [key, entry] : _schema) {
        if(entry.units == BOOL || entry.units == ACTUATOR) {
            _state[key] = "off";
        }
    }

    if(_state.empty()) {
        _deviceState = DEVICE_STATE_ERROR;
        LOGT_ERROR("VALVEMASTER: schema contains no BOOL or ACTUATOR keys");
        return false;
    }

    if(!loadBindingsFromSchema()) {
        _deviceState = DEVICE_STATE_ERROR;
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: schema initialized with %zu controllable keys", _state.size());
    return true;
}

/**
 * @brief Load node/valve bindings from schema otherProps.
 *
 * This is where pIoTServer property names become real hardware targets.
 *
 * A schema key without valid node/valve properties is a configuration error.
 */
bool VALVEMASTER_Device::loadBindingsFromSchema()
{
    for(const auto& [key, entry] : _schema) {
        if(entry.units != BOOL && entry.units != ACTUATOR) {
            continue;
        }

        ValveBinding binding;

        if(!json_get_u8(entry.otherProps, "node", binding.node)) {
            LOGT_ERROR("VALVEMASTER: key '%s' missing otherProps.node", key.c_str());
            return false;
        }

        if(!json_get_u8(entry.otherProps, "valve", binding.valve)) {
            LOGT_ERROR("VALVEMASTER: key '%s' missing otherProps.valve", key.c_str());
            return false;
        }

        if(!valid_node(binding.node)) {
            LOGT_ERROR("VALVEMASTER: key '%s' has invalid node %u",
                       key.c_str(),
                       binding.node);
            return false;
        }

        if(!valid_channel(binding.valve)) {
            LOGT_ERROR("VALVEMASTER: key '%s' has invalid valve/channel %u",
                       key.c_str(),
                       binding.valve);
            return false;
        }

        _bindings[key] = binding;

        LOGT_DEBUG("VALVEMASTER: key '%s' maps to node %u valve %u",
                   key.c_str(),
                   binding.node,
                   binding.valve);
    }

    return true;
}

/**
 * @brief Parse I2C address from device properties.
 *
 * Property:
 *
 *   address = "0x09"
 *
 * If omitted, defaults to 0x09.
 */
bool VALVEMASTER_Device::parseI2CAddress()
{
    string addressText = "0x09";

    json props;
    getProperties(props);

    if(props.contains("address") && props["address"].is_string()) {
        addressText = props["address"].get<string>();
    }

    uint8_t parsed = 0;

    if(!parse_u8_address(addressText, parsed)) {
        LOGT_ERROR("VALVEMASTER: invalid I2C address '%s'", addressText.c_str());
        return false;
    }

    _i2cAddress = parsed;

    LOGT_DEBUG("VALVEMASTER: configured I2C address 0x%02x", _i2cAddress);
    return true;
}

/* ============================================================================
 * Low-level I2C register access
 * ========================================================================== */

/**
 * @brief Read one Valve Master register.
 */
bool VALVEMASTER_Device::readRegister(uint8_t reg, uint8_t& valueOut)
{
    uint8_t value = 0;

    if(!_i2c.readByte(reg, value)) {
        LOGT_ERROR("VALVEMASTER: read register 0x%02x failed", reg);
        return false;
    }

    valueOut = value;
    return true;
}

/**
 * @brief Write one Valve Master register.
 */
bool VALVEMASTER_Device::writeRegister(uint8_t reg, uint8_t value)
{
    if(!_i2c.writeByte(reg, value)) {
        LOGT_ERROR("VALVEMASTER: write register 0x%02x = 0x%02x failed",
                   reg,
                   value);
        return false;
    }

    return true;
}

/**
 * @brief Queue a Valve Master command.
 *
 * The firmware defers command execution out of the TWI ISR, so this only queues
 * the command. The caller must wait for STATUS_BUSY to clear and then check
 * REG_RESULT.
 */
bool VALVEMASTER_Device::writeCommand(uint8_t command)
{
    return writeRegister(REG_COMMAND, command);
}

/* ============================================================================
 * Valve Master status / command completion
 * ========================================================================== */

/**
 * @brief Read firmware version and status summary from the Valve Master.
 *
 * Called during start() and useful as a bring-up sanity check.
 */
bool VALVEMASTER_Device::readMasterSummary()
{
    if(!readRegister(REG_VERSION_HI, _versionHi)) {
        return false;
    }

    if(!readRegister(REG_VERSION_LO, _versionLo)) {
        return false;
    }

    if(!readRegister(REG_STATUS, _lastStatus)) {
        return false;
    }

    if(!readRegister(REG_RESULT, _lastResult)) {
        return false;
    }

    if(!readRegister(REG_POWER_STATE, _lastPowerState)) {
        return false;
    }

    if(!readRegister(REG_NODE_COUNT, _lastNodeCount)) {
        return false;
    }

    _fieldPowerOn = (_lastPowerState != 0);

    LOGT_DEBUG("VALVEMASTER: firmware version %u.%u", _versionHi, _versionLo);
    LOGT_DEBUG("VALVEMASTER: status=0x%02x result=0x%02x power=%u nodes=%u",
               _lastStatus,
               _lastResult,
               _lastPowerState,
               _lastNodeCount);

    return true;
}

/**
 * @brief Wait until the Valve Master clears STATUS_BUSY.
 *
 * This is used after every command write.
 *
 * @param timeoutMs Maximum wait time.
 * @return true if not busy before timeout.
 */
bool VALVEMASTER_Device::waitNotBusy(uint32_t timeoutMs)
{
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeoutMs);

    while(chrono::steady_clock::now() < deadline) {
        uint8_t status = 0;

        if(!readRegister(REG_STATUS, status)) {
            return false;
        }

        _lastStatus = status;

        if((status & STATUS_BUSY) == 0) {
            return true;
        }

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    LOGT_ERROR("VALVEMASTER: timeout waiting for BUSY clear");
    return false;
}

/**
 * @brief Verify REG_RESULT is RESULT_OK.
 *
 * If the command failed, this logs both the hex result and readable name.
 */
bool VALVEMASTER_Device::checkResultOk(const char* operation)
{
    uint8_t result = 0;

    if(!readRegister(REG_RESULT, result)) {
        return false;
    }

    _lastResult = result;

    if(result != RESULT_OK) {
        LOGT_ERROR("VALVEMASTER: %s failed result 0x%02x %s",
                   operation ? operation : "operation",
                   result,
                   result_name(result));
        return false;
    }

    return true;
}

/* ============================================================================
 * Field-bus power control
 * ========================================================================== */

/**
 * @brief Turn on switched 12 V field-bus power.
 *
 * This commands the Valve Master to pulse the latching relay SET coil. The
 * Valve Master firmware also waits for its configured bus power-up delay before
 * clearing BUSY.
 *
 * The plugin caches the resulting power state in _fieldPowerOn.
 */
bool VALVEMASTER_Device::powerOn()
{
    if(!isConnected()) {
        LOGT_ERROR("VALVEMASTER: powerOn rejected, device is not connected");
        return false;
    }

    if(_fieldPowerOn) {
        LOGT_DEBUG("VALVEMASTER: field power already on");
        return true;
    }

    LOGT_DEBUG("VALVEMASTER: power on command");

    if(!writeCommand(CMD_POWER_ON)) {
        return false;
    }

    if(!waitNotBusy(1000)) {
        return false;
    }

    if(!checkResultOk("powerOn")) {
        return false;
    }

    if(!readRegister(REG_POWER_STATE, _lastPowerState)) {
        return false;
    }

    _fieldPowerOn = (_lastPowerState != 0);

    LOGT_DEBUG("VALVEMASTER: field power %s", _fieldPowerOn ? "on" : "off");

    return _fieldPowerOn;
}

/**
 * @brief Turn off switched 12 V field-bus power.
 *
 * This commands the Valve Master to pulse the latching relay RESET coil.
 */
bool VALVEMASTER_Device::powerOff()
{
    if(!isConnected()) {
        LOGT_ERROR("VALVEMASTER: powerOff rejected, device is not connected");
        return false;
    }

    if(!_fieldPowerOn) {
        LOGT_DEBUG("VALVEMASTER: field power already off");
        return true;
    }

    LOGT_DEBUG("VALVEMASTER: power off command");

    if(!writeCommand(CMD_POWER_OFF)) {
        return false;
    }

    if(!waitNotBusy(1000)) {
        return false;
    }

    if(!checkResultOk("powerOff")) {
        return false;
    }

    if(!readRegister(REG_POWER_STATE, _lastPowerState)) {
        return false;
    }

    _fieldPowerOn = (_lastPowerState != 0);

    LOGT_DEBUG("VALVEMASTER: field power %s", _fieldPowerOn ? "on" : "off");

    return !_fieldPowerOn;
}

/* ============================================================================
 * RS-485 discovery / diagnostics through the Valve Master
 * ========================================================================== */

/**
 * @brief Run broadcast WHO discovery and read the node map.
 *
 * This sends CMD_WHO to the Valve Master. The Valve Master broadcasts :FFW on
 * the RS-485 bus, waits for slotted slave replies, and stores discovered node
 * addresses in REG_NODE_MAP.
 */
bool VALVEMASTER_Device::probeBus()
{
    if(!powerOn()) {
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: WHO scan command");

    if(!writeCommand(CMD_WHO)) {
        return false;
    }

    if(!waitNotBusy(WHO_SCAN_TIMEOUT_MS)) {
        return false;
    }

    if(!checkResultOk("whoScan")) {
        return false;
    }

    uint8_t count = 0;

    if(!readRegister(REG_NODE_COUNT, count)) {
        return false;
    }

    _lastNodeCount = count;

    LOGT_DEBUG("VALVEMASTER: WHO found %u node(s)", count);

    if(count > NODE_MAP_BYTES) {
        count = NODE_MAP_BYTES;
    }

    for(uint8_t i = 0; i < count; i++) {
        uint8_t node = 0;

        if(!readRegister(static_cast<uint8_t>(REG_NODE_MAP + i), node)) {
            return false;
        }

        LOGT_DEBUG("VALVEMASTER: node[%u] = %u", i, node);
    }

    return true;
}

/**
 * @brief Ping one RS-485 slave node through the Valve Master.
 */
bool VALVEMASTER_Device::pingNode(uint8_t node)
{
    if(!valid_node(node)) {
        LOGT_ERROR("VALVEMASTER: bad ping node %u", node);
        return false;
    }

    if(!powerOn()) {
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: ping node %u", node);

    if(!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if(!writeCommand(CMD_PING)) {
        return false;
    }

    if(!waitNotBusy(PING_TIMEOUT_MS)) {
        return false;
    }

    if(!checkResultOk("pingNode")) {
        return false;
    }

    uint8_t replyNode = 0;
    uint8_t replyCmd = 0;

    if(!readRegister(REG_REPLY_NODE, replyNode)) {
        return false;
    }

    if(!readRegister(REG_REPLY_CMD, replyCmd)) {
        return false;
    }

    if(replyNode != node || replyCmd != static_cast<uint8_t>('A')) {
        LOGT_ERROR("VALVEMASTER: ping node %u unexpected reply node=%u cmd=0x%02x",
                   node,
                   replyNode,
                   replyCmd);
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: ping node %u OK", node);
    return true;
}

/**
 * @brief Ping every node currently listed in the Valve Master node map.
 */
bool VALVEMASTER_Device::pingDiscoveredNodes()
{
    uint8_t count = 0;

    if(!readRegister(REG_NODE_COUNT, count)) {
        return false;
    }

    _lastNodeCount = count;

    if(count > NODE_MAP_BYTES) {
        count = NODE_MAP_BYTES;
    }

    LOGT_DEBUG("VALVEMASTER: pinging %u discovered node(s)", count);

    for(uint8_t i = 0; i < count; i++) {
        uint8_t node = 0;

        if(!readRegister(static_cast<uint8_t>(REG_NODE_MAP + i), node)) {
            return false;
        }

        if(!valid_node(node)) {
            LOGT_ERROR("VALVEMASTER: node map entry %u invalid node %u", i, node);
            return false;
        }

        if(!pingNode(node)) {
            return false;
        }
    }

    return true;
}

/**
 * @brief Query firmware version from one RS-485 slave node.
 */
bool VALVEMASTER_Device::getNodeVersion(uint8_t node, uint8_t& versionHiOut, uint8_t& versionLoOut)
{
    if(!valid_node(node)) {
        LOGT_ERROR("VALVEMASTER: bad version node %u", node);
        return false;
    }

    if(!powerOn()) {
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: get version node %u", node);

    if(!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if(!writeCommand(CMD_GET_NODE_VERSION)) {
        return false;
    }

    if(!waitNotBusy(VERSION_TIMEOUT_MS)) {
        return false;
    }

    if(!checkResultOk("getNodeVersion")) {
        return false;
    }

    uint8_t replyNode = 0;
    uint8_t replyCmd = 0;
    uint8_t replyArg0 = 0;
    uint8_t replyArg1 = 0;

    if(!readRegister(REG_REPLY_NODE, replyNode)) {
        return false;
    }

    if(!readRegister(REG_REPLY_CMD, replyCmd)) {
        return false;
    }

    if(!readRegister(REG_REPLY_ARG0, replyArg0)) {
        return false;
    }

    if(!readRegister(REG_REPLY_ARG1, replyArg1)) {
        return false;
    }

    if(replyNode != node || replyCmd != static_cast<uint8_t>('V')) {
        LOGT_ERROR("VALVEMASTER: version node %u unexpected reply node=%u cmd=0x%02x arg0=0x%02x arg1=0x%02x",
                   node,
                   replyNode,
                   replyCmd,
                   replyArg0,
                   replyArg1);
        return false;
    }

    versionHiOut = replyArg0;
    versionLoOut = replyArg1;

    LOGT_DEBUG("VALVEMASTER: node %u version %u.%u", node, versionHiOut, versionLoOut);
    return true;
}

/**
 * @brief Query firmware version from every discovered node.
 */
bool VALVEMASTER_Device::versionScanDiscoveredNodes()
{
    uint8_t count = 0;

    if(!readRegister(REG_NODE_COUNT, count)) {
        return false;
    }

    _lastNodeCount = count;

    if(count > NODE_MAP_BYTES) {
        count = NODE_MAP_BYTES;
    }

    LOGT_DEBUG("VALVEMASTER: version scanning %u discovered node(s)", count);

    for(uint8_t i = 0; i < count; i++) {
        uint8_t node = 0;
        uint8_t versionHi = 0;
        uint8_t versionLo = 0;

        if(!readRegister(static_cast<uint8_t>(REG_NODE_MAP + i), node)) {
            return false;
        }

        if(!valid_node(node)) {
            LOGT_ERROR("VALVEMASTER: node map entry %u invalid node %u", i, node);
            return false;
        }

        if(!getNodeVersion(node, versionHi, versionLo)) {
            return false;
        }
    }

    return true;
}

/* ============================================================================
 * Device start / stop / connection state
 * ========================================================================== */

bool VALVEMASTER_Device::start()
{
    if(_schema.empty()) {
        _deviceState = DEVICE_STATE_ERROR;
        LOGT_ERROR("VALVEMASTER: start failed, schema is empty");
        return false;
    }

    if(!parseI2CAddress()) {
        _deviceState = DEVICE_STATE_ERROR;
        return false;
    }

    int error = 0;

    LOGT_DEBUG("VALVEMASTER: opening I2C address 0x%02x", _i2cAddress);

    if(!_i2c.begin(_i2cAddress, error)) {
        LOGT_ERROR("VALVEMASTER: I2C begin failed for address 0x%02x error=%d",
                   _i2cAddress,
                   error);

        _isConnected = false;
        _fieldPowerOn = false;
        _deviceState = DEVICE_STATE_DISCONNECTED;
        return false;
    }

    if(!_i2c.smbQuick()) {
        LOGT_ERROR("VALVEMASTER: no I2C ACK at address 0x%02x", _i2cAddress);

        _i2c.stop();
        _isConnected = false;
        _fieldPowerOn = false;
        _deviceState = DEVICE_STATE_DISCONNECTED;
        return false;
    }

    _isConnected = true;

    if(!readMasterSummary()) {
        _i2c.stop();
        _isConnected = false;
        _fieldPowerOn = false;
        _deviceState = DEVICE_STATE_DISCONNECTED;
        return false;
    }

    _deviceState = DEVICE_STATE_CONNECTED;
    return true;
}

void VALVEMASTER_Device::stop()
{
    _i2c.stop();
    _isConnected = false;
    _fieldPowerOn = false;
    _deviceState = DEVICE_STATE_DISCONNECTED;
}

bool VALVEMASTER_Device::isConnected()
{
    return _deviceState == DEVICE_STATE_CONNECTED && _isConnected && _i2c.isAvailable();
}

bool VALVEMASTER_Device::setEnabled(bool enable)
{
    _isEnabled = enable;
    return true;
}

/* ============================================================================
 * Schema key lookup and real valve control
 * ========================================================================== */

bool VALVEMASTER_Device::getBindingForKey(const string& key, ValveBinding& bindingOut)
{
    auto it = _bindings.find(key);

    if(it == _bindings.end()) {
        return false;
    }

    bindingOut = it->second;
    return true;
}

/**
 * @brief Send CMD_SET_CHANNEL for one node/channel.
 *
 * This is the real valve actuation path used by setValues().
 *
 * Register contract:
 *   REG_ARG0 = node address
 *   REG_ARG1 = valve/channel number
 *   REG_ARG2 = 1 for on/open, 0 for off/close
 *   REG_COMMAND = CMD_SET_CHANNEL
 */
bool VALVEMASTER_Device::setValveChannel(uint8_t node, uint8_t channel, bool on)
{
    if(!valid_node(node)) {
        LOGT_ERROR("VALVEMASTER: bad set-channel node %u", node);
        return false;
    }

    if(!valid_channel(channel)) {
        LOGT_ERROR("VALVEMASTER: bad set-channel valve/channel %u", channel);
        return false;
    }

    if(!powerOn()) {
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: set channel node=%u channel=%u state=%s",
               node,
               channel,
               on ? "on" : "off");

    if(!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if(!writeRegister(REG_ARG1, channel)) {
        return false;
    }

    if(!writeRegister(REG_ARG2, on ? 1u : 0u)) {
        return false;
    }

    if(!writeCommand(CMD_SET_CHANNEL)) {
        return false;
    }

    if(!waitNotBusy(SET_CHANNEL_TIMEOUT_MS)) {
        return false;
    }

    return checkResultOk("setChannel");
}

/**
 * @brief Send one Valve Master close-all command.
 *
 * This does not loop over schema keys. The Valve Master broadcasts close-all
 * over RS-485 once, and each slave node closes its own local valve outputs.
 */
bool VALVEMASTER_Device::closeAllValves()
{
    if(!powerOn()) {
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: close-all command");

    if(!writeCommand(CMD_CLOSE_ALL)) {
        return false;
    }

    if(!waitNotBusy(CLOSE_ALL_TIMEOUT_MS)) {
        return false;
    }

    return checkResultOk("closeAll");
}

/**
 * @brief Set one or more schema values.
 *
 * Each schema key maps to a hardware node/channel binding loaded from schema
 * otherProps.
 *
 * Example:
 *   SPRK_1 = on
 *     -> node 1, valve 1, open
 *
 *   SPRK_2 = off
 *     -> node 1, valve 2, close
 */
bool VALVEMASTER_Device::setValues(keyValueMap_t kv)
{
    if(!_isEnabled || !isConnected()) {
        LOGT_ERROR("VALVEMASTER: setValues rejected, device is not enabled or not connected");
        return false;
    }

    for(const auto& [key, value] : kv) {
        if(!_state.contains(key)) {
            LOGT_ERROR("VALVEMASTER: setValues rejected unknown key '%s'", key.c_str());
            return false;
        }

        bool parsed = false;
        if(!parse_bool_string(value, parsed)) {
            LOGT_ERROR("VALVEMASTER: setValues rejected invalid bool value '%s' for key '%s'",
                       value.c_str(),
                       key.c_str());
            return false;
        }

        ValveBinding binding;

        if(!getBindingForKey(key, binding)) {
            LOGT_ERROR("VALVEMASTER: no binding for key '%s'", key.c_str());
            return false;
        }

        LOGT_DEBUG("VALVEMASTER: schema key '%s' maps to node=%u valve=%u state=%s",
                   key.c_str(),
                   binding.node,
                   binding.valve,
                   parsed ? "on" : "off");

        if(!setValveChannel(binding.node, binding.valve, parsed)) {
            LOGT_ERROR("VALVEMASTER: setValues failed for key '%s'", key.c_str());
            return false;
        }

        /*
         * This is a cached command-state update. It means the command completed
         * successfully according to the Valve Master. It does not prove water
         * flow or physical valve position.
         */
        _state[key] = parsed ? "on" : "off";

        LOGT_DEBUG("VALVEMASTER: %s = %s",
                   key.c_str(),
                   _state[key].c_str());
    }

    return true;
}

bool VALVEMASTER_Device::getValues(keyValueMap_t& results)
{
    if(!isConnected()) {
        return false;
    }

    for(const auto& [key, value] : _state) {
        results[key] = value;
    }

    return true;
}

/**
 * @brief Turn all valves off using the hardware close-all command.
 *
 * Important:
 *   This sends CMD_CLOSE_ALL once.
 *
 * It does not individually close every schema key. The Valve Master sends a
 * broadcast close-all command on RS-485. Each slave node handles its own local
 * close sequence.
 */
bool VALVEMASTER_Device::allOff()
{
    if(!isConnected()) {
        LOGT_ERROR("VALVEMASTER: allOff rejected, device is not connected");
        return false;
    }

    if(!closeAllValves()) {
        LOGT_ERROR("VALVEMASTER: allOff close-all command failed");
        return false;
    }

    /*
     * After close-all succeeds, all schema values are cached as off.
     * Again, this is command-state, not sensed valve position.
     */
    for(auto& [key, value] : _state) {
        value = "off";
    }

    LOGT_DEBUG("VALVEMASTER: allOff complete");
    return true;
}

/* ============================================================================
 * Driver/plugin version
 * ============================================================================
 *
 * This is the pIoTServer VALVEMASTER plugin/driver version.
 *
 * The Makefile should normally provide this with:
 *
 *   -DVALVEMASTER_DRIVER_VERSION=\"0.1\"
 *
 * Keep the fallback here so local/manual builds still work.
 *
 * Do not confuse this with:
 *   - Valve Master firmware version read from REG_VERSION_HI/LO
 *   - slave ValveNode firmware versions read over RS-485
 */

#ifndef VALVEMASTER_DRIVER_VERSION
#define VALVEMASTER_DRIVER_VERSION "1.0"
#endif

bool VALVEMASTER_Device::getVersion(string& version)
{
    version = string("VALVEMASTER driver ") + VALVEMASTER_DRIVER_VERSION;
    return true;
}

/* ============================================================================
 * Lab/test API
 * ============================================================================
 *
 * These methods are called through exported C hooks in VALVEMASTER_factory.cpp.
 * They are intentionally not part of the pIoTServerDevice base API.
 */

bool VALVEMASTER_Device::testPowerOn()
{
    return powerOn();
}

bool VALVEMASTER_Device::testPowerOff()
{
    return powerOff();
}

bool VALVEMASTER_Device::testProbeBus()
{
    return probeBus();
}

bool VALVEMASTER_Device::testPingDiscoveredNodes()
{
    return pingDiscoveredNodes();
}

bool VALVEMASTER_Device::testVersionScanDiscoveredNodes()
{
    return versionScanDiscoveredNodes();
}
