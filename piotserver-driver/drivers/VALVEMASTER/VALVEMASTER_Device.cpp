#include "VALVEMASTER_Device.hpp"

#include "LogMgr.hpp"
#include "Utils.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <sstream>

using namespace std;


// MARK: - Valve Master I2C Register Map

/**
 * @section valvemaster_i2c_register_map Valve Master I2C Register Map
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


// MARK: - Valve Master Command Values

/**
 * @section valvemaster_command_values Valve Master Command Values
 *
 * These command bytes are written to REG_COMMAND.
 *
 * Command arguments must be written first. REG_COMMAND is written last.
 * The firmware executes the command from its main loop, not inside the TWI ISR.
 * The host then polls STATUS_BUSY and checks REG_RESULT.
 */

static constexpr uint8_t CMD_POWER_ON          = 0x01;
static constexpr uint8_t CMD_POWER_OFF         = 0x02;
static constexpr uint8_t CMD_WHO               = 0x03;
static constexpr uint8_t CMD_PING              = 0x04;
static constexpr uint8_t CMD_SET_CHANNEL       = 0x05;
static constexpr uint8_t CMD_GET_CHANNEL_STATUS = 0x06;
static constexpr uint8_t CMD_GET_NODE_VERSION  = 0x07;
static constexpr uint8_t CMD_CLOSE_ALL         = 0x0F;


// MARK: - Valve Master Status and Result Values

/**
 * @section valvemaster_status_result_values Valve Master Status and Result Values
 *
 * STATUS_BUSY means the Valve Master is still executing a command. The host
 * must not issue another command until STATUS_BUSY clears.
 *
 * STATUS_ERROR is tracked by firmware and mirrored through REG_RESULT.
 *
 * STATUS_POWER_ON reflects the Valve Master's current belief about switched
 * field-bus power.
 */

static constexpr uint8_t STATUS_BUSY          = (1u << 0);
static constexpr uint8_t STATUS_ERROR         = (1u << 1);
static constexpr uint8_t STATUS_POWER_ON      = (1u << 2);

static constexpr uint8_t RESULT_OK            = 0x00;


// MARK: - Node Map and Timing Constants

/**
 * @section valvemaster_node_map_timing Node Map and Timing Constants
 *
 * NODE_MAP_BYTES is the maximum number of node entries exposed by the Valve
 * Master firmware.
 *
 * Timeouts here are host-side waits for the Valve Master BUSY bit to clear.
 * The Valve Master firmware has its own RS-485 timeouts internally.
 */

static constexpr uint8_t NODE_MAP_BYTES                 = 32;

static constexpr uint32_t WHO_SCAN_TIMEOUT_MS           = 10000;
static constexpr uint32_t PING_TIMEOUT_MS               = 2000;
static constexpr uint32_t VERSION_TIMEOUT_MS            = 2000;
static constexpr uint32_t SET_CHANNEL_TIMEOUT_MS        = 3000;
static constexpr uint32_t GET_CHANNEL_STATUS_TIMEOUT_MS = 3000;
static constexpr uint32_t CLOSE_ALL_TIMEOUT_MS          = 3000;

/*
 * Field power-on settle time.
 *
 * After switching the 12 V field bus on, downstream AVR nodes need time to
 * boot, initialize GPIO/UART/RS-485 direction, load EEPROM identity, finish any
 * startup LED behavior, and enter receive mode.
 *
 * This belongs in the driver, not only in the harness.
 */
static constexpr uint32_t FIELD_POWER_ON_SETTLE_MS = 5000;

/*
 * Field power-off settle time.
 *
 * The Valve Master can switch field power off quickly, but downstream AVR
 * nodes and indicator LEDs may still coast on stored charge for several
 * seconds. Give the field bus time to collapse before the next power-on / WHO
 * sequence.
 */
static constexpr uint32_t FIELD_POWER_OFF_SETTLE_MS = 10000;

/*
 * WHO discovery retry policy.
 *
 * WHO is broadcast discovery and is inherently less reliable than unicast
 * version/ping. Use multiple passes and union the node list.
 */
static constexpr uint8_t WHO_DISCOVERY_PASSES = 3;
static constexpr uint32_t WHO_DISCOVERY_RETRY_GAP_MS = 250;


// MARK: - Local Parsing and Formatting Helpers

/**
 * @brief Parse an I2C 7-bit address from decimal or C-style hex text.
 *
 * Examples:
 *
 *   - "9"
 *   - "0x09"
 *
 * @param text Address string.
 * @param[out] out Parsed 7-bit address.
 * @return true if valid.
 */
static bool parse_u8_address(const string& text, uint8_t& out)
{
    string trimmed = Utils::trim(text);

    if(trimmed.empty()) {
        return false;
    }

    char* end = nullptr;
    errno = 0;

    unsigned long value = strtoul(trimmed.c_str(), &end, 0);

    if(errno != 0 || end == trimmed.c_str() || *end != '\0') {
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
    string trimmed = Utils::trim(text);

    if(trimmed.empty()) {
        return false;
    }

    char* end = nullptr;
    errno = 0;

    unsigned long value = strtoul(trimmed.c_str(), &end, 0);

    if(errno != 0 || end == trimmed.c_str() || *end != '\0') {
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
 * @brief Extract an unsigned long from JSON.
 *
 * This accepts unsigned numbers, non-negative signed numbers, and strings such
 * as "30" or "0x1e".
 *
 * @param j JSON value.
 * @param[out] out Parsed unsigned long value.
 * @return true if the value was valid.
 */
static bool json_value_to_unsigned_long(const json& j, unsigned long& out)
{
    if(j.is_number_unsigned()) {
        out = j.get<unsigned long>();
        return true;
    }

    if(j.is_number_integer()) {
        long value = j.get<long>();

        if(value < 0) {
            return false;
        }

        out = static_cast<unsigned long>(value);
        return true;
    }

    if(j.is_string()) {
        string text = Utils::trim(j.get<string>());

        if(text.empty()) {
            return false;
        }

        char* end = nullptr;
        errno = 0;

        unsigned long value = strtoul(text.c_str(), &end, 0);

        if(errno != 0 || end == text.c_str() || *end != '\0') {
            return false;
        }

        out = value;
        return true;
    }

    return false;
}

/**
 * @brief Validate a real assigned RS-485 node address.
 *
 * Address 0 is reserved for unassigned nodes. Address 255 is broadcast.
 *
 * @param node Candidate node address.
 * @return true if node is a valid assigned node address.
 */
static bool valid_node(uint8_t node)
{
    return node >= 1u && node <= 254u;
}

/**
 * @brief Validate a public valve/channel number.
 *
 * Current slave firmware supports only channels 1 and 2, but the Valve Master
 * API is designed for up to 16 channels. A future capability query should
 * report what each node actually supports.
 *
 * @param channel Candidate valve/channel number.
 * @return true if channel is within the public API range.
 */
static bool valid_channel(uint8_t channel)
{
    return channel >= 1u && channel <= 16u;
}

/**
 * @brief Convert a Valve Master result code to readable text.
 *
 * These names mirror valvenode_master.c.
 *
 * @param result REG_RESULT value.
 * @return Static readable result name.
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

/**
 * @brief Format cached node versions for diagnostics.
 *
 * @param versions Node version map.
 * @return Compact readable version string.
 */
static string format_node_versions(const map<uint8_t, VALVEMASTER_Device::NodeVersion>& versions)
{
    std::ostringstream oss;
    bool first = true;

    for(const auto& [node, version] : versions) {
        if(!first) {
            oss << ",";
        }

        first = false;

        oss << static_cast<unsigned int>(node)
            << "=";

        if(version.versionHi < 10) {
            oss << "0";
        }

        oss << static_cast<unsigned int>(version.versionHi)
            << ".";

        if(version.versionLo < 10) {
            oss << "0";
        }

        oss << static_cast<unsigned int>(version.versionLo);
    }

    return oss.str();
}

/**
 * @brief Return a readable action name.
 *
 * @param type Action type.
 * @return Static action name string.
 */
const char* VALVEMASTER_Device::actionName(action_type_t type) const
{
    switch(type) {
    case ACTION_SET_VALUES:                return "SET_VALUES";
    case ACTION_CLOSE_ALL:                 return "CLOSE_ALL";
    case ACTION_POWER_ON_TEST:             return "POWER_ON_TEST";
    case ACTION_POWER_OFF_TEST:            return "POWER_OFF_TEST";
    case ACTION_PROBE_BUS:                 return "PROBE_BUS";
    case ACTION_PING_DISCOVERED:           return "PING_DISCOVERED";
    case ACTION_VERSION_SCAN_DISCOVERED:   return "VERSION_SCAN_DISCOVERED";
    case ACTION_NONE:
    default:                               return "NONE";
    }
}

/**
 * @brief Update cached diagnostic values.
 *
 * Caller must hold _mutex.
 */
void VALVEMASTER_Device::updateDiagnosticStateLocked()
{
    _state[VALUE_FIELD_POWER] = _fieldPowerOn ? "on" : "off";
    _state[VALUE_ACTION_BUSY] = _actionBusy ? "true" : "false";
    _state[VALUE_LAST_ACTION] = _lastActionName;
    _state[VALUE_LAST_ACTION_OK] = _lastActionSucceeded ? "true" : "false";
    _state[VALUE_DISCOVERED_NODE_COUNT] = std::to_string(_discoveredNodes.size());

    if(!_nodeVersions.empty()) {
        _state[VALUE_NODE_VERSIONS] = format_node_versions(_nodeVersions);
    }
    else {
        _state[VALUE_NODE_VERSIONS] = "";
    }

    _dataDidChange = true;
}


// MARK: - VALVEMASTER_Device Lifecycle

VALVEMASTER_Device::VALVEMASTER_Device(string devID, string driverName)
{
    setDeviceID(devID, driverName);

    _deviceState = DEVICE_STATE_UNKNOWN;

    _schema.clear();
    _state.clear();
    _bindings.clear();

    _isSetup = false;
    _dataDidChange = false;

    _i2cAddress = 0x09;
    _powerHoldSec = DEFAULT_POWER_HOLD_SEC;
    _isConnected = false;
    _fieldPowerOn = false;

    _lastStatus = 0;
    _lastResult = 0;
    _lastPowerState = 0;
    _lastNodeCount = 0;
    _versionHi = 0;
    _versionLo = 0;

    _discoveredNodes.clear();
    _nodeVersions.clear();

    _running = false;
    _stopRequested = false;
    _actionBusy = false;
    _powerHoldActive = false;
    _lastActionName.clear();
    _lastActionSucceeded = true;

    json j = {
        { PROP_DEVICE_MFG_URL,        "https://www.vinthewrench.com/" },
        { PROP_DEVICE_MFG_PART,       "Valve Master I2C RS-485 irrigation controller" },
        { JSON_ARG_ADDRESS,           "0x09" },
        { JSON_ARG_POWER_HOLD_SEC,    DEFAULT_POWER_HOLD_SEC }
    };

    setProperties(j);
}

VALVEMASTER_Device::~VALVEMASTER_Device()
{
    stop();
}


// MARK: - Schema Setup

bool VALVEMASTER_Device::initWithSchema(deviceSchemaMap_t deviceSchema)
{
    std::lock_guard<std::mutex> lock(_mutex);

    _schema = deviceSchema;
    _state.clear();
    _bindings.clear();
    _isSetup = false;

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

    updateDiagnosticStateLocked();

    _isSetup = true;
    _dataDidChange = true;

    LOGT_DEBUG("VALVEMASTER: schema initialized with %zu values", _state.size());
    return true;
}

bool VALVEMASTER_Device::loadBindingsFromSchema()
{
    for(const auto& [key, entry] : _schema) {
        if(entry.units != BOOL && entry.units != ACTUATOR) {
            continue;
        }

        if(entry.readOnly) {
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

bool VALVEMASTER_Device::parseI2CAddress()
{
    string addressText = "0x09";
    unsigned long powerHoldSec = DEFAULT_POWER_HOLD_SEC;

    json props;
    getProperties(props);

    if(props.contains(JSON_ARG_ADDRESS) && props[JSON_ARG_ADDRESS].is_string()) {
        addressText = props[JSON_ARG_ADDRESS].get<string>();
    }

    uint8_t parsed = 0;

    if(!parse_u8_address(addressText, parsed)) {
        LOGT_ERROR("VALVEMASTER: invalid I2C address '%s'", addressText.c_str());
        return false;
    }

    if(props.contains(JSON_ARG_POWER_HOLD_SEC)) {
        if(!json_value_to_unsigned_long(props[JSON_ARG_POWER_HOLD_SEC], powerHoldSec)) {
            LOGT_ERROR("VALVEMASTER: invalid %s property", JSON_ARG_POWER_HOLD_SEC);
            return false;
        }
    }

    _i2cAddress = parsed;
    _powerHoldSec = powerHoldSec;

    LOGT_DEBUG("VALVEMASTER: configured I2C address 0x%02x", _i2cAddress);
    LOGT_DEBUG("VALVEMASTER: configured power_hold_sec %lu", _powerHoldSec);

    return true;
}


// MARK: - Action Thread Helpers

bool VALVEMASTER_Device::queueAction(const action_request_t& request, bool clearPendingSetValues)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if(!_running || _stopRequested || !_isConnected) {
        LOGT_ERROR("VALVEMASTER: queueAction rejected action=%s running=%d stop=%d connected=%d",
                   actionName(request.type),
                   _running ? 1 : 0,
                   _stopRequested ? 1 : 0,
                   _isConnected ? 1 : 0);
        return false;
    }

    if(clearPendingSetValues) {
        _actionQueue.erase(
            std::remove_if(_actionQueue.begin(),
                           _actionQueue.end(),
                           [](const action_request_t& queued) {
                               return queued.type == ACTION_SET_VALUES;
                           }),
            _actionQueue.end());
    }

    _actionQueue.push_back(request);
    updateDiagnosticStateLocked();
    _actionCv.notify_all();

    return true;
}

void VALVEMASTER_Device::armPowerHoldTimer()
{
    std::lock_guard<std::mutex> lock(_mutex);

    if(_powerHoldSec == 0) {
        _powerHoldActive = false;
        updateDiagnosticStateLocked();
        return;
    }

    _powerHoldDeadline = chrono::steady_clock::now() + chrono::seconds(_powerHoldSec);
    _powerHoldActive = true;

    LOGT_DEBUG("VALVEMASTER: auto power-off armed for %lu second(s)", _powerHoldSec);

    updateDiagnosticStateLocked();
    _actionCv.notify_one();
}

void VALVEMASTER_Device::cancelPowerHoldTimer()
{
    std::lock_guard<std::mutex> lock(_mutex);

    _powerHoldActive = false;
    updateDiagnosticStateLocked();
}

bool VALVEMASTER_Device::powerHoldExpired()
{
    std::lock_guard<std::mutex> lock(_mutex);

    return _powerHoldActive && chrono::steady_clock::now() >= _powerHoldDeadline;
}

bool VALVEMASTER_Device::delayWithStopCheck(uint32_t delayMs, const char* reason)
{
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(delayMs);

    if(reason != nullptr && reason[0] != '\0') {
        LOGT_DEBUG("VALVEMASTER: waiting %u ms for %s", delayMs, reason);
    }

    std::unique_lock<std::mutex> lock(_mutex);

    while(!_stopRequested) {
        if(chrono::steady_clock::now() >= deadline) {
            return true;
        }

        _actionCv.wait_until(lock, deadline, [this]() {
            return _stopRequested;
        });
    }

    LOGT_DEBUG("VALVEMASTER: delay interrupted by stop request");
    return false;
}

bool VALVEMASTER_Device::waitForIdle(uint32_t timeoutMs)
{
    auto deadline = chrono::steady_clock::now() + chrono::milliseconds(timeoutMs);
    std::unique_lock<std::mutex> lock(_mutex);

    for(;;) {
        if(!_actionBusy && _actionQueue.empty()) {
            return true;
        }

        if(_actionCv.wait_until(lock, deadline) == std::cv_status::timeout) {
            return !_actionBusy && _actionQueue.empty();
        }
    }
}

bool VALVEMASTER_Device::testWaitForIdle(uint32_t timeoutMs)
{
    return waitForIdle(timeoutMs);
}

void VALVEMASTER_Device::setLastActionStatus(const string& actionNameText, bool didSucceed)
{
    std::lock_guard<std::mutex> lock(_mutex);

    _lastActionName = actionNameText;
    _lastActionSucceeded = didSucceed;

    updateDiagnosticStateLocked();
    _actionCv.notify_all();
}

void VALVEMASTER_Device::actionThread()
{
    LOGT_DEBUG("VALVEMASTER: actionThread started");

    for(;;) {
        action_request_t request;
        bool haveRequest = false;
        bool doAutoPowerOff = false;

        {
            std::unique_lock<std::mutex> lock(_mutex);

            if(_stopRequested) {
                break;
            }

            if(_actionQueue.empty()) {
                if(_fieldPowerOn && _powerHoldActive) {
                    bool wokeForWork = _actionCv.wait_until(
                        lock,
                        _powerHoldDeadline,
                        [this]() {
                            return _stopRequested || !_actionQueue.empty();
                        });

                    if(_stopRequested) {
                        break;
                    }

                    if(wokeForWork) {
                        continue;
                    }

                    if(_fieldPowerOn && _powerHoldActive &&
                       chrono::steady_clock::now() >= _powerHoldDeadline) {
                        _powerHoldActive = false;
                        _actionBusy = true;
                        updateDiagnosticStateLocked();
                        doAutoPowerOff = true;
                    }
                } else {
                    _actionCv.wait(lock, [this]() {
                        return _stopRequested || !_actionQueue.empty();
                    });

                    if(_stopRequested) {
                        break;
                    }

                    continue;
                }
            } else {
                request = _actionQueue.front();
                _actionQueue.pop_front();
                _actionBusy = true;
                updateDiagnosticStateLocked();
                haveRequest = true;
            }
        }

        if(doAutoPowerOff) {
            LOGT_DEBUG("VALVEMASTER: auto power-off delay expired");

            bool didSucceed = powerOff();

            if(didSucceed) {
                didSucceed = delayWithStopCheck(FIELD_POWER_OFF_SETTLE_MS,
                                                "field power-off settle");
            }

            setLastActionStatus("AUTO_POWER_OFF", didSucceed);

            {
                std::lock_guard<std::mutex> lock(_mutex);
                _actionBusy = false;
                updateDiagnosticStateLocked();
                _actionCv.notify_all();
            }

            continue;
        }

        if(haveRequest) {
            bool didSucceed = executeAction(request);

            setLastActionStatus(actionName(request.type), didSucceed);

            if(request.callback) {
                request.callback(didSucceed);
            }

            {
                std::lock_guard<std::mutex> lock(_mutex);
                _actionBusy = false;
                updateDiagnosticStateLocked();
                _actionCv.notify_all();
            }

            continue;
        }
    }

    cancelPowerHoldTimer();

    if(_fieldPowerOn) {
        LOGT_DEBUG("VALVEMASTER: actionThread shutdown powering field bus off");

        if(powerOff()) {
            delayWithStopCheck(FIELD_POWER_OFF_SETTLE_MS,
                               "shutdown field power-off settle");
        }
    }

    {
        std::lock_guard<std::mutex> lock(_mutex);
        _actionBusy = false;
        _running = false;
        updateDiagnosticStateLocked();
        _actionCv.notify_all();
    }

    LOGT_DEBUG("VALVEMASTER: actionThread exited");
}

bool VALVEMASTER_Device::executeAction(const action_request_t& request)
{
    bool didSucceed = false;

    LOGT_DEBUG("VALVEMASTER: executing action %s", actionName(request.type));

    switch(request.type) {
    case ACTION_SET_VALUES:
        didSucceed = executeSetValuesAction(request);

        if(didSucceed) {
            armPowerHoldTimer();
        } else {
            cancelPowerHoldTimer();
            powerOff();
        }
        break;

    case ACTION_CLOSE_ALL:
        didSucceed = closeAllValves();
        cancelPowerHoldTimer();

        if(powerOff()) {
            delayWithStopCheck(FIELD_POWER_OFF_SETTLE_MS,
                               "field power-off settle");
        }
        break;

    case ACTION_POWER_ON_TEST:
        didSucceed = powerOn();

        if(didSucceed) {
            armPowerHoldTimer();
        }
        break;

    case ACTION_POWER_OFF_TEST:
        cancelPowerHoldTimer();
        didSucceed = powerOff();

        if(didSucceed) {
            didSucceed = delayWithStopCheck(FIELD_POWER_OFF_SETTLE_MS,
                                            "field power-off settle");
        }
        break;

    case ACTION_PROBE_BUS:
        didSucceed = probeBus();

        if(didSucceed) {
            armPowerHoldTimer();
        } else {
            cancelPowerHoldTimer();
            powerOff();
        }
        break;

    case ACTION_PING_DISCOVERED:
        didSucceed = pingDiscoveredNodes();

        if(didSucceed) {
            armPowerHoldTimer();
        } else {
            cancelPowerHoldTimer();
            powerOff();
        }
        break;

    case ACTION_VERSION_SCAN_DISCOVERED:
        didSucceed = versionScanDiscoveredNodes();

        if(didSucceed) {
            armPowerHoldTimer();
        } else {
            cancelPowerHoldTimer();
            powerOff();
        }
        break;

    case ACTION_NONE:
    default:
        LOGT_ERROR("VALVEMASTER: unknown action type %d", request.type);
        didSucceed = false;
        break;
    }

    LOGT_DEBUG("VALVEMASTER: action %s %s",
               actionName(request.type),
               didSucceed ? "succeeded" : "failed");

    return didSucceed;
}

bool VALVEMASTER_Device::executeSetValuesAction(const action_request_t& request)
{
    for(const auto& [key, requestedState] : request.values) {
        ValveBinding binding;

        if(!getBindingForKey(key, binding)) {
            LOGT_ERROR("VALVEMASTER: actionThread no binding for key '%s'", key.c_str());
            return false;
        }

        LOGT_DEBUG("VALVEMASTER: actionThread set '%s' node=%u valve=%u state=%s",
                   key.c_str(),
                   binding.node,
                   binding.valve,
                   requestedState ? "on" : "off");

        if(!setValveChannel(binding.node, binding.valve, requestedState)) {
            LOGT_ERROR("VALVEMASTER: actionThread set failed for key '%s'", key.c_str());
            return false;
        }
    }

    return true;
}


// MARK: - Low-Level I2C Register Access

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

bool VALVEMASTER_Device::writeCommand(uint8_t command)
{
    return writeRegister(REG_COMMAND, command);
}


// MARK: - Valve Master Status and Command Completion

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

    {
        std::lock_guard<std::mutex> lock(_mutex);
        updateDiagnosticStateLocked();
    }

    LOGT_DEBUG("VALVEMASTER: firmware version %u.%u", _versionHi, _versionLo);
    LOGT_DEBUG("VALVEMASTER: status=0x%02x result=0x%02x power=%u nodes=%u",
               _lastStatus,
               _lastResult,
               _lastPowerState,
               _lastNodeCount);

    return true;
}

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

        delayWithStopCheck(10, nullptr);
    }

    LOGT_ERROR("VALVEMASTER: timeout waiting for BUSY clear");
    return false;
}

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


// MARK: - Field-Bus Power Control

bool VALVEMASTER_Device::powerOn()
{
    if(!_isConnected || !_i2c.isAvailable()) {
        LOGT_ERROR("VALVEMASTER: powerOn rejected, device is not connected");
        return false;
    }

    if(_fieldPowerOn) {
        LOGT_DEBUG("VALVEMASTER: field power already on");

        {
            std::lock_guard<std::mutex> lock(_mutex);
            updateDiagnosticStateLocked();
        }

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

    {
        std::lock_guard<std::mutex> lock(_mutex);
        updateDiagnosticStateLocked();
    }

    LOGT_DEBUG("VALVEMASTER: field power %s", _fieldPowerOn ? "on" : "off");

    return _fieldPowerOn;
}

bool VALVEMASTER_Device::powerOff()
{
    if(!_isConnected || !_i2c.isAvailable()) {
        LOGT_ERROR("VALVEMASTER: powerOff rejected, device is not connected");
        return false;
    }

    if(!_fieldPowerOn) {
        LOGT_DEBUG("VALVEMASTER: field power already off");

        {
            std::lock_guard<std::mutex> lock(_mutex);
            updateDiagnosticStateLocked();
        }

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

    {
        std::lock_guard<std::mutex> lock(_mutex);
        updateDiagnosticStateLocked();
    }

    LOGT_DEBUG("VALVEMASTER: field power %s", _fieldPowerOn ? "on" : "off");

    return !_fieldPowerOn;
}

bool VALVEMASTER_Device::ensureFieldPowerOn(bool& didPowerOnOut)
{
    didPowerOnOut = false;

    bool wasOn = _fieldPowerOn;

    if(!powerOn()) {
        return false;
    }

    didPowerOnOut = !wasOn && _fieldPowerOn;
    return true;
}

bool VALVEMASTER_Device::settleAfterFieldPowerOnIfNeeded(bool didPowerOn, const char* reason)
{
    if(!didPowerOn) {
        return true;
    }

    return delayWithStopCheck(FIELD_POWER_ON_SETTLE_MS,
                              reason ? reason : "field power-on settle");
}


// MARK: - RS-485 Discovery and Diagnostics Through Valve Master

bool VALVEMASTER_Device::probeBus()
{
    bool didPowerOn = false;

    if(!ensureFieldPowerOn(didPowerOn)) {
        return false;
    }

    if(!settleAfterFieldPowerOnIfNeeded(didPowerOn, "slave wake after field power-on")) {
        return false;
    }

    vector<uint8_t> discovered;

    for(uint8_t pass = 0; pass < WHO_DISCOVERY_PASSES; pass++) {
        if(pass > 0) {
            if(!delayWithStopCheck(WHO_DISCOVERY_RETRY_GAP_MS,
                                   "WHO retry gap")) {
                return false;
            }
        }

        LOGT_DEBUG("VALVEMASTER: WHO scan command pass %u",
                   static_cast<unsigned int>(pass + 1));

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

        if(count > NODE_MAP_BYTES) {
            count = NODE_MAP_BYTES;
        }

        LOGT_DEBUG("VALVEMASTER: WHO pass %u found %u node(s)",
                   static_cast<unsigned int>(pass + 1),
                   count);

        for(uint8_t i = 0; i < count; i++) {
            uint8_t node = 0;

            if(!readRegister(static_cast<uint8_t>(REG_NODE_MAP + i), node)) {
                return false;
            }

            if(!valid_node(node)) {
                LOGT_ERROR("VALVEMASTER: node map entry %u invalid node %u",
                           i,
                           node);
                return false;
            }

            if(std::find(discovered.begin(), discovered.end(), node) == discovered.end()) {
                discovered.push_back(node);
                LOGT_DEBUG("VALVEMASTER: discovered node %u", node);
            }
        }
    }

    std::sort(discovered.begin(), discovered.end());

    {
        std::lock_guard<std::mutex> lock(_mutex);

        _lastNodeCount = static_cast<uint8_t>(
            discovered.size() > 255u ? 255u : discovered.size());

        _discoveredNodes = discovered;
        _nodeVersions.clear();

        updateDiagnosticStateLocked();
    }

    LOGT_DEBUG("VALVEMASTER: WHO union found %zu node(s)", discovered.size());

    return true;
}

bool VALVEMASTER_Device::pingNode(uint8_t node)
{
    if(!valid_node(node)) {
        LOGT_ERROR("VALVEMASTER: bad ping node %u", node);
        return false;
    }

    bool didPowerOn = false;

    if(!ensureFieldPowerOn(didPowerOn)) {
        return false;
    }

    if(!settleAfterFieldPowerOnIfNeeded(didPowerOn, "ping slave wake")) {
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

bool VALVEMASTER_Device::pingDiscoveredNodes()
{
    vector<uint8_t> nodes;

    {
        std::lock_guard<std::mutex> lock(_mutex);
        nodes = _discoveredNodes;
    }

    if(nodes.empty()) {
        LOGT_DEBUG("VALVEMASTER: no cached discovered nodes, running WHO before ping");

        if(!probeBus()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(_mutex);
        nodes = _discoveredNodes;
    }

    LOGT_DEBUG("VALVEMASTER: pinging %zu discovered node(s)", nodes.size());

    for(uint8_t node : nodes) {
        if(!pingNode(node)) {
            return false;
        }
    }

    return true;
}

bool VALVEMASTER_Device::getNodeVersion(uint8_t node, uint8_t& versionHiOut, uint8_t& versionLoOut)
{
    if(!valid_node(node)) {
        LOGT_ERROR("VALVEMASTER: bad version node %u", node);
        return false;
    }

    bool didPowerOn = false;

    if(!ensureFieldPowerOn(didPowerOn)) {
        return false;
    }

    if(!settleAfterFieldPowerOnIfNeeded(didPowerOn, "version slave wake")) {
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

bool VALVEMASTER_Device::versionScanDiscoveredNodes()
{
    vector<uint8_t> nodes;

    {
        std::lock_guard<std::mutex> lock(_mutex);
        nodes = _discoveredNodes;
    }

    if(nodes.empty()) {
        LOGT_DEBUG("VALVEMASTER: no cached discovered nodes, running WHO before version scan");

        if(!probeBus()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(_mutex);
        nodes = _discoveredNodes;
    }

    LOGT_DEBUG("VALVEMASTER: version scanning %zu discovered node(s)", nodes.size());

    map<uint8_t, NodeVersion> versions;

    for(uint8_t node : nodes) {
        uint8_t versionHi = 0;
        uint8_t versionLo = 0;

        if(!getNodeVersion(node, versionHi, versionLo)) {
            return false;
        }

        NodeVersion version;
        version.versionHi = versionHi;
        version.versionLo = versionLo;
        versions[node] = version;
    }

    {
        std::lock_guard<std::mutex> lock(_mutex);
        _nodeVersions = versions;
        updateDiagnosticStateLocked();
    }

    return true;
}


// MARK: - Device Start, Stop, and Connection State

bool VALVEMASTER_Device::start()
{
    if(_schema.empty()) {
        _deviceState = DEVICE_STATE_ERROR;
        LOGT_ERROR("VALVEMASTER: start failed, schema is empty");
        return false;
    }

    if(!_isSetup) {
        _deviceState = DEVICE_STATE_ERROR;
        LOGT_ERROR("VALVEMASTER: start failed, schema setup is incomplete");
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
        _dataDidChange = true;
        return false;
    }

    if(!_i2c.smbQuick()) {
        LOGT_ERROR("VALVEMASTER: no I2C ACK at address 0x%02x", _i2cAddress);

        _i2c.stop();
        _isConnected = false;
        _fieldPowerOn = false;
        _deviceState = DEVICE_STATE_DISCONNECTED;
        _dataDidChange = true;
        return false;
    }

    _isConnected = true;

    if(!readMasterSummary()) {
        _i2c.stop();
        _isConnected = false;
        _fieldPowerOn = false;
        _deviceState = DEVICE_STATE_DISCONNECTED;
        _dataDidChange = true;
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(_mutex);

        _stopRequested = false;
        _running = true;
        _actionBusy = false;
        _powerHoldActive = false;
        _actionQueue.clear();
        updateDiagnosticStateLocked();
    }

    _thread = std::thread(&VALVEMASTER_Device::actionThread, this);

    _deviceState = DEVICE_STATE_CONNECTED;
    _dataDidChange = true;

    LOGT_DEBUG("VALVEMASTER: device started");

    return true;
}

void VALVEMASTER_Device::stop()
{
    {
        std::lock_guard<std::mutex> lock(_mutex);

        if(!_running && !_thread.joinable()) {
            _i2c.stop();

            _isConnected = false;
            _fieldPowerOn = false;
            _deviceState = DEVICE_STATE_DISCONNECTED;
            updateDiagnosticStateLocked();
            return;
        }

        _stopRequested = true;
        _actionQueue.clear();
        _powerHoldActive = false;
        updateDiagnosticStateLocked();
    }

    _actionCv.notify_all();

    if(_thread.joinable()) {
        _thread.join();
    }

    _i2c.stop();

    {
        std::lock_guard<std::mutex> lock(_mutex);

        _running = false;
        _stopRequested = false;
        _isConnected = false;
        _fieldPowerOn = false;
        _deviceState = DEVICE_STATE_DISCONNECTED;
        updateDiagnosticStateLocked();
    }

    LOGT_DEBUG("VALVEMASTER: device stopped");
}

bool VALVEMASTER_Device::isConnected()
{
    std::lock_guard<std::mutex> lock(_mutex);

    return _deviceState == DEVICE_STATE_CONNECTED && _isConnected && _i2c.isAvailable();
}

bool VALVEMASTER_Device::setEnabled(bool enable)
{
    std::lock_guard<std::mutex> lock(_mutex);

    _isEnabled = enable;
    _dataDidChange = true;

    return true;
}


// MARK: - Schema Key Lookup and Valve Control

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
 * @brief Query one ValveNode channel status through the Valve Master.
 *
 * The Valve Master firmware sends CMD_GET_CHANNEL_STATUS to the RS-485 node.
 * The slave replies with an R frame containing:
 *
 *   arg0 = channel
 *   arg1 = state
 *
 * Current slave state characters:
 *
 *   O = open/on
 *   C = closed/off
 *
 * @param node ValveNode address.
 * @param channel Valve channel number.
 * @param onOut Destination state. true means open/on, false means closed/off.
 * @return true if the status query succeeded and returned a valid state.
 */
bool VALVEMASTER_Device::getValveChannelStatus(uint8_t node, uint8_t channel, bool& onOut)
{
    if(!valid_node(node)) {
        LOGT_ERROR("VALVEMASTER: bad status node %u", node);
        return false;
    }

    if(!valid_channel(channel)) {
        LOGT_ERROR("VALVEMASTER: bad status valve/channel %u", channel);
        return false;
    }

    bool didPowerOn = false;

    if(!ensureFieldPowerOn(didPowerOn)) {
        return false;
    }

    if(!settleAfterFieldPowerOnIfNeeded(didPowerOn, "status slave wake")) {
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: get channel status node=%u channel=%u",
               node,
               channel);

    if(!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if(!writeRegister(REG_ARG1, channel)) {
        return false;
    }

    if(!writeCommand(CMD_GET_CHANNEL_STATUS)) {
        return false;
    }

    if(!waitNotBusy(GET_CHANNEL_STATUS_TIMEOUT_MS)) {
        return false;
    }

    if(!checkResultOk("getChannelStatus")) {
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

    if(replyNode != node || replyCmd != static_cast<uint8_t>('R')) {
        LOGT_ERROR("VALVEMASTER: status node %u unexpected reply node=%u cmd=0x%02x arg0=0x%02x arg1=0x%02x",
                   node,
                   replyNode,
                   replyCmd,
                   replyArg0,
                   replyArg1);
        return false;
    }

    if(replyArg0 != channel) {
        LOGT_ERROR("VALVEMASTER: status node %u channel mismatch requested=%u reported=%u",
                   node,
                   channel,
                   replyArg0);
        return false;
    }

    if(replyArg1 == static_cast<uint8_t>('O') ||
       replyArg1 == static_cast<uint8_t>('o')) {
        onOut = true;
        return true;
    }

    if(replyArg1 == static_cast<uint8_t>('C') ||
       replyArg1 == static_cast<uint8_t>('c')) {
        onOut = false;
        return true;
    }

    LOGT_ERROR("VALVEMASTER: status node %u channel %u invalid state 0x%02x",
               node,
               channel,
               replyArg1);

    return false;
}

/**
 * @brief Send CMD_SET_CHANNEL for one node/channel.
 *
 * After the set command succeeds, immediately query CMD_GET_CHANNEL_STATUS and
 * verify the node reports the requested state. This is not constant polling.
 * It is command completion verification before the driver allows field power
 * to idle out.
 *
 * @param node ValveNode address.
 * @param channel Valve channel number.
 * @param on true to open/on, false to close/off.
 * @return true if the Valve Master reported success and status verification matched.
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

    bool didPowerOn = false;

    if(!ensureFieldPowerOn(didPowerOn)) {
        return false;
    }

    if(!settleAfterFieldPowerOnIfNeeded(didPowerOn, "set-channel slave wake")) {
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

    if(!checkResultOk("setChannel")) {
        return false;
    }

    bool verifiedOn = false;

    if(!getValveChannelStatus(node, channel, verifiedOn)) {
        LOGT_ERROR("VALVEMASTER: set channel verification failed node=%u channel=%u requested=%s",
                   node,
                   channel,
                   on ? "on" : "off");
        return false;
    }

    if(verifiedOn != on) {
        LOGT_ERROR("VALVEMASTER: set channel verification mismatch node=%u channel=%u requested=%s reported=%s",
                   node,
                   channel,
                   on ? "on" : "off",
                   verifiedOn ? "on" : "off");
        return false;
    }

    LOGT_DEBUG("VALVEMASTER: set channel verified node=%u channel=%u state=%s",
               node,
               channel,
               on ? "on" : "off");

    return true;
}

bool VALVEMASTER_Device::closeAllValves()
{
    bool didPowerOn = false;

    if(!ensureFieldPowerOn(didPowerOn)) {
        return false;
    }

    if(!settleAfterFieldPowerOnIfNeeded(didPowerOn, "close-all slave wake")) {
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

bool VALVEMASTER_Device::setValues(keyValueMap_t kv)
{
    action_request_t request;
    request.type = ACTION_SET_VALUES;

    {
        std::lock_guard<std::mutex> lock(_mutex);

        if(!_isEnabled || !_isConnected || !_running || _stopRequested) {
            LOGT_ERROR("VALVEMASTER: setValues rejected, device is not enabled/running/connected");
            return false;
        }

        for(const auto& [key, value] : kv) {
            if(!_state.contains(key)) {
                LOGT_ERROR("VALVEMASTER: setValues rejected unknown key '%s'", key.c_str());
                return false;
            }

            if(_bindings.find(key) == _bindings.end()) {
                LOGT_ERROR("VALVEMASTER: setValues rejected read-only or diagnostic key '%s'",
                           key.c_str());
                return false;
            }

            bool parsed = false;

            if(!stringToBool(value, parsed)) {
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

            request.values.push_back(std::make_pair(key, parsed));

            _state[key] = parsed ? "on" : "off";
            _dataDidChange = true;

            LOGT_DEBUG("VALVEMASTER: queued schema key '%s' node=%u valve=%u state=%s",
                       key.c_str(),
                       binding.node,
                       binding.valve,
                       parsed ? "on" : "off");
        }
    }

    if(request.values.empty()) {
        return true;
    }

    return queueAction(request);
}

bool VALVEMASTER_Device::hasUpdates()
{
    std::lock_guard<std::mutex> lock(_mutex);

    return _dataDidChange;
}

bool VALVEMASTER_Device::getValues(keyValueMap_t& results)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if(_deviceState != DEVICE_STATE_CONNECTED || !_isConnected) {
        return false;
    }

    for(const auto& [key, value] : _state) {
        results[key] = value;
    }

    _dataDidChange = false;

    return true;
}

bool VALVEMASTER_Device::allOff()
{
    action_request_t request;
    request.type = ACTION_CLOSE_ALL;

    {
        std::lock_guard<std::mutex> lock(_mutex);

        if(!_isConnected || !_running || _stopRequested) {
            LOGT_ERROR("VALVEMASTER: allOff rejected, device is not running/connected");
            return false;
        }

        for(auto& [key, value] : _state) {
            if(_bindings.find(key) != _bindings.end()) {
                value = "off";
            }
        }

        updateDiagnosticStateLocked();
    }

    return queueAction(request, true);
}


// MARK: - Driver / Plugin Version

#ifndef VALVEMASTER_DRIVER_VERSION
#define VALVEMASTER_DRIVER_VERSION "1.0"
#endif

bool VALVEMASTER_Device::getVersion(string& version)
{
    version = string("VALVEMASTER driver ") + VALVEMASTER_DRIVER_VERSION;
    return true;
}


// MARK: - Lab / Test API

bool VALVEMASTER_Device::testPowerOn()
{
    action_request_t request;
    request.type = ACTION_POWER_ON_TEST;

    return queueAction(request);
}

bool VALVEMASTER_Device::testPowerOff()
{
    action_request_t request;
    request.type = ACTION_POWER_OFF_TEST;

    return queueAction(request, true);
}

bool VALVEMASTER_Device::testProbeBus()
{
    action_request_t request;
    request.type = ACTION_PROBE_BUS;

    return queueAction(request);
}

bool VALVEMASTER_Device::testPingDiscoveredNodes()
{
    action_request_t request;
    request.type = ACTION_PING_DISCOVERED;

    return queueAction(request);
}

bool VALVEMASTER_Device::testVersionScanDiscoveredNodes()
{
    action_request_t request;
    request.type = ACTION_VERSION_SCAN_DISCOVERED;

    return queueAction(request);
}

/* testWaitForIdle() is implemented near waitForIdle(). */
