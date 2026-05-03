//
//  Valve_master.cpp
//
//  Client-side interface for the Valve Master I2C controller.
//
//  This implementation matches the register/command contract used by
//  valvenode_master_sim.c and intended for the future product firmware.
//

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <chrono>
#include <thread>

#include "Valve_master.hpp"
#include "LogMgr.hpp"

/**
 * @brief Construct a Valve_master object.
 *
 * The object starts closed. Call begin() before attempting to communicate
 * with the Valve Master hardware.
 */
Valve_master::Valve_master()
{
    _isSetup = false;
}

/**
 * @brief Destroy the Valve_master object.
 *
 * Stops the underlying I2C interface if it is open.
 */
Valve_master::~Valve_master()
{
    stop();
}

/**
 * @brief Open the Valve Master using the default or supplied I2C address.
 *
 * This overload discards the lower-level errno value. Use begin(address, error)
 * if the caller needs the reason for failure.
 *
 * @param deviceAddress 7-bit I2C address of the Valve Master.
 * @return true if the device was opened and initialized, false otherwise.
 */
bool Valve_master::begin(uint8_t deviceAddress)
{
    int error = 0;

    return begin(deviceAddress, error);
}

/**
 * @brief Open the Valve Master using the supplied I2C address.
 *
 * @param deviceAddress 7-bit I2C address of the Valve Master.
 * @param error Receives the errno value from the underlying I2C open/setup on failure.
 * @return true if the device was opened and initialized, false otherwise.
 */
bool Valve_master::begin(uint8_t deviceAddress, int &error)
{
    _isSetup = _i2cPort.begin(deviceAddress, error);

    LOGT_DEBUG("Valve_master(%02x) begin: %s",
               deviceAddress,
               _isSetup ? "OK" : "FAIL");

    if (!_isSetup) {
        return false;
    }

    if (!softReset()) {
        stop();
        return false;
    }

    return true;
}

/**
 * @brief Close the Valve Master interface.
 */
void Valve_master::stop()
{
    if (_isSetup) {
        LOGT_DEBUG("Valve_master(%02x) stop\n", _i2cPort.getDevAddr());

        _isSetup = false;
        _i2cPort.stop();
    }
}

/**
 * @brief Return the currently configured I2C device address.
 *
 * @return 7-bit I2C address from the underlying I2C object.
 */
uint8_t Valve_master::getDevAddr()
{
    return _i2cPort.getDevAddr();
}

/**
 * @brief Check whether the Valve Master interface is open.
 *
 * @return true if begin() has completed and stop() has not been called.
 */
bool Valve_master::isOpen()
{
    return _isSetup;
}

/**
 * @brief Perform class-level startup initialization.
 *
 * This currently verifies that basic register reads are possible by reading
 * the master firmware version.
 *
 * @return true if startup checks succeeded.
 */
bool Valve_master::softReset()
{
    FirmwareVersion version = 0;

    LOGT_DEBUG("Valve_master(%02x) softReset\n", _i2cPort.getDevAddr());

    return getMasterVersion(version);
}

/**
 * @brief Read a single Valve Master register.
 *
 * @param reg Register address to read.
 * @param valueOut Receives the register value on success.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::readRegister(uint8_t reg, uint8_t &valueOut)
{
    uint8_t value = 0;

    if (!_i2cPort.readByte(reg, value)) {
        LOGT_ERROR("Valve_master(%02X) read register %02X failed: %s",
                   _i2cPort.getDevAddr(), reg, strerror(errno));
        return false;
    }

    valueOut = value;
    return true;
}

/**
 * @brief Write a single Valve Master register.
 *
 * @param reg Register address to write.
 * @param value Byte value to write.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::writeRegister(uint8_t reg, uint8_t value)
{
    if (!_i2cPort.writeByte(reg, value)) {
        LOGT_ERROR("Valve_master(%02X) write register %02X = %02X failed: %s",
                   _i2cPort.getDevAddr(), reg, value, strerror(errno));
        return false;
    }

    return true;
}

/**
 * @brief Write a command to the Valve Master command register.
 *
 * Writing REG_COMMAND queues command execution in the firmware. The caller may
 * then poll STATUS_BUSY and read REG_RESULT / reply registers.
 *
 * @param command Command byte to write to REG_COMMAND.
 * @return true if the command byte was written successfully.
 */
bool Valve_master::writeCommand(uint8_t command)
{
    return writeRegister(REG_COMMAND, command);
}

/**
 * @brief Submit a command and wait for STATUS_BUSY to clear.
 *
 * @param command Command byte.
 * @param timeoutMs Maximum wait time in milliseconds.
 * @return true if the command was written and BUSY cleared.
 */
bool Valve_master::submitCommandAndWait(uint8_t command, uint32_t timeoutMs)
{
    if (!writeCommand(command)) {
        return false;
    }

    return waitNotBusy(timeoutMs);
}

/**
 * @brief Read REG_RESULT and verify it is RESULT_OK.
 *
 * @param operation Operation name for logging.
 * @return true if REG_RESULT is RESULT_OK.
 */
bool Valve_master::checkResultOk(const char *operation)
{
    uint8_t result = 0;

    if (!getLastResult(result)) {
        return false;
    }

    if (result != RESULT_OK) {
        LOGT_ERROR("Valve_master(%02X) %s failed result %02X",
                   _i2cPort.getDevAddr(),
                   operation ? operation : "operation",
                   result);
        return false;
    }

    return true;
}

/**
 * @brief Read the Valve Master status register.
 *
 * @param statusOut Receives the status byte on success.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::getStatus(uint8_t &statusOut)
{
    return readRegister(REG_STATUS, statusOut);
}

/**
 * @brief Read the slave-node power relay state.
 *
 * @param stateOut Receives the power state byte.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::getPowerState(uint8_t &stateOut)
{
    return readRegister(REG_POWER_STATE, stateOut);
}

/**
 * @brief Read the result of the most recently completed or rejected command.
 *
 * @param resultOut Receives one of the RESULT_* values.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::getLastResult(uint8_t &resultOut)
{
    return readRegister(REG_RESULT, resultOut);
}

/**
 * @brief Read the reply registers.
 *
 * @param replyOut Receives the decoded reply register values.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::getReply(Reply &replyOut)
{
    uint8_t node = 0;
    uint8_t cmd = 0;
    uint8_t arg0 = 0;
    uint8_t arg1 = 0;

    if (!readRegister(REG_REPLY_NODE, node)) {
        return false;
    }

    if (!readRegister(REG_REPLY_CMD, cmd)) {
        return false;
    }

    if (!readRegister(REG_REPLY_ARG0, arg0)) {
        return false;
    }

    if (!readRegister(REG_REPLY_ARG1, arg1)) {
        return false;
    }

    replyOut.node = node;
    replyOut.cmd = cmd;
    replyOut.arg0 = arg0;
    replyOut.arg1 = arg1;

    return true;
}

/**
 * @brief Read the Valve Master firmware version.
 *
 * @param versionOut Receives the packed firmware version.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::getMasterVersion(FirmwareVersion &versionOut)
{
    uint8_t hi = 0;
    uint8_t lo = 0;

    if (!readRegister(REG_VERSION_HI, hi)) {
        return false;
    }

    if (!readRegister(REG_VERSION_LO, lo)) {
        return false;
    }

    versionOut = static_cast<FirmwareVersion>((static_cast<uint16_t>(hi) << 8) | lo);
    return true;
}

/**
 * @brief Query a slave node and return its firmware version.
 *
 * @param node RS485 node address, valid range 1-254.
 * @param versionOut Receives the packed node firmware version.
 * @return true on success, false on validation, timeout, I2C, or command result failure.
 */
bool Valve_master::getNodeVersion(uint8_t node, FirmwareVersion &versionOut)
{
    if (!validNode(node)) {
        LOGT_ERROR("Valve_master(%02X) bad node address: %u",
                   _i2cPort.getDevAddr(), node);
        return false;
    }

    if (!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if (!submitCommandAndWait(CMD_GET_NODE_VERSION, 2000)) {
        LOGT_ERROR("Valve_master(%02X) getNodeVersion node %u timed out",
                   _i2cPort.getDevAddr(), node);
        return false;
    }

    if (!checkResultOk("getNodeVersion")) {
        return false;
    }

    Reply reply;

    if (!getReply(reply)) {
        return false;
    }

    if (reply.node != node || reply.cmd != 'V') {
        LOGT_ERROR("Valve_master(%02X) getNodeVersion unexpected reply node=%u cmd=%02X",
                   _i2cPort.getDevAddr(), reply.node, reply.cmd);
        return false;
    }

    versionOut = static_cast<FirmwareVersion>((static_cast<uint16_t>(reply.arg0) << 8) |
                                              reply.arg1);
    return true;
}

/**
 * @brief Submit the POWER_ON command and wait for completion.
 *
 * @return true on success.
 */
bool Valve_master::powerOn()
{
    if (!submitCommandAndWait(CMD_POWER_ON, 1000)) {
        return false;
    }

    return checkResultOk("powerOn");
}

/**
 * @brief Submit the POWER_OFF command and wait for completion.
 *
 * @return true on success.
 */
bool Valve_master::powerOff()
{
    if (!submitCommandAndWait(CMD_POWER_OFF, 1000)) {
        return false;
    }

    return checkResultOk("powerOff");
}

/**
 * @brief Ping a node.
 *
 * @param node RS485 node address, valid range 1-254.
 * @return true if the node ACKed.
 */
bool Valve_master::pingNode(uint8_t node)
{
    if (!validNode(node)) {
        LOGT_ERROR("Valve_master(%02X) bad node address: %u",
                   _i2cPort.getDevAddr(), node);
        return false;
    }

    if (!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if (!submitCommandAndWait(CMD_PING, 1000)) {
        return false;
    }

    if (!checkResultOk("pingNode")) {
        return false;
    }

    Reply reply;

    if (!getReply(reply)) {
        return false;
    }

    return reply.node == node && reply.cmd == 'A';
}

/**
 * @brief Set one channel on or off.
 *
 * @param node RS485 node address, valid range 1-254.
 * @param channel Public channel number, valid range 1-16.
 * @param on true for ON/OPEN, false for OFF/CLOSE.
 * @return true on success.
 */
bool Valve_master::setChannel(uint8_t node, uint8_t channel, bool on)
{
    if (!validNode(node)) {
        LOGT_ERROR("Valve_master(%02X) bad node address: %u",
                   _i2cPort.getDevAddr(), node);
        return false;
    }

    if (!validChannel(channel)) {
        LOGT_ERROR("Valve_master(%02X) bad channel: %u",
                   _i2cPort.getDevAddr(), channel);
        return false;
    }

    if (!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if (!writeRegister(REG_ARG1, channel)) {
        return false;
    }

    if (!writeRegister(REG_ARG2, on ? 1 : 0)) {
        return false;
    }

    if (!submitCommandAndWait(CMD_SET_CHANNEL, 2000)) {
        return false;
    }

    return checkResultOk("setChannel");
}

/**
 * @brief Compatibility wrapper using irrigation wording.
 *
 * @param node RS485 node address.
 * @param valveNumber Public valve number.
 * @param on true for ON/OPEN, false for OFF/CLOSE.
 * @return true on success.
 */
bool Valve_master::setValve(uint8_t node, uint8_t valveNumber, bool on)
{
    return setChannel(node, valveNumber, on);
}

/**
 * @brief Query one channel's current state.
 *
 * @param node RS485 node address.
 * @param channel Public channel number.
 * @param stateOut Receives 'O' for open or 'C' for closed on success.
 * @return true on success.
 */
bool Valve_master::getChannelStatus(uint8_t node, uint8_t channel, uint8_t &stateOut)
{
    if (!validNode(node)) {
        LOGT_ERROR("Valve_master(%02X) bad node address: %u",
                   _i2cPort.getDevAddr(), node);
        return false;
    }

    if (!validChannel(channel)) {
        LOGT_ERROR("Valve_master(%02X) bad channel: %u",
                   _i2cPort.getDevAddr(), channel);
        return false;
    }

    if (!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if (!writeRegister(REG_ARG1, channel)) {
        return false;
    }

    if (!submitCommandAndWait(CMD_GET_CHANNEL_STATUS, 2000)) {
        return false;
    }

    if (!checkResultOk("getChannelStatus")) {
        return false;
    }

    Reply reply;

    if (!getReply(reply)) {
        return false;
    }

    if (reply.node != node || reply.cmd != 'R' || reply.arg0 != channel) {
        LOGT_ERROR("Valve_master(%02X) getChannelStatus unexpected reply node=%u cmd=%02X arg0=%02X",
                   _i2cPort.getDevAddr(), reply.node, reply.cmd, reply.arg0);
        return false;
    }

    stateOut = reply.arg1;
    return true;
}

/**
 * @brief Submit WHO discovery and wait for completion.
 *
 * @return true on success.
 */
bool Valve_master::whoScan()
{
    if (!submitCommandAndWait(CMD_WHO, 7000)) {
        return false;
    }

    return checkResultOk("whoScan");
}

/**
 * @brief Read the number of nodes in the current node map.
 *
 * @param countOut Receives node count.
 * @return true on success.
 */
bool Valve_master::getNodeCount(uint8_t &countOut)
{
    return readRegister(REG_NODE_COUNT, countOut);
}

/**
 * @brief Read the discovered node list.
 *
 * The firmware exposes a compact node list beginning at REG_NODE_MAP.
 * REG_NODE_COUNT gives the number of valid entries. At most NODE_MAP_BYTES
 * entries are read.
 *
 * @param nodesOut Caller-provided NODE_MAP_BYTES buffer.
 * @param countOut Receives number of valid entries copied.
 * @return true on success.
 */
bool Valve_master::readNodeMap(uint8_t nodesOut[NODE_MAP_BYTES], uint8_t &countOut)
{
    if (nodesOut == nullptr) {
        LOGT_ERROR("Valve_master(%02X) readNodeMap null output pointer",
                   _i2cPort.getDevAddr());
        return false;
    }

    uint8_t count = 0;

    if (!getNodeCount(count)) {
        return false;
    }

    if (count > NODE_MAP_BYTES) {
        count = NODE_MAP_BYTES;
    }

    if (count > 0) {
        if (!_i2cPort.readBytes(REG_NODE_MAP, count, nodesOut)) {
            LOGT_ERROR("Valve_master(%02X) read NODE_MAP failed: %s",
                       _i2cPort.getDevAddr(), strerror(errno));
            return false;
        }
    }

    for (uint8_t i = count; i < NODE_MAP_BYTES; ++i) {
        nodesOut[i] = 0;
    }

    countOut = count;
    return true;
}

/**
 * @brief Ask one node to enter identify/blink mode.
 *
 * @param node RS485 node address.
 * @return true on success.
 */
bool Valve_master::identifyNode(uint8_t node)
{
    if (!validNode(node)) {
        LOGT_ERROR("Valve_master(%02X) bad node address: %u",
                   _i2cPort.getDevAddr(), node);
        return false;
    }

    if (!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if (!submitCommandAndWait(CMD_IDENTIFY, 1000)) {
        return false;
    }

    return checkResultOk("identifyNode");
}

/**
 * @brief Broadcast cancel config/identify modes.
 *
 * @return true on success.
 */
bool Valve_master::cancel()
{
    if (!submitCommandAndWait(CMD_CANCEL, 1000)) {
        return false;
    }

    return checkResultOk("cancel");
}

/**
 * @brief Put a node into config mode.
 *
 * node 0 means unassigned/config address 00.
 *
 * @param node Node address 1-254, or 0 for unassigned.
 * @return true on success.
 */
bool Valve_master::config(uint8_t node)
{
    if (node != 0 && !validNode(node)) {
        LOGT_ERROR("Valve_master(%02X) bad config node: %u",
                   _i2cPort.getDevAddr(), node);
        return false;
    }

    if (!writeRegister(REG_ARG0, node)) {
        return false;
    }

    if (!submitCommandAndWait(CMD_CONFIG, 1000)) {
        return false;
    }

    return checkResultOk("config");
}

/**
 * @brief Assign a new address to the node currently in config mode.
 *
 * @param newNode New assigned node address, valid range 1-254.
 * @return true on success.
 */
bool Valve_master::assign(uint8_t newNode)
{
    if (!validNode(newNode)) {
        LOGT_ERROR("Valve_master(%02X) bad new node address: %u",
                   _i2cPort.getDevAddr(), newNode);
        return false;
    }

    if (!writeRegister(REG_ARG0, newNode)) {
        return false;
    }

    if (!submitCommandAndWait(CMD_ASSIGN, 1000)) {
        return false;
    }

    return checkResultOk("assign");
}

/**
 * @brief Move an already-addressed node to a new address.
 *
 * This is a driver-side helper sequence:
 *   1. config(oldNode)
 *   2. assign(newNode)
 *
 * @param oldNode Existing node address.
 * @param newNode New node address.
 * @return true on success.
 */
bool Valve_master::moveNode(uint8_t oldNode, uint8_t newNode)
{
    if (!validNode(oldNode) || !validNode(newNode)) {
        LOGT_ERROR("Valve_master(%02X) bad move node old=%u new=%u",
                   _i2cPort.getDevAddr(), oldNode, newNode);
        return false;
    }

    if (oldNode == newNode) {
        LOGT_ERROR("Valve_master(%02X) move old node and new node are the same: %u",
                   _i2cPort.getDevAddr(), oldNode);
        return false;
    }

    if (!config(oldNode)) {
        return false;
    }

    return assign(newNode);
}

/**
 * @brief Check whether the Valve Master firmware is busy.
 *
 * @param busyOut Receives true if STATUS_BUSY is set.
 * @return true on success, false on I2C failure.
 */
bool Valve_master::isBusy(bool &busyOut)
{
    uint8_t status = 0;

    if (!getStatus(status)) {
        return false;
    }

    busyOut = (status & STATUS_BUSY) != 0;
    return true;
}

/**
 * @brief Wait for the Valve Master BUSY bit to clear.
 *
 * Polls STATUS_BUSY every 10 ms until it clears or the timeout expires.
 *
 * @param timeoutMs Maximum time to wait in milliseconds.
 * @return true if BUSY cleared before timeout, false on timeout or I2C failure.
 */
bool Valve_master::waitNotBusy(uint32_t timeoutMs)
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (std::chrono::steady_clock::now() < deadline) {
        bool busy = false;

        if (!isBusy(busy)) {
            return false;
        }

        if (!busy) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    LOGT_ERROR("Valve_master(%02X) timeout waiting for BUSY clear",
               _i2cPort.getDevAddr());

    return false;
}

/**
 * @brief Submit the CLEAR_ERROR command.
 *
 * @return true on success.
 */
bool Valve_master::clearError()
{
    if (!submitCommandAndWait(CMD_CLEAR_ERROR, 1000)) {
        return false;
    }

    return checkResultOk("clearError");
}

/**
 * @brief Submit the SET_ERROR command.
 *
 * This is mainly for simulator / bench testing. Firmware behavior should set
 * STATUS_ERROR. The real product firmware can later also drive the fault LED
 * and fault output line.
 *
 * @return true on success.
 */
bool Valve_master::setError()
{
    if (!submitCommandAndWait(CMD_SET_ERROR, 1000)) {
        return false;
    }

    return checkResultOk("setError");
}

/**
 * @brief Validate a node address.
 *
 * @param node Node address.
 * @return true if valid for assigned nodes.
 */
bool Valve_master::validNode(uint8_t node)
{
    return node >= MIN_NODE_ADDR && node <= MAX_NODE_ADDR;
}

/**
 * @brief Validate a public channel number.
 *
 * @param channel Channel number.
 * @return true if in the master API range.
 */
bool Valve_master::validChannel(uint8_t channel)
{
    return channel >= MIN_CHANNEL && channel <= MAX_CHANNEL;
}

/**
 * @brief Extract the major byte from a packed firmware version.
 *
 * @param version Packed 16-bit firmware version.
 * @return Major version byte.
 */
uint8_t Valve_master::versionMajor(FirmwareVersion version)
{
    return static_cast<uint8_t>((version >> 8) & 0xFF);
}

/**
 * @brief Extract the minor byte from a packed firmware version.
 *
 * @param version Packed 16-bit firmware version.
 * @return Minor version byte.
 */
uint8_t Valve_master::versionMinor(FirmwareVersion version)
{
    return static_cast<uint8_t>(version & 0xFF);
}
