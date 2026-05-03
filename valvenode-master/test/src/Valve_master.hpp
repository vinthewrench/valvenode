#ifndef Valve_master_hpp
#define Valve_master_hpp

#include <stdint.h>
#include <stddef.h>

#include "I2C.hpp"

/**
 * @file Valve_master.hpp
 * @brief Host-side C++ driver for the ATmega88PB Valve Master.
 *
 * This file defines the Valve_master class used by the Linux-side test tool.
 * The class talks to the Valve Master firmware over I2C and exposes a small
 * command API for RS-485 valve-node discovery, configuration, valve control,
 * and status reporting.
 */

/**
 * @class Valve_master
 * @brief Host-side I2C driver for the ATmega88PB Valve Master.
 *
 * Valve_master provides the C++ host interface used by the test CLI to talk
 * to the real Valve Master firmware over Linux I2C.
 *
 * The Valve Master sits between the host and the RS-485 valve-node field bus:
 *
 * @code
 * Host test tool
 *   -> Linux I2C
 *   -> ATmega88PB Valve Master at address 0x09
 *   -> switched 12 V field bus
 *   -> half-duplex RS-485 valve nodes
 * @endcode
 *
 * This class wraps the Valve Master register map and command model. It handles
 * register reads, register writes, command submission, status polling, node-map
 * reads, version reads, and reply/result inspection.
 *
 * The class does not directly speak RS-485. RS-485 framing, retries, node
 * discovery, and valve actuation are handled by the Valve Master firmware.
 */
class Valve_master
{
public:

    /** @brief Default 7-bit I2C address for the Valve Master. */
    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x09;

    /** @name I2C register map */
    ///@{
    static constexpr uint8_t REG_COMMAND        = 0x00;
    static constexpr uint8_t REG_STATUS         = 0x01;
    static constexpr uint8_t REG_ARG0           = 0x02;
    static constexpr uint8_t REG_ARG1           = 0x03;
    static constexpr uint8_t REG_ARG2           = 0x04;
    static constexpr uint8_t REG_RESULT         = 0x05;
    static constexpr uint8_t REG_POWER_STATE    = 0x06;
    static constexpr uint8_t REG_NODE_COUNT     = 0x07;
    static constexpr uint8_t REG_REPLY_NODE     = 0x08;
    static constexpr uint8_t REG_REPLY_CMD      = 0x09;
    static constexpr uint8_t REG_REPLY_ARG0     = 0x0A;
    static constexpr uint8_t REG_REPLY_ARG1     = 0x0B;

    static constexpr uint8_t REG_VERSION_HI     = 0x10;
    static constexpr uint8_t REG_VERSION_LO     = 0x11;

    static constexpr uint8_t REG_NODE_MAP       = 0x20;
    static constexpr uint8_t NODE_MAP_BYTES     = 32;
    ///@}

    /** @name Valve Master command values */
    ///@{
    static constexpr uint8_t CMD_NONE                   = 0x00;
    static constexpr uint8_t CMD_POWER_ON               = 0x01;
    static constexpr uint8_t CMD_POWER_OFF              = 0x02;
    static constexpr uint8_t CMD_WHO                    = 0x03;
    static constexpr uint8_t CMD_PING                   = 0x04;
    static constexpr uint8_t CMD_SET_CHANNEL            = 0x05;
    static constexpr uint8_t CMD_GET_CHANNEL_STATUS     = 0x06;
    static constexpr uint8_t CMD_GET_NODE_VERSION       = 0x07;
    static constexpr uint8_t CMD_IDENTIFY               = 0x08;
    static constexpr uint8_t CMD_CANCEL                 = 0x09;
    static constexpr uint8_t CMD_CONFIG                 = 0x0A;
    static constexpr uint8_t CMD_ASSIGN                 = 0x0B;
    static constexpr uint8_t CMD_CLEAR_ERROR            = 0x0C;
    static constexpr uint8_t CMD_SET_ERROR              = 0x0D;
    ///@}

    /** @name REG_STATUS bit definitions */
    ///@{
    static constexpr uint8_t STATUS_BUSY        = (1u << 0);
    static constexpr uint8_t STATUS_ERROR       = (1u << 1);
    static constexpr uint8_t STATUS_POWER_ON    = (1u << 2);
    ///@}

    /** @name REG_RESULT values */
    ///@{
    static constexpr uint8_t RESULT_OK                   = 0x00;
    static constexpr uint8_t RESULT_BAD_COMMAND          = 0x01;
    static constexpr uint8_t RESULT_BAD_NODE             = 0x02;
    static constexpr uint8_t RESULT_BAD_CHANNEL          = 0x03;
    static constexpr uint8_t RESULT_NODE_NOT_FOUND       = 0x04;
    static constexpr uint8_t RESULT_UNSUPPORTED_CHANNEL  = 0x05;
    static constexpr uint8_t RESULT_CONFIG_REQUIRED      = 0x06;
    static constexpr uint8_t RESULT_ADDRESS_IN_USE       = 0x07;
    static constexpr uint8_t RESULT_BUSY                 = 0x08;
    ///@}

    /** @name Node and channel limits */
    ///@{
    static constexpr uint8_t MIN_NODE_ADDR       = 1;
    static constexpr uint8_t MAX_NODE_ADDR       = 254;
    static constexpr uint8_t MIN_CHANNEL         = 1;
    static constexpr uint8_t MAX_CHANNEL         = 16;
    ///@}

    /**
     * @brief Packed firmware version value.
     *
     * The high byte is the major version and the low byte is the minor version.
     */
    using FirmwareVersion = uint16_t;

    /**
     * @brief Last reply fields reported by the Valve Master firmware.
     *
     * These values are copied from the firmware reply registers after a command
     * completes. The meaning of arg0 and arg1 depends on the command.
     */
    struct Reply {
        uint8_t node;  ///< Replying node address.
        uint8_t cmd;   ///< Reply command or response code.
        uint8_t arg0;  ///< First reply argument.
        uint8_t arg1;  ///< Second reply argument.
    };

    /**
     * @brief Construct a closed Valve Master driver instance.
     */
    Valve_master();

    /**
     * @brief Close the I2C device if it is still open.
     */
    ~Valve_master();

    /**
     * @brief Open the default Linux I2C bus using the supplied I2C address.
     *
     * @param deviceAddress 7-bit I2C address of the Valve Master.
     *
     * @return true if the I2C port was opened and configured.
     * @return false on open or address-selection failure.
     */
    bool begin(uint8_t deviceAddress = DEFAULT_I2C_ADDR);

    /**
     * @brief Open the default Linux I2C bus using the supplied I2C address.
     *
     * @param deviceAddress 7-bit I2C address of the Valve Master.
     * @param error Receives a platform error code on failure.
     *
     * @return true if the I2C port was opened and configured.
     * @return false on open or address-selection failure.
     */
    bool begin(uint8_t deviceAddress, int &error);

    /**
     * @brief Close the I2C port and mark the driver as not set up.
     */
    void stop();

    /**
     * @brief Report whether the I2C driver is open and set up.
     *
     * @return true if begin() succeeded and stop() has not been called.
     * @return false otherwise.
     */
    bool isOpen();

    /**
     * @brief Return the active Valve Master I2C address.
     *
     * @return 7-bit I2C address currently used by the I2C wrapper.
     */
    uint8_t getDevAddr();

    /**
     * @brief Read the Valve Master status register.
     *
     * @param status Receives REG_STATUS bit flags.
     *
     * @return true if the register read succeeded.
     * @return false on I2C failure.
     */
    bool getStatus(uint8_t &status);

    /**
     * @brief Read the switched field-bus power state.
     *
     * @param state Receives the power-state register value.
     *
     * @return true if the register read succeeded.
     * @return false on I2C failure.
     */
    bool getPowerState(uint8_t &state);

    /**
     * @brief Read the most recent command result code.
     *
     * @param result Receives one of the RESULT_* values.
     *
     * @return true if the register read succeeded.
     * @return false on I2C failure.
     */
    bool getLastResult(uint8_t &result);

    /**
     * @brief Read the last reply fields from the Valve Master.
     *
     * @param replyOut Receives the reply node, command, and arguments.
     *
     * @return true if all reply registers were read successfully.
     * @return false on I2C failure.
     */
    bool getReply(Reply &replyOut);

    /**
     * @brief Read the Valve Master firmware version.
     *
     * @param versionOut Receives the packed firmware version.
     *
     * @return true if the version registers were read successfully.
     * @return false on I2C failure.
     */
    bool getMasterVersion(FirmwareVersion &versionOut);

    /**
     * @brief Query a valve node for its firmware version.
     *
     * @param node Valve-node address.
     * @param versionOut Receives the packed node firmware version.
     *
     * @return true if the command completed successfully.
     * @return false on invalid input, timeout, command failure, or I2C failure.
     */
    bool getNodeVersion(uint8_t node, FirmwareVersion &versionOut);

    /**
     * @brief Switch on the 12 V valve-node field bus.
     *
     * @return true if the command completed successfully.
     * @return false on timeout, command failure, or I2C failure.
     */
    bool powerOn();

    /**
     * @brief Switch off the 12 V valve-node field bus.
     *
     * @return true if the command completed successfully.
     * @return false on timeout, command failure, or I2C failure.
     */
    bool powerOff();

    /**
     * @brief Ping a valve node.
     *
     * @param node Valve-node address.
     *
     * @return true if the node responded successfully.
     * @return false on invalid node, timeout, node failure, or I2C failure.
     */
    bool pingNode(uint8_t node);

    /**
     * @brief Set a node channel on or off.
     *
     * @param node Valve-node address.
     * @param channel Channel number.
     * @param on true to turn the channel on, false to turn it off.
     *
     * @return true if the command completed successfully.
     * @return false on invalid input, timeout, command failure, or I2C failure.
     */
    bool setChannel(uint8_t node, uint8_t channel, bool on);

    /**
     * @brief Query a node channel state.
     *
     * @param node Valve-node address.
     * @param channel Channel number.
     * @param stateOut Receives the reported channel state.
     *
     * @return true if the command completed successfully.
     * @return false on invalid input, timeout, command failure, or I2C failure.
     */
    bool getChannelStatus(uint8_t node, uint8_t channel, uint8_t &stateOut);

    /**
     * @brief Set a valve output on or off.
     *
     * @param node Valve-node address.
     * @param valveNumber Valve number.
     * @param on true to open or energize, false to close or de-energize.
     *
     * @return true if the command completed successfully.
     * @return false on invalid input, timeout, command failure, or I2C failure.
     */
    bool setValve(uint8_t node, uint8_t valveNumber, bool on);

    /**
     * @brief Broadcast a discovery request and update the firmware node map.
     *
     * @return true if the scan command completed successfully.
     * @return false on timeout, command failure, or I2C failure.
     */
    bool whoScan();

    /**
     * @brief Read the number of discovered valve nodes.
     *
     * @param countOut Receives the node count.
     *
     * @return true if the register read succeeded.
     * @return false on I2C failure.
     */
    bool getNodeCount(uint8_t &countOut);

    /**
     * @brief Read the discovered-node bitmap from the Valve Master.
     *
     * @param nodesOut Caller-provided buffer of NODE_MAP_BYTES bytes.
     * @param countOut Receives the current discovered-node count.
     *
     * @return true if the node map and count were read successfully.
     * @return false on I2C failure.
     */
    bool readNodeMap(uint8_t nodesOut[NODE_MAP_BYTES], uint8_t &countOut);

    /**
     * @brief Ask a node to identify itself.
     *
     * @param node Valve-node address.
     *
     * @return true if the command completed successfully.
     * @return false on invalid node, timeout, command failure, or I2C failure.
     */
    bool identifyNode(uint8_t node);

    /**
     * @brief Cancel the current or pending Valve Master operation.
     *
     * @return true if the cancel command completed successfully.
     * @return false on timeout, command failure, or I2C failure.
     */
    bool cancel();

    /**
     * @brief Put one node, or the system, into configuration mode.
     *
     * @param node Node address, or 0 for firmware-defined default behavior.
     *
     * @return true if the command completed successfully.
     * @return false on timeout, command failure, or I2C failure.
     */
    bool config(uint8_t node = 0);

    /**
     * @brief Assign a new address to a node in configuration mode.
     *
     * @param newNode New valve-node address.
     *
     * @return true if the assignment completed successfully.
     * @return false on invalid address, timeout, address conflict, command failure, or I2C failure.
     */
    bool assign(uint8_t newNode);

    /**
     * @brief Move an existing node from one address to another.
     *
     * @param oldNode Existing valve-node address.
     * @param newNode New valve-node address.
     *
     * @return true if the move completed successfully.
     * @return false on invalid address, timeout, address conflict, command failure, or I2C failure.
     */
    bool moveNode(uint8_t oldNode, uint8_t newNode);

    /**
     * @brief Check whether the Valve Master firmware is busy.
     *
     * @param busyOut Receives true when STATUS_BUSY is set.
     *
     * @return true if status was read successfully.
     * @return false on I2C failure.
     */
    bool isBusy(bool &busyOut);

    /**
     * @brief Wait until the Valve Master firmware clears STATUS_BUSY.
     *
     * @param timeoutMs Maximum wait time in milliseconds.
     *
     * @return true if the firmware became idle before timeout.
     * @return false on timeout or I2C failure.
     */
    bool waitNotBusy(uint32_t timeoutMs);

    /**
     * @brief Clear the Valve Master firmware error state.
     *
     * @return true if the command completed successfully.
     * @return false on timeout, command failure, or I2C failure.
     */
    bool clearError();

    /**
     * @brief Force the Valve Master firmware error state.
     *
     * This is primarily useful for test and diagnostic paths.
     *
     * @return true if the command completed successfully.
     * @return false on timeout, command failure, or I2C failure.
     */
    bool setError();

    /**
     * @brief Check whether a node address is within the legal unicast range.
     *
     * @param node Node address to test.
     *
     * @return true if node is from MIN_NODE_ADDR through MAX_NODE_ADDR.
     * @return false otherwise.
     */
    static bool validNode(uint8_t node);

    /**
     * @brief Check whether a channel number is within the legal range.
     *
     * @param channel Channel number to test.
     *
     * @return true if channel is from MIN_CHANNEL through MAX_CHANNEL.
     * @return false otherwise.
     */
    static bool validChannel(uint8_t channel);

    /**
     * @brief Extract the major version from a packed firmware version.
     *
     * @param version Packed firmware version.
     *
     * @return Major version byte.
     */
    static uint8_t versionMajor(FirmwareVersion version);

    /**
     * @brief Extract the minor version from a packed firmware version.
     *
     * @param version Packed firmware version.
     *
     * @return Minor version byte.
     */
    static uint8_t versionMinor(FirmwareVersion version);

private:

    /**
     * @brief Issue the low-level soft reset path, if supported by the backend.
     *
     * @return true if the reset operation succeeded.
     * @return false on I2C failure or unsupported operation.
     */
    bool softReset();

    /**
     * @brief Write a command byte to REG_COMMAND.
     *
     * @param command Command value to submit.
     *
     * @return true if the register write succeeded.
     * @return false on I2C failure.
     */
    bool writeCommand(uint8_t command);

    /**
     * @brief Read one Valve Master register.
     *
     * @param reg Register address.
     * @param valueOut Receives the register value.
     *
     * @return true if the read succeeded.
     * @return false on I2C failure.
     */
    bool readRegister(uint8_t reg, uint8_t &valueOut);

    /**
     * @brief Write one Valve Master register.
     *
     * @param reg Register address.
     * @param value Register value to write.
     *
     * @return true if the write succeeded.
     * @return false on I2C failure.
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Submit a command and wait for firmware completion.
     *
     * @param command Command value to submit.
     * @param timeoutMs Maximum wait time in milliseconds.
     *
     * @return true if the command completed and RESULT_OK was reported.
     * @return false on timeout, error result, or I2C failure.
     */
    bool submitCommandAndWait(uint8_t command, uint32_t timeoutMs);

    /**
     * @brief Check whether REG_RESULT reports RESULT_OK.
     *
     * @param operation Human-readable operation name used by diagnostic output.
     *
     * @return true if REG_RESULT is RESULT_OK.
     * @return false if REG_RESULT reports an error or cannot be read.
     */
    bool checkResultOk(const char *operation);

    I2C  _i2cPort;  ///< Linux I2C wrapper used for register access.
    bool _isSetup;  ///< true after successful begin(), false after construction or stop().
};

#endif /* Valve_master_hpp */
