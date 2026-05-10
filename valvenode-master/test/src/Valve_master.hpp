#ifndef Valve_master_hpp
#define Valve_master_hpp

#include <stdint.h>
#include <stddef.h>

#include "I2C.hpp"

class Valve_master
{
public:

    static constexpr uint8_t DEFAULT_I2C_ADDR = 0x09;

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
    static constexpr uint8_t CMD_CLOSE_ALL              = 0x0F;

    static constexpr uint8_t STATUS_BUSY        = (1u << 0);
    static constexpr uint8_t STATUS_ERROR       = (1u << 1);
    static constexpr uint8_t STATUS_POWER_ON    = (1u << 2);

    static constexpr uint8_t RESULT_OK                   = 0x00;
    static constexpr uint8_t RESULT_BAD_COMMAND          = 0x01;
    static constexpr uint8_t RESULT_BAD_NODE             = 0x02;
    static constexpr uint8_t RESULT_BAD_CHANNEL          = 0x03;
    static constexpr uint8_t RESULT_NODE_NOT_FOUND       = 0x04;
    static constexpr uint8_t RESULT_UNSUPPORTED_CHANNEL  = 0x05;
    static constexpr uint8_t RESULT_CONFIG_REQUIRED      = 0x06;
    static constexpr uint8_t RESULT_ADDRESS_IN_USE       = 0x07;
    static constexpr uint8_t RESULT_BUSY                 = 0x08;

    static constexpr uint8_t MIN_NODE_ADDR       = 1;
    static constexpr uint8_t MAX_NODE_ADDR       = 254;
    static constexpr uint8_t MIN_CHANNEL         = 1;
    static constexpr uint8_t MAX_CHANNEL         = 16;

    using FirmwareVersion = uint16_t;

    struct Reply {
        uint8_t node;
        uint8_t cmd;
        uint8_t arg0;
        uint8_t arg1;
    };

    Valve_master();
    ~Valve_master();

    bool begin(uint8_t deviceAddress = DEFAULT_I2C_ADDR);
    bool begin(uint8_t deviceAddress, int &error);
    void stop();

    bool isOpen();
    uint8_t getDevAddr();

    bool getStatus(uint8_t &status);
    bool getPowerState(uint8_t &state);
    bool getLastResult(uint8_t &result);
    bool getReply(Reply &replyOut);

    bool getMasterVersion(FirmwareVersion &versionOut);
    bool getNodeVersion(uint8_t node, FirmwareVersion &versionOut);

    bool powerOn();
    bool powerOff();

    bool pingNode(uint8_t node);
    bool setChannel(uint8_t node, uint8_t channel, bool on);
    bool getChannelStatus(uint8_t node, uint8_t channel, uint8_t &stateOut);

    bool setValve(uint8_t node, uint8_t valveNumber, bool on);
    bool closeAll();

    bool whoScan();
    bool getNodeCount(uint8_t &countOut);
    bool readNodeMap(uint8_t nodesOut[NODE_MAP_BYTES], uint8_t &countOut);

    bool identifyNode(uint8_t node);
    bool cancel();
    bool config(uint8_t node = 0);
    bool assign(uint8_t newNode);
    bool moveNode(uint8_t oldNode, uint8_t newNode);

    bool isBusy(bool &busyOut);
    bool waitNotBusy(uint32_t timeoutMs);

    bool clearError();
    bool setError();

    static bool validNode(uint8_t node);
    static bool validChannel(uint8_t channel);
    static uint8_t versionMajor(FirmwareVersion version);
    static uint8_t versionMinor(FirmwareVersion version);

private:

    bool softReset();

    bool writeCommand(uint8_t command);
    bool readRegister(uint8_t reg, uint8_t &valueOut);
    bool writeRegister(uint8_t reg, uint8_t value);

    bool submitCommandAndWait(uint8_t command, uint32_t timeoutMs);
    bool checkResultOk(const char *operation);

    I2C  _i2cPort;
    bool _isSetup;
};

#endif /* Valve_master_hpp */
