#pragma once

#include <cstdint>
#include <map>
#include <string>

#include "pIoTServerDevice.hpp"
#include "pIoTServerSchema.hpp"
#include "I2C.hpp"

using namespace std;

class VALVEMASTER_Device : public pIoTServerDevice {
public:
    VALVEMASTER_Device(string devID, string driverName);

    bool initWithSchema(deviceSchemaMap_t deviceSchema) override;
    bool start() override;
    void stop() override;
    bool isConnected() override;
    bool setEnabled(bool enable) override;

    bool setValues(keyValueMap_t kv) override;
    bool getValues(keyValueMap_t& results) override;
    bool allOff() override;
    bool getVersion(string& version) override;

    /*
     * Lab/test API.
     *
     * These are intentionally not part of pIoTServerDevice.
     * The plugin harness may call these through plugin-side C hooks.
     */
    bool testPowerOn();
    bool testPowerOff();
    bool testProbeBus();
    bool testPingDiscoveredNodes();
    bool testVersionScanDiscoveredNodes();

private:
    struct ValveBinding {
        uint8_t node = 0;
        uint8_t valve = 0;
    };

    bool parseI2CAddress();
    bool loadBindingsFromSchema();
    bool getBindingForKey(const string& key, ValveBinding& bindingOut);

    bool readRegister(uint8_t reg, uint8_t& valueOut);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool writeCommand(uint8_t command);

    bool readMasterSummary();
    bool waitNotBusy(uint32_t timeoutMs);
    bool checkResultOk(const char* operation);

    bool powerOn();
    bool powerOff();
    bool probeBus();

    bool pingNode(uint8_t node);
    bool pingDiscoveredNodes();

    bool getNodeVersion(uint8_t node, uint8_t& versionHiOut, uint8_t& versionLoOut);
    bool versionScanDiscoveredNodes();

    /*
     * Real valve command helpers.
     *
     * setValveChannel() sends CMD_SET_CHANNEL for one node/channel.
     * closeAllValves() sends the Valve Master broadcast close-all command,
     * CMD_CLOSE_ALL, once. It does not loop over schema valves.
     */
    bool setValveChannel(uint8_t node, uint8_t channel, bool on);
    bool closeAllValves();

    deviceSchemaMap_t _schema;

    /*
     * Cached schema state.
     *
     * These values reflect successful commands sent through the Valve Master.
     * They do not prove physical valve position, water flow, or solenoid presence.
     */
    keyValueMap_t _state;

    map<string, ValveBinding> _bindings;

    I2C _i2c;

    uint8_t _i2cAddress = 0x09;
    bool    _isConnected = false;
    bool    _fieldPowerOn = false;

    uint8_t _lastStatus = 0;
    uint8_t _lastResult = 0;
    uint8_t _lastPowerState = 0;
    uint8_t _lastNodeCount = 0;
    uint8_t _versionHi = 0;
    uint8_t _versionLo = 0;
};
