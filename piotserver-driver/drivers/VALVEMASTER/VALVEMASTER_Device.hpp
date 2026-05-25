#pragma once

/**
 * @file VALVEMASTER_Device.hpp
 * @brief pIoTServer device plugin for the Valve Master I2C/RS-485 controller.
 *
 * The VALVEMASTER plugin is the Linux-side device driver for the ATmega88PB
 * Valve Master board. The plugin talks to the Valve Master over I2C. The Valve
 * Master board owns field-bus power, RS-485 direction control, and the actual
 * ValveNode slave protocol.
 *
 * The plugin does not speak RS-485 directly.
 *
 * Driver stack:
 *
 *   pIoTServer
 *     -> VALVEMASTER_Device
 *       -> I2C register interface
 *         -> ATmega88PB Valve Master
 *           -> switched field-bus power
 *           -> RS-485
 *           -> ValveNode slaves
 *
 * Threading model:
 *
 *   Public API calls are fast and cache/queue oriented.
 *
 *   actionThread() owns slow hardware operations:
 *     - field-bus power on/off
 *     - Valve Master I2C command execution
 *     - auto power-off delay
 *
 *   stop() is allowed to block because it must safely stop the thread and
 *   power down the field line.
 *
 * Cached valve state in this driver means "last requested/desired state
 * accepted by the driver." It does not prove physical valve position, water
 * flow, wiring continuity, solenoid presence, or even completed field-bus
 * command execution.
 */

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "pIoTServerDevice.hpp"
#include "pIoTServerSchema.hpp"
#include "I2C.hpp"
#include "CommonDefs.hpp"
#include "pIoTServerMgrCommon.hpp"

using namespace std;

class VALVEMASTER_Device : public pIoTServerDevice {
public:

    static constexpr const char* JSON_ARG_ADDRESS = "address";
    static constexpr const char* JSON_ARG_POWER_HOLD_SEC = "power_hold_sec";
    static constexpr unsigned long DEFAULT_POWER_HOLD_SEC = 60;

    static constexpr const char* VALUE_FIELD_POWER = "field_power";
    static constexpr const char* VALUE_ACTION_BUSY = "action_busy";
    static constexpr const char* VALUE_LAST_ACTION = "last_action";
    static constexpr const char* VALUE_LAST_ACTION_OK = "last_action_ok";
    static constexpr const char* VALUE_DISCOVERED_NODE_COUNT = "discovered_node_count";
    static constexpr const char* VALUE_NODE_VERSIONS = "node_versions";

    struct NodeVersion {
        uint8_t versionHi = 0;
        uint8_t versionLo = 0;
    };

    VALVEMASTER_Device(string devID, string driverName);
    virtual ~VALVEMASTER_Device();

    bool initWithSchema(deviceSchemaMap_t deviceSchema);
    bool start();
    void stop();
    bool isConnected();
    bool setEnabled(bool enable);
    bool setValues(keyValueMap_t kv);
    bool hasUpdates();
    bool getValues(keyValueMap_t& results);
    bool allOff();
    bool getVersion(string& version);

    bool testPowerOn();
    bool testPowerOff();
    bool testProbeBus();
    bool testPingDiscoveredNodes();
    bool testVersionScanDiscoveredNodes();
    bool testWaitForIdle(uint32_t timeoutMs);

private:

    struct ValveBinding {
        uint8_t node = 0;
        uint8_t valve = 0;
    };

    typedef enum
    {
        ACTION_NONE = 0,
        ACTION_SET_VALUES,
        ACTION_CLOSE_ALL,
        ACTION_POWER_ON_TEST,
        ACTION_POWER_OFF_TEST,
        ACTION_PROBE_BUS,
        ACTION_PING_DISCOVERED,
        ACTION_VERSION_SCAN_DISCOVERED
    } action_type_t;

    struct action_request_t
    {
        action_type_t type = ACTION_NONE;
        keyBoolVector_t values;
        boolCallback_t callback = nullptr;
    };

    bool parseI2CAddress();
    bool loadBindingsFromSchema();
    bool getBindingForKey(const string& key, ValveBinding& bindingOut);

    const char* actionName(action_type_t type) const;
    void actionThread();
    bool queueAction(const action_request_t& request, bool clearPendingSetValues = false);
    bool executeAction(const action_request_t& request);
    bool executeSetValuesAction(const action_request_t& request);
    void armPowerHoldTimer();
    void cancelPowerHoldTimer();
    bool delayWithStopCheck(uint32_t delayMs, const char* reason);
    bool waitForIdle(uint32_t timeoutMs);
    void setLastActionStatus(const string& actionName, bool didSucceed);

    bool readRegister(uint8_t reg, uint8_t& valueOut);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool writeCommand(uint8_t command);

    bool readMasterSummary();
    bool waitNotBusy(uint32_t timeoutMs);
    bool checkResultOk(const char* operation);

    bool powerOn();
    bool powerOff();
    bool ensureFieldPowerOn(bool& didPowerOnOut);
    bool settleAfterFieldPowerOnIfNeeded(bool didPowerOn, const char* reason);

    bool probeBus();
    bool pingNode(uint8_t node);
    bool pingDiscoveredNodes();
    bool getNodeVersion(uint8_t node, uint8_t& versionHiOut, uint8_t& versionLoOut);
    bool versionScanDiscoveredNodes();

    bool setValveChannel(uint8_t node, uint8_t channel, bool on);
    bool getValveChannelStatus(uint8_t node, uint8_t channel, bool& onOut);
    bool closeAllValves();

    void updateDiagnosticStateLocked();

    deviceSchemaMap_t _schema;
    keyValueMap_t _state;
    map<string, ValveBinding> _bindings;

    bool _isSetup = false;
    bool _dataDidChange = false;

    mutable std::mutex _mutex;

    I2C _i2c;
    uint8_t _i2cAddress = 0x09;
    unsigned long _powerHoldSec = DEFAULT_POWER_HOLD_SEC;

    bool _isConnected = false;
    bool _fieldPowerOn = false;

    uint8_t _lastStatus = 0;
    uint8_t _lastResult = 0;
    uint8_t _lastPowerState = 0;
    uint8_t _lastNodeCount = 0;
    uint8_t _versionHi = 0;
    uint8_t _versionLo = 0;

    std::vector<uint8_t> _discoveredNodes;
    std::map<uint8_t, NodeVersion> _nodeVersions;

    std::thread _thread;
    std::condition_variable _actionCv;
    std::deque<action_request_t> _actionQueue;

    bool _running = false;
    bool _stopRequested = false;
    bool _actionBusy = false;
    bool _powerHoldActive = false;

    std::chrono::steady_clock::time_point _powerHoldDeadline;

    string _lastActionName;
    bool _lastActionSucceeded = true;
};
