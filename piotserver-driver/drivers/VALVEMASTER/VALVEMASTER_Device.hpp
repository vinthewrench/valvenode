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

#include "pIoTServerDevice.hpp"
#include "pIoTServerSchema.hpp"
#include "I2C.hpp"
#include "CommonDefs.hpp"
#include "pIoTServerMgrCommon.hpp"

using namespace std;

/**
 * @class VALVEMASTER_Device
 * @brief pIoTServer plugin wrapper for the Valve Master board.
 *
 * This class provides the pIoTServer-facing device API and hides the low-level
 * I2C register contract used to command the Valve Master firmware.
 *
 * Schema entries are bound to physical ValveNode outputs by using schema
 * metadata:
 *
 *   - otherProps.node
 *   - otherProps.valve
 *
 * A pIoTServer key therefore maps to a specific node/channel pair without the
 * driver needing to know application-level names such as zones, beds, or crops.
 */
class VALVEMASTER_Device : public pIoTServerDevice {
public:

    /**
     * @name VALVEMASTER property names
     * @{
     */

    /** @brief I2C 7-bit address property, e.g. "0x09". */
    static constexpr const char* JSON_ARG_ADDRESS = "address";

    /** @brief Field-bus auto power-off hold time in seconds. */
    static constexpr const char* JSON_ARG_POWER_HOLD_SEC = "power_hold_sec";

    /** @brief Default field-bus auto power-off hold time in seconds. */
    static constexpr unsigned long DEFAULT_POWER_HOLD_SEC = 60;

    /** @} */


    /**
     * @name VALVEMASTER diagnostic value names
     * @{
     */

    /** @brief Cached field-bus power state, "on" or "off". */
    static constexpr const char* VALUE_FIELD_POWER = "field_power";

    /** @brief Worker-thread busy state, "true" or "false". */
    static constexpr const char* VALUE_ACTION_BUSY = "action_busy";

    /** @brief Last completed action name. */
    static constexpr const char* VALUE_LAST_ACTION = "last_action";

    /** @brief Last completed action result, "true" or "false". */
    static constexpr const char* VALUE_LAST_ACTION_OK = "last_action_ok";

    /** @} */


    /**
     * @name Construction / lifetime
     * @{
     */

    /**
     * @brief Construct a Valve Master plugin instance.
     *
     * The constructor initializes cached driver state and stores the pIoTServer
     * device identity through setDeviceID().
     *
     * @param devID pIoTServer device identifier.
     * @param driverName pIoTServer driver name.
     */
    VALVEMASTER_Device(string devID, string driverName);

    /**
     * @brief Destroy the plugin instance and release runtime resources.
     *
     * The destructor calls stop(), matching the existing driver style used by
     * other pIoTServer plugins.
     */
    virtual ~VALVEMASTER_Device();

    /** @} */


    /**
     * @name pIoTServer device API
     * @{
     */

    /**
     * @brief Initialize the plugin from the pIoTServer schema.
     *
     * This loads schema bindings from pIoTServer keys to Valve Master
     * node/channel pairs.
     *
     * @param deviceSchema Schema map supplied by pIoTServer.
     * @return true if the schema was accepted.
     */
    bool initWithSchema(deviceSchemaMap_t deviceSchema);

    /**
     * @brief Start the plugin.
     *
     * This opens/configures the I2C connection to the Valve Master board, reads
     * initial master summary state, and starts actionThread().
     *
     * @return true if the device started successfully.
     */
    bool start();

    /**
     * @brief Stop the plugin and release the I2C transport.
     *
     * stop() may block. It asks actionThread() to exit, wakes it, waits for it
     * to power down the field line, joins the thread, and then closes I2C.
     */
    void stop();

    /**
     * @brief Report whether the Valve Master transport is currently connected.
     *
     * @return true if the I2C transport is connected.
     */
    bool isConnected();

    /**
     * @brief Enable or disable the plugin.
     *
     * This currently controls whether command paths accept work. Disabling the
     * device does not directly command field power. allOff() and stop() own the
     * immediate safety actions.
     *
     * @param enable true to enable command handling, false to disable it.
     * @return true if the enabled state was accepted.
     */
    bool setEnabled(bool enable);

    /**
     * @brief Queue one or more pIoTServer value changes.
     *
     * Each schema key is mapped to a ValveNode node/channel binding. A true
     * value requests that valve channel open. A false value requests it closed.
     *
     * This method returns after the request is validated, cached, and queued.
     * It does not wait for the field-bus command to complete.
     *
     * @param kv Key/value changes supplied by pIoTServer.
     * @return true if all requested changes were accepted and queued.
     */
    bool setValues(keyValueMap_t kv);

    /**
     * @brief Report whether cached values have changed since the last read.
     *
     * @return true if getValues() should be called to collect updated state.
     */
    bool hasUpdates();

    /**
     * @brief Copy cached driver state into the pIoTServer result map.
     *
     * Cached valve values represent requested/desired state accepted by this
     * driver. They do not prove physical valve state or completed field-bus
     * command execution.
     *
     * Diagnostic values such as VALUE_FIELD_POWER and VALUE_ACTION_BUSY are also
     * returned in the result map.
     *
     * @param results Destination map for pIoTServer-visible values.
     * @return true if values were copied.
     */
    bool getValues(keyValueMap_t& results);

    /**
     * @brief Queue a close-all operation through the Valve Master.
     *
     * This clears pending set-value actions, marks all cached schema values off,
     * queues one hardware close-all command, and returns immediately.
     *
     * actionThread() performs the actual close-all command and then powers down
     * the field bus immediately.
     *
     * @return true if the close-all request was accepted.
     */
    bool allOff();

    /**
     * @brief Return the plugin version string.
     *
     * @param version Destination string.
     * @return true if the version string was supplied.
     */
    bool getVersion(string& version);

    /** @} */


    /**
     * @name Hardware bring-up / lab test API
     *
     * These methods are intentionally not part of pIoTServerDevice. The
     * standalone plugin harness may call these through plugin-side C hooks
     * exported by VALVEMASTER_factory.cpp.
     *
     * In the threaded driver, these calls queue the requested operation and
     * return once accepted. They do not necessarily wait for hardware completion.
     *
     * @{
     */

    /** @brief Queue field-bus power-on test behavior. */
    bool testPowerOn();

    /** @brief Queue field-bus power-off test behavior. */
    bool testPowerOff();

    /** @brief Queue Valve Master WHO/node-discovery behavior. */
    bool testProbeBus();

    /** @brief Queue ping of nodes discovered by the Valve Master. */
    bool testPingDiscoveredNodes();

    /** @brief Queue firmware-version query of discovered nodes. */
    bool testVersionScanDiscoveredNodes();

    /** @} */

private:

    /**
     * @struct ValveBinding
     * @brief Maps a pIoTServer schema key to a ValveNode node/channel pair.
     */
    struct ValveBinding {
        uint8_t node = 0;
        uint8_t valve = 0;
    };

    /**
     * @brief Action type executed by actionThread().
     */
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

    /**
     * @struct action_request_t
     * @brief One queued asynchronous driver action.
     *
     * ACTION_SET_VALUES uses values to preserve the ordered list of key/state
     * requests from setValues().
     *
     * callback is optional. It is included now to match the existing codebase
     * style and to support later lab/WHO/version closure work without changing
     * the queue shape again.
     */
    struct action_request_t
    {
        action_type_t type = ACTION_NONE;

        /** @brief Ordered key/state requests used by ACTION_SET_VALUES. */
        keyBoolVector_t values;

        /** @brief Optional completion callback. */
        boolCallback_t callback = nullptr;
    };


    /**
     * @name Schema binding helpers
     * @{
     */

    /**
     * @brief Parse runtime configuration from device properties.
     *
     * Current top-level properties:
     *
     *   - JSON_ARG_ADDRESS
     *   - JSON_ARG_POWER_HOLD_SEC
     *
     * @return true if configured runtime properties are valid.
     */
    bool parseI2CAddress();

    /**
     * @brief Load schema key to node/channel bindings.
     *
     * Read-only schema entries are ignored by the binding loader, so diagnostic
     * values may be exposed without requiring node/valve metadata.
     *
     * @return true if all required bindings were loaded successfully.
     */
    bool loadBindingsFromSchema();

    /**
     * @brief Look up the ValveNode binding for a pIoTServer key.
     *
     * @param key pIoTServer schema/result key.
     * @param bindingOut Destination binding.
     * @return true if the key has a binding.
     */
    bool getBindingForKey(const string& key, ValveBinding& bindingOut);

    /** @} */


    /**
     * @name Action thread helpers
     * @{
     */

    /**
     * @brief Return a readable action name.
     *
     * @param type Action type.
     * @return Static action name string.
     */
    const char* actionName(action_type_t type) const;

    /**
     * @brief Main worker thread body.
     *
     * actionThread() is the only path that should execute slow field-bus
     * hardware work after start() completes.
     */
    void actionThread();

    /**
     * @brief Queue one asynchronous action.
     *
     * @param request Action to queue.
     * @param clearPendingSetValues true to remove pending ACTION_SET_VALUES
     *        before queuing this request.
     * @return true if the action was accepted.
     */
    bool queueAction(const action_request_t& request, bool clearPendingSetValues = false);

    /**
     * @brief Execute one queued action.
     *
     * This runs on actionThread().
     *
     * @param request Action request to execute.
     * @return true if the hardware action succeeded.
     */
    bool executeAction(const action_request_t& request);

    /**
     * @brief Execute ACTION_SET_VALUES on actionThread().
     *
     * @param request Queued set-values request.
     * @return true if all requested hardware commands completed successfully.
     */
    bool executeSetValuesAction(const action_request_t& request);

    /**
     * @brief Arm the automatic field-bus power-off delay.
     *
     * Called by actionThread() after normal field-bus activity.
     */
    void armPowerHoldTimer();

    /**
     * @brief Cancel the automatic field-bus power-off delay.
     */
    void cancelPowerHoldTimer();

    /**
     * @brief Return true if the automatic field-bus power-off deadline expired.
     *
     * @return true if the hold timer expired.
     */
    bool powerHoldExpired();

    /**
     * @brief Update cached last-action status and diagnostic result values.
     *
     * @param actionName Human-readable action name.
     * @param didSucceed true if the action succeeded.
     */
    void setLastActionStatus(const string& actionName, bool didSucceed);

    /** @} */


    /**
     * @name I2C register helpers
     *
     * These helpers talk directly to the Valve Master over I2C. After the
     * threaded driver starts, they should only be called by actionThread() or
     * by start() before actionThread() is launched.
     *
     * @{
     */

    /**
     * @brief Read one Valve Master register.
     *
     * @param reg Register address.
     * @param valueOut Destination byte.
     * @return true if the read succeeded.
     */
    bool readRegister(uint8_t reg, uint8_t& valueOut);

    /**
     * @brief Write one Valve Master register.
     *
     * @param reg Register address.
     * @param value Value to write.
     * @return true if the write succeeded.
     */
    bool writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Write the command register.
     *
     * Command arguments must be written before this call.
     *
     * @param command Command byte.
     * @return true if the command byte was written.
     */
    bool writeCommand(uint8_t command);

    /** @} */


    /**
     * @name Valve Master command helpers
     *
     * These helpers are thread-owned after start() launches actionThread().
     *
     * @{
     */

    /**
     * @brief Read cached summary registers from the Valve Master.
     *
     * @return true if the summary registers were read.
     */
    bool readMasterSummary();

    /**
     * @brief Wait until the Valve Master firmware clears STATUS_BUSY.
     *
     * @param timeoutMs Maximum wait time in milliseconds.
     * @return true if the Valve Master became idle before timeout.
     */
    bool waitNotBusy(uint32_t timeoutMs);

    /**
     * @brief Check REG_RESULT after an operation.
     *
     * @param operation Human-readable operation name for logging.
     * @return true if REG_RESULT reports success.
     */
    bool checkResultOk(const char* operation);

    /** @} */


    /**
     * @name Field power helpers
     *
     * These helpers are thread-owned after actionThread() starts.
     *
     * @{
     */

    /**
     * @brief Command field-bus power on.
     *
     * Updates VALUE_FIELD_POWER in the cached result map.
     *
     * @return true if the Valve Master reported success.
     */
    bool powerOn();

    /**
     * @brief Command field-bus power off.
     *
     * Updates VALUE_FIELD_POWER in the cached result map.
     *
     * @return true if the Valve Master reported success.
     */
    bool powerOff();

    /** @} */


    /**
     * @name Node discovery helpers
     *
     * These helpers are thread-owned after actionThread() starts.
     *
     * @{
     */

    /**
     * @brief Ask the Valve Master to discover responding ValveNode slaves.
     *
     * @return true if the Valve Master completed discovery successfully.
     */
    bool probeBus();

    /**
     * @brief Ping one ValveNode through the Valve Master.
     *
     * @param node ValveNode address.
     * @return true if the node replied successfully.
     */
    bool pingNode(uint8_t node);

    /**
     * @brief Ping all nodes currently reported by the Valve Master node map.
     *
     * @return true if all discovered nodes replied successfully.
     */
    bool pingDiscoveredNodes();

    /**
     * @brief Query a ValveNode firmware version through the Valve Master.
     *
     * @param node ValveNode address.
     * @param versionHiOut Destination major/high version byte.
     * @param versionLoOut Destination minor/low version byte.
     * @return true if the query succeeded.
     */
    bool getNodeVersion(uint8_t node, uint8_t& versionHiOut, uint8_t& versionLoOut);

    /**
     * @brief Query firmware versions for discovered ValveNode slaves.
     *
     * @return true if all version queries succeeded.
     */
    bool versionScanDiscoveredNodes();

    /** @} */


    /**
     * @name Valve command helpers
     *
     * These helpers are thread-owned after actionThread() starts.
     *
     * @{
     */

    /**
     * @brief Command one ValveNode channel open or closed.
     *
     * setValveChannel() sends CMD_SET_CHANNEL for one node/channel.
     *
     * @param node ValveNode address.
     * @param channel Valve channel number on the node.
     * @param on true to open, false to close.
     * @return true if the Valve Master reported command success.
     */
    bool setValveChannel(uint8_t node, uint8_t channel, bool on);

    /**
     * @brief Command all valves closed using the Valve Master broadcast command.
     *
     * closeAllValves() sends CMD_CLOSE_ALL once. It does not loop over schema
     * valves in the plugin.
     *
     * @return true if the Valve Master reported command success.
     */
    bool closeAllValves();

    /** @} */


    /**
     * @name Diagnostic state helpers
     * @{
     */

    /**
     * @brief Refresh cached diagnostic values in _state.
     *
     * Caller must already hold _mutex.
     *
     * Diagnostic values are stored in the same _state map returned by
     * getValues(), alongside schema-backed valve values.
     *
     * Updated keys:
     *   - VALUE_FIELD_POWER
     *   - VALUE_ACTION_BUSY
     *   - VALUE_LAST_ACTION
     *   - VALUE_LAST_ACTION_OK
     */
    void updateDiagnosticStateLocked();

    /** @} */


    /**
     * @name Schema and cached pIoTServer state
     * @{
     */

    /** @brief Schema supplied by pIoTServer. */
    deviceSchemaMap_t _schema;

    /**
     * @brief Cached pIoTServer-visible desired and diagnostic state.
     *
     * Schema keys reflect requested state accepted by the driver. They do not
     * prove physical valve position, water flow, wiring continuity, solenoid
     * presence, or completed field-bus command execution.
     *
     * Diagnostic keys include:
     *   - VALUE_FIELD_POWER
     *   - VALUE_ACTION_BUSY
     *   - VALUE_LAST_ACTION
     *   - VALUE_LAST_ACTION_OK
     */
    keyValueMap_t _state;

    /** @brief Map from pIoTServer key to ValveNode node/channel binding. */
    map<string, ValveBinding> _bindings;

    /** @brief True after initWithSchema() has accepted schema bindings. */
    bool _isSetup = false;

    /** @brief True when cached state changed since getValues() last ran. */
    bool _dataDidChange = false;

    /** @} */


    /**
     * @name Transport and Valve Master cached state
     * @{
     */

    /** @brief Mutex protecting queue, cached state, and connection state. */
    mutable std::mutex _mutex;

    /** @brief I2C transport wrapper. */
    I2C _i2c;

    /** @brief Valve Master I2C address. Defaults to 0x09. */
    uint8_t _i2cAddress = 0x09;

    /**
     * @brief Seconds to keep field-bus power on after command activity.
     *
     * The default is DEFAULT_POWER_HOLD_SEC. The harness may override this to
     * a shorter value for timeout testing.
     */
    unsigned long _powerHoldSec = DEFAULT_POWER_HOLD_SEC;

    /** @brief True when the I2C transport is connected. */
    bool _isConnected = false;

    /** @brief Last known field-bus power state. */
    bool _fieldPowerOn = false;

    /** @brief Last value read from REG_STATUS. */
    uint8_t _lastStatus = 0;

    /** @brief Last value read from REG_RESULT. */
    uint8_t _lastResult = 0;

    /** @brief Last value read from REG_POWER_STATE. */
    uint8_t _lastPowerState = 0;

    /** @brief Last value read from REG_NODE_COUNT. */
    uint8_t _lastNodeCount = 0;

    /** @brief Last value read from REG_VERSION_HI. */
    uint8_t _versionHi = 0;

    /** @brief Last value read from REG_VERSION_LO. */
    uint8_t _versionLo = 0;

    /** @} */


    /**
     * @name Action thread state
     * @{
     */

    /** @brief Worker thread that owns slow Valve Master hardware actions. */
    std::thread _thread;

    /** @brief Condition variable used to wake actionThread(). */
    std::condition_variable _actionCv;

    /** @brief FIFO queue of pending field-bus actions. */
    std::deque<action_request_t> _actionQueue;

    /** @brief True while actionThread() should keep running. */
    bool _running = false;

    /** @brief True when stop() has requested actionThread() shutdown. */
    bool _stopRequested = false;

    /** @brief True when actionThread() is executing a slow hardware operation. */
    bool _actionBusy = false;

    /** @brief True when a field-power auto-off deadline is active. */
    bool _powerHoldActive = false;

    /** @brief Deadline for automatic field-bus power-off. */
    std::chrono::steady_clock::time_point _powerHoldDeadline;

    /** @brief Last completed action name for diagnostics/logging. */
    string _lastActionName;

    /** @brief Last completed action result. */
    bool _lastActionSucceeded = true;

    /** @} */
};
