#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <dlfcn.h>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "pIoTServerDevice.hpp"
#include "pIoTServerSchema.hpp"
#include "LogMgr.hpp"
#include "VALVEMASTER_Device.hpp"

using namespace std;

/*
 * Test harness revision.
 *
 * This is the standalone plugin_harness revision, not the VALVEMASTER plugin
 * version, not the Valve Master firmware version, and not the ValveNode slave
 * firmware version.
 */
static constexpr const char* HARNESS_REVISION = "1.1";


/* ============================================================================
 * Global flags
 * ========================================================================== */

int gVerbose_flag = 0;
int gDebug_flag   = 0;
int gPrint_flag   = 0;
int gQuiet_flag   = 0;

/* ============================================================================
 * Types / constants
 * ========================================================================== */

using factory_fn_t = pIoTServerDevice* (*)(std::string devID, std::string driverName);
using test_hook_fn_t = bool (*)(pIoTServerDevice* dev);
using wait_idle_hook_fn_t = bool (*)(pIoTServerDevice* dev, uint32_t timeoutMs);

/*
 * Field-bus startup settle time.
 *
 * This is deliberately generous for the harness. After field power comes on,
 * each slave AVR has to:
 *
 *   - power up
 *   - initialize GPIO
 *   - initialize UART
 *   - enter RS-485 receive mode
 *   - load EEPROM identity
 *   - finish startup LED blink behavior
 *   - be ready for broadcast traffic
 *
 * If WHO misses nodes, increase this first.
 */
static constexpr uint32_t SLAVE_WAKE_SETTLE_MS = 5000;

/*
 * Settling time after the startup close-all broadcast.
 *
 * The startup allOff() is a real field-bus command. Give every node time to
 * receive it, pulse local latching solenoids, and return to receive mode before
 * starting WHO discovery.
 *
 * If WHO is still missing nodes after increasing SLAVE_WAKE_SETTLE_MS, increase
 * this next.
 */
static constexpr uint32_t STARTUP_ALLOFF_SETTLE_MS = 3000;

/*
 * Gap between real valve actions.
 *
 * This gives the latching solenoid pulse, VNH driver, wiring, and water system
 * a little breathing room between commands. It also makes the test easier to
 * watch in logs and in the field.
 */
static constexpr uint32_t VALVE_ACTION_GAP_MS = 2000;

/*
 * Harness field-power hold time.
 *
 * The production driver default is 60 seconds. The harness deliberately
 * overrides it to 10 seconds so auto power-off can be verified during a normal
 * test run.
 */
static constexpr uint32_t HARNESS_POWER_HOLD_SEC = 10;

/*
 * Wait long enough for the driver's auto power-off timer to expire.
 *
 * This must be longer than HARNESS_POWER_HOLD_SEC. The extra two seconds allow
 * for scheduling jitter, logging, and the actual Valve Master power-off command.
 */
static constexpr uint32_t AUTO_POWER_OFF_VERIFY_WAIT_MS =
    (HARNESS_POWER_HOLD_SEC * 1000u) + 2000u;

/*
 * Maximum time to wait for ordinary queued async driver actions to finish.
 *
 * This is enough for normal set-channel, power-on, power-off, and close-all
 * actions.
 */
static constexpr uint32_t ACTION_IDLE_TIMEOUT_MS = 20000;

/*
 * Maximum time to wait for WHO/version style bus discovery actions.
 *
 * The driver may now do:
 *
 *   - 5 sec field power-on settle
 *   - 3 WHO passes
 *   - retry gaps between passes
 *
 * So 20 seconds is too short for discovery.
 */
static constexpr uint32_t ACTION_DISCOVERY_IDLE_TIMEOUT_MS = 45000;

/* ============================================================================
 * Logging / fatal helpers
 * ========================================================================== */

/**
 * @brief Fatal error helper.
 *
 * If LogMgr terminal printing is enabled, report through LOGT_ERROR.
 * Otherwise use stderr so usage/build failures are still visible.
 */
static void die(const string& msg)
{
    if(gPrint_flag) {
        LOGT_ERROR("ERROR: %s", msg.c_str());
    }
    else {
        fprintf(stderr, "ERROR: %s\n", msg.c_str());
    }

    exit(1);
}

/**
 * @brief Progress note helper.
 *
 * Harness progress is intentionally separate from driver debug logging.
 * With -p, progress goes through LogMgr so timestamps line up with plugin logs.
 */
static void note(const string& msg)
{
    if(gQuiet_flag) {
        return;
    }

    if(gPrint_flag) {
        LOGT_INFO("%s", msg.c_str());
    }
}

/**
 * @brief Wait between ordinary valve actions.
 */
static void wait_between_valve_actions()
{
    note("Waiting before next valve action.");
    this_thread::sleep_for(chrono::milliseconds(VALVE_ACTION_GAP_MS));
}

/**
 * @brief Wait after startup allOff() before bus discovery.
 *
 * This is intentionally separate from the ordinary valve-action gap because
 * startup close-all happens before WHO discovery.
 *
 * If nodes are missed during WHO, this delay is one of the first things to
 * increase after the slave wake settle time.
 */
static void wait_after_startup_alloff()
{
    note("Waiting after startup allOff before bus discovery.");
    this_thread::sleep_for(chrono::milliseconds(STARTUP_ALLOFF_SETTLE_MS));
}

/**
 * @brief Wait long enough to verify actionThread auto power-off behavior.
 *
 * The driver should log the auto power-off expiration and field-power-off
 * command during this idle period.
 */
static void wait_for_auto_power_off_check()
{
    note("Waiting to verify auto power-off hold timer.");
    this_thread::sleep_for(chrono::milliseconds(AUTO_POWER_OFF_VERIFY_WAIT_MS));
}

/* ============================================================================
 * Argument parsing
 * ========================================================================== */

static void usage(const char* progname)
{
    cerr << "usage: " << progname << " [-q] [-p] [-v|-d] <plugin-path>" << endl;
    cerr << "  -q, --quiet     suppress harness progress output" << endl;
    cerr << "  -p, --print     enable LogMgr terminal printing" << endl;
    cerr << "  -d, --debug     enable debug logging" << endl;
    cerr << "  -v, --verbose   enable verbose logging" << endl;
}

static void parse_args(int argc, char* argv[], string& pluginPath)
{
    for(int i = 1; i < argc; i++) {
        string arg = argv[i];

        if(arg == "-q" || arg == "--quiet") {
            gQuiet_flag++;
        }
        else if(arg == "-p" || arg == "--print") {
            gPrint_flag++;
        }
        else if(arg == "-v" || arg == "--verbose") {
            gVerbose_flag++;
        }
        else if(arg == "-d" || arg == "--debug") {
            gDebug_flag++;
        }
        else if(arg == "-h" || arg == "--help") {
            usage(argv[0]);
            exit(0);
        }
        else if(pluginPath.empty()) {
            pluginPath = arg;
        }
        else {
            cerr << "unexpected argument: " << arg << endl;
            usage(argv[0]);
            exit(1);
        }
    }

    if(pluginPath.empty()) {
        usage(argv[0]);
        exit(1);
    }
}

/* ============================================================================
 * LogMgr setup
 * ========================================================================== */

static void setup_logging()
{
    if(gPrint_flag) {
        START_LOGPRINT;
    }

    if(gVerbose_flag) {
        LogMgr::shared()->_logFlags = LogMgr::LogLevelVerbose;
    }
    else if(gDebug_flag) {
        LogMgr::shared()->_logFlags = LogMgr::LogLevelDebug;
    }
}

/* ============================================================================
 * Test schema / properties
 * ========================================================================== */

/**
 * @brief Default schema file used by the harness.
 *
 * make run is normally launched from piotserver-driver, so this path is
 * relative to that directory.
 */
static constexpr const char* DEFAULT_SCHEMA_FILE = "test/valvemaster_test_schema.json";

/**
 * @brief Ordered schema keys loaded from the JSON schema file.
 *
 * deviceSchemaMap_t is ordered by key, which is not the order we want for the
 * field test. This vector preserves the JSON file order for command cycling.
 */
static vector<string> gSchemaKeys;

/**
 * @brief Read a JSON file from disk.
 *
 * @param path JSON file path.
 * @return Parsed JSON object.
 */
static nlohmann::json read_json_file_or_die(const string& path)
{
    ifstream input(path);

    if(!input.is_open()) {
        die("cannot open schema file: " + path);
    }

    nlohmann::json j;

    try {
        input >> j;
    }
    catch(const std::exception& e) {
        die("cannot parse schema file " + path + ": " + e.what());
    }

    return j;
}

/**
 * @brief Extract a required string from a JSON object.
 */
static string json_required_string_or_die(const nlohmann::json& j,
                                          const string& key,
                                          const string& context)
{
    if(!j.contains(key) || !j[key].is_string()) {
        die(context + " missing string field '" + key + "'");
    }

    return j[key].get<string>();
}

/**
 * @brief Extract a required unsigned byte from a JSON object.
 */
static unsigned int json_required_u8_or_die(const nlohmann::json& j,
                                            const string& key,
                                            const string& context)
{
    if(!j.contains(key) || !j[key].is_number_unsigned()) {
        die(context + " missing unsigned numeric field '" + key + "'");
    }

    unsigned int value = j[key].get<unsigned int>();

    if(value > 255u) {
        die(context + " field '" + key + "' exceeds 255");
    }

    return value;
}

/**
 * @brief Build a VALVEMASTER schema for harness testing from JSON.
 *
 * The JSON file supplies only the test mapping data: key, title, node, and
 * valve. The harness still sets the actual pIoTServer schema constants here,
 * using the real names from pIoTServerSchema.hpp: BOOL and TR_IGNORE.
 *
 * This avoids inventing enum/type names that do not exist in the codebase.
 */
static deviceSchemaMap_t make_valvemaster_schema()
{
    const string schemaPath = DEFAULT_SCHEMA_FILE;
    nlohmann::json root = read_json_file_or_die(schemaPath);

    if(!root.contains("values") || !root["values"].is_array()) {
        die(schemaPath + " missing array field 'values'");
    }

    deviceSchemaMap_t schema;
    gSchemaKeys.clear();

    for(size_t index = 0; index < root["values"].size(); index++) {
        const nlohmann::json& item = root["values"][index];
        const string context = schemaPath + " values[" + to_string(index) + "]";

        const string key = json_required_string_or_die(item, "key", context);
        const string title = json_required_string_or_die(item, "title", context);
        const unsigned int node = json_required_u8_or_die(item, "node", context);
        const unsigned int valve = json_required_u8_or_die(item, "valve", context);

        if(schema.find(key) != schema.end()) {
            die(context + " duplicate key '" + key + "'");
        }

        deviceSchema_t entry;
        entry.title = title;
        entry.units = BOOL;
        entry.tracking = TR_IGNORE;
        entry.readOnly = false;
        entry.queryDelay = 5;
        entry.otherProps = {
            {"node", node},
            {"valve", valve}
        };

        schema[key] = entry;
        gSchemaKeys.push_back(key);
    }

    if(schema.empty()) {
        die(schemaPath + " contains no schema values");
    }

    note("Loaded schema file: " + schemaPath);
    return schema;
}

/**
 * @brief Return the ordered schema keys used by this harness.
 *
 * The order comes from the JSON schema file, not from deviceSchemaMap_t.
 */
static vector<string> make_ordered_schema_keys()
{
    if(gSchemaKeys.empty()) {
        die("schema key list is empty; make_valvemaster_schema() was not called");
    }

    return gSchemaKeys;
}

/**
 * @brief Build plugin properties used by the harness.
 *
 * The address property is parsed by VALVEMASTER_Device::parseI2CAddress().
 */
static nlohmann::json make_valvemaster_props()
{
    nlohmann::json props;

    props[VALVEMASTER_Device::JSON_ARG_ADDRESS] = "0x09";
    props["device_type"] = "VALVEMASTER";
    props["title"] = "Valve Master I2C";
    props["interval"] = 5;

    /*
     * The driver default is 60 seconds. The harness deliberately sets this
     * lower so a normal test run can prove that actionThread powers the field
     * line off after the idle hold delay expires.
     */
    props[VALVEMASTER_Device::JSON_ARG_POWER_HOLD_SEC] = HARNESS_POWER_HOLD_SEC;

    return props;
}

/* ============================================================================
 * Value / property display
 * ========================================================================== */

static void print_values(const keyValueMap_t& values)
{
    if(gQuiet_flag || !gPrint_flag) {
        return;
    }

    for(const auto& [key, value] : values) {
        LOGT_INFO("  %s = %s", key.c_str(), value.c_str());
    }
}

/**
 * @brief Print plugin/device properties returned by pIoTServerDevice::getProperties().
 *
 * This verifies what properties are actually visible to the driver before and
 * after the harness calls setProperties().
 */
static void print_properties_or_die(pIoTServerDevice* dev,
                                    void* handle,
                                    const string& header)
{
    (void)handle;

    nlohmann::json props;

    dev->getProperties(props);

    note(header);

    if(gQuiet_flag || !gPrint_flag) {
        return;
    }

    LOGT_INFO("%s", props.dump(4).c_str());
}

/* ============================================================================
 * Plugin symbol loading
 * ========================================================================== */

/**
 * @brief Load one VALVEMASTER lab/test hook from the plugin.
 *
 * These hooks are exported by VALVEMASTER_factory.cpp. They let this harness
 * test hardware bring-up functions that are not part of pIoTServerDevice.
 */
static test_hook_fn_t load_test_hook(void* handle, const char* symbol)
{
    dlerror();

    auto hook = reinterpret_cast<test_hook_fn_t>(dlsym(handle, symbol));
    const char* sym_error = dlerror();

    if(sym_error) {
        die(sym_error);
    }

    if(hook == nullptr) {
        die(string("test hook not found: ") + symbol);
    }

    return hook;
}

/**
 * @brief Load the VALVEMASTER wait-for-idle lab hook from the plugin.
 */
static wait_idle_hook_fn_t load_wait_idle_hook(void* handle, const char* symbol)
{
    dlerror();

    auto hook = reinterpret_cast<wait_idle_hook_fn_t>(dlsym(handle, symbol));
    const char* sym_error = dlerror();

    if(sym_error) {
        die(sym_error);
    }

    if(hook == nullptr) {
        die(string("test hook not found: ") + symbol);
    }

    return hook;
}

/**
 * @brief Wait for the driver's actionThread queue to drain.
 */
static void wait_for_driver_idle_or_die(pIoTServerDevice* dev,
                                        wait_idle_hook_fn_t testWaitForIdle,
                                        test_hook_fn_t testPowerOff,
                                        void* handle,
                                        const string& description,
                                        uint32_t timeoutMs = ACTION_IDLE_TIMEOUT_MS)
{
    note(description);

    if(testWaitForIdle(dev, timeoutMs)) {
        return;
    }

    testPowerOff(dev);
    delete dev;
    dlclose(handle);
    die("timed out waiting for VALVEMASTER actionThread idle: " + description);
}

/* ============================================================================
 * Device value helpers
 * ========================================================================== */

/**
 * @brief Read and log the VALVEMASTER plugin/driver version string.
 *
 * This exercises pIoTServerDevice::getVersion().
 *
 * Important:
 *   getVersion() is the driver/plugin version API.
 *   It is not the Valve Master firmware version.
 *   It is not the RS-485 slave ValveNode firmware version.
 *
 * This should work before dev->start(), because it should not require I2C
 * hardware access.
 */
static void print_driver_version_or_die(pIoTServerDevice* dev,
                                        void* handle)
{
    string version;

    if(dev->getVersion(version)) {
        note(string("Driver version: ") + version);
        return;
    }

    delete dev;
    dlclose(handle);
    die("getVersion() failed");
}

/**
 * @brief Print the standalone test harness revision.
 *
 * This identifies the test harness itself, separate from:
 *
 *   - VALVEMASTER plugin/driver version
 *   - Valve Master ATmega88PB firmware version
 *   - ValveNode slave firmware versions
 */
static void print_harness_revision()
{
    note(string("Test harness revision: ") + HARNESS_REVISION);
}

/**
 * @brief Read and print all cached schema and diagnostic values.
 */
static void print_values_or_die(pIoTServerDevice* dev,
                                test_hook_fn_t testPowerOff,
                                void* handle,
                                const string& header)
{
    keyValueMap_t values;

    if(dev->getValues(values)) {
        note(header);
        print_values(values);
        return;
    }

    testPowerOff(dev);
    delete dev;
    dlclose(handle);
    die("getValues() failed");
}

/**
 * @brief Read one cached value from getValues().
 *
 * @param dev Device instance.
 * @param key Value key.
 * @param valueOut Destination string.
 * @return true if getValues() succeeded and the key exists.
 */
static bool get_value(pIoTServerDevice* dev,
                      const string& key,
                      string& valueOut)
{
    keyValueMap_t values;

    if(!dev->getValues(values)) {
        return false;
    }

    auto it = values.find(key);
    if(it == values.end()) {
        return false;
    }

    valueOut = it->second;
    return true;
}

/**
 * @brief Verify the driver's cached field_power diagnostic value.
 *
 * @param dev Device instance.
 * @param testPowerOff Power-off hook used for cleanup on failure.
 * @param handle dlopen() handle.
 * @param expected Expected value, normally "on" or "off".
 * @param description Log description.
 */
static void verify_field_power_or_die(pIoTServerDevice* dev,
                                      test_hook_fn_t testPowerOff,
                                      void* handle,
                                      const string& expected,
                                      const string& description)
{
    string value;

    if(!get_value(dev, VALVEMASTER_Device::VALUE_FIELD_POWER, value)) {
        testPowerOff(dev);
        delete dev;
        dlclose(handle);
        die(string("missing ") + VALVEMASTER_Device::VALUE_FIELD_POWER + " value");
    }

    note(description + ": " + VALVEMASTER_Device::VALUE_FIELD_POWER + " = " + value);

    if(value != expected) {
        testPowerOff(dev);
        delete dev;
        dlclose(handle);
        die(string("expected ") +
            VALVEMASTER_Device::VALUE_FIELD_POWER +
            "=" +
            expected +
            " but got " +
            value);
    }
}

/**
 * @brief Set a single schema key through pIoTServerDevice::setValues().
 *
 * This exercises the actual driver schema binding path:
 *
 *   schema key -> otherProps.node/valve -> queued ACTION_SET_VALUES
 *   actionThread -> CMD_SET_CHANNEL
 */
static void set_one_value_or_die(pIoTServerDevice* dev,
                                 test_hook_fn_t testPowerOff,
                                 void* handle,
                                 const string& key,
                                 const string& value,
                                 const string& description)
{
    keyValueMap_t kv;
    kv[key] = value;

    note(description);

    if(dev->setValues(kv)) {
        return;
    }

    testPowerOff(dev);
    delete dev;
    dlclose(handle);
    die("setValues(" + key + " " + value + ") failed");
}

/**
 * @brief Cycle every schema-backed valve on and then off.
 *
 * This exercises the pIoTServerDevice::setValues() queue path for every schema
 * key. The command returns immediately after validation and queueing. The
 * harness waits for actionThread idle after each request so the printed values
 * and logs line up with completed hardware actions.
 */
static void cycle_each_schema_valve_or_die(pIoTServerDevice* dev,
                                           wait_idle_hook_fn_t testWaitForIdle,
                                           test_hook_fn_t testPowerOff,
                                           void* handle,
                                           const vector<string>& keys)
{
    for(const auto& key : keys) {
        set_one_value_or_die(dev,
                             testPowerOff,
                             handle,
                             key,
                             "on",
                             "Turning " + key + " on.");

        wait_for_driver_idle_or_die(dev,
                                    testWaitForIdle,
                                    testPowerOff,
                                    handle,
                                    "Waiting for " + key + " on action to complete.");

        print_values_or_die(dev,
                            testPowerOff,
                            handle,
                            "After " + key + " on:");

        wait_between_valve_actions();

        set_one_value_or_die(dev,
                             testPowerOff,
                             handle,
                             key,
                             "off",
                             "Turning " + key + " off.");

        wait_for_driver_idle_or_die(dev,
                                    testWaitForIdle,
                                    testPowerOff,
                                    handle,
                                    "Waiting for " + key + " off action to complete.");

        print_values_or_die(dev,
                            testPowerOff,
                            handle,
                            "After " + key + " off:");

        wait_between_valve_actions();
    }
}

/**
 * @brief Turn every schema-backed valve on before the close-all test.
 *
 * This deliberately uses setValues() once per key so the log shows each schema
 * command clearly. The harness waits for each queued hardware action to finish
 * before sending the next one. After this, the harness calls allOff(), which
 * must send one hardware CMD_CLOSE_ALL broadcast rather than individually
 * closing keys.
 */
static void turn_all_schema_valves_on_or_die(pIoTServerDevice* dev,
                                             wait_idle_hook_fn_t testWaitForIdle,
                                             test_hook_fn_t testPowerOff,
                                             void* handle,
                                             const vector<string>& keys)
{
    for(const auto& key : keys) {
        set_one_value_or_die(dev,
                             testPowerOff,
                             handle,
                             key,
                             "on",
                             "Turning " + key + " on for all-off command test.");

        wait_for_driver_idle_or_die(dev,
                                    testWaitForIdle,
                                    testPowerOff,
                                    handle,
                                    "Waiting for " + key + " all-off setup action to complete.");

        print_values_or_die(dev,
                            testPowerOff,
                            handle,
                            "After " + key + " on for all-off test:");

        wait_between_valve_actions();
    }
}

/**
 * @brief Call pIoTServerDevice::allOff().
 *
 * This must exercise the VALVEMASTER close-all path:
 *
 *   dev->allOff()
 *     -> VALVEMASTER_Device::allOff()
 *     -> queued ACTION_CLOSE_ALL
 *     -> actionThread
 *     -> closeAllValves()
 *     -> one CMD_CLOSE_ALL command
 *
 * It must not close schema valves one by one.
 */
static void all_off_or_die(pIoTServerDevice* dev,
                           test_hook_fn_t testPowerOff,
                           void* handle)
{
    note("Calling hardware allOff().");

    if(dev->allOff()) {
        return;
    }

    testPowerOff(dev);
    delete dev;
    dlclose(handle);
    die("allOff() failed");
}

/* ============================================================================
 * Main
 * ========================================================================== */

int main(int argc, char* argv[])
{
    string pluginPath;

    parse_args(argc, argv, pluginPath);
    setup_logging();

    print_harness_revision();

    const char* plugin_path = pluginPath.c_str();

    /* ------------------------------------------------------------------------
     * Load plugin and factory
     * --------------------------------------------------------------------- */

    note(string("Loading plugin: ") + plugin_path);

    void* handle = dlopen(plugin_path, RTLD_NOW);
    if(!handle) {
        die(dlerror());
    }

    dlerror();

    auto factory = reinterpret_cast<factory_fn_t>(dlsym(handle, "factory"));
    const char* sym_error = dlerror();

    if(sym_error) {
        dlclose(handle);
        die(sym_error);
    }

    note("OK: factory symbol found");

    /* ------------------------------------------------------------------------
     * Load VALVEMASTER lab/test hooks
     * --------------------------------------------------------------------- */

    auto testPowerOn = load_test_hook(handle, "VALVEMASTER_testPowerOn");
    auto testPowerOff = load_test_hook(handle, "VALVEMASTER_testPowerOff");
    auto testProbeBus = load_test_hook(handle, "VALVEMASTER_testProbeBus");

    auto testVersionScanDiscoveredNodes = load_test_hook(handle, "VALVEMASTER_testVersionScanDiscoveredNodes");
    auto testWaitForIdle = load_wait_idle_hook(handle, "VALVEMASTER_testWaitForIdle");

    /*
     * Keep this hook loaded even though the normal harness path does not currently
     * use it. Ping may become useful later as a lighter diagnostic than WHO or
     * version scan, especially for targeted node-presence checks.
     */
    auto testPingDiscoveredNodes =
        load_test_hook(handle, "VALVEMASTER_testPingDiscoveredNodes");
    (void)testPingDiscoveredNodes;


    note("OK: VALVEMASTER test hooks found");

    /* ------------------------------------------------------------------------
     * Create and initialize device
     * --------------------------------------------------------------------- */

    pIoTServerDevice* dev = factory("valvemaster_1", "VALVEMASTER");
    if(!dev) {
        dlclose(handle);
        die("factory returned nullptr");
    }

    note("OK: device object created");

    print_driver_version_or_die(dev, handle);

    print_properties_or_die(dev,
                            handle,
                            "Device properties before harness setProperties():");

    auto schema = make_valvemaster_schema();
    if(!dev->initWithSchema(schema)) {
        delete dev;
        dlclose(handle);
        die("initWithSchema() failed");
    }

    note("OK: schema initialized");

    auto props = make_valvemaster_props();
    dev->setProperties(props);

    note("OK: properties set");

    print_properties_or_die(dev,
                            handle,
                            "Device properties after harness setProperties():");

    if(!dev->start()) {
        delete dev;
        dlclose(handle);
        die("start() failed");
    }

    note("OK: device started");

    /* ------------------------------------------------------------------------
     * Field-bus power-up and slave AVR wake time
     * --------------------------------------------------------------------- */

    note("Powering field bus on.");
    if(!testPowerOn(dev)) {
        delete dev;
        dlclose(handle);
        die("VALVEMASTER_testPowerOn() failed");
    }

    wait_for_driver_idle_or_die(dev,
                                testWaitForIdle,
                                testPowerOff,
                                handle,
                                "Waiting for startup power-on action to complete.");

    note("Waiting for slave AVR nodes to wake.");
    this_thread::sleep_for(chrono::milliseconds(SLAVE_WAKE_SETTLE_MS));

    verify_field_power_or_die(dev,
                              testPowerOff,
                              handle,
                              "on",
                              "Verified field power after startup power-on");

    /* ------------------------------------------------------------------------
     * Startup close-all safety command
     * --------------------------------------------------------------------- */

    all_off_or_die(dev, testPowerOff, handle);

    wait_for_driver_idle_or_die(dev,
                                testWaitForIdle,
                                testPowerOff,
                                handle,
                                "Waiting for startup hardware allOff to complete.");

    print_values_or_die(dev, testPowerOff, handle, "After startup hardware allOff:");
    wait_after_startup_alloff();

    /* ------------------------------------------------------------------------
     * RS-485 discovery and diagnostics
     *
     * These test hooks are async queue submissions. The harness must wait for
     * actionThread to become idle before treating discovery/version scan as
     * complete.
     * --------------------------------------------------------------------- */

    note("Probing RS-485 valve-node bus.");
    if(!testProbeBus(dev)) {
        testPowerOff(dev);
        delete dev;
        dlclose(handle);
        die("VALVEMASTER_testProbeBus() failed");
    }

    wait_for_driver_idle_or_die(dev,
                                testWaitForIdle,
                                testPowerOff,
                                handle,
                                "Waiting for WHO discovery to complete.",
                                ACTION_DISCOVERY_IDLE_TIMEOUT_MS);

    note("Version scanning discovered valve nodes.");
    if(!testVersionScanDiscoveredNodes(dev)) {
        testPowerOff(dev);
        delete dev;
        dlclose(handle);
        die("VALVEMASTER_testVersionScanDiscoveredNodes() failed");
    }

    wait_for_driver_idle_or_die(dev,
                                testWaitForIdle,
                                testPowerOff,
                                handle,
                                "Waiting for version scan to complete.",
                                ACTION_DISCOVERY_IDLE_TIMEOUT_MS);

    print_values_or_die(dev, testPowerOff, handle, "Initial values:");

    const vector<string> schemaKeys = make_ordered_schema_keys();

    /* ------------------------------------------------------------------------
     * One-by-one schema command test
     * --------------------------------------------------------------------- */

    cycle_each_schema_valve_or_die(dev,
                                   testWaitForIdle,
                                   testPowerOff,
                                   handle,
                                   schemaKeys);

    /*
     * The previous valve activity should leave field power on and the driver's
     * hold timer armed. Wait here without issuing another command so the log
     * should show:
     *
     *   auto power-off delay expired
     *   power off command
     *   field power off
     *
     * This is now a real pass/fail test using the driver's field_power
     * diagnostic value.
     */
    wait_for_auto_power_off_check();

    wait_for_driver_idle_or_die(dev,
                                testWaitForIdle,
                                testPowerOff,
                                handle,
                                "Waiting for auto power-off settle to complete.");

    print_values_or_die(dev,
                        testPowerOff,
                        handle,
                        "After auto power-off timeout:");

    verify_field_power_or_die(dev,
                              testPowerOff,
                              handle,
                              "off",
                              "Verified auto power-off timeout");

    /* ------------------------------------------------------------------------
     * All-on followed by single hardware close-all command
     *
     * The next setValues() should cause the driver to power the field bus back
     * on because the timeout test above proved it was off.
     * --------------------------------------------------------------------- */

    turn_all_schema_valves_on_or_die(dev,
                                     testWaitForIdle,
                                     testPowerOff,
                                     handle,
                                     schemaKeys);

    all_off_or_die(dev, testPowerOff, handle);

    wait_for_driver_idle_or_die(dev,
                                testWaitForIdle,
                                testPowerOff,
                                handle,
                                "Waiting for hardware allOff to complete.");

    print_values_or_die(dev, testPowerOff, handle, "After hardware allOff:");
    wait_between_valve_actions();

    /* ------------------------------------------------------------------------
     * Field-bus power-down and cleanup
     * --------------------------------------------------------------------- */

    note("Powering field bus off.");
    if(!testPowerOff(dev)) {
        delete dev;
        dlclose(handle);
        die("VALVEMASTER_testPowerOff() failed");
    }

    wait_for_driver_idle_or_die(dev,
                                testWaitForIdle,
                                testPowerOff,
                                handle,
                                "Waiting for final power-off action to complete.");

    note("Stopping device.");
    dev->stop();

    delete dev;
    dlclose(handle);

    if(gPrint_flag) {
        LOGT_INFO("PASS harness revision %s", HARNESS_REVISION);
    }
    else {
        printf("PASS harness revision %s\n", HARNESS_REVISION);
    }

    return 0;
}
