#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <iostream>
#include <string>
#include <thread>

#include "pIoTServerDevice.hpp"
#include "pIoTServerSchema.hpp"
#include "LogMgr.hpp"

using namespace std;

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

/*
 * After the Valve Master turns on field-bus power, the slave AVR nodes need
 * time to boot, initialize UART, enter RS-485 receive mode, and be ready to
 * receive the startup close-all command.
 */
static constexpr uint32_t SLAVE_WAKE_SETTLE_MS = 1500;

/*
 * Gap between real valve actions.
 *
 * This gives the latching solenoid pulse, VNH driver, wiring, and water system
 * a little breathing room between commands. It also makes the test easier to
 * watch in logs and in the field.
 */
static constexpr uint32_t VALVE_ACTION_GAP_MS = 2000;

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
 * @brief Wait between valve actions.
 */
static void wait_between_valve_actions()
{
    note("Waiting before next valve action.");
    this_thread::sleep_for(chrono::milliseconds(VALVE_ACTION_GAP_MS));
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
 * @brief Build a small VALVEMASTER schema for harness testing.
 *
 * This schema intentionally maps two different pIoTServer keys to real
 * ValveNode outputs:
 *
 *   SPRK_1 -> node 1, valve 1
 *   SPRK_2 -> node 3, valve 1
 *
 * That proves the driver is not hardcoded to one node and one valve. It must
 * read node/valve binding from schema otherProps.
 */
static deviceSchemaMap_t make_valvemaster_schema()
{
    deviceSchemaMap_t schema;

    deviceSchema_t sprk1;
    sprk1.title = "A. Chard/Kale";
    sprk1.units = BOOL;
    sprk1.tracking = TR_IGNORE;
    sprk1.readOnly = false;
    sprk1.queryDelay = 5;
    sprk1.otherProps = {
        {"node", 1},
        {"valve", 1}
    };
    schema["SPRK_1"] = sprk1;

    deviceSchema_t sprk2;
    sprk2.title = "B. Cucumbers";
    sprk2.units = BOOL;
    sprk2.tracking = TR_IGNORE;
    sprk2.readOnly = false;
    sprk2.queryDelay = 5;
    sprk2.otherProps = {
        {"node", 3},
        {"valve", 1}
    };
    schema["SPRK_2"] = sprk2;

    return schema;
}

/**
 * @brief Build plugin properties used by the harness.
 *
 * The address property is parsed by VALVEMASTER_Device::parseI2CAddress().
 */
static nlohmann::json make_valvemaster_props()
{
    nlohmann::json props;

    props["address"] = "0x09";
    props["device_type"] = "VALVEMASTER";
    props["title"] = "Valve Master I2C";
    props["interval"] = 5;

    return props;
}

/* ============================================================================
 * Value display
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
 * @brief Read and print all cached schema values.
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
 * @brief Set a single schema key through pIoTServerDevice::setValues().
 *
 * This exercises the actual driver schema binding path:
 *
 *   schema key -> otherProps.node/valve -> CMD_SET_CHANNEL
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
 * @brief Call pIoTServerDevice::allOff().
 *
 * This must exercise the VALVEMASTER close-all path:
 *
 *   dev->allOff()
 *     -> VALVEMASTER_Device::allOff()
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
    auto testPingDiscoveredNodes = load_test_hook(handle, "VALVEMASTER_testPingDiscoveredNodes");
    auto testVersionScanDiscoveredNodes = load_test_hook(handle, "VALVEMASTER_testVersionScanDiscoveredNodes");

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

    /* ------------------------------------------------------------------------
     * Driver/plugin version API smoke test
     *
     * getVersion() reports the driver/plugin version. It should not require
     * dev->start(), I2C, Valve Master firmware access, or RS-485 slave nodes.
     * --------------------------------------------------------------------- */

    print_driver_version_or_die(dev, handle);

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

    note("Waiting for slave AVR nodes to wake.");
    this_thread::sleep_for(chrono::milliseconds(SLAVE_WAKE_SETTLE_MS));

    /* ------------------------------------------------------------------------
     * Startup close-all safety command
     *
     * Real system policy should eventually do this during field-bus power-up.
     * The harness does it explicitly:
     *
     *   power on
     *   wait for slave AVR boot
     *   broadcast close-all once
     *   wait 2 seconds
     *
     * This puts every listening node into a known closed command-state before
     * discovery and before schema valve testing.
     * --------------------------------------------------------------------- */

    all_off_or_die(dev, testPowerOff, handle);
    print_values_or_die(dev, testPowerOff, handle, "After startup hardware allOff:");
    wait_between_valve_actions();

    /* ------------------------------------------------------------------------
     * RS-485 discovery and diagnostics
     * --------------------------------------------------------------------- */

    note("Probing RS-485 valve-node bus.");
    if(!testProbeBus(dev)) {
        testPowerOff(dev);
        delete dev;
        dlclose(handle);
        die("VALVEMASTER_testProbeBus() failed");
    }

    note("Pinging discovered valve nodes.");
    if(!testPingDiscoveredNodes(dev)) {
        testPowerOff(dev);
        delete dev;
        dlclose(handle);
        die("VALVEMASTER_testPingDiscoveredNodes() failed");
    }

    note("Version scanning discovered valve nodes.");
    if(!testVersionScanDiscoveredNodes(dev)) {
        testPowerOff(dev);
        delete dev;
        dlclose(handle);
        die("VALVEMASTER_testVersionScanDiscoveredNodes() failed");
    }

    /* ------------------------------------------------------------------------
     * Initial schema value read
     * --------------------------------------------------------------------- */

    print_values_or_die(dev, testPowerOff, handle, "Initial values:");

    /* ------------------------------------------------------------------------
     * One-by-one schema actuation test
     *
     * This verifies that each schema key maps to the correct node/valve pair.
     * Each action is separated by at least 2 seconds.
     * --------------------------------------------------------------------- */

    set_one_value_or_die(dev, testPowerOff, handle,
                         "SPRK_1",
                         "on",
                         "Turning SPRK_1 on.");
    print_values_or_die(dev, testPowerOff, handle, "After SPRK_1 on:");
    wait_between_valve_actions();

    set_one_value_or_die(dev, testPowerOff, handle,
                         "SPRK_1",
                         "off",
                         "Turning SPRK_1 off.");
    print_values_or_die(dev, testPowerOff, handle, "After SPRK_1 off:");
    wait_between_valve_actions();

    set_one_value_or_die(dev, testPowerOff, handle,
                         "SPRK_2",
                         "on",
                         "Turning SPRK_2 on.");
    print_values_or_die(dev, testPowerOff, handle, "After SPRK_2 on:");
    wait_between_valve_actions();

    set_one_value_or_die(dev, testPowerOff, handle,
                         "SPRK_2",
                         "off",
                         "Turning SPRK_2 off.");
    print_values_or_die(dev, testPowerOff, handle, "After SPRK_2 off:");
    wait_between_valve_actions();

    /* ------------------------------------------------------------------------
     * All-on followed by single hardware close-all command
     *
     * This verifies two separate things:
     *
     *   1. setValues() can turn multiple schema-backed valves on.
     *   2. allOff() sends one Valve Master CMD_CLOSE_ALL command.
     *
     * allOff() must not loop over the schema and individually close each valve.
     * --------------------------------------------------------------------- */

    set_one_value_or_die(dev, testPowerOff, handle,
                         "SPRK_1",
                         "on",
                         "Turning SPRK_1 on for all-off command test.");
    print_values_or_die(dev, testPowerOff, handle, "After SPRK_1 on for all-off test:");
    wait_between_valve_actions();

    set_one_value_or_die(dev, testPowerOff, handle,
                         "SPRK_2",
                         "on",
                         "Turning SPRK_2 on for all-off command test.");
    print_values_or_die(dev, testPowerOff, handle, "After SPRK_2 on for all-off test:");
    wait_between_valve_actions();

    all_off_or_die(dev, testPowerOff, handle);
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

    note("Stopping device.");
    dev->stop();

    delete dev;
    dlclose(handle);

    if(gPrint_flag) {
        LOGT_INFO("PASS");
    }
    else {
        printf("PASS\n");
    }

    return 0;
}
