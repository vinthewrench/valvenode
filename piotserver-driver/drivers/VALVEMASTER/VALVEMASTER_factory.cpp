/**
 * @file VALVEMASTER_factory.cpp
 * @brief Factory and lab-test C exports for the VALVEMASTER pIoTServer plugin.
 *
 * pIoTServer loads device plugins through the exported factory() symbol. The
 * factory must stay simple: create the plugin object and return it. Hardware
 * setup belongs in VALVEMASTER_Device::start(), not here.
 *
 * This file also exports optional lab/bring-up hooks used by the standalone
 * plugin test harness. Those hooks intentionally downcast the generic
 * pIoTServerDevice pointer back to VALVEMASTER_Device so the harness can call
 * Valve Master specific diagnostics.
 */

#include <cstdint>
#include <string>

#include "VALVEMASTER_Device.hpp"

using namespace std;


/**
 * @brief Create a VALVEMASTER plugin instance.
 *
 * This is the symbol pIoTServer uses when loading the device plugin.
 *
 * No I2C probing, schema loading, test operation, or runtime setup should be
 * performed in the factory. Keep the factory dumb.
 *
 * @param devID pIoTServer device identifier.
 * @param driverName pIoTServer driver name.
 * @return Newly allocated plugin instance.
 */
extern "C" pIoTServerDevice* factory(std::string devID, string driverName)
{
    return new VALVEMASTER_Device(devID, driverName);
}


// MARK: - Hardware Bring-Up / Lab Test Hooks

/**
 * @brief Convert a generic pIoTServerDevice pointer into a VALVEMASTER device.
 *
 * The standalone test harness passes plugin instances around as the base class.
 * These helpers only work for VALVEMASTER_Device objects created by factory().
 *
 * @param device Generic pIoTServer device pointer.
 * @return VALVEMASTER_Device pointer, or nullptr if the cast fails.
 */
static VALVEMASTER_Device* asValveMaster(pIoTServerDevice* device)
{
    return dynamic_cast<VALVEMASTER_Device*>(device);
}

/**
 * @brief Test field-bus power-on behavior.
 *
 * @param device Plugin instance returned by factory().
 * @return true if the test operation succeeded.
 */
extern "C" bool VALVEMASTER_testPowerOn(pIoTServerDevice* device)
{
    VALVEMASTER_Device* valveMaster = asValveMaster(device);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testPowerOn();
}

/**
 * @brief Test field-bus power-off behavior.
 *
 * @param device Plugin instance returned by factory().
 * @return true if the test operation succeeded.
 */
extern "C" bool VALVEMASTER_testPowerOff(pIoTServerDevice* device)
{
    VALVEMASTER_Device* valveMaster = asValveMaster(device);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testPowerOff();
}

/**
 * @brief Run Valve Master WHO/node-discovery behavior.
 *
 * @param device Plugin instance returned by factory().
 * @return true if the test operation succeeded.
 */
extern "C" bool VALVEMASTER_testProbeBus(pIoTServerDevice* device)
{
    VALVEMASTER_Device* valveMaster = asValveMaster(device);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testProbeBus();
}

/**
 * @brief Ping nodes discovered by the Valve Master.
 *
 * @param device Plugin instance returned by factory().
 * @return true if the test operation succeeded.
 */
extern "C" bool VALVEMASTER_testPingDiscoveredNodes(pIoTServerDevice* device)
{
    VALVEMASTER_Device* valveMaster = asValveMaster(device);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testPingDiscoveredNodes();
}

/**
 * @brief Query firmware versions from discovered nodes.
 *
 * @param device Plugin instance returned by factory().
 * @return true if the test operation succeeded.
 */
extern "C" bool VALVEMASTER_testVersionScanDiscoveredNodes(pIoTServerDevice* device)
{
    VALVEMASTER_Device* valveMaster = asValveMaster(device);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testVersionScanDiscoveredNodes();
}


/**
 * @brief Wait until queued VALVEMASTER lab/test work is complete.
 *
 * @param device Plugin instance returned by factory().
 * @param timeoutMs Maximum wait in milliseconds.
 * @return true if the device became idle before timeout.
 */
extern "C" bool VALVEMASTER_testWaitForIdle(pIoTServerDevice* device, uint32_t timeoutMs)
{
    VALVEMASTER_Device* valveMaster = asValveMaster(device);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testWaitForIdle(timeoutMs);
}
