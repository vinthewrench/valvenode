#include "VALVEMASTER_Device.hpp"

extern "C" pIoTServerDevice* factory(std::string devID, std::string driverName)
{
    return new VALVEMASTER_Device(devID, driverName);
}

/*
 * Lab/test hooks.
 *
 * These are not part of the pIoTServerDevice API. The standalone plugin
 * harness may find these with dlsym() and use them for hardware bring-up.
 */

extern "C" bool VALVEMASTER_testPowerOn(pIoTServerDevice* dev)
{
    auto* valveMaster = dynamic_cast<VALVEMASTER_Device*>(dev);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testPowerOn();
}

extern "C" bool VALVEMASTER_testPowerOff(pIoTServerDevice* dev)
{
    auto* valveMaster = dynamic_cast<VALVEMASTER_Device*>(dev);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testPowerOff();
}

extern "C" bool VALVEMASTER_testProbeBus(pIoTServerDevice* dev)
{
    auto* valveMaster = dynamic_cast<VALVEMASTER_Device*>(dev);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testProbeBus();
}

extern "C" bool VALVEMASTER_testPingDiscoveredNodes(pIoTServerDevice* dev)
{
    auto* valveMaster = dynamic_cast<VALVEMASTER_Device*>(dev);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testPingDiscoveredNodes();
}

extern "C" bool VALVEMASTER_testVersionScanDiscoveredNodes(pIoTServerDevice* dev)
{
    auto* valveMaster = dynamic_cast<VALVEMASTER_Device*>(dev);

    if(valveMaster == nullptr) {
        return false;
    }

    return valveMaster->testVersionScanDiscoveredNodes();
}
