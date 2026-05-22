//
//  pIoTServerDevice.hpp
//  pIoTServer
//
//  Created by vinnie on 12/28/24.
//
 
#ifndef pIoTServerDevice_h
#define pIoTServerDevice_h

#include <string>
#include <map>
#include <vector>
#include "json.hpp"

#include "pIoTServerMgrCommon.hpp"
#include "pIoTServerSchema.hpp"
#include "PropValKeys.hpp"
#include "EventAction.hpp"
#include "Utils.hpp"

using namespace std;
using namespace nlohmann;
 

typedef struct deviceSchema_t {
    string              title;
    valueSchemaUnits_t  units       = UNKNOWN;
    valueTracking_t     tracking    = TR_IGNORE;
    bool                readOnly    = true;
    bool                isFormula   = false;
    uint64_t            queryDelay  = UINT64_MAX;
    uint8_t             pinNo     = 0;      //  bit/BCM or maybe a channel number
    unsigned long       flags     = 0;      // used for GPIO
    json                otherProps = NULL;
} deviceSchema_t;


typedef  std::map<std::string, deviceSchema_t> deviceSchemaMap_t;



class pIoTServerDevice {
 
public:
 
    typedef enum  {
        DEVICE_STATE_UNKNOWN = 0,
        DEVICE_STATE_DISCONNECTED,
        DEVICE_STATE_CONNECTED,
        DEVICE_STATE_ERROR,
        DEVICE_STATE_TIMEOUT
    }device_state_t;
    
    typedef enum {
        INVALID = 0,
        PROCESS_VALUES,
        ERROR,
        FAIL,
        CONTINUE,
        NOTHING,
    }response_result_t;
    
    // this is used to create a  built in drivers
    typedef std::function< pIoTServerDevice* (string devID)> builtInDevicefactoryCallback_t;
 
    virtual ~pIoTServerDevice() {} // Virtual destructor

    virtual bool initWithSchema(deviceSchemaMap_t deviceSchema) = 0;

    virtual bool start() = 0;
    
    // optional - preflight gets called after all devices are started.
    virtual bool preflight() { return true;};
 
    virtual void stop() = 0;
    virtual bool isConnected()  = 0;
    
    virtual bool setEnabled(bool enable) = 0;
    virtual bool isEnabled() {return _isEnabled;};
    
    virtual bool  getDeviceID(string  &devID)
    {devID = _deviceID; return (!_deviceID.empty());};
    
    virtual bool  getDriverName(string  &dName)
    {dName = _driverName; return !_driverName.empty();};

    virtual bool  getVersion(string  &version)
            {return false;};

    virtual bool setValues(keyBoolVector_t states) { return false;};
    
    virtual bool setValues(keyValueMap_t kv) { return false;};
    virtual bool getValues( keyValueMap_t &) { return false;};
    virtual bool hasUpdates() { return true;};
    
    virtual bool allOff() { return false;};
    
    virtual void  setProperties(json  j) {
        for (auto& [key, value] : j.items()) {
            string propKey = Utils::trim(key);
            if(propKey == JSON_ARG_ENABLE) continue;
            _deviceProperties[propKey] = value;
        };
    }
 
    virtual void  getProperties(json  &j) {
        j = _deviceProperties;
        string devID; getDeviceID(devID);       // PROP_DEVICE_ID is a psuedo prop
        j[PROP_DEVICE_ID] = devID;
        j[JSON_ARG_ENABLE] = isEnabled();
        
        string version;
        if(getVersion(version)){
            j[JSON_ARG_VERSION] = version;
         }

        string drvrName;
        if(getDriverName(drvrName)){
            j[PROP_PLUGIN_NAME] = drvrName;
         }

    };
    
    string getDeviceTitle() {
        string title;
        if(_deviceProperties.count(PROP_TITLE))
            title = _deviceProperties[PROP_TITLE];
        return title;
    }
    
    virtual device_state_t getDeviceState() { return _deviceState;};

    static std::string stateString(device_state_t state){
            
            std::string result;
            
            switch(state){
                    
                case DEVICE_STATE_DISCONNECTED:
                    result = "Disconnected";
                    break;
                case DEVICE_STATE_CONNECTED:
                    result = "Connected";
                    break;
                case DEVICE_STATE_ERROR:
                    result = "Error";
                    break;
                case DEVICE_STATE_TIMEOUT:
                    result = "Timeout";
                    break;
                default:
                    result = "Unknown";
                    break;
            }
            
            return result;
        }

    // optional event processing for device
    virtual void eventNotification(EventTrigger trig) {};
 
protected:
    
    virtual void setDeviceID(string devID , string driverName)
        {_deviceID = devID;
        _driverName = driverName;};
 
 
    string          _deviceID;
    string          _driverName;
    
    bool            _isEnabled = true;
    device_state_t  _deviceState;
    json            _deviceProperties;

};

// this is the API used by plugins to create a new device
typedef pIoTServerDevice* pIoTServerDeviceFactory_t(string devID, string driverName);


#endif /* pIoTServerDevice_h */
