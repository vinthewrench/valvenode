//
//  pIoTServerSchema.hpp
//  pIoTServer
//
//  Created by vinnie on 12/31/24.
//
#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <functional>
#include <map>


typedef enum {
    TR_IGNORE               = 0,
    TR_DONT_RECORD          = 1, // use latest value, dont track
    TR_TRACK_LATEST_VALUE   = 2,    // TRACK LATEST ONLY
    TR_TRACK_CHANGES        = 3,
    TR_TRACK_RANGE          = 4,

}valueTracking_t;


typedef enum {
    INVALID = 0,
    BOOL,                // Bool ON/OFF
    INT,                // Int
    MAH,                // mAh milliAmp hours
    PERCENT,         // (per hundred) sign ‰
    WATTS,             // W
    MILLIVOLTS,        // mV
    MILLIAMPS,        // mA
    SECONDS,            // sec
    MINUTES,            // mins
    DEGREES_C,        // degC
    VOLTS,            // V
    HERTZ,            // Hz
    AMPS,                // A
    BINARY,            // Binary 8 bits 000001
    RH,                // Relative Humidity Percentage
    HPA,               // barometric pressure in Hectopascal * 0.00029530 to get inHg
    STRING,            // string
    IGNORE,
    TIME_T,             // unix time
    FLOAT,              // floating number
    POM,                // phase of moon  0 - 1, 0.5 = full,
    EQUATION,           // equation string
    LUX,                // Lux is used to measure the amount of light output in a given area.
                        // One lux is equal to one lumen per square meter.
    ACTUATOR,           //  ACTUATOR action
    BOOSTER,            // BOOSTER  Relay ID
    MASTER_RELAY,       // Master Relay ID

    SERIAL_NO,          //Serial Number (String)

    UNKNOWN,
}valueSchemaUnits_t;

typedef  std::pair<std::string, valueSchemaUnits_t> keySchemaPair_t;
