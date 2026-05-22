//
//  PropValKeys.hpp
//  carradio
//
//  Created by Vincent Moscaritolo on 5/9/22.
//
#pragma once


#include <strings.h>
#include <cstring>

using namespace std;

// REST URLS

constexpr string_view NOUN_VERSION                 = "version";
constexpr string_view NOUN_DATE                     = "date";
constexpr string_view NOUN_STATE                    = "state";
constexpr string_view NOUN_SCHEMA                   = "schema";
constexpr string_view NOUN_VALUES                   = "values";
constexpr string_view NOUN_PROPERTIES               = "props";
constexpr string_view NOUN_HISTORY                  = "history";

constexpr string_view NOUN_RANGE                  = "range";

constexpr string_view NOUN_LOG                       = "log";
constexpr string_view NOUN_ALERTS                   = "alerts";
constexpr string_view NOUN_DEVICES                  = "devices";
constexpr string_view NOUN_SEQUENCES              = "sequences";
constexpr string_view NOUN_SEQUENCE_GROUPS        = "sequence.groups";

constexpr string_view NOUN_TEST                = "test";


constexpr string_view SUBPATH_FILEPATH            = "filepath";
constexpr string_view SUBPATH_STATE               = "state";
constexpr string_view SUBPATH_RUN_ACTION        = "run.actions";
constexpr string_view SUBPATH_COUNT              = "count";
constexpr string_view SUBPATH_RANGE              = "range";

// REST Body Keys

constexpr string_view JSON_ARG_SUCCESS            = "success";

constexpr string_view JSON_ARG_DATE            = "date";
constexpr string_view JSON_ARG_VERSION   = "version";
constexpr string_view JSON_ARG_SERVER_PROCNAME   = "proc_name";
constexpr string_view JSON_ARG_MODEL               = "model";

constexpr string_view JSON_ARG_BUILD_TIME    = "buildtime";
constexpr string_view JSON_ARG_UPTIME        = "uptime";
constexpr string_view JSON_ARG_MESSAGE        = "message";            // for logfile

constexpr string_view JSON_ARG_STATE            = "state";
constexpr string_view JSON_ARG_STATE_STRING        = "state.string";
 
constexpr string_view JSON_ARG_PROPERTIES    = "properties";
constexpr string_view JSON_ARG_DEVICES          = "devices";
 
constexpr string_view JSON_ARG_SCHEMA            = "schema";
constexpr string_view JSON_ARG_VALUES            = "values";
constexpr string_view JSON_ARG_EVENT           = "event";
constexpr string_view JSON_ARG_KEYS              = "keys";

constexpr string_view JSON_ARG_RANGE             = "range";

constexpr string_view JSON_ARG_MANUAL_KEYS       = "keys.manual";
constexpr string_view JSON_ARG_AUTOMATIC_KEYS       = "keys.automatic";

constexpr string_view JSON_HDR_DAYS            = "days";
constexpr string_view JSON_HDR_HOURS            = "hours";

constexpr string_view JSON_HDR_LIMIT            = "limit";
constexpr string_view JSON_HDR_OFFSET           = "offset";

constexpr string_view JSON_ARG_COUNT           = "count";

constexpr string_view JSON_ARG_NAME            = "name";
constexpr string_view JSON_ARG_UNITS            = "units";
constexpr string_view JSON_ARG_ETAG            = "ETag";
constexpr string_view JSON_ARG_SUFFIX            = "suffix";
constexpr string_view JSON_ARG_DISPLAYSTR       = "display";
constexpr string_view JSON_ARG_VALUE            = "value";
constexpr string_view JSON_ARG_ALERT           = "alert";
constexpr string_view JSON_ARG_ALERT_STRING     = "alert.string";
constexpr string_view JSON_ARG_ALERT_DETAILS    = "alert.details";

constexpr string_view JSON_ARG_TIME             = "time";
constexpr string_view JSON_PROP_MAX             = "max";
constexpr string_view JSON_PROP_MIN             = "min";
 
//* for triggers */
constexpr string_view JSON_TIME_EPHMERAL      = "epoch.time";

constexpr string_view JSON_TIME_BASE            = "timeBase";
constexpr string_view JSON_TIME_OFFSET          = "offset";
constexpr string_view JSON_TIME_TOD             = "time";
constexpr string_view JSON_TIME_DOW             = "weekday";

constexpr string_view JSON_TIME_CRON            = "cron";

constexpr string_view JSON_EVENT_TRIGGER_STRING   = "trigger.string";

constexpr string_view JSON_EVENT_STARTUP        = "startup";
constexpr string_view JSON_EVENT_SHUTDOWN       = "shutdown";
constexpr string_view JSON_EVENT_MANUAL         = "manual";

constexpr string_view JSON_ARG_ENABLE            = "enable";

constexpr string_view JSON_ARG_I2C            = "i2c";
constexpr string_view JSON_ARG_W1DEVICE        = "w1";

constexpr string_view JSON_ARG_INITIAL_VALUE     = "initial.value";
constexpr string_view JSON_ARG_FORMULA          = "formula";
 
constexpr string_view JSON_ARG_SUNRISE          = "sunrise";
constexpr string_view JSON_ARG_SUNSET           = "sunset";
constexpr string_view JSON_ARG_CIVIL_SUNRISE    = "civil sunrise";
constexpr string_view JSON_ARG_CIVIL_SUNSET     = "civil sunset";

constexpr string_view JSON_VAL_START            = "start";
constexpr string_view JSON_VAL_STOP            = "stop";

constexpr string_view JSON_VAL_ON               = "on";
constexpr string_view JSON_VAL_OFF              = "off";
constexpr string_view JSON_VAL_AUTO             = "auto";

constexpr string_view JSON_ARG_OS_SYSNAME    = "os.sysname";
constexpr string_view JSON_ARG_OS_NODENAME    = "os.nodename";
constexpr string_view JSON_ARG_OS_RELEASE    = "os.release";
constexpr string_view JSON_ARG_OS_MACHINE    = "os.machine";
constexpr string_view JSON_ARG_OS_VERSION    = "os.version";

constexpr string_view JSON_ARG_SOLAR                 = "solar";
constexpr string_view JSON_ARG_SOLAR_CIVIL_SUNRISE     = "civilSunRise";
constexpr string_view JSON_ARG_SOLAR_SUNRISE         = "sunRise";
constexpr string_view JSON_ARG_SOLAR_SUNSET             ="sunSet";
constexpr string_view JSON_ARG_SOLAR_CIVIL_SUNSET     ="civilSunSet";
constexpr string_view JSON_ARG_SOLAR_LATITUDE         = "latitude";
constexpr string_view JSON_ARG_SOLAR_LONGITUDE         = "longitude";
constexpr string_view JSON_ARG_SOLAR_GMTOFFSET         = "gmtOffset";
constexpr string_view JSON_ARG_SOLAR_TIMEZONE         = "timeZone";
constexpr string_view JSON_ARG_SOLAR_MIDNIGHT         = "midnight";

constexpr string_view JSON_ARG_SOLAR_POM_VISABLE       = "pom.visable";
constexpr string_view JSON_ARG_SOLAR_POM_PHASE         = "pom.phase";
constexpr string_view JSON_ARG_SOLAR_POM_STR            = "pom.name";
 
// Well known keys
 
constexpr static string_view PROP_ACCESS_KEYS           = "access_keys";
constexpr static string_view PROP_API_KEY           = "api_key";
constexpr static string_view PROP_API_SECRET        = "api_secret";

constexpr static string_view PROP_CONFIG_LOGFILE_PATH        = "logfile_path";
constexpr static string_view PROP_CONFIG_LOGFILE_FLAGS       = "logfile_flags";
 inline static const string  PROP_CONFIG_LATLONG                  = "lat_long";

constexpr static string_view PROP_CONFIG          = "config";

constexpr static string_view PROP_DEVICES          = "devices";
constexpr static string_view PROP_DEVICE_TYPE      = "device_type";
constexpr static string_view PROP_DEVICE_MFG_URL    = "mfg_url";
constexpr static string_view PROP_DEVICE_MFG_PART    = "mfg_part";
constexpr static string_view PROP_DEVICE_ID         = "deviceID";

constexpr static string_view PROP_DEVICE_PARAMS     = "params";

constexpr static string_view PROP_SEQUENCE         = "sequence";
constexpr static string_view JSON_ARG_OVERIDE_MANUAL     = "override_manualmode";

constexpr static string_view PROP_SENSOR_QUERY_INTERVAL  = "interval";
constexpr static string_view PROP_TRACKING                 = "tracking";

constexpr static string_view JSON_ARG_IGNORE                = "ignore";
constexpr static string_view JSON_ARG_DONT_RECORD           = "dont.record";
constexpr static string_view JSON_ARG_TRACK_LATEST_VALUE    = "track.latest";
constexpr static string_view JSON_ARG_TRACK_CHANGES         = "track.changes";
constexpr static string_view JSON_ARG_TRACK_RANGE           = "track.range";

constexpr static string_view PROP_SENSOR_ID_1WIRE       = "1WIRE";

constexpr static string_view PROP_SENSOR_ID_QWIIC_BUTTON  = "QWIICBUTTON";
constexpr static string_view PROP_SENSOR_ID_VELM6030        = "VELM6030";

constexpr static string_view PROP_SENSOR_ID_TANKDEPTH      = "TANK_DEPTH";

constexpr static string_view PROP_DEVICE_QWR_16566        = "QWR_16566";   //  COM-16566  Quad relay
constexpr static string_view PROP_DEVICE_QWR_15093        = "QWR_15093";  //  COM-15093  single relay
constexpr static string_view PROP_DEVICE_QWR_16810        = "QWR_16810";  //  COM-16810  DUAL SSR
 
constexpr static string_view PROP_PLUGIN_NAME      = "plugin.name";

constexpr static string_view PROP_DEVICE_GPIO       = "GPIO";
constexpr static string_view PROP_DEVICE_PINS       = "pins";
constexpr static string_view PROP_DEVICE_GPIO_BCM   = "BCM";

constexpr static string_view PROP_DEVICE_PCA9671_BIT   = "bit";

constexpr static string_view PROP_DEVICE_ACTUATOR    = "ACTUATOR";
constexpr static string_view PROP_DEVICE_SPRINKLER    = "SPRINKLER";
 
/* these are now  plugins
 
 //constexpr static string_view PROP_SENSOR_ID_TMP10X      = "TMP10X";
 //constexpr static string_view PROP_SENSOR_ID_BME280      = "BME280";
 //constexpr static string_view PROP_SENSOR_ID_MCP3427     = "MCP3427";
 //constexpr static string_view PROP_SENSOR_ID_MCP23008    = "MCP23008";
 //constexpr static string_view PROP_SENSOR_ID_SHT30        = "SHT30";
 //constexpr static string_view PROP_SENSOR_ID_SHT25        = "SHT25";
 //constexpr static string_view PROP_SENSOR_ID_ADS1115     = "ADS1115";
 //constexpr static string_view PROP_DEVICE_PCA9671         = "PCA9671";
 //constexpr static string_view PROP_DEVICE_SAMPLE         = "SAMPLE";

 */
constexpr static string_view PROP_DEVICE_BUILT_IN    = "@@@@";


constexpr static string_view PROP_READONLY        = "read_only";
constexpr static string_view PROP_TITLE           = "title";
constexpr static string_view PROP_DATA_TYPE       = "data_type";
constexpr static string_view PROP_ADDRESS         = "address";
constexpr static string_view PROP_KEY             = "key";
constexpr static string_view PROP_DESCRIPTION     = "description";
constexpr static string_view PROP_KEYS             = "keys";
constexpr static string_view PROP_OTHER            = "other.props";

constexpr static string_view PROP_GPIO_MODE         = "gpio.mode";
constexpr static string_view PROP_ARG_GPIO_OUTPUT    = "output";
constexpr static string_view PROP_ARG_GPIO_INPUT      = "input";
constexpr static string_view PROP_GPIO_FLAGS         = "gpio.flags";

 
// built in read only values
inline static const string VAL_CPU_TEMPERATURE      = "CPU_TEMPERATURE";
inline static const string VAL_CPU_FAN              = "CPU_FAN";
inline static const string VAL_SOLAR_SUNSET         = "SOLAR_SUNSET";
inline static const string VAL_SOLAR_SUNRISE        = "SOLAR_SUNRISE";
inline static const string VAL_SOLAR_CIVIL_SUNRISE  = "SOLAR_CIVIL_SUNRISE";
inline static const string VAL_SOLAR_CIVIL_SUNSET   = "SOLAR_CIVIL_SUNSET";
inline static const string VAL_SOLAR_MOONPHASE      = "SOLAR_MOONPHASE";
inline static const string VAL_SOLAR_LASTMIDNIGHT   = "SOLAR_LASTMIDNIGHT";


// json data

inline static const string PROP_FILE_VERSION         = "prop.file.version";
inline static const string  PROP_LAST_WRITE_DATE        = "last_write";
inline static const string  PROP_FILE_ETAG              = "file.etag";


// debug DAB stuff
constexpr static string_view PROP_DAB_USER          = "dab_user";
constexpr static string_view PROP_DAB_SECRET        = "dab_secret";

constexpr static string_view PROP_ARG_GROUPID        = "groupID";
constexpr static string_view PROP_ARG_GROUPIDS        = "groupIDs";
constexpr static string_view JSON_ARG_SEQUENCE_ID         = "sequenceID";
constexpr static string_view JSON_ARG_SEQUENCE_IDS         = "sequenceIDs";
constexpr static string_view JSON_ARG_TIMED_SEQUENCES    = "sequences.timed";
constexpr static string_view JSON_ARG_FUTURE_SEQUENCES    = "sequences.future";
constexpr static string_view JSON_ARG_CRON_SEQUENCES   = "sequences.cron";
constexpr static string_view PROP_SEQUENCE_GROUPS        = "sequence.groups";     
 
constexpr static string_view JSON_ARG_ACTION            = "action";
constexpr static string_view JSON_ARG_STEPS            = "steps";

constexpr static string_view JSON_ARG_CONDITION         = "condition";
constexpr static string_view JSON_ARG_ON_ABORT         = "on_abort";


constexpr static string_view JSON_ARG_POST_ACTION       = "post.action";
constexpr static string_view JSON_ARG_TRIGGER            = "trigger";
constexpr static string_view JSON_ARG_ABORT            = "abort";

constexpr static string_view JSON_ARG_DURATION       = "duration";

constexpr static string_view JSON_ARG_SCHEDULE           = "schedule";

constexpr static string_view JSON_ARG_STEP            = "step";
constexpr static string_view JSON_ARG_CRON            = "cron";

