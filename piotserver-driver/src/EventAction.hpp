//
//  EventTrigger.hpp
 //
//  Created by Vincent Moscaritolo on 12/29/21.
//

#ifndef EventAction_hpp
#define EventAction_hpp


#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
#include <map>
#include "Utils.hpp"
#include "SolarTimeMgr.hpp"

#include "json.hpp"

using namespace std;
 

string_view preprocess_cronstring(string_view expr);
 
class EventTrigger;

typedef std::function<bool(EventTrigger)> actionCallback_t;

//MARK: - Action

class Action {
 
public:
    constexpr static string_view JSON_CMD               = "cmd";
    constexpr static string_view JSON_CMD_SET           = "SET";
    constexpr static string_view JSON_CMD_RUN_SEQ     = "RUN.SEQUENCE";
    constexpr static string_view JSON_CMD_EVAL          = "EVAL";
    constexpr static string_view JSON_CMD_LOG          = "LOG";
 
    constexpr static string_view JSON_CMD_CALLBACK      = "<callback>";
 
    constexpr static string_view JSON_ACTION_KEY            = "key";
    constexpr static string_view JSON_ACTION_VALUE          = "value";
    constexpr static string_view JSON_ACTION_EXPRESSION        = "expression";
 
    
    Action();
    Action(string cmd, string key, string value);
    Action(nlohmann::json j);
    Action(std::string);
    Action(actionCallback_t cb);

    Action( const Action &actIn){
        _cmd = actIn._cmd;
        _key = actIn._key;
        _value = actIn._value;
        _expression = actIn._expression;
        _cb = actIn._cb;
    }

    inline void operator = (const Action &right ) {
        _cmd = right._cmd;
        _key = right._key;
        _value = right._value;
        _expression = right._expression;
        _cb = right._cb;
  
    }
    
    std::string idString() const;
    std::string printString() const;
    const nlohmann::json JSON();
    
    bool isValid() {return !(_cmd.empty());};
 
    bool isCallBack() {return  (_cmd == JSON_CMD_CALLBACK) && (_cb != NULL); };
    
    bool invokeCallBack(EventTrigger);
    
    const string            cmd() { return _cmd;};
    const string             key(){return     _key;};
    const string            value(){return  _value;};
    const string            expression(){return  _expression;};
     
private:
    void initWithJSON(nlohmann::json j);
    
protected:
    string         _cmd;
    string         _key;
    string          _value;
    string          _expression;
    actionCallback_t _cb = NULL;
};

//MARK: - EventTrigger

class EventTrigger {

 friend class Sequence;

public:
 
    // these shouldnt change, they become persistant
    typedef enum  {
        EVENT_TYPE_UNKNOWN         = 0,
        EVENT_TYPE_TIME            = 2,
        EVENT_TYPE_APP             = 3,
        EVENT_TYPE_EPHEMERAL       = 4,
        EVENT_TYPE_CRON           = 5,
    }eventType_t;

    typedef enum {
        TOD_INVALID     = 0,
        TOD_ABSOLUTE    = 1,
        
        TOD_SUNRISE        = 2,
        TOD_SUNSET        = 3,
        TOD_CIVIL_SUNRISE = 4,
        TOD_CIVIL_SUNSET     = 5,
    } tod_offset_t;

    typedef union {
        uint8_t byte;
        struct {
            bool Sun : 1;
            bool Mon : 1;
            bool Tue : 1;
            bool Wed : 1;
            bool Thu : 1;
            bool Fri : 1;
            bool Sat : 1;
            bool unused : 1;
        } day;
    } dayOfWeek_t;
    
    static const uint8_t everyDayOfWeek = 0xFF;
    
    typedef struct {
        tod_offset_t    timeBase;
        int16_t         timeBaseOffset;
        time_t          lastRun;
        dayOfWeek_t     dayOfWeek;
    } timeEventInfo_t;

    typedef enum {
        APP_EVENT_INVALID     = 0,
        APP_EVENT_STARTUP    = 1,
        APP_EVENT_SHUTDOWN    = 2,
        APP_EVENT_MANUAL    = 3
    } app_event_t;

    
    /*
     NB:  EVENT_TYPE_CRON,  It's cron not chron..
     
     An old guy with a beard told me it meant “Command Run ON”.
     cron is not named after Chronos, the God of Time, but rather Cronus, the King of Titans,
     Keeper of the Old Order. The Root UNIX Scrolls tell of an eventual age when order will be
     overthrown in the world, and the regularity that is overseen by cron will crumble with it.
     When the Greeks wrote Titanomachy, they were foretelling the Fall of UNIX without knowing it.
     */
    
    typedef struct {
        string     cronString;
        time_t     next;
    }cron_event_t;
    
    EventTrigger();

    EventTrigger(const EventTrigger &etIn){
            copy(etIn, this);
    }
    
    EventTrigger(app_event_t appEvent);
    EventTrigger(cron_event_t cronEvent);
  
    EventTrigger(tod_offset_t timeBase, int16_t timeBaseOffset = 0 );
    EventTrigger(tod_offset_t timeBase, int16_t timeBaseOffset, dayOfWeek_t dayOfWeek);
    EventTrigger(time_t time); // Ephmeral event.
 
    EventTrigger(std::string);
    EventTrigger(nlohmann::json j);
    nlohmann::json JSON();
    const std::string printString(bool fullString = true);
   
    bool isValid();
    bool isTimed();
    bool isEphemeral();
    bool isAppEvent();
    bool isCronEvent();
 
    eventType_t getEventType() { return  _eventType;};
    
    bool getTimedEventInfo(timeEventInfo_t &info);
    bool getAppEventInfo(app_event_t &info);
    
    bool shouldTriggerFromAppEvent(app_event_t a);
    bool shouldTriggerFromTimeEvent(const solarTimes_t &solar, time_t time);
    bool shouldTriggerInFuture(const solarTimes_t &solar, time_t time);
    bool canTriggerOnDay(time_t time);
    
    bool calculateTriggerTime(const solarTimes_t &solar, int16_t &minsFromMidnight);

    bool nextCronTime(time_t &time);
    bool scheduleNextCronTime();
    
    bool setLastRun(time_t time);
    
    time_t getLastRun(){
        return (_eventType == EVENT_TYPE_TIME)?_timeEvent.lastRun:0;
    }

    inline void operator = (const EventTrigger &right ) {
        copy(right, this);
    }

private:
    
    void initWithJSON(nlohmann::json j);
    void copy(const EventTrigger &evt, EventTrigger *eventOut);

    eventType_t            _eventType;

    // C++ can't handle a union with a c++ class  (cron_event_t) in it..
 //   union{
        time_t                 _ephmeralTime;
        timeEventInfo_t        _timeEvent;
        app_event_t            _appEvent;
        cron_event_t          _cronEvent;
//    };
 
};


#endif /* EventAction_hpp */
