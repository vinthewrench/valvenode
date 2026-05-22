//
//  CommonDefs.hpp

//
//  Created by Vincent Moscaritolo on 3/19/21.
//

#ifndef CommonDefs_h
#define CommonDefs_h

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <functional>
#include <stdexcept>
#include <vector>
#include <string>
#include <limits>
#include <map>
#include <time.h>

typedef std::function<void(bool didSucceed)> boolCallback_t;
typedef std::function<void()> voidCallback_t;
typedef std::vector<std::string> stringvector;
typedef std::map<std::string,bool> boolMap_t;

const time_t MAX_TIME = std::numeric_limits<time_t>::max();
 
static inline bool XOR(bool a, bool b)
{
   return (!a && b) || (a && !b);
}



class Exception: virtual public std::runtime_error {
   
protected:
   
   int error_number;               ///< Error Number
   unsigned line;                        // line number
   const char* function    ;             //function name
public:
   
   /** Constructor (C++ STL string, int, int).
    *  @param msg The error message
    *  @param err_num Error number
    */
   explicit Exception(const std::string& msg, int err_num = 0):
   std::runtime_error(msg)
   {
       line = __LINE__;
       function = __FUNCTION__;
       error_number = err_num;
   }
   
   
   /** Destructor.
    *  Virtual to allow for subclassing.
    */
   virtual ~Exception() throw () {}
   
   /** Returns error number.
    *  @return #error_number
    */
   virtual int getErrorNumber() const throw() {
       return error_number;
   }
};


template <class T>
class ClassName
{
public:
 static std::string Get()
 {
    // Get function name, which is "ClassName<class T>::Get"
    // The template parameter 'T' is the class name we're looking for
    std::string name = __FUNCTION__;
    // Remove "ClassName<class " ("<class " is 7 characters long)
    size_t pos = name.find_first_of('<');
    if (pos != std::string::npos)
       name = name.substr(pos + 7);
    // Remove ">::Get"
    pos = name.find_last_of('>');
    if (pos != std::string::npos)
       name = name.substr(0, pos);
    return name;
 }
};

template <class T>
std::string GetClassName(const T* _this = NULL)
{
 return ClassName<T>::Get();
}

#endif /* CommonDefs_h */
