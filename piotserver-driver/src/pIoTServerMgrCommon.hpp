//
//  CoopMgrCommon.h
//  coopserver
//
//  Created by Vincent Moscaritolo on 12/20/21.
//

#ifndef coopMgrCommon_h
#define coopMgrCommon_h

#include <stdexcept>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <functional>
#include <map>

#include "CommonDefs.hpp"

typedef uint64_t eTag_t;
#define MAX_ETAG UINT64_MAX
 
typedef  std::map<std::string, std::string> keyValueMap_t;

typedef std::vector<std::pair<std::string,bool>> keyBoolVector_t;
 
 
class pIoTServerException: virtual public Exception {
	 
protected:

	 int error_number;               ///< Error Number
	 
public:

	 /** Constructor (C++ STL string, int, int).
	  *  @param msg The error message
	  *  @param err_num Error number
	  */
	 explicit
    pIoTServerException(const std::string& msg, int err_num = 0):
        Exception(msg, err_num),
		  std::runtime_error(msg)
		  {
				error_number = err_num;
		  }

	
	 /** Destructor.
	  *  Virtual to allow for subclassing.
	  */
	 virtual ~pIoTServerException() throw () {}
	 
	 /** Returns error number.
	  *  @return #error_number
	  */
	 virtual int getErrorNumber() const throw() {
		  return error_number;
	 }
};
 

#endif /* coopMgrCommon_h */
