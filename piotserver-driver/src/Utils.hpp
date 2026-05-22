// ============================================================================
//    Author: Kenneth Perkins
//    Date:   Dec 10, 2020
//    Taken From: http://programmingnotes.org/
//    File:  Utils.h
//    Description: Handles general utility functions
// ============================================================================
#pragma once
#include <string>
#include <algorithm>
#include <cctype>
#include <vector>
#include <sstream>
#include <iomanip>
#include <regex>

 
namespace Utils {

template<class Element, class Container> bool in_array(const Element & element, const Container & container)
{
    return std::find(std::begin(container), std::end(container), element)
            != std::end(container);
}



	 /**
	 * FUNCTION: trimEnd
	 * USE: Returns a new string with all trailing whitespace characters removed
	 * @param source =  The source string
	 * @return: A new string with all the trailing whitespace characters removed
	 */
	inline  std::string trimEnd(std::string source) {
		  source.erase(std::find_if(source.rbegin(), source.rend(), [](char c) {
				return !std::isspace(static_cast<unsigned char>(c));
		  }).base(), source.end());
		  return source;
	 }

	 /**
	 * FUNCTION: trimStart
	 * USE: Returns a new string with all leading whitespace characters removed
	 * @param source - The source string
	 * @return: A new string with all the leading whitespace characters removed
	 */
	inline  std::string trimStart(std::string source) {
		  source.erase(source.begin(), std::find_if(source.begin(), source.end(), [](char c) {
				return !std::isspace(static_cast<unsigned char>(c));
		  }));
		  return source;
	 }

	 /**
	 * FUNCTION: trim
	 * USE: Returns a new string with all the leading and trailing whitespace
	 *   characters removed
	 * @param source  - The source string
	 * @return: A new string with all the leading and trailing whitespace
	 *   characters removed
	 */
	 inline std::string trim(std::string source) {
		  return trimEnd(trimStart(source));
	 }

inline std::string urlencode(const std::string &s)
{
    static const char lookup[]= "0123456789abcdef";
    std::stringstream e;
    for(int i=0, ix= (int)s.length(); i<ix; i++)
    {
        const char& c = s[i];
        if ( (48 <= c && c <= 57) ||//0-9
             (65 <= c && c <= 90) ||//abc...xyz
             (97 <= c && c <= 122) || //ABC...XYZ
             (c=='-' || c=='_' || c=='.' || c=='~')
        )
        {
            e << c;
        }
        else
        {
            e << '%';
            e << lookup[ (c&0xF0)>>4 ];
            e << lookup[ (c&0x0F) ];
        }
    }
    return e.str();
}

}// http://programmingnotes.org/

template<typename T>
std::vector<T>
split(const T & str, const T & delimiters) {
	std::vector<T> v;
	 typename T::size_type start = 0;
	 auto pos = str.find_first_of(delimiters, start);
	 while(pos != T::npos) {
		  if(pos != start) // ignore empty tokens
				v.emplace_back(str, start, pos - start);
		  start = pos + 1;
		  pos = str.find_first_of(delimiters, start);
	 }
	 if(start < str.length()) // ignore trailing delimiter
		  v.emplace_back(str, start, str.length() - start); // add what's left of the string
	 return v;
}

/**
  * FUNCTION: replaceAll
  * USE: Replaces all occurrences of the 'oldValue' string with the
  *   'newValue' string
  * @param source The source string
  * @param oldValue The string to be replaced
  * @param newValue The string to replace all occurrences of oldValue
  * @return: A new string with all occurrences replaced
  */
inline std::string replaceAll(const std::string& source
		, const std::string& oldValue, const std::string& newValue) {
		if (oldValue.empty()) {
			 return source;
		}
		std::string newString;
		newString.reserve(source.length());
		std::size_t lastPos = 0;
		std::size_t findPos;
		while (std::string::npos != (findPos = source.find(oldValue, lastPos))) {
			 newString.append(source, lastPos, findPos - lastPos);
			 newString += newValue;
			 lastPos = findPos + oldValue.length();
		}
		newString += source.substr(lastPos);
		return newString;
  }

inline bool caseInSensStringCompare(std::string str1, std::string str2)
{
	return ((str1.size() == str2.size())
			  && std::equal(str1.begin(), str1.end(), str2.begin(), [](char & c1, char & c2){
		return (c1 == c2 || std::toupper(c1) == std::toupper(c2));
	}));
}

inline bool isHexString(std::string str){
    return  regex_match( str, std::regex("^0?[xX][0-9a-fA-F]+$"));
}

inline bool isNumberString(std::string str){
    return  regex_match( str, std::regex("^[-+]?[0-9]*\\.?[0-9]+$"));
}

inline bool isIntegerString(std::string str){
    return  regex_match( str, std::regex("^[-+]?[0-9]+$"));
}


inline bool isBinaryString(std::string str){
    return  regex_match( str, std::regex("^[0-1]+$"));
}


inline bool stringToDouble(std::string str, double &out ){
    bool isValid = false;
    double dValue  = 0;
    
    if (isNumberString(str)){
        char   *p;
        dValue = strtod(str.c_str(), &p);
        if(*p == 0){
            isValid = true;
        }
    }
 
    if(isValid)
        out = dValue;
    
    return isValid;
}

inline bool stringToBool(std::string str, bool &stateOut ){
    
    bool state = false;
    bool isBool = false;
    unsigned long intVal = 0;
    
    str = Utils::trim(str);
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    
    if(caseInSensStringCompare(str,"off")) {
        state = false;
        isBool = true;
    }
    else  if(caseInSensStringCompare(str,"on")) {
        state = true;
        isBool = true;
    }
    else if(caseInSensStringCompare(str,"true")) {
        state = true;
        isBool = true;
    }
    else  if(caseInSensStringCompare(str,"false")) {
        state = false;
        isBool = true;
    }
    else  if(caseInSensStringCompare(str,"1")) {
        state = true;
        isBool = true;
    }
    else  if(caseInSensStringCompare(str,"0")) {
        state = false;
        isBool = true;
    }
    
    else if(isNumberString(str)
            && ( std::sscanf(str.c_str(), "%ld", &intVal) == 1)
            && (intVal == 0 || intVal == 1)){
        isBool = true;
        state = intVal == 1;
    }
    else if( isHexString(str)
            && ( std::sscanf(str.c_str(), "%lx", &intVal) == 1)
            && (intVal == 0 || intVal == 1)){
        isBool = true;
        state = intVal == 1;
    }
    
    if(isBool)
        stateOut = state;
    
    return isBool;
}

  

//
//template <typename T> inline std::string int_to_hex(T val, size_t width=sizeof(T)*2)
//{
//	std::stringstream ss;
//	ss << std::setfill('0') << std::setw(width) << std::hex << (val|0);
//	return ss.str();
//}


template <class T, class T2 = typename std::enable_if<std::is_integral<T>::value>::type>
static std::string to_hex(const T & data, bool addPrefix = false);
 
/*
template<class T, class>
inline std::string to_hex(const T & data, bool addPrefix)
{
	 std::stringstream sstream;
	 sstream << std::hex;
	 std::string ret;
	 if (typeid(T) == typeid(char) || typeid(T) == typeid(unsigned char) || sizeof(T)==1)
	 {
		  sstream << static_cast<int>(data);
		  ret = sstream.str();
		  if (ret.length() > 2)
		  {
				ret = ret.substr(ret.length() - 2, 2);
		  }
	 }
	 else
	 {
		  sstream << data;
		  ret = sstream.str();
	 }
	 return (addPrefix ? u8"0x" : u8"") + ret;
}
*/


template<class T, class> std::string to_hex(const T & data, bool addPrefix)
{
	std::stringstream sstream;
	sstream << std::hex;
	std::string ret;
	if (typeid(T) == typeid(char) || typeid(T) == typeid(unsigned char) || sizeof(T)==1)
	{
		sstream << std::setw(sizeof(T) * 2) << std::setfill('0') <<  static_cast<int>(data);
		 ret = sstream.str();
	}
	else
	{
	  sstream << std::setw(sizeof(T) * 2) << std::setfill('0') << data;
	  ret = sstream.str();
  }

    if(addPrefix)
        ret = std::string("0x") + ret;
  
	return   ret;
}
 

inline std::string hexString(uint8_t data[], size_t len) {
    
    constexpr char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7',
        '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    
    std::string s(len * 2, ' ');
    for (int i = 0; i < len; ++i) {
        s[2 * i]     = hexmap[(data[i] & 0xF0) >> 4];
        s[2 * i + 1] = hexmap[data[i] & 0x0F];
    }
    return s;
}

/*
 
 #include <sstream>
 #include <iomanip>
 #include <span>

 std::string hexString(std::span<const uint8_t> data) {
     std::stringstream ss;
     ss << std::hex << std::setfill('0');

     for (auto x : data) {
         ss << static_cast<int>(x) << " ";
     }

     return ss.str();
 }

 */
