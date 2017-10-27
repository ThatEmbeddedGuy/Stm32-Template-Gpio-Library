/*
 * stringLibs.h
 *
 *  Created on: 10 авг. 2016 г.
 *      Author: tihonov
 */

#ifndef STRINGLIBS_H_
#define STRINGLIBS_H_

#include <algorithm>
#include <string>
#include <string.h>
#include <sstream>
const char * const BoolToString(bool b);
std::string bufferToString(char* buffer, int bufflen);

template<typename T>
std::string toString(const T& value)
{

	    std::ostringstream s;
	    s <<  value;
	    return s.str();
}

inline std::string toString(const uint8_t& value)
{
	    std::ostringstream s;
	    s << (int) value;
	    return s.str();
}


enum class SerializeOrder
{
forward,
backward
};


template  <typename A,typename B>
void serialize(const A&  from, B& to, size_t size=sizeof(A), const SerializeOrder order=SerializeOrder::forward)
{
const void* fromptr=&from;
void* toptr=&to;
	if (order==SerializeOrder::forward)
		memcpy(toptr,fromptr,size);
	if (order==SerializeOrder::backward)
	{
		std::reverse_copy(toptr, toptr + size, fromptr);
	}
}


#endif /* STRINGLIBS_H_ */
