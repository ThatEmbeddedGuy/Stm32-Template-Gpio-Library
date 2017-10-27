/*
 * stringLibs.cpp
 *
 *  Created on: 10 авг. 2016 г.
 *      Author: tihonov
 */




#include "stringLibs.h"


const char * const BoolToString(bool b)
{
	return b ? "true" : "false";

}

std::string bufferToString(char* buffer, int bufflen)
{
	std::string ret(buffer, bufflen);
	return ret;
}
