/*
 * freertosLibs.h
 *
 *  Created on: 18 авг. 2016 г.
 *      Author: tihonov
 */

#ifndef FREERTOSLIBS_H_
#define FREERTOSLIBS_H_



#include "FreeRTOS.h"
#include "task.h"



class iTask
{
public:
	virtual void run(void* pvParameters)=0;
	virtual ~iTask(){};
};

void attach(iTask *task,  char *title,uint16_t stack);





#endif /* FREERTOSLIBS_H_ */
