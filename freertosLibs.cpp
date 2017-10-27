/*
 * freertosLibs.cpp
 *
 *  Created on: 18 авг. 2016 г.
 *      Author: tihonov
 */




#include "FreertosLibs.h"

 typedef void (iTask::*FNMETHOD) (void *);

 void attach(iTask *task,  char *title,uint16_t stack) {
 	FNMETHOD run = &iTask::run;

 	pdTASK_CODE proc = ((pdTASK_CODE)(task->*run));
 	xTaskCreate(
 		proc,
 		(const char *) title,
		stack,
 		task,
 		2,
 		(xTaskHandle *) 0);

 }

