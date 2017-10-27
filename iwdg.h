/*
 * iwdg.h
 *
 *  Created on: 7 нояб. 2016 г.
 *      Author: tihonov
 */

#ifndef INTERRUPT_IWDG_H_
#define INTERRUPT_IWDG_H_
#include "cmsis_device.h"

class iwdg
{
public:
	iwdg()=default;
static void init(uint32_t us)
{
	IWDG->KR = 0x5555;
	IWDG->PR = 3;   // Prescaler /32  (1 ms)
	IWDG->RLR = us;        
	IWDG->KR = 0xCCCC;        // Start
};
static void reset()
{
 IWDG->KR =  0xAAAA; };
};



#endif /* INTERRUPT_IWDG_H_ */
