/*
 * interruptmanager.h
 *
 *  Created on: 23 но€б. 2015 г.
 *      Author: tihonov
 */

#ifndef INTERRUPT_INTERRUPTMANAGER_H_
#define INTERRUPT_INTERRUPTMANAGER_H_

#include "stm32f407xx.h"

typedef void (*pHandlerPointer_t)();

class InterruptManager {
private:

public:
	static void DefaultHandler(void) {
		asm volatile("nop"::);
	}
	;
	static pHandlerPointer_t volatile IsrVectors[81];
	static void AddHandler(pHandlerPointer_t handler, IRQn_Type irq) {
		IsrVectors[irq] = handler;
	}
	;
	static void RemoveHanlder(IRQn_Type irq) {
		IsrVectors[irq] = DefaultHandler;
	}
	;
	static void Raise(IRQn_Type irq) {
		IsrVectors[irq]();
	}
	;
};



#endif /* INTERRUPT_INTERRUPTMANAGER_H_ */
