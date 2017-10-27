/*
 * interruptmanager.h
 *
 *  Created on: 23 нояб. 2015 г.
 *      Author: tihonov
 */

#ifndef INTERRUPT_INTERRUPTMANAGER_H_
#define INTERRUPT_INTERRUPTMANAGER_H_

#include "stm32f4xx.h"
#include<stdint.h>
#include "string.h"
//Add another vecotrs implimentaton if you use another mcu
#if defined(STM32F407xx)
#include "vectorsf407.h"
#endif
//******************************************************************************************
//How to use this manager
//
// First define INTERRUPTMANAGER_ENABLED INTERRUPTMANAGER_SRAM or INTERRUPTMANAGER_SRAM
//INTERRUPTMANAGER::init(); then init interruptmanager
//INTERRUPTMANAGER::AddHandler  add or remove interrupts
//INTERRUPTMANAGER::AddHandler(InterruptFunction, EXTI4_IRQn);
//NVIC_SetPriority(EXTI4_IRQn, 6);
//NVIC_EnableIRQ (EXTI4_IRQn);
//NVIC_DisableIRQ (EXTI4_IRQn);
//INTERRUPTMANAGER::RemoveHandler(InterruptFunction, EXTI4_IRQn);
//INTERRUPTMANAGER::AddHandler(this->IrqHandle, ETH_IRQn);
//NVIC_SetPriority(ETH_IRQn, 6);
//NVIC_EnableIRQ (ETH_IRQn);
//******************************************************************************************


//******************************************************************************************
//Select which interruptmanager will be used in your project
//******************************************************************************************
#define INTERRUPTMANAGER_SRAM 1
#define INTERRUPTMANAGER_FLASH 2

// uncomment one of this sections
#define INTERRUPTMANAGER_ENABLED INTERRUPTMANAGER_SRAM
//#define INTERRUPTMANAGER_ENABLED INTERRUPTMANAGER_FLASH

#ifndef INTERRUPTMANAGER_ENABLED
#define INTERRUPTMANAGER_ENABLED INTERRUPTMANAGER_FLASH
#endif

//******************************************************************************************
//Specify this parameters by adding
//******************************************************************************************
#if defined(STM32F407xx)
#define VECTORTABLE_SIZE        (100)
#define VECTORTABLE_FIRST_SPEC_IRQ_OFT (16) // offset of first specific interrupt
#endif


//******************************************************************************************
//Function pointer
//******************************************************************************************
typedef void (*pHandlerPointer_t)();

//******************************************************************************************
//Sram interrupt manager
//******************************************************************************************
#if INTERRUPTMANAGER_ENABLED==INTERRUPTMANAGER_SRAM

#define INTERRUPTMANAGER InterruptManagerSram

extern pHandlerPointer_t volatile sramVectorsTable[VECTORTABLE_SIZE];
#define VECTORTABLE__FIRST_SPEC_IRQ_ALIGNMENT   sramVectorsTable + VECTORTABLE_FIRST_SPEC_IRQ_OFT



class InterruptManagerSram {
private:
	static  pHandlerPointer_t  volatile *vectors;
public:
	static void init()
	{
		__disable_irq();
		memcpy((void*)sramVectorsTable, (void*)SCB->VTOR, sizeof(sramVectorsTable));
		SCB->VTOR=(uint32_t)sramVectorsTable;
		__DSB ();
		__enable_irq();
		__ISB();
	};
	static void defaultHandler() {
		asm volatile("nop"::);
	};

	static void addHandler(pHandlerPointer_t handler, IRQn_Type irq) {
		vectors[irq] = handler;
	};
	static void removeHanlder(IRQn_Type irq) {
		vectors[irq] = defaultHandler;
	};
	static void raise(IRQn_Type irq) {
		vectors[irq]();
	};

};
#endif


//******************************************************************************************
//Flash interrupt manager
//******************************************************************************************
#if INTERRUPTMANAGER_ENABLED==INTERRUPTMANAGER_FLASH

#define INTERRUPTMANAGER InterruptManagerFlash


class InterruptManagerFlash {
private:
	static pHandlerPointer_t volatile vectors[VECTORTABLE_SIZE];
public:
	static void init()	{	}
	static void defaultHandler() {
		asm volatile("nop"::);
	};

	static void addHandler(pHandlerPointer_t handler, IRQn_Type irq) {
		vectors[irq] = handler;
	};
	static void removeHanlder(IRQn_Type irq) {
		vectors[irq] = defaultHandler;
	};
	static void raise(IRQn_Type irq) {
		vectors[irq]();
	};
};

#endif



#endif /* INTERRUPT_INTERRUPTMANAGER_H_ */
