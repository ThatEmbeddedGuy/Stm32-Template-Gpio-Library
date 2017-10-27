/*
 * interruptmanager.cpp
 *
 *  Created on: 23 нояб. 2015 г.
 *      Author: tihonov
 */
#include "interruptmanager.h"



//******************************************************************************************
//Sram interrupt manager
//******************************************************************************************
#if INTERRUPTMANAGER_ENABLED==INTERRUPTMANAGER_SRAM
pHandlerPointer_t volatile sramVectorsTable[VECTORTABLE_SIZE]  __attribute__((aligned(256)));;
pHandlerPointer_t  volatile *InterruptManagerSram::vectors = (pHandlerPointer_t volatile*)VECTORTABLE__FIRST_SPEC_IRQ_ALIGNMENT;
#endif

//******************************************************************************************
//Flash interrupt manager
//******************************************************************************************
#if INTERRUPTMANAGER_ENABLED==INTERRUPTMANAGER_FLASH
pHandlerPointer_t volatile InterruptManagerFlash::vectors[VECTORTABLE_SIZE] { 0 };
#endif



