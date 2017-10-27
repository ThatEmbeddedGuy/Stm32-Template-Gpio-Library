/*
 * iInterruptable.h
 *
 *  Created on: 23 авг. 2016 г.
 *      Author: tihonov
 */

#ifndef INTERRUPT_IINTERRUPTABLE_H_
#define INTERRUPT_IINTERRUPTABLE_H_

#include "interruptmanager.h"
#include <array>
//******************************************************************************************
//Interface provides interrupt functional to any class
//******************************************************************************************
class iInterruptable
{
public:
	iInterruptable(){};
	virtual ~iInterruptable(){};
	virtual void	interruptHandle(){};
};

//******************************************************************************************
//Static template class of interrupt
//use case
//InterruptSubject<TIM4_IRQn,6> timIrqSubject;
//timIrqSubject.init();
//DebugString debugstring1("debug1"); // class extends interface iInterruptable
//DebugString debugstring2("debug2");
//timIrqSubject.AttachInterrupt(&debugstring1);
//timIrqSubject.AttachInterrupt(&debugstring2);
//******************************************************************************************
template <IRQn_Type irqN, uint8_t maxInterrupts>
class InterruptSubject{
private:
	static	void SetVector(){ INTERRUPTMANAGER::addHandler(interruptSubjectHandle, irqN); };
	static std::array <iInterruptable*, maxInterrupts> ArrayOfInterruptableClasses;
public:
	InterruptSubject(){}
	static	void interruptSubjectHandle();
	static void init(){ SetVector(); };
	static	void AttachInterrupt(iInterruptable * pInterrupt);
	static	void DetatchInterrupt(iInterruptable * pInterrupt);

};

//******************************************************************************************
//Template definitions of static variables
//******************************************************************************************
template <IRQn_Type irqN, uint8_t maxInterrupts>
std::array<iInterruptable*, maxInterrupts> InterruptSubject<irqN, maxInterrupts>::ArrayOfInterruptableClasses = { NULL };


//******************************************************************************************
//Template definitions of public functions
//******************************************************************************************
template <IRQn_Type irqN, uint8_t maxInterrupts>
void InterruptSubject<irqN, maxInterrupts>::interruptSubjectHandle()
{
	for (size_t i = 0; i < ArrayOfInterruptableClasses.max_size(); i++){
		if (ArrayOfInterruptableClasses[i] != NULL)	ArrayOfInterruptableClasses[i]->interruptHandle();
	};
}

template <IRQn_Type irqN, uint8_t maxInterrupts>
void InterruptSubject<irqN, maxInterrupts>::AttachInterrupt(iInterruptable * pInterrupt)
{
	uint8_t i = 0;
	bool attached = false;
	while (!attached)
	{
		if (ArrayOfInterruptableClasses[i] == NULL)
		{
			ArrayOfInterruptableClasses[i] = pInterrupt;
			attached = true;
		}
		i++;
		if (i == ArrayOfInterruptableClasses.max_size()) attached = true;
	}
}

template <IRQn_Type irqN, uint8_t maxInterrupts>
void InterruptSubject<irqN, maxInterrupts>::DetatchInterrupt(iInterruptable * pInterrupt)
{
	uint8_t i = 0;
	bool detached = false;
	while (!detached)
	{
		if (ArrayOfInterruptableClasses[i] == pInterrupt)
		{
			ArrayOfInterruptableClasses[i] = NULL;
			detached = true;
		}
		i++;
		if (i == ArrayOfInterruptableClasses.max_size()) detached = true;
	}
}



#endif /* INTERRUPT_IINTERRUPTABLE_H_ */
