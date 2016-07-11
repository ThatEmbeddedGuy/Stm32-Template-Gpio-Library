/*
 * PulseGenerator.h
 *
 *  Created on: 7 июля 2016 г.
 *      Author: tihonov
 */
//******************************************************************************************
//
//Class for generating impulses by timer.
//use case
//
//PulseGenerator<TIM4> pulseGeneratorTim4(); //Default constructor sets positive polarity
//PulseGenerator<TIM2> pulseGeneratorTim2(PulseGeneratorPolarity_Negative);//You may set polarity in constructor
//pulseGeneratorTim2.init();
//pulseGeneratorTim4.init();
//pulseGeneratorTim4.GeneratePulseASync(GPIOA,10,25); Wait  timer 4 available, generates impulse, don't wait end of impulse  (On GPIOA port, 10 pin, for 25 microseconds)
//pulseGeneratorTim4.GeneratePulseSync(GPIOA,10,35); Wait  timer 4 available, generates impulse, wait end of impulse
//pulseGeneratorTim2.GeneratePulseSync(GPIOA,20,45);  Wait  timer 2 available, generates impulse, wait end of impulse
//Each timer can generate 1 impulse at the same time, functions waits for availability of timer, and then generates impulse.
//You can create instances of Pulse generators everywhere, they synchronizing automatically
//******************************************************************************************

#ifndef PULSEGENERATOR_H_
#define PULSEGENERATOR_H_

#include "stm32f407xx.h"
#include "Gpio.h"
#include "interruptmanager.h"

//******************************************************************************************
//Defines
//******************************************************************************************
#define DEFAULT_SYS_FREQ 168000000

//******************************************************************************************
//Public types
//******************************************************************************************
enum PulseGeneratorPolarity_t
{
	PulseGeneratorPolarity_Positive,
	PulseGeneratorPolarity_Negative
};
enum PulseGeneratorTimers_t
{
	PulseGeneratorTIM1,
	PulseGeneratorTIM2,
	PulseGeneratorTIM3,
	PulseGeneratorTIM4,
	PulseGeneratorTIM5,
	PulseGeneratorTIM6,
	PulseGeneratorTIM7,
	PulseGeneratorTIM8,
	PulseGeneratorTIM9,
	PulseGeneratorTIM10,
	PulseGeneratorTIM11,
	PulseGeneratorTIM12,
	PulseGeneratorTIM13,
	PulseGeneratorTIM14
};


template <PulseGeneratorTimers_t TIM>
class PulseGenerator
{
public:
	//******************************************************************************************
	//Public functions
	//******************************************************************************************
	PulseGenerator(bool Polarity){polarity=Polarity;}
	PulseGenerator():polarity(0){}
	void init(void);
	bool tryGeneratePulse(GPIO_TypeDef *Port,uint16_t  Pin, int32_t widthMs);
	void generatePulseAsync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs);
	void generatePulseSync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs);
	void setSysFreq(uint32_t sysFreq){freq=sysFreq;}
	static void irqHandler(void);
	~PulseGenerator();
private:
	//******************************************************************************************
	//Private variables
	//******************************************************************************************
	static bool isBusy;
	static GPIO_TypeDef *currentPort;
	static uint16_t currentPin;
	static bool Currentpolarity;
	static bool isInited;
	static uint32_t freq;
	bool polarity;
	//******************************************************************************************
	//Private functions
	//******************************************************************************************
	static inline  void SetUp(void) {currentPort->BSRR=1<<currentPin;};
	static inline  void SetDown(void){currentPort->BSRR=1<<(currentPin+16);}
	static inline  void SetState(bool state) {state  ? SetUp() : SetDown(); }
	static void InitInterrupt(void);
	void EnableTimer(void);
	static void TimCLockEnable(void) ;
	static constexpr TIM_TypeDef * getTimBase(void);
};

//******************************************************************************************
//Template definitions of static variables
//******************************************************************************************
template <PulseGeneratorTimers_t TIM>
bool PulseGenerator<TIM>::isBusy=0;

template <PulseGeneratorTimers_t TIM>
bool PulseGenerator<TIM>::isInited=0;

template <PulseGeneratorTimers_t TIM>
GPIO_TypeDef * PulseGenerator<TIM>::currentPort=NULL;

template <PulseGeneratorTimers_t TIM>
uint16_t PulseGenerator<TIM>::currentPin=0;

template <PulseGeneratorTimers_t TIM>
bool PulseGenerator<TIM>::Currentpolarity=0;

template <PulseGeneratorTimers_t TIM>
uint32_t PulseGenerator<TIM>::freq=DEFAULT_SYS_FREQ;

//******************************************************************************************
//Template definitions of public functions
//******************************************************************************************
template <PulseGeneratorTimers_t TIM>
void PulseGenerator<TIM>::init()
{
	if (!isInited)
	{

		TimCLockEnable();
		TIM_Base_InitTypeDef TimInitStruct;
		TimInitStruct.ClockDivision=TIM_CLOCKDIVISION_DIV1;
		TimInitStruct.CounterMode=TIM_COUNTERMODE_UP;
		TimInitStruct.Period=1;
		TimInitStruct.Prescaler=freq/2000000;
		TimInitStruct.RepetitionCounter=0;
		TIM_Base_SetConfig(getTimBase(),&TimInitStruct);
		InitInterrupt();
		isInited=true;
	}
}

template <PulseGeneratorTimers_t TIM>
void PulseGenerator<TIM>::generatePulseAsync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs){while (tryGeneratePulse(Port,Pin,widthMs));}

template <PulseGeneratorTimers_t TIM>
void PulseGenerator<TIM>::generatePulseSync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs)
{
	generatePulseAsync(Port,Pin,widthMs);
	while(isBusy);
}

template <PulseGeneratorTimers_t TIM>
bool PulseGenerator<TIM>::tryGeneratePulse(GPIO_TypeDef *Port,uint16_t  Pin, int32_t widthMs)
{
	if(!isBusy)
	{
		Currentpolarity=polarity;
		currentPort=Port;
		currentPin=Pin;
		isBusy=1;
		SetState(!Currentpolarity);
		getTimBase()->CNT=0;
		getTimBase()->ARR=widthMs-1;
		getTimBase()->DIER|=TIM_DIER_UIE;
		EnableTimer();
		return 0;
	}	else
		return 1;
}

template <PulseGeneratorTimers_t TIM>
 void PulseGenerator<TIM>::irqHandler()
{
	getTimBase()->SR&=~TIM_FLAG_UPDATE;
	getTimBase()->CR1&=~TIM_CR1_CEN;
	SetState(Currentpolarity);
	isBusy=0;
}
//******************************************************************************************
//Template definitions of private functions
//******************************************************************************************
template <PulseGeneratorTimers_t TIM>
constexpr TIM_TypeDef * PulseGenerator<TIM>::getTimBase()
{
	switch (TIM)
	{
	case (PulseGeneratorTIM1):return TIM1;
	case (PulseGeneratorTIM2):return TIM2;
	case (PulseGeneratorTIM3):return TIM3;
	case (PulseGeneratorTIM4):return TIM4;
	case (PulseGeneratorTIM5):return TIM5;
	case (PulseGeneratorTIM6):return TIM6;
	case (PulseGeneratorTIM7):return TIM7;
	case (PulseGeneratorTIM8):return TIM8;
	case (PulseGeneratorTIM9):return TIM9;
	case (PulseGeneratorTIM10):return TIM10;
	case (PulseGeneratorTIM11):return TIM11;
	case (PulseGeneratorTIM12):return TIM12;
	case (PulseGeneratorTIM13):return TIM13;
	case (PulseGeneratorTIM14):return TIM14;
	default:while(1);
	}
}

template <PulseGeneratorTimers_t TIM>
void PulseGenerator<TIM>::TimCLockEnable() {
	switch (TIM)
	{
	case(PulseGeneratorTIM1):{	__TIM1_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM2):{	__TIM2_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM3):{	__TIM3_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM4):{	__TIM4_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM5):{	__TIM5_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM6):{	__TIM6_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM7):{	__TIM7_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM8):{	__TIM8_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM9):{	__TIM9_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM10):{	__TIM10_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM11):{	__TIM11_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM12):{	__TIM12_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM13):{	__TIM13_CLK_ENABLE();break;	}
	case(PulseGeneratorTIM14):{	__TIM14_CLK_ENABLE();break;	}
	}
}
template <PulseGeneratorTimers_t TIM>
void PulseGenerator<TIM>::EnableTimer()
{
	getTimBase()->CNT=0;
	getTimBase()->CR1|=TIM_CR1_CEN;
};

template <PulseGeneratorTimers_t TIM>
 void PulseGenerator<TIM>::InitInterrupt()
{
	IRQn_Type IrqNumber;
	switch (TIM)
	{
	case(PulseGeneratorTIM1):{IrqNumber=TIM1_UP_TIM10_IRQn;break;	}
	case(PulseGeneratorTIM2):{IrqNumber=TIM2_IRQn;break;	}
	case(PulseGeneratorTIM3):{IrqNumber=TIM3_IRQn;break;	}
	case(PulseGeneratorTIM4):{IrqNumber=TIM4_IRQn;break;	}
	case(PulseGeneratorTIM5):{IrqNumber=TIM5_IRQn;break;	}
	case(PulseGeneratorTIM6):{IrqNumber=TIM6_DAC_IRQn;break;	}
	case(PulseGeneratorTIM7):{IrqNumber=TIM7_IRQn;break;	}
	case(PulseGeneratorTIM8):{IrqNumber=TIM8_UP_TIM13_IRQn;break;	}
	case(PulseGeneratorTIM9):{IrqNumber=TIM1_BRK_TIM9_IRQn;break;	}
	case(PulseGeneratorTIM10):{IrqNumber=TIM1_UP_TIM10_IRQn;break;	}
	case(PulseGeneratorTIM11):{IrqNumber=TIM1_TRG_COM_TIM11_IRQn;break;	}
	case(PulseGeneratorTIM12):{IrqNumber=TIM8_BRK_TIM12_IRQn;break;	}
	case(PulseGeneratorTIM13):{IrqNumber=TIM8_UP_TIM13_IRQn;break;	}
	case(PulseGeneratorTIM14):{IrqNumber=TIM8_TRG_COM_TIM14_IRQn;break;	}
	}
	HAL_NVIC_SetPriority(IrqNumber, 0xF, 0);
	HAL_NVIC_EnableIRQ (IrqNumber);
	InterruptManager::AddHandler(irqHandler, IrqNumber);
}


#endif /* PULSEGENERATOR_H_ */
