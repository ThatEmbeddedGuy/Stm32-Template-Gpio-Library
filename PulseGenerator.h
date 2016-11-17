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
//PulseGenerator<PulseGeneratorTimers::Tim4> pulseGeneratorTim4(); //Default constructor sets positive polarity
//PulseGenerator<PulseGeneratorTimers::Tim2> pulseGeneratorTim2(PulseGeneratorPolarity::Negative);//You may set polarity in constructor
//pulseGeneratorTim2.init();
//pulseGeneratorTim4.init();
//pulseGeneratorTim4.GeneratePulseASync(GPIOA,10,25); Wait  timer 4 availability, generates impulse, don't wait end of impulse  (On GPIOA port, 10 pin, for 25 microseconds)
//pulseGeneratorTim4.GeneratePulseSync(GPIOA,10,35); Wait  timer 4 availability, generates impulse, wait end of impulse
//pulseGeneratorTim2.GeneratePulseSync(GPIOA,20,45);  Wait  timer 2 availability, generates impulse, wait end of impulse
//Each timer can generate 1 impulse at the same time, functions waits for availability of timer, and then generates impulse.
//You can create instances of Pulse generators everywhere, they synchronizing automatically
//******************************************************************************************

#ifndef PULSEGENERATOR_H_
#define PULSEGENERATOR_H_
#include <atomic>
#include "cmsis_device.h"
#include "interruptmanager.h"


//******************************************************************************************
//Defines
//******************************************************************************************
#define DEFAULT_SYS_FREQ 168000000
#define DEFAULT_TIM_NVIC_PRIORITY 0xF

//******************************************************************************************
//Public types
//******************************************************************************************
enum class PulseGeneratorPolarity
{
	Positive,
	Negative
};
enum class PulseGeneratorTimers
{
	Tim1,
	Tim2,
	Tim3,
	Tim4,
	Tim5,
	Tim6,
	Tim7,
	Tim8,
	Tim9,
	Tim10,
	Tim11,
	Tim12,
	Tim13,
	Tim14
};


template <PulseGeneratorTimers TIM>
class PulseGenerator
{
public:
	//******************************************************************************************
	//Public functions
	//******************************************************************************************
	PulseGenerator(bool Polarity):polarity(Polarity){}
	PulseGenerator():polarity(0){}
	void init(void);
	bool tryGeneratePulse(GPIO_TypeDef *Port,uint16_t  Pin, int32_t widthMs);
	void generatePulseAsync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs);
	void generatePulseSync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs);
	void setSysFreq(uint32_t sysFreq){freq=sysFreq;}
	void setPolarity(bool pol){polarity=pol;}
	volatile static  void irqHandler(void);
	virtual ~PulseGenerator(){};
private:
	//******************************************************************************************
	//Private variables
	//******************************************************************************************
	volatile static std::atomic<bool>    isBusy;
	static GPIO_TypeDef *currentPort;
	static uint16_t currentPin;
	static bool currentPolarity;
	static bool isInited;
	static uint32_t freq;
	bool polarity;
	//******************************************************************************************
	//Private functions
	//******************************************************************************************
	static inline  void setPinUp(void) {currentPort->BSRR=1<<currentPin;};
	static inline  void setPinDown(void){currentPort->BSRR=1<<(currentPin+16);}
	static inline  void setPinState(bool state) {state  ? setPinUp() : setPinDown(); }
	static void initInterrupt(void);
	static void initTimer(void);
	void enableTimer(void);
	static void timCLockEnable(void) ;
	static constexpr TIM_TypeDef * getTimBase(void);
};

//******************************************************************************************
//Template definitions of static variables
//******************************************************************************************
template <PulseGeneratorTimers TIM>
volatile  std::atomic<bool>  PulseGenerator<TIM>::isBusy(0);

template <PulseGeneratorTimers TIM>
bool PulseGenerator<TIM>::isInited=0;

template <PulseGeneratorTimers TIM>
GPIO_TypeDef * PulseGenerator<TIM>::currentPort=NULL;

template <PulseGeneratorTimers TIM>
uint16_t PulseGenerator<TIM>::currentPin=0;

template <PulseGeneratorTimers TIM>
bool PulseGenerator<TIM>::currentPolarity=0;

template <PulseGeneratorTimers TIM>
uint32_t PulseGenerator<TIM>::freq=DEFAULT_SYS_FREQ;

//******************************************************************************************
//Template definitions of public functions
//******************************************************************************************

#ifndef TIM_CLOCKDIVISION_DIV1
#define TIM_CLOCKDIVISION_DIV1 0
#endif

#ifndef TIM_COUNTERMODE_UP
#define TIM_COUNTERMODE_UP 0
#endif


template <PulseGeneratorTimers TIM>
void PulseGenerator<TIM>::init()
{
	if (!isInited)
	{
		timCLockEnable();
		initTimer();
		initInterrupt();
		isInited=true;
	}
}

template <PulseGeneratorTimers TIM>
void PulseGenerator<TIM>::initTimer()
{
	constexpr uint32_t RepetitionCounter = 0;
	constexpr uint32_t Period = 1;
	constexpr uint32_t ClockDivision = TIM_CLOCKDIVISION_DIV1;
	constexpr uint32_t CounterMode = TIM_COUNTERMODE_UP;
	const uint32_t Prescaler=freq/2000000;

	uint32_t tmpcr1 = 0;
	tmpcr1 = getTimBase()->CR1;

	if(IS_TIM_CC3_INSTANCE(getTimBase()) != RESET)
	{
		tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
		tmpcr1 |= CounterMode;
	}
	if(IS_TIM_CC1_INSTANCE(getTimBase()) != RESET)
	{
		tmpcr1 &= ~TIM_CR1_CKD;
		tmpcr1 |= ClockDivision;
	}
	getTimBase()->CR1 = tmpcr1;
	getTimBase()->ARR = Period ;
	getTimBase()->PSC = Prescaler;
	if(IS_TIM_ADVANCED_INSTANCE(getTimBase()) != RESET)
	{
		getTimBase()->RCR = RepetitionCounter;
	}
	getTimBase()->EGR = TIM_EGR_UG;
}


template <PulseGeneratorTimers TIM>
void PulseGenerator<TIM>::generatePulseAsync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs){while (tryGeneratePulse(Port,Pin,widthMs));}

template <PulseGeneratorTimers TIM>
void PulseGenerator<TIM>::generatePulseSync(GPIO_TypeDef *Port,uint16_t  Pin,uint32_t widthMs)
{
	generatePulseAsync(Port,Pin,widthMs);
	while(isBusy);
}

template <PulseGeneratorTimers TIM>
bool PulseGenerator<TIM>::tryGeneratePulse(GPIO_TypeDef *Port,uint16_t  Pin, int32_t widthMs)
{
	bool freeState=0;
	if(isBusy.compare_exchange_strong(freeState, true,std::memory_order_acq_rel))
	{
		currentPolarity=polarity;
		currentPort=Port;
		currentPin=Pin;
		setPinState(!currentPolarity);
		getTimBase()->CNT=0;
		getTimBase()->ARR=widthMs-1;
		getTimBase()->DIER|=TIM_DIER_UIE;
		enableTimer();
		return 0;
	}	else
		return 1;
}

template <PulseGeneratorTimers TIM>
volatile void PulseGenerator<TIM>::irqHandler()
{
	getTimBase()->SR&=~TIM_SR_UIF;
	getTimBase()->CR1&=~TIM_CR1_CEN;
	setPinState(currentPolarity);
	isBusy=0;
}
//******************************************************************************************
//Template definitions of private functions
//******************************************************************************************
template <PulseGeneratorTimers TIM>
constexpr TIM_TypeDef * PulseGenerator<TIM>::getTimBase()
{
	switch (TIM)
	{
	case (PulseGeneratorTimers::Tim1):return TIM1;
	case (PulseGeneratorTimers::Tim2):return TIM2;
	case (PulseGeneratorTimers::Tim3):return TIM3;
	case (PulseGeneratorTimers::Tim4):return TIM4;
	case (PulseGeneratorTimers::Tim5):return TIM5;
	case (PulseGeneratorTimers::Tim6):return TIM6;
	case (PulseGeneratorTimers::Tim7):return TIM7;
	case (PulseGeneratorTimers::Tim8):return TIM8;
	case (PulseGeneratorTimers::Tim9):return TIM9;
	case (PulseGeneratorTimers::Tim10):return TIM10;
	case (PulseGeneratorTimers::Tim11):return TIM11;
	case (PulseGeneratorTimers::Tim12):return TIM12;
	case (PulseGeneratorTimers::Tim13):return TIM13;
	case (PulseGeneratorTimers::Tim14):return TIM14;
	default:while(1);
	}
}

template <PulseGeneratorTimers TIM>
void PulseGenerator<TIM>::timCLockEnable() {
	switch (TIM)
	{
	case(PulseGeneratorTimers::Tim1):{	RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;break;	}
	case(PulseGeneratorTimers::Tim2):{	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;break;	}
	case(PulseGeneratorTimers::Tim3):{	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;break;	}
	case(PulseGeneratorTimers::Tim4):{	RCC->APB1ENR|=RCC_APB1ENR_TIM4EN;break;	}
	case(PulseGeneratorTimers::Tim5):{	RCC->APB1ENR|=RCC_APB1ENR_TIM5EN;break;	}
	case(PulseGeneratorTimers::Tim6):{	RCC->APB1ENR|=RCC_APB1ENR_TIM6EN;break;	}
	case(PulseGeneratorTimers::Tim7):{	RCC->APB1ENR|=RCC_APB1ENR_TIM7EN;break;	}
	case(PulseGeneratorTimers::Tim8):{	RCC->APB2ENR|=RCC_APB2ENR_TIM8EN;break;	}
	case(PulseGeneratorTimers::Tim9):{	RCC->APB2ENR|=RCC_APB2ENR_TIM9EN;break;	}
	case(PulseGeneratorTimers::Tim10):{	RCC->APB2ENR|=RCC_APB2ENR_TIM10EN;break;}
	case(PulseGeneratorTimers::Tim11):{	RCC->APB2ENR|=RCC_APB2ENR_TIM11EN;break;}
	case(PulseGeneratorTimers::Tim12):{	RCC->APB1ENR|=RCC_APB1ENR_TIM12EN;break;}
	case(PulseGeneratorTimers::Tim13):{	RCC->APB1ENR|=RCC_APB1ENR_TIM13EN;break;}
	case(PulseGeneratorTimers::Tim14):{	RCC->APB1ENR|=RCC_APB1ENR_TIM14EN;break;}
	}
}
template <PulseGeneratorTimers TIM>
void PulseGenerator<TIM>::enableTimer()
{
	getTimBase()->CNT=0;
	getTimBase()->CR1|=TIM_CR1_CEN;
};

template <PulseGeneratorTimers TIM>
void PulseGenerator<TIM>::initInterrupt()
{
	IRQn_Type irqNumber;
	switch (TIM)
	{
	case(PulseGeneratorTimers::Tim1):{irqNumber=TIM1_UP_TIM10_IRQn;break;		}
	case(PulseGeneratorTimers::Tim2):{irqNumber=TIM2_IRQn;break;				}
	case(PulseGeneratorTimers::Tim3):{irqNumber=TIM3_IRQn;break;				}
	case(PulseGeneratorTimers::Tim4):{irqNumber=TIM4_IRQn;break;				}
	case(PulseGeneratorTimers::Tim5):{irqNumber=TIM5_IRQn;break;				}
	case(PulseGeneratorTimers::Tim6):{irqNumber=TIM6_DAC_IRQn;break;			}
	case(PulseGeneratorTimers::Tim7):{irqNumber=TIM7_IRQn;break;				}
	case(PulseGeneratorTimers::Tim8):{irqNumber=TIM8_UP_TIM13_IRQn;break;		}
	case(PulseGeneratorTimers::Tim9):{irqNumber=TIM1_BRK_TIM9_IRQn;break;		}
	case(PulseGeneratorTimers::Tim10):{irqNumber=TIM1_UP_TIM10_IRQn;break;		}
	case(PulseGeneratorTimers::Tim11):{irqNumber=TIM1_TRG_COM_TIM11_IRQn;break;	}
	case(PulseGeneratorTimers::Tim12):{irqNumber=TIM8_BRK_TIM12_IRQn;break;		}
	case(PulseGeneratorTimers::Tim13):{irqNumber=TIM8_UP_TIM13_IRQn;break;		}
	case(PulseGeneratorTimers::Tim14):{irqNumber=TIM8_TRG_COM_TIM14_IRQn;break;	}
	}

	INTERRUPTMANAGER::addHandler(pHandlerPointer_t(irqHandler), irqNumber);

	NVIC_SetPriority(irqNumber, DEFAULT_TIM_NVIC_PRIORITY);
	NVIC_EnableIRQ (irqNumber);

}


#endif /* PULSEGENERATOR_H_ */
