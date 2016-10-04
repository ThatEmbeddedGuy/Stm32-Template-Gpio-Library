/*
 * exti.h
 *
 *  Created on: 25 нояб. 2015 г.
 *      Author: tihonov
 */

#ifndef EXTI_EXTI_H_
#define EXTI_EXTI_H_

#include <stdio.h>
#include "stm32f4xx.h"

enum ExtiMode_t {
	ExtiMode_Interrupt = 0x00, ExtiMode_Event = 0x04
};

enum ExtiTrigger_t {
	ExtiTrigger_Rising = 0x08,
	ExtiTrigger_Falling = 0x0C,
	ExtiTrigger_RisingFalling = 0x10
};

enum ExtiState_t {
	ExtiState_Disabled, ExtiState_Enabled
};

enum ExtiLine_t {
	ExtiLine_0 = ((uint32_t) 0x00001), /*!< External interrupt line 0 */
	ExtiLine_1 = ((uint32_t) 0x00002), /*!< External interrupt line 1 */
	ExtiLine_2 = ((uint32_t) 0x00004), /*!< External interrupt line 2 */
	ExtiLine_3 = ((uint32_t) 0x00008), /*!< External interrupt line 3 */
	ExtiLine_4 = ((uint32_t) 0x00010), /*!< External interrupt line 4 */
	ExtiLine_5 = ((uint32_t) 0x00020), /*!< External interrupt line 5 */
	ExtiLine_6 = ((uint32_t) 0x00040), /*!< External interrupt line 6 */
	ExtiLine_7 = ((uint32_t) 0x00080), /*!< External interrupt line 7 */
	ExtiLine_8 = ((uint32_t) 0x00100), /*!< External interrupt line 8 */
	ExtiLine_9 = ((uint32_t) 0x00200), /*!< External interrupt line 9 */
	ExtiLine_10 = ((uint32_t) 0x00400), /*!< External interrupt line 10 */
	ExtiLine_11 = ((uint32_t) 0x00800), /*!< External interrupt line 11 */
	ExtiLine_12 = ((uint32_t) 0x01000), /*!< External interrupt line 12 */
	ExtiLine_13 = ((uint32_t) 0x02000), /*!< External interrupt line 13 */
	ExtiLine_14 = ((uint32_t) 0x04000), /*!< External interrupt line 14 */
	ExtiLine_15 = ((uint32_t) 0x08000), /*!< External interrupt line 15 */
	ExtiLine_16 = ((uint32_t) 0x10000), /*!< External interrupt line 16 Connected to the PVD Output */
	ExtiLine_17 = ((uint32_t) 0x20000), /*!< External interrupt line 17 Connected to the RTC Alarm event */
	ExtiLine_18 = ((uint32_t) 0x40000), /*!< External interrupt line 18 Connected to the USB OTG FS Wakeup from suspend event */
	ExtiLine_19 = ((uint32_t) 0x80000), /*!< External interrupt line 19 Connected to the Ethernet Wakeup event */
	ExtiLine_20 = ((uint32_t) 0x00100000), /*!< External interrupt line 20 Connected to the USB OTG HS (configured in FS) Wakeup event  */
	ExtiLine_21 = ((uint32_t) 0x00200000), /*!< External interrupt line 21 Connected to the RTC Tamper and Time Stamp events */
	ExtiLine_22 = ((uint32_t) 0x00400000) /*!< External interrupt line 22 Connected to the RTC Wakeup event */
};

//typedef ExtiGpio<PortE,7,ExtiMode_Interrupt,ExtiTrigger_RisingFalling> Trigger1;
//Trigger1::Init;

template<GpioPort_t port, uint8_t pin, ExtiMode_t mode, ExtiTrigger_t Trigger>

class ExtiGpio {
public:
	ExtiGpio();
	static void Init() {
		constexpr uint32_t line = 1 << pin;

		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
		RCC->AHB1ENR = RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN
				| RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN
				| RCC_AHB1ENR_GPIOEEN;

		uint32_t tmp;

		tmp = SYSCFG->EXTICR[pin >> 2];
		tmp &= ~(((uint32_t)0x0F) << (4 * (pin & 0x03)));
		tmp |= ((uint32_t)port << (4 * (pin & 0x03)));
        SYSCFG->EXTICR[pin >> 2] = tmp;

		//Снимаем все флаги
		EXTI->IMR &= ~line;
		EXTI->EMR &= ~line;

		mode == ExtiMode_Interrupt ?
		EXTI->IMR |= line :
		EXTI->EMR |= line;

		switch (Trigger) {
		case ExtiTrigger_Rising:
			EXTI->RTSR |= line;
			break;

		case ExtiTrigger_Falling:
			EXTI->FTSR |= line;
			break;

		case ExtiTrigger_RisingFalling:
			EXTI->RTSR |= line;
			EXTI->FTSR |= line;
			break;


		}

	}

	static void ClearFlag()
	{
		constexpr uint32_t line = 1 << pin;
		EXTI->PR=line;
	}
	static void DeInit()

	{
		constexpr uint32_t line = 1 << pin;
		SYSCFG->EXTICR[pin >> 0x02] &= ~((uint32_t) 0x0F)
				<< (0x04 * (pin & (uint8_t) 0x03));

		EXTI->IMR &= ~line;
		EXTI->EMR &= ~line;
	}
	;

	virtual ~ExtiGpio();

};

#endif /* EXTI_EXTI_H_ */
