/*
 * Gpio.h
 *
 *  Created on: 18 но€б. 2015 г.
 *      Author: tihonov
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdio.h>
#include "stm32f4xx.h"

//------------------------------------------
// Template class for gpio ports in stm32
// use case
//typedef Gpio<PortE,7,GpioMode_Out,GpioOutType_PP,GpioSpeed_100MHz,GpioPuPd_NoPull> led1;
//
//led1::Init;
//led1::SetState(true);
//led::Toggle();
//------------------------------------------

enum GpioPort_t {
	PortA,
	PortB,
	PortC,
	PortD,
	PortE,
	PortF
};

enum GpioMode_t {
	GpioMode_In = 0x00,
	GpioMode_Out = 0x01,
	GpioMode_AlternateFunction = 0x02,
	GpioMode_Analog = 0x03,
	GpioMode_Exti=0xFF

};

enum GpioOutType_t {
	GpioOutType_PP = 0x00,
	GpioOutType_OD = 0x01
};

enum GpioSpeed_t {
	GpioSpeed_2MHz = 0x00,
	GpioSpeed_25MHz = 0x01,
	GpioSpeed_50MHz = 0x02,
	GpioSpeed_100MHz = 0x03
};

enum GpioPuPd_t {
	GpioPuPd_NoPull = 0x00,
	GpioPuPd_PullUp = 0x01,
	GpioPuPd_PullDown = 0x02
};


enum GpioAf_t
{
    GpioAf_0             =0x00,
    GpioAf_RTC_50Hz      =GpioAf_0,  /*  RTC_50Hz Alternate Function mapping */
    GpioAf_MCO           =GpioAf_0,  /*  MCO (MCO1 and MCO2) Alternate Function mapping */
    GpioAf_TAMPER        =GpioAf_0,  /*  TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
    GpioAf_SWJ           =GpioAf_0,  /*  SWJ (SWD and JTAG) Alternate Function mapping */
    GpioAf_TRACE         =GpioAf_0,  /*  TRACE Alternate Function mapping */

    GpioAf_1             =0x01,
    GpioAf_TIM1          =GpioAf_1,  /*  TIM1 Alternate Function mapping */
    GpioAf_TIM2          =GpioAf_1,  /*  TIM2 Alternate Function mapping */

    GpioAf_2             =0x02,
    GpioAf_TIM3          =GpioAf_2,  /*  TIM3 Alternate Function mapping */
    GpioAf_TIM4          =GpioAf_2,  /*  TIM4 Alternate Function mapping */
    GpioAf_TIM5          =GpioAf_2,  /*  TIM5 Alternate Function mapping */

    GpioAf_3             =0x03,
    GpioAf_TIM8          =GpioAf_3,  /*  TIM8 Alternate Function mapping */
    GpioAf_TIM9          =GpioAf_3,  /*  TIM9 Alternate Function mapping */
    GpioAf_TIM10         =GpioAf_3,  /*  TIM10 Alternate Function mapping */
    GpioAf_TIM11         =GpioAf_3,  /*  TIM11 Alternate Function mapping */

    GpioAf_4             =0x04,
    GpioAf_I2C1          =GpioAf_4,  /*  I2C1 Alternate Function mapping */
    GpioAf_I2C2          =GpioAf_4,  /*  I2C2 Alternate Function mapping */
    GpioAf_I2C3          =GpioAf_4,  /*  I2C3 Alternate Function mapping */

    GpioAf_5             =0x05,
    GpioAf_SPI1          =GpioAf_5,  /*  SPI1 Alternate Function mapping */
    GpioAf_SPI2          =GpioAf_5,  /*  SPI2/I2S2 Alternate Function mapping */
    GpioAf_I2S3ext       =GpioAf_5, /* I2S3ext_SD Alternate Function mapping  */

    GpioAf_6             =0x06,
    GpioAf_6_SPI3        =GpioAf_6,  /*  SPI3/I2S3 Alternate Function mapping */
    GpioAf_I2S2ext       =GpioAf_6, /* I2S2ext_SD Alternate Function mapping */

    GpioAf_7              =0x07,
    GpioAf_USART1         =GpioAf_7,  /*  USART1 Alternate Function mapping */
    GpioAf_USART2         =GpioAf_7,  /*  USART2 Alternate Function mapping */
    GpioAf_USART3         =GpioAf_7,  /*  USART3 Alternate Function mapping */

    GpioAf_8              =0x08,
    GpioAf_UART4          =GpioAf_8,  /*  UART4 Alternate Function mapping */
    GpioAf_UART5          =GpioAf_8,  /*  UART5 Alternate Function mapping */
    GpioAf_USART6         =GpioAf_8,  /*  USART6 Alternate Function mapping */

    GpioAf_9              =0x09,
    GpioAf_CAN1           =GpioAf_9,  /*  CAN1 Alternate Function mapping */
    GpioAf_CAN2           =GpioAf_9,  /*  CAN2 Alternate Function mapping */
    GpioAf_TIM12          =GpioAf_9,  /*  TIM12 Alternate Function mapping */
    GpioAf_TIM13          =GpioAf_9,  /*  TIM13 Alternate Function mapping */
    GpioAf_TIM14          =GpioAf_9,  /*  TIM14 Alternate Function mapping */

    GpioAf_10             =0x0A,
    GpioAf_OTG_FS         =GpioAf_10,  /*  OTG_FS Alternate Function mapping */
    GpioAf_OTG_HS         =GpioAf_10,  /*  OTG_HS Alternate Function mapping */

    GpioAf_11             =0x0B,
    GpioAf_ETH            =GpioAf_11,  /*  ETHERNET Alternate Function mapping */

    GpioAf_12             =0x0C,
    GpioAf_FSMC           =GpioAf_12,  /*  FSMC Alternate Function mapping */
    GpioAf_OTG_HS_FS      =GpioAf_12,  /*  OTG HS configured in FS, Alternate Function mapping */

    GpioAf_SDIO           =GpioAf_12,  /*  SDIO Alternate Function mapping */
    GpioAf_13             =0x0D,
    GpioAf_DCMI           =GpioAf_13,  /*  DCMI Alternate Function mapping */

    GpioAf_15             =0x0F,
    GpioAf_EVENTOUT       =GpioAf_15   /*  EVENTOUT Alternate Function mapping */
};







template<GpioPort_t port,
		 uint8_t pin,
		 GpioMode_t mode,
		 GpioOutType_t GpioOutType,
		 GpioSpeed_t GpioSpeed,
		 GpioPuPd_t GpioPuPd>

class Gpio {
public:
	Gpio();
	 static void Init()
	{
		RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOEEN;

		if (IsNotExti()) GetPortBase()->MODER|=	 mode<<pin*2 ;
		GetPortBase()->OSPEEDR|= GpioSpeed<<pin*2;
		GetPortBase()->OTYPER|=  GpioOutType<<pin;
		GetPortBase()->PUPDR|=   GpioPuPd<<pin*2;

	}

	inline static void DeInit(){};
	inline static void SetAf(GpioAf_t af){

        const uint32_t MskOft   = (uint32_t)(pin & (uint32_t)0x07) * 4;
        const uint32_t AfMask   = ~((uint32_t)0xf << MskOft) ;
        const uint32_t AfValue  = ((uint32_t)(af) << MskOft);
        const uint32_t AfrIdx   = pin >> 3;

        GetPortBase()->AFR[AfrIdx] &= AfMask;
        GetPortBase()->AFR[AfrIdx] |= AfValue;


 };
	inline static void SetState(bool state) {state  ? SetUp() : SetDown()  ;  }
	inline static void SetUp(void) {GetPortBase()->BSRR=1<<pin;};
	inline static void SetDown(void){GetPortBase()->BSRR=1<<(pin+16);}
	inline static bool GetState(void) {return GetPortBase()->IDR&1<<pin;};
	inline static void Toggle(void) {SetState(!GetState());};

	virtual ~Gpio();

private:
	static constexpr GPIO_TypeDef* GetPortBase(void)	{ return ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(port))); };
	static constexpr bool IsNotExti(void) {return  !(mode==GpioMode_Exti);  };
};



#endif /* GPIO_H_ */
