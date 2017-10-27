/*
 * Gpio.h
 *
 *  Created on: 18 нояб. 2015 г.
 *      Author: tihonov
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdio.h>
#include "stm32f4xx.h"


enum class Port: unsigned char {
	A=0,
	B,
	C,
	D,
	E,
	F
};

enum class GpioMode {
	In = 0x00,
	Out = 0x01,
	AlternateFunction = 0x02,
	Analog = 0x03,
	Exti=0xFF

};

enum class GpioOutType {
	PP = 0x00,
	OD = 0x01
};

enum class GpioSpeed {
	s2MHz = 0x00,
	s25MHz = 0x01,
	s50MHz = 0x02,
	s100MHz = 0x03
};

enum class GpioPuPd {
	NoPull = 0x00,
	PullUp = 0x01,
	PullDown = 0x02
};


enum class GpioAf
{
	Af0           =0x00,
	rtc50Hz       =Af0,  /*  RTC_50Hz Alternate Function mapping */
	mco           =Af0,  /*  MCO (MCO1 and MCO2) Alternate Function mapping */
	tamper        =Af0,  /*  TAMPER (TAMPER_1 and TAMPER_2) Alternate Function mapping */
	swj           =Af0,  /*  SWJ (SWD and JTAG) Alternate Function mapping */
	trace         =Af0,  /*  TRACE Alternate Function mapping */

	Af1            =0x01,
	tim1          =Af1,  /*  TIM1 Alternate Function mapping */
	tim2          =Af1,  /*  TIM2 Alternate Function mapping */

	Af2            =0x02,
	tim3          =Af2,  /*  TIM3 Alternate Function mapping */
	tim4          =Af2,  /*  TIM4 Alternate Function mapping */
	tim5          =Af2,  /*  TIM5 Alternate Function mapping */

	Af3           =0x03,
	tim8          =Af3,  /*  TIM8 Alternate Function mapping */
	tim9          =Af3,  /*  TIM9 Alternate Function mapping */
	tim10         =Af3,  /*  TIM10 Alternate Function mapping */
	tim11         =Af3,  /*  TIM11 Alternate Function mapping */

	Af4           =0x04,
	i2c1          =Af4,  /*  I2C1 Alternate Function mapping */
	i2c2          =Af4,  /*  I2C2 Alternate Function mapping */
	i2c3          =Af4,  /*  I2C3 Alternate Function mapping */

	Af5           =0x05,
	spi1          =Af5,  /*  SPI1 Alternate Function mapping */
	spi2          =Af5,  /*  SPI2/I2S2 Alternate Function mapping */
	i2s3ext       =Af5, /* I2S3ext_SD Alternate Function mapping  */

	Af6           =0x06,
	spi3          =Af6,  /*  SPI3/I2S3 Alternate Function mapping */
	i2s2ext       =Af6, /* I2S2ext_SD Alternate Function mapping */

	Af7            =0x07,
	usart1         =Af7,  /*  usart Alternate Function mapping */
	usart2         =Af7,  /*  USART2 Alternate Function mapping */
	usart3         =Af7,  /*  USART3 Alternate Function mapping */

	Af8            =0x08,
	uart4          =Af8,  /*  UART4 Alternate Function mapping */
	uart5          =Af8,  /*  UART5 Alternate Function mapping */
	uart6          =Af8,  /*  USART6 Alternate Function mapping */

	Af9             =0x09,
	can1           =Af9,  /*  CAN1 Alternate Function mapping */
	can2           =Af9,  /*  CAN2 Alternate Function mapping */
	tim12          =Af9,  /*  TIM12 Alternate Function mapping */
	tim13          =Af9,  /*  TIM13 Alternate Function mapping */
	tim14          =Af9,  /*  TIM14 Alternate Function mapping */

	Af10           =0x0A,
	otgFs          =Af10,  /*  OTG_FS Alternate Function mapping */
	otgHs          =Af10,  /*  OTG_HS Alternate Function mapping */

	Af11           =0x0B,
	eth            =Af11,  /*  ETHERNET Alternate Function mapping */

	Af12           =0x0C,
	fsmc 	       =Af12,  /*  FSMC Alternate Function mapping */
	otgHsFs        =Af12,  /*  OTG HS configured in FS, Alternate Function mapping */

	sdio           =Af12,  /*  SDIO Alternate Function mapping */
	Af13             =0x0D,
	dcmi           =Af13,  /*  DCMI Alternate Function mapping */

	Af15            =0x0F,
	eventout       =Af15   /*  EVENTOUT Alternate Function mapping */
};

//Example
//Gpio led1(Port::A,7);
//led1.init(GpioMode::Out,GpioOutType::PP,GpioSpeed::s100MHz,GpioPuPd::PullDown);
//led1.toggle();
//constexpr Gpio led2(Port::A,7);  //параметры передаются на этапе компиляции
//led2.init(GpioMode::Out,GpioOutType::PP,GpioSpeed::s100MHz,GpioPuPd::PullDown);
//led2.toggle();

class Gpio {
public:
	constexpr Gpio(Port port,uint8_t pin):mPort(getPortBase(port)),mPin(pin){};
	 ~Gpio()=default;
	  void init const(
			GpioMode mode,
			GpioOutType GpioOutType,
			GpioSpeed GpioSpeed,
			GpioPuPd GpioPuPd)
	{
		RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOEEN;
		if (isNotExti(mode)) mPort->MODER|=	 (uint8_t)mode<<mPin*2 ;
		mPort->OSPEEDR|= (uint8_t)GpioSpeed<<mPin*2;
		mPort->OTYPER|=  (uint8_t)GpioOutType<<mPin;
		mPort->PUPDR|=   (uint8_t)GpioPuPd<<mPin*2;
	}
	  void setAf(GpioAf af) const{
		const uint32_t MskOft   = (uint32_t)(mPin & (uint32_t)0x07) * 4;
		const uint32_t AfMask   = ~((uint32_t)0xf << MskOft) ;
		const uint32_t AfValue  = ((uint32_t)(af) << MskOft);
		const uint32_t AfrIdx   = mPin >> 3;

		mPort->AFR[AfrIdx] &= AfMask;
		mPort->AFR[AfrIdx] |= AfValue;
	};

	  void setState(bool state) const{state  ? setUp() : setDown()  ;  }
	  void setUp() const  {mPort->BSRR=1<<mPin;};
	  void setDown()const{mPort->BSRR=1<<(mPin+16);}
	  bool getState()const {return mPort->IDR&1<<mPin;};
	  void toggle()const {setState(!getState());};

private:
 constexpr static 	GPIO_TypeDef* getPortBase(Port port) const	{
	switch (port)
	{
	case Port::A :
		return GPIOA;
		break;
	case Port::B :
		return GPIOB;
		break;
	case Port::C :
		return GPIOC;
		break;
	case Port::D :
		return GPIOD;
		break;
	case Port::E :
		return GPIOE;
		break;
	case Port::F :
		return GPIOF;
		break;
	}
 };

	  bool isNotExti(GpioMode mode)const {return  !(mode==GpioMode::Exti);  };
	GPIO_TypeDef* mPort;
	uint16_t mPin;
};



//Example
//typedef static_Gpio<Port::A,7,GpioMode::Out,GpioOutType::PP,GpioSpeed::s100MHz,GpioPuPd::PullDown> led1;
//led1::init();
//led1::toggle();

template<Port port,
uint8_t pin,
GpioMode mode,
GpioOutType GpioOutType,
GpioSpeed GpioSpeed,
GpioPuPd GpioPuPd>

class static_Gpio {
public:
	static_Gpio(){};
	 static void init()
	{
		RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN|RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIODEN|RCC_AHB1ENR_GPIOEEN;

		if (isNotExti()) getPortBase()->MODER|=	 (uint8_t)mode<<pin*2 ;
		getPortBase()->OSPEEDR|= (uint8_t)GpioSpeed<<pin*2;
		getPortBase()->OTYPER|=  (uint8_t)GpioOutType<<pin;
		getPortBase()->PUPDR|=   (uint8_t)GpioPuPd<<pin*2;

	}
	 static void deInit(){};
	 static void setAf(GpioAf af){

		const uint32_t MskOft   = (uint32_t)(pin & (uint32_t)0x07) * 4;
		const uint32_t AfMask   = ~((uint32_t)0xf << MskOft) ;
		const uint32_t AfValue  = ((uint32_t)(af) << MskOft);
		const uint32_t AfrIdx   = pin >> 3;

		getPortBase()->AFR[AfrIdx] &= AfMask;
		getPortBase()->AFR[AfrIdx] |= AfValue;
	};

	 static void setState(bool state) {state  ? setUp() : setDown()  ;  }
	 static void setUp() {getPortBase()->BSRR=1<<pin;};
	 static void setDown(){getPortBase()->BSRR=1<<(pin+16);}
	 static bool getState() {return getPortBase()->IDR&1<<pin;};
	 static void toggle() {setState(!getState());};
	virtual ~static_Gpio();
	static constexpr GPIO_TypeDef* getPortBase()	{ return ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*((uint8_t)port))); }; // port
	static constexpr uint16_t getPin() {return  pin;  };
	static constexpr bool isNotExti() {return  !(mode==GpioMode::Exti);  };
private:

};



#endif /* GPIO_H_ */
