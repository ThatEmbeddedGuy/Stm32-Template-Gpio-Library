/*
 * led.h
 *
 *  Created on: 7 июля 2016 г.
 *      Author: tihonov
 */

#ifndef LED_H_
#define LED_H_


template<Port port, uint8_t pin>
class Led
{

public:
	Led(){};
	static void init() 	{mLed::init();}
	static void setState(bool state) {mLed::setState(state);};
	static void toggle(bool state)  {mLed::toggle();}
	static void setUp(void) {mLed::setState(true);};
	static void setDown(void) {mLed::setState(false);};
private:
	typedef static_Gpio<port,pin,GpioMode::Out,GpioOutType::PP,GpioSpeed::s100MHz,GpioPuPd::PullUp> mLed;

};



#endif /* LED_H_ */
