/*
 * led.h
 *
 *  Created on: 7 июля 2016 г.
 *      Author: tihonov
 */

#ifndef LED_H_
#define LED_H_


template<GpioPort_t port, uint8_t pin>
class Led
{

public:
	Led(){};
	void Init() 	{mLed::Init();}
	void SetState(bool state) {mLed::SetState();};
	void Toggle(bool state)  {mLed::Toggle();}
private:
	typedef Gpio<port,pin,GpioMode_Out,GpioOutType_PP,GpioSpeed_100MHz,GpioPuPd_PullUp> mLed;

};



#endif /* LED_H_ */
