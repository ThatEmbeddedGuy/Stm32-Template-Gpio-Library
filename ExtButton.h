/*
 * ExtButton.h
 *
 *  Created on: 27 θώνÿ 2016 γ.
 *      Author: tihonov
 */

#ifndef EXTBUTTON_H_
#define EXTBUTTON_H_


//------------------------------------------
// Template class for External Buttons
// use case
//typedef ExtButton<PortA,4> Button1;
//typedef ExtButton<PortA,5> Button2;
//
// bool Button1State,Button2State;
//
//Button1::Init();
//Button2::Init();
//
//Button1State=Button1::GetState();
//Button2State=Button2::GetState();
//------------------------------------------


#include "Gpio.h"

template<GpioPort_t port, uint8_t pin>
class ExtButton
{

public:
	ExtButton();
	static void Init() 	{mButton::Init();}
	static bool GetState() {return mButton::GetState();};
private:
	typedef Gpio<port,pin,GpioMode_In,GpioOutType_PP,GpioSpeed_100MHz,GpioPuPd_PullUp> mButton;
};
#endif /* EXTBUTTON_H_ */
