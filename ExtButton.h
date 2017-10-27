/*
 * extButton.h
 *
 *  Created on: 28 июня 2016 г.
 *      Author: tihonov
 */

#ifndef EXTBUTTON_H_
#define EXTBUTTON_H_


template<Port port, uint8_t pin>
class ExtButton
{

public:
	ExtButton(){;	};
	void init() 	{mButton::init();}
	bool getState() {return mButton::getState();  };
	void setSubscribtion(bool state)  {IsSubscribed=state;}
	bool getsubscribtion() {return (IsSubscribed);}
private:
	typedef static_Gpio<port,pin,GpioMode::In,GpioOutType::PP,GpioSpeed::s100MHz,GpioPuPd::PullUp> mButton;
};


#endif /* EXTBUTTON_H_ */
