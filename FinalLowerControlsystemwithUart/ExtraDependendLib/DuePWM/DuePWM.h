/*  

Library:     pwm01.h (version 01)
Date:        2/11/2013
Written By:  randomvibe 

Library:	  DuePWM (version 1, based on pwm01.h version 01)
Date:		  6/17/2013
Rewritten By: exedor

Purpose:     
   Setup one or two unique PWM frequenices directly in sketch program,
   set PWM duty cycle, and stop PWM function.

User Functions:
   ctor: DuePWM();
   ctor: DuePWM(uint32_t clockA_freq, uint32_t clockB_freq)
   DuePWM::setFreq1( uint32_t clockA_freq ) // in Hz
   DuePWM::setFreq2( uint32_t clockB_freq ) // in Hz
   DuePWM::pinFreq1( uint32_t pin ) // pin must be 6 through 9
   DuePWM::pinFreq2( uint32_t pin ) // pin must be 6 through 9
   DuePWM::pinDuty( uint32_t pin_number, uint32_t duty_cycle) ~ Write PWM duty cycle 0 through 255
   DuePWM::stop( uint32_t pin_number) ~ Force PWM stop

Notes:
   - Applies to Arduino-Due board, PWM pins 6, 7, 8 & 9, all others ignored
   - Libary Does not operate on the TIO pins.
   - Unique frequencies set via PWM Clock-A ("CLKA") and Clock-B ("CLKB")
     Therefore, up to two unique frequencies allowed.
   - Set max duty cycle counts (pwm_max_duty_Ncount) equal to 255
     per Arduino approach.  This value is best SUITED for low frequency
     applications (2hz to 40,000hz) such as PWM motor drivers, 
     38khz infrared transmitters, etc.
   - Future library versions will address high frequency applications.
   - Arduino's "wiring_analog.c" function was very helpful in this effort.

 Rewrite Notes:
   - I hope the other isn't terribly upset with me for redoing his great original work :)
   
*/

#ifndef DuePWM_h
#define DuePWM_h

#include <Arduino.h>


class DuePWM
{
  public:
	DuePWM();
	DuePWM(uint32_t clockA_freq, uint32_t clockB_freq);

	//void pwm_set_resolution(int res);
	void  setFreq1(uint32_t  clockA_freq);
	void  setFreq2(uint32_t  clockB_freq);

	// MAIN PWM INITIALIZATION
	//--------------------------------
	void  pinFreq1( uint32_t  pin );
	void  pinFreq2( uint32_t  pin );
	
	// WRITE DUTY CYCLE
	//--------------------------------
	void  pinDuty( uint32_t  pin,  uint32_t  duty );

	// FORCE PWM STOP
	//--------------------------------
	void  stop( uint32_t  pin );
	
  private:
	//static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) 
	//{
	//	if (from == to)
	//		return value;
	//	if (from > to)
	//		return value >> (from-to);
	//	else
	//		return value << (to-from);
	//}

	//int       pwm_res_nbit;
	uint32_t  pwm_clockA_freq;
	uint32_t  pwm_clockB_freq;
	static const uint32_t  pwm_max_duty_Ncount = 255;
};
#endif
