#include <DuePWM.h>


#define PWM_FREQ1  2
#define PWM_FREQ2  5000

// DuePWM can be instantiated using a default constrcutor like this:
// DuePWM pwm();
// OR
// DuePWM pwm;
// In these cases it will use the default values of PWM_FREQUENCY
// defined in variant.h which is currently 1kHz and is equivalent to:
// DuePWM pwm(PWM_FREQUENCY, PWM_FREQUENCY);
DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );

void setup() 
{
    //uint32_t  pwm_duty = 32767;
    uint32_t pwm_duty = 127; // 50% duty cycle

    pwm.setFreq1( PWM_FREQ1 );
    pwm.setFreq2( PWM_FREQ2 );
    // I have done away with pwm_set_resolution in the interest of speed
    // The arduino standard seems to be 8 bit resolution on all platforms
    // so in the interest of speed and efficiency, just assume it's 8 bit.
    // Set PWM Resolution
    //pwm_set_resolution(16);  

    // Setup PWM Once (Up to two unique frequencies allowed
    //-----------------------------------------------------    
    pwm.pinFreq1( 6 );  // Pin 6 freq set to "pwm_freq1" on clock A
    pwm.pinFreq2( 7 );  // Pin 7 freq set to "pwm_freq2" on clock B
    pwm.pinFreq2( 8 );  // Pin 8 freq set to "pwm_freq2" on clock B
    pwm.pinFreq2( 9 );  // Pin 9 freq set to "pwm_freq2" on clock B
      
    // Write PWM Duty Cycle Anytime After PWM Setup
    //-----------------------------------------------------    
    pwm.pinDuty( 6, pwm_duty );  // 50% duty cycle on Pin 6
    pwm.pinDuty( 7, pwm_duty );  // 50% duty cycle on Pin 7
    pwm.pinDuty( 8, pwm_duty );  // 50% duty cycle on Pin 8
    pwm.pinDuty( 9, pwm_duty );  // 50% duty cycle on Pin 9

    delay(30000);  // 30sec Delay; PWM signal will still stream
    
    // Change frequency 1 on PWM clock A to 10kHz
    // Change frequency 2 on PWM clock B to 20kHz
    pwm.setFreq1( 10000 );
    pwm.setFreq2( 20000 );
    
    pwm_duty = 255;
    pwm.pinDuty( 6, pwm_duty );  // 100% duty cycle on Pin 6
    pwm.pinDuty( 7, pwm_duty );  // 100% duty cycle on Pin 7
    pwm.pinDuty( 8, pwm_duty );  // 100% duty cycle on Pin 8
    pwm.pinDuty( 9, pwm_duty );  // 100% duty cycle on Pin 9

    // Pin 6 should now show 10kHz and 7, 8, and 9, 20kHz respectively.
    // Duty cycles remain unchanged.
    
    delay(30000);  // 30sec Delay; PWM signal will stream new freq
    
    // Force PWM Stop On All Pins
    //-----------------------------    
    pwm.stop( 6 );
    pwm.stop( 7 );
    pwm.stop( 8 );
    pwm.stop( 9 );
}

void loop() 
{  
}

