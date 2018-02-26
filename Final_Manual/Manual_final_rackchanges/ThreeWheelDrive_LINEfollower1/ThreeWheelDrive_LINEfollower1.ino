#include <TimerThree.h>
#include <SPI.h>
#define SerialPS2 Serial3
//#define Newcircuit


//-----------------------------------------------DELAYTIME_FOR_PS2------------------------- 
#define PS3
#define basePS2_Y 128.0
#define wirelessdelay 0
#ifdef PS3
#define wirelessdelay 50 
#define basePS2_Y 127.0
#endif


/* Structs/Variables for driving */
      
typedef struct wheels {
  float trans_rpm; //RPM for translational Velocity
  float ang_rpm; //RPM due to angular velocity
  int angle; //Position of tyre from x-axis
  float rpmmax; //Max RPM of wheel
  int pinpwm;//Pin PWM of motor to be connected to
  int pina;//a = 1, b=0 positive
  int pinb;
  int rpm;
} wheel; 


//---------------------------------PINS------------------------------------------------------------

const int anglea = 90;
const int angleb = 225;
const int anglec = 315;
const int rpmmax = 600;
const int pinpwma = 3;   //4; //Channel1
const int pinpwmb = 5;  //Channel2
const int pinpwmc = 2;   //2;  //Channel1
const int pinaa = 33;   //29;
const int pinab = 37;     //25;     //ALL WHEELS ARE TRYING TO ROTATE BOT CLOCKWISE WHEN A HIGH AND B LOW
const int pinba = 25;    //37;
const int pinbb = 29;    //33;
const int pinca = 41;    //41;
const int pincb = 45;
const float HeadTheta=54.2;

//------------------------------END-----------------------------------------------------------------------

//----------------------------------------PID-------------------------------------------------------------

struct gain {
  float kd;
  float kp;
  float ki;
  float required;
  float maxControl;
  float minControl;
  float error;
  float previousError;
  float derivativeError;
  float integralError;
};

int rpmrotate=80;

//------------------------------PS2---------------------------------------------------------------------



float distAnalogleft,distAnalogright;

#include <PS2X_lib.h>  //for v1.6

#define PS2_DAT        50  //14    
#define PS2_CMD        51  //15
#define PS2_SEL        31  //16
#define PS2_CLK        52  //17

#ifdef Newcircuit
#define PS2_SEL        6
#endif
/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
//#define pressures   false
//#define rumble      true
//#define rumble      false

PS2X ps2x; // create PS2 Controller Class

double xSquare_leftdistance=0,ySquare_leftdistance=0,xSquare_rightdistance=0,ySquare_rightdistance=0,xCircle_leftdistance=0,yCircle_leftdistance=0,xCircle_rightdistance=0,yCircle_rightdistance=0;
double LeftAnalogTheta=0,RightAnalogTheta=0;//w1=0,w2=0;
float LeftAnalogDistance=0,RightAnalogDistance=0;//r1=0,r2=0; //r1-Leftanalogdist w1-Leftanalogtheta

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

//---------------------------------------------END------------------------------------------------------------------------------------------------

const float rad = 1;
const float pi = 3.14159;

int Stopflag = 1, Linearflag = 0, rotateFlag=1; //rotateFlag 1 IMU, 0 Analog

wheel wheela = {0, 0, anglea, rpmmax, pinpwma, pinaa, pinab,0}, wheelb = {0, 0, angleb, rpmmax, pinpwmb, pinba, pinbb,0}, wheelc = {0, 0, anglec, rpmmax, pinpwmc, pinca, pincb,0};
wheel *pwheel[3]={&wheela,&wheelb,&wheelc};

float omegaControl;
float PWMfactor=2;
int passflag=0;
int racklift=0;

void setup() {                                                                                           
  Serial.begin(9600);
  Timer3.initialize(500);
  SerialPS2.begin(9600);
  initDriving(pwheel);
  initPS2();
}
int frontpressed=0,backpressed=0;

void loop() {  
      getPS2value();    
      scalePS2value();
      PS2executePressed();      
      LeftAnalogTheta=((int)(LeftAnalogTheta+180)%360);
      if(passflag)
      PWMfactor=1.8;
      else
      PWMfactor=2.75; 
       if((LeftAnalogTheta < 110.0 && LeftAnalogTheta > 70.0))// || (LeftAnalogTheta < 290.0 && LeftAnalogTheta > 250.0)) 
       {  
        PWMfactor+=0.25; //forward0backward
        }
       else if((LeftAnalogTheta < 45.0 && LeftAnalogTheta >=0.0) ||  (LeftAnalogTheta < 360.0 && LeftAnalogTheta >310.0) || (LeftAnalogTheta < 200.0 && LeftAnalogTheta >160.0)) 
       { 
        PWMfactor-=0.35;
       }
      if(rotateFlag){
        if(RightAnalogTheta >= 270 || RightAnalogTheta <= 90)
            omegaControl = -RightAnalogDistance;//Rotate Left
            
        else if(RightAnalogTheta >= 90 && RightAnalogTheta <= 270) 
            omegaControl = RightAnalogDistance;//Rotate Right
      }
      if(Stopflag==0){  
         if(frontpressed)
          calcRPM(omegaControl*rpmrotate,270,rpmmax,pwheel);          
         else if(backpressed)
          calcRPM(omegaControl*rpmrotate,90,rpmmax,pwheel);          
         else
          calcRPM(omegaControl*rpmrotate,LeftAnalogTheta,LeftAnalogDistance*rpmmax,pwheel);
      }
      else if(Stopflag==1){
        for(int k =0;k<3;k++){
          pwheel[k]->rpm=0;
        }
      }
      startMotion(pwheel);
} 

 
