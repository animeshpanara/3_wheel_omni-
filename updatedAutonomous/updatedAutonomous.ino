 /*
 * TO DO:
 * Check if IMU giving readings- Timer interrupt to calc loop counter value, if same then rese
 * Limit ang_velocity
 * Different PID rotation!
 */
 //this is sparta
// 0 //Compass

 
//#include <TimerOne.h>
#include <Wire.h>
#include <LSM303.h>
#include <SPI.h>
//#include <DueTimer.h>
//#include <Wire.h>
//#include <DuePWM.h>
//#include<avr/wdt.h>


//#define rpmmax 300
#define Time 0.2
#define MaxPwm 255
#define GearRatio 0.87
#define desiredpwm(x) (x*255.0)/320.0  
#define PWM_FREQ1 2500
//DuePWM pwm(PWM_FREQ1,3000);

const int omegaMode=1;
LSM303 compass;

void TicksA();
//The following code is for setting up IMU constants
//**********************************************************************
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define M_X_MIN -2292
#define M_Y_MIN -2374
#define M_Z_MIN -2605
#define M_X_MAX +2672
#define M_Y_MAX +2674
#define M_Z_MAX +2255 

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data,
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw
//min: { -2308,  -2293,  -1676}    max: { +2752,  +2620,  +1846}

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;
float G_Dt=0.02;    // Integratio n time (DCM algorithm)  We will run the integration loop at 50Hz if possible
float CompassHeadingOffset=0;
float CompassHeading;


  
//*********************************************************************

//init variables for LSA08
//**********************************************************************

typedef struct LSA08{
  char Address;
  int Theta;
  int JunctionPin;
  volatile int JunctionCount;
  int OePin;
}LSA;
float LSAforwardprev;
float LSAbackwardprev;
int LSArotateprev;

LSA LSAf={0x01,270,51,0,53},LSAr={0x02,180,47,0,49},LSAl={0x03,0,41,0,43}, LSAb={0x04,90,37,0,39},*LSAArray[4]={&LSAf,&LSAr,&LSAl,&LSAb};
//LSA LSAf={0x01,90,51,0,53},LSAr={0x02,0,47,0,49},LSAl={0x03,180,18,0,39},*LSAArray[3]={&LSAf,&LSAr,&LSAl};
const byte rx = 17;    // Defining pin 0 as Rx
const byte tx = 16;    // Defining pin 1 as Tx
//const byte OutputEnable[3] = {35,37,39};
const float LSAlength = 11.1;
const float LSAdistance = 46.18;
//const int JucntionPulse[3]={3,2,18};
//int JucntionCount[3]={0};
//char add[3]={0x01,0x02,0x03};
//int Theta[3]={270,0,180};
int Test[3]={6,1,0};
int ActiveSensor=0;
//enum LSA08{LSA08a,LSA08b,LSA08c};
enum activeLSA {f,r,l,b,s};
const int S=0,A=1,B=2,T1=3,T2=4,T3=5;
char arr[6][6][15]={
                   {{s},{r,s},{r,r,s},{r,f,f,s},{r,r,f,f,s},{r,r,f,f,f,f,f,s}},
 
                   {{l,s},{s},{r,s},{f,f,s},{r,f,f,s},{r,f,f,f,f,f,s}},
      
                   {{l,l,s},{l,s},{s},{l,f,f,s},{f,f,s},{f,f,f,f,f,s}},
    
                   {{b,b,l,s},{b,b,s},{b,b,r,s},{s},{b,b,r,f,f,s},{b,b,r,f,f,f,f,f,s}},
     
                   {{b,b,l,l,s},{b,b,l,s},{b,b,s},{b,b,l,f,f,s},{s},{f,f,f,s}},
      
                   {{b,b,b,b,b,l,l,s},{b,b,b,b,b,l,s},{b,b,b,b,b,s},{b,b,b,b,b,l,f,f,s},{b,b,b,s},{s}}

    };

//char arr[6][6][15]={
//                   {{s},{f,s},{f,f,s},{f,l,l,s},{f,f,l,l,s},{f,f,l,l,l,l,l,s}},
// 
//                   {{b,s},{s},{f,s},{l,s},{f,l,l,s},{f,l,l,l,l,l,s}},
//      
//                   {{b,b,s},{b,s},{s},{b,l,l,s},{l,l,s},{l,l,l,l,l,s}},
//    
//                   {{r,r,b,s},{r,r,s},{r,r,f,s},{s},{r,r,f,l,l,s},{r,r,f,l,l,l,l,l,s}},
//     
//                   {{r,r,b,b,s},{r,r,b,s},{r,r,s},{r,r,b,l,l,s},{s},{l,l,l,s}},
//      
//                   {{r,r,r,r,r,b,b,s},{r,r,r,r,r,b,s},{r,r,r,r,r,s},{r,r,r,r,r,b,l,l,s},{r,r,r,s},{s}}
//
//    };






//The following code is to set up driving constants/structs

//******************************************************
typedef struct wheels {
  float trans_rpm; //RPM for translational Velocity
  float ang_rpm; //RPM due to angular velocity
  int angle; //Position of tyre from x-axis
  float rpmmax; //Max RPM of wheel
  int pinpwm;//Pin PWM of motor to be connected to
  int pina;//a = 1, b=0 positive
  int pinb;
  int rpm;
  int prev_rpm;
} wheel;

const int anglea = 90;
const int angleb = 210;
const int anglec = 330;
const int RPMMAX = 300;
const int pinpwma = 7;
const int pinpwmb = 8;
const int pinpwmc = 6;
const int pinaa = 29;
const int pinab = 27;     //ALL WHEELS ARE TRYING TO ROTATE BOT CLOCKWISE WHEN A HIGH AND B LOW
const int pinba = 31;
const int pinbb = 33;
const int pinca = 23;
const int pincb = 25;
const int alignrpm=150;
const int DAC_PinTZ2=11;
const int DAC_PinTZ3=13;
const int ThrowPin=12;
const float HeadTheta=54.2;
int rpmmax=RPMMAX;
//*****************************************************************



//The following code is to set up PID constants
//***************************************************


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
//****************************************************************




//Constants for manual driving 
//*******************************************************************
  
//****************************************************************************


//Constants for Y Positional Encoder
//*******************************************************************************
typedef struct encoder{
  int channelA;
  int channelB;
  long int count;
  long int previousCount;
  int rpm; 
  int ppr;
  void (*Tickfunction)(void);
  float diameter;
} encoder;


//************************************************************************************
LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
int pos[2];
volatile struct gain Linegain[4], Omegagain, Aligngain, Aligngainperp,Aligngain1, Aligngainperp1,Rotategain;
volatile struct gain *pLinegain[] = {&Linegain[0],&Linegain[1],&Linegain[2], &Linegain[3]}, *pOmegagain = &Omegagain, *pAligngain = &Aligngain, *pAligngainperp = &Aligngainperp,*pAligngain1 = &Aligngain1, *pAligngainperp1 = &Aligngainperp1;
volatile struct gain *pRotategain=&Rotategain;
const float rad = 1;
const float pi = 3.14159;
volatile int Stopflag = 0, Linearflag = 0;
wheel wheela = {0, 0, anglea, rpmmax, pinpwma, pinaa, pinab,0,0}, wheelb = {0, 0, angleb, rpmmax, pinpwmb, pinba, pinbb,0,0}, wheelc = {0, 0, anglec, rpmmax, pinpwmc, pinca, pincb,0,0};
wheel *pwheel[3]={&wheela,&wheelb,&wheelc};
//encoder encoderA={1,1,0,0,0,1024,&TicksA,5.6};
//encoder * pencoderA = &encoderA;     
bool transmit;
int circleFlag=0;
volatile enum activeLSA dir,rdir, pdir, rpdir;
volatile int posindex=0;
volatile int ActiveLineSensor, ActiveOmegaSensor, PerpendicularLineSensor;
volatile int junctionPassed=0;


#define REDBOXSetup 0 
//DRIVING VARIABLES AND FLAGS
int alignedFlag=0;
int yawdeg;
volatile int cnttt=0;
float Linecontrol=0, Omegacontrol=0; 
bool Dirchange=0;
int Rotateflag=-1;
int ToleranceOfAlignment=10;
bool Throwcomplete=0;
bool LoadFlag=0;
int alignCounter=0;
int alignCounter1=0;
int cyclecomplete=1;
int ThrowLocation=3;


void setup() {
  //wdt_disable();
  Serial.begin(9600);
  Serial1.begin(9600);
  Wire.begin();
  for(int k =0;k<3;k++){
          pwheel[k]->rpm=0;
        }
        TransmitRPM(pwheel);
  interrupts();
  Serial.println("Starting code!");
  delay(1000);
  
  initLSA(9600,LSAArray[0]->OePin);            //const int minControl = -255;      const int maxControl = 255;
  initLSA(9600,LSAArray[1]->OePin);
  initLSA(9600,LSAArray[2]->OePin);
  initLSA(9600,LSAArray[3]->OePin);
  
  PIDinit(0.3,0.5,0,0,-255,255,pLinegain[0]);
  PIDinit(0.7,2.0,0,0,-255,255,pLinegain[1]);
  PIDinit(0.7,2.0,0,0,-255,255,pLinegain[2]);
  PIDinit(0.4,0.5,0,0,-255,255,pLinegain[3]);
  
  PIDinit(0.4,0.0,0,0,-rpmmax,rpmmax,pOmegagain);//Kp=0.67,Kd=0.7,Ki=0
  PIDinit(0.6,0.5,0,0,-35,35,pAligngain);
  PIDinit(0.9,1.0 ,0,0,-35,35,pAligngainperp);
  PIDinit(0.9,1.0,0,0,-22,22,pAligngain1);
  PIDinit(0.6,0.5,0,0,-22,22,pAligngainperp1);
  PIDinit(3,30,0,0,-120,120,pRotategain);
//  PIDinit(0.4,0.5,0,0,-35,35,pAligngain);
//  PIDinit(0.7,1.0 ,0,0,-35,35,pAligngainperp);
//  PIDinit(0.7,1.0,0,0,-22,22,pAligngain1);
//  PIDinit(0.4,0.5,0,0,-22,22,pAligngainperp1);
//  
  //clearJunction(LSAArray[0]->Address);
  
  pinMode(LSAArray[0]->JunctionPin,INPUT);
  pinMode(LSAArray[1]->JunctionPin,INPUT);
  pinMode(LSAArray[2]->JunctionPin,INPUT);
  pinMode(LSAArray[3]->JunctionPin,INPUT);
  
//  attachInterrupt(digitalPinToInterrupt(LSAArray[0]->JunctionPin),updateJunction,RISING);
//  attachInterrupt(digitalPinToInterrupt(LSAArray[1]->JunctionPin),updateJunction,RISING);
//  attachInterrupt(digitalPinToInterrupt(LSAArray[2]->JunctionPin),updateJunction,RISING);
//  attachInterrupt(digitalPinToInterrupt(LSAArray[3]->JunctionPin),updateJunction,RISING);
  
  clearJunction(LSAArray[0]->Address);
  clearJunction(LSAArray[1]->Address);
  clearJunction(LSAArray[2]->Address);
  clearJunction(LSAArray[3]->Address);
//
//  int count=0;
//  while(count!=2){
//    if(Serial.available()>0){
//    String data =Serial.readString();
//    pos[count]=atoi(data.c_str());
//    count++;
//   }
//  }

  
  pos[0]=0;
  pos[1]=1;
  dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex]; 
  rdir = (enum activeLSA)abs((int)dir-3);
  pdir = (enum activeLSA)(((int)dir+2)%4);   
  Stopflag=0;
  ActiveLineSensor=dir;
  ActiveOmegaSensor=rdir;
  PerpendicularLineSensor= pdir;
  
  if(REDBOXSetup){
    while(abs(GetLSAReading(LSAArray[r]->OePin))>25){
    Serial.println("Setup2");
    calcRPM(0,LSAArray[f]->Theta,rpmmax/3,pwheel);
    TransmitRPM(pwheel);            
    }
    while(abs(GetLSAReading(LSAArray[r]->OePin))<200){
    calcRPM(0,LSAArray[f]->Theta,rpmmax/3,pwheel);
    TransmitRPM(pwheel);             
    }
    while(abs(GetLSAReading(LSAArray[r]->OePin))>25){
    Serial.println("Setup2");
    calcRPM(0,LSAArray[f]->Theta,rpmmax/3,pwheel);
    TransmitRPM(pwheel);            
    }
  }
 // wdt_enable(WDTO_1S);
}

void loop(){
        transmit = false;
      if(Stopflag==0){
          if(digitalRead(LSAArray[ActiveLineSensor]->JunctionPin))
            {
              while(digitalRead(LSAArray[ActiveLineSensor]->JunctionPin));
              LSAArray[ActiveLineSensor]->JunctionCount=getJunction(LSAArray[ActiveLineSensor]->Address); //Why not in interrupt????
              posindex++;
              dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex];
              rdir =(enum activeLSA)abs(dir-3);
              pdir =(enum activeLSA)(((int)dir+2)%4); //f,r,l,b  f=l r=b      
              //junctionPassed=0;
              
              if((int)dir!=4){
                if(dir==ActiveLineSensor){
                    ActiveLineSensor = dir;
                    ActiveOmegaSensor = rdir;
                    PerpendicularLineSensor= pdir; 
                }
                else{
                  Dirchange=1;
                  rpmmax/=2; 
                }
                    
              }
           }
          else{
             Linecontrol=LineControl(LSAArray[ActiveLineSensor]->OePin,15,35,pLinegain[ActiveLineSensor]);
             if(!(digitalRead(LSAArray[ActiveLineSensor]->JunctionPin)||(digitalRead(LSAArray[ActiveOmegaSensor]->JunctionPin))))
             Omegacontrol=OmegaControl(LSAArray[ActiveLineSensor]->OePin,LSAArray[ActiveOmegaSensor]->OePin,40,pOmegagain);
             }
             Serial.println("LineControl:"+String(Linecontrol)+"OmegaControl:"+String(Omegacontrol));
            if(arr[pos[0]][pos[1]][posindex]==4 && alignedFlag==0)
            {
                  if(abs(GetLSAReading(LSAArray[PerpendicularLineSensor]->OePin))<15)
                  {
                    
                     for(int k =0;k<3;k++){
                       pwheel[k]->rpm=0;
                     }
                       alignedFlag=1; 
                  }
                  else
                  {
                      calcRPM(-Omegacontrol,LSAArray[ActiveLineSensor]->Theta-Linecontrol,rpmmax/3,pwheel);
                  }
            }
          else if(arr[pos[0]][pos[1]][posindex]==4 && Rotateflag==2)
          {//set align flag 0 before AND afterrotate 1 
            //RotateBot(0,20);
            RotateBot1(0,5);
            ToleranceOfAlignment=5;
            Rotateflag=-1;
          }
          else if(arr[pos[0]][pos[1]][posindex]==4 && Rotateflag==1)
          {//set align flag 0 before AND afterrotate 1 
            //RotateBot(1,20);
            RotateBot1(1,5);
            ToleranceOfAlignment=1;
            Rotateflag=-2;
          }
          else if(arr[pos[0]][pos[1]][posindex]==4 && alignedFlag==1)
            {
              //ActiveLineSensor;
              alignCounter1+=alignBot1(ActiveLineSensor,ToleranceOfAlignment);
              //if(pos[1]!=1){
              if(Rotateflag==0)
                {
                  Rotateflag=1; 
                }
              if(Rotateflag==-1 && cyclecomplete==0)
                 {
                          if(LoadFlag==0)
                          LoadBot();
                          //wait for loading
                  }
              
              
//              if(alignCounter>=2){
//                if(Rotateflag==-1)
//                {
//                  //alignCounter=0;
//                  if(LoadFlag==0)
//                  LoadBot();
//                }
//              }
              if(alignCounter1>=2)
              {
                        alignCounter1=0;
                        //calcRPM(0,0,0,pwheel);
                        if(Rotateflag==-2)
                        {
                        Rotateflag=2;
                        Throwcomplete=1;
                        } 
                        //if(Rotateflag==0)Rotateflag=1;
                        if(Rotateflag==-1 && cyclecomplete==1 && ThrowLocation<=5)
                        {
                          NextThrowCycle(ThrowLocation);
                          ThrowLocation++;
                          //wait for loading
                        }
              }
            }
          else if(arr[pos[0]][pos[1]][posindex+1]!=dir)
            calcRPM(-Omegacontrol,LSAArray[ActiveLineSensor]->Theta-Linecontrol,rpmmax/1.5,pwheel);
          else if(arr[pos[0]][pos[1]][posindex]!=4)
            calcRPM(-Omegacontrol,LSAArray[ActiveLineSensor]->Theta-Linecontrol,rpmmax,pwheel);
          if(Dirchange==1){
                    if(abs(GetLSAReading(LSAArray[dir]->OePin))<=30){
                      ActiveLineSensor=dir;
                      ActiveOmegaSensor = rdir;        
                      LSAforwardprev=0;
                      PerpendicularLineSensor= pdir;
                      Dirchange=0;
                      rpmmax*=2;
                      }
                  }
      }            
      else {
        Serial.println(" Stopped ");
         for(int k =0;k<3;k++){
          pwheel[k]->rpm=0;
        }
      }
      for(int l =0;l<3;l++)
      {
        Serial.println("Wheel"+String(l)+": "+pwheel[l]->rpm);
        if(pwheel[l]->prev_rpm!=pwheel[l]->rpm)
        transmit = true;
      }
       //Serial.println("Dir:"+String(dir));
        
      if(true)
      TransmitRPM(pwheel);
      
   //   wdt_reset();
} 

//void serialEvent(){
//  while(Serial.available()>0){
//    int a = atoi(Serial.readString().c_str());
//    pOmegagain->kd=a/1000.0;
//    Serial.println(pOmegagain->kd);
//  }
//  }

//void updateJunction(){
//        if(digitalRead(LSAArray[ActiveLineSensor]->JunctionPin))
//        {
//          junctionPassed=1;
//        }
//}

