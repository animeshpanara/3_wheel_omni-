#include <Wire.h>
#define MaxPwm 255
#define REDBOXSetup 1
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

/*init variables for LSA08*/

typedef struct LSA08{
  char Address;
  int Theta;
  int JunctionPin;
  int JunctionCount;
  int OePin;
}LSA;

float LSAforwardprev;
float LSAbackwardprev;
int LSArotateprev;

LSA LSAf={0x01,270,51,0,53},LSAr={0x02,180,47,0,49},LSAl={0x03,0,41,0,43}, LSAb={0x04,90,37,0,39},*LSAArray[4]={&LSAf,&LSAr,&LSAl,&LSAb};
const byte rx = 17;    // Defining pin 0 as Rx
const byte tx = 16;    // Defining pin 1 as Tx
const float LSAlength = 11.1;
const float LSAdistance = 46.18;
int ActiveSensor=0;
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

/*The following code is to set up driving constants/structs*/

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


/*The following code is to set up PID constants*/

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


/* objects/variables used*/

struct gain Linegain[4], Omegagain, Aligngain, Aligngainperp,Aligngain1, Aligngainperp1,Rotategain,AlignOmegagain;
struct gain *pLinegain[] = {&Linegain[0],&Linegain[1],&Linegain[2], &Linegain[3]}, *pOmegagain = &Omegagain, *pAligngain = &Aligngain, *pAligngainperp = &Aligngainperp,*pAligngain1 = &Aligngain1, *pAligngainperp1 = &Aligngainperp1;
struct gain *pRotategain=&Rotategain, *pAlignOmegagain = &AlignOmegagain;
const float rad = 1;
const float pi = 3.1415;
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
const int alignrpm=210;
const int DAC_PinTZ2 = 11;
const int DAC_PinTZ3 = 13;
const int ThrowPin = 12;
//const float HeadTheta=54.2;
int rpmmax=RPMMAX;
int timerStart = 1;
long int alignTime;
int Stopflag = 0;

wheel wheela = {0, 0, anglea, rpmmax, pinpwma, pinaa, pinab,0,0}, wheelb = {0, 0, angleb, rpmmax, pinpwmb, pinba, pinbb,0,0}, wheelc = {0, 0, anglec, rpmmax, pinpwmc, pinca, pincb,0,0};
wheel *pwheel[3]={&wheela,&wheelb,&wheelc};
bool transmit;
enum activeLSA dir,rdir, pdir, rpdir;
int posindex=0;
int ActiveLineSensor, ActiveOmegaSensor, PerpendicularLineSensor;
int junctionPassed=0;
int alignedFlag=0;
float Linecontrol=0, Omegacontrol=0; 
bool Dirchange=0;
int Rotateflag=-1;
float ToleranceOfAlignment=8;
bool Throwcomplete=0;
bool LoadFlag=0;
int alignCounter=0;
int alignCounter1=0;
int cyclecomplete=1;
int ThrowLocation=3;
int pos[2];

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
  
  PIDinit(0.2,0.0,0,0,-255,255,pLinegain[0]);
  PIDinit(0.7,2.0,0,0,-255,255,pLinegain[1]);
  PIDinit(0.7,2.0,0,0,-255,255,pLinegain[2]);
  PIDinit(0.5,0.0,0,0,-255,255,pLinegain[3]);
  PIDinit(0.8,0.0,0,0,-rpmmax,rpmmax,pAlignOmegagain);//Kp=0.67,Kd=0.7,Ki=0
  PIDinit(0.4,0.0,0,0,-rpmmax,rpmmax,pOmegagain);//Kp=0.67,Kd=0.7,Ki=0
  PIDinit(0.6,0.6,0.005,0,-35,35,pAligngain);
  PIDinit(0.9,1.1,0.01,0,-35,35,pAligngainperp);
  PIDinit(0.9,1.1,0.01,0,-35,35,pAligngain1);
  PIDinit(0.6,0.6,0.005,0,-35,35,pAligngainperp1);
  PIDinit(3,30,0,0,-120,120,pRotategain);

  pinMode(LSAArray[0]->JunctionPin,INPUT);
  pinMode(LSAArray[1]->JunctionPin,INPUT);
  pinMode(LSAArray[2]->JunctionPin,INPUT);
  pinMode(LSAArray[3]->JunctionPin,INPUT);
  initThrowing();

  clearJunction(LSAArray[0]->Address);
  clearJunction(LSAArray[1]->Address);
  clearJunction(LSAArray[2]->Address);
  clearJunction(LSAArray[3]->Address);

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
int checkbacksensor=1; 
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
          //Serial.println("LineControl:"+String(Linecontrol)+"OmegaControl:"+String(Omegacontrol));
          if(arr[pos[0]][pos[1]][posindex]==4 && alignedFlag==0){
                if(abs(GetLSAReading(LSAArray[PerpendicularLineSensor]->OePin))<15){
                   for(int k =0;k<3;k++){
                     pwheel[k]->rpm=0;
                   }
                     alignedFlag=1; 
                }
                else{
                    calcRPM(-Omegacontrol,LSAArray[ActiveLineSensor]->Theta-Linecontrol,rpmmax/3,pwheel);
                }
          }
        else if(arr[pos[0]][pos[1]][posindex]==4 && Rotateflag==2){                                                                                       //set align flag 0 before AND afterrotate 1 
          //RotateBot(0,5);
          RotateBot(0,5);
          ToleranceOfAlignment=5 ;
          Rotateflag=-1;
        }
        else if(arr[pos[0]][pos[1]][posindex]==4 && Rotateflag==1){                                                                                        //set align flag 0 before AND afterrotate 1 
          //RotateBot(1,5);
          RotateBot(1,5);
          ToleranceOfAlignment=6;
          Rotateflag=-2;
        }
        else if(arr[pos[0]][pos[1]][posindex]==4 && alignedFlag==1){
           alignCounter1+=alignBot(ActiveLineSensor,ToleranceOfAlignment); 
           if(Rotateflag==0){
              Rotateflag=1; 
           }
           if(Rotateflag==-1 && cyclecomplete==0){
              if(LoadFlag==0)
              LoadBot();
              //cyclecomplete=1; 
              //delay after throw moving towards loading
           }
               
            if(alignCounter1>=2){
              alignCounter1=0;
              if(Rotateflag==-2)
              {
              Rotateflag=2;
              //throw here
              ThrowShuttleCock();
              Throwcomplete=1;
              } 
              if(Rotateflag==-1 && cyclecomplete==1 && ThrowLocation<=5)
              {
                NextThrowCycle(ThrowLocation);
                delay(5000);
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
                    if(checkbacksensor){
                        if(abs(GetLSAReading(LSAArray[dir]->OePin))<=30){
                          ActiveLineSensor=dir;
                          ActiveOmegaSensor = rdir;        
                          //LSAforwardprev=0;
                          PerpendicularLineSensor= pdir;
                          checkbacksensor=0;
                          }
                      }
                      if(digitalRead(LSAArray[ActiveOmegaSensor]->JunctionPin)){
                      Dirchange=0;
                      rpmmax*=2;
                      checkbacksensor=1;
                      }
          }
    }            
    else{
      Serial.println(" Stopped ");
      for(int k =0;k<3;k++){
        pwheel[k]->rpm=0;
      }
    }
    for(int l =0;l<3;l++){
      //Serial.println("Wheel"+String(l)+": "+pwheel[l]->rpm);
      if(pwheel[l]->prev_rpm!=pwheel[l]->rpm)
      transmit = true;
    }
    
    if(transmit)
    TransmitRPM(pwheel);
      
   //   wdt_reset();
} 


