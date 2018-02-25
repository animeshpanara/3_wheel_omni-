#include <WSWire.h>
//#include <Wire.h>
#define REDBOXSetup 1
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#include <Adafruit_MCP4725.h>
#define PiSerial Serial3
const int dacTZ3=1020;//1000
const int dacTZ2=750;
const int dacTZ1=600;
const int rotateRPM=120;

Adafruit_MCP4725 dac;
uint32_t DACcounter;

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

//LSA LSAf={0x01,270,51,0,53},LSAr={0x02,180,47,0,49},LSAl={0x03,0,41,0,43}, LSAb={0x04,90,37,0,39},*LSAArray[4]={&LSAf,&LSAr,&LSAl,&LSAb};
LSA LSAf={0x01,270,19,0,31},LSAr={0x02,180,2,0,27},LSAl={0x03,0,18,0,29}, LSAb={0x04,90,3,0,25},*LSAArray[4]={&LSAf,&LSAr,&LSAl,&LSAb};

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
const int RPMMAX = 400;
const int alignrpm=210;
const int TriggerPin=9;
const int EchoPin=8;
const int ThrowLed=47; 
const int CamLed=51;
const int ThrowPin = 43;
//const int SpeedOfSound=340;
int rpmmax=RPMMAX;

/*************************************   new_Mechanism    *********************************************************************************************/
const int Motordir1pin=13;    
const int Motordir2pin=11; 
const int PWMpin=12;
const int LimitLpin=7;
const int LimitRpin=5; 
const int reloadLiftpin=53;
const int MechanismRPM=90;
/**********************************************************************************************************************************/
int maxcolor=0;
/**********************************************************************************************************************************/

int timerStart = 1;
long int alignTime;
int alignCounter=0; 
int alignCounter1=0;
int Stopflag = 0;
bool transmit;
enum activeLSA dir,rdir, pdir, rpdir;
int posindex=0;
int ActiveLineSensor, ActiveOmegaSensor, PerpendicularLineSensor;
float Linecontrol=0, Omegacontrol=0; 

bool LoadFlag=0;
int alignedFlag=0;
int Rotateflag=-1;
int cyclecomplete=1;

bool Throwcomplete=1;

bool Dirchange=0;
float ToleranceOfAlignment=8;


int pos[2];
int Ircounter=0;
int CheckIrflag=0;
//int ThrowLocation=3;

/**********************************************************************************************************************************/

int checkbacksensor=1;
long int junctimer=0;

/**********************************************************************************************************************************/
wheel wheela = {0, 0, anglea, rpmmax, 0, 0, 0,0,0}, wheelb = {0, 0, angleb, rpmmax, 0, 0, 0,0,0}, wheelc = {0, 0, anglec, rpmmax, 0, 0, 0,0,0};
wheel *pwheel[3]={&wheela,&wheelb,&wheelc};

void setup() {
  //wdt_disable();
  Serial.begin(9600);
  PiSerial.begin(115200);
  Serial2.begin(9600);
  Wire.begin();
  initIrSensor();
  dac.begin(0x60);
  
  for(int k =0;k<3;k++){
    pwheel[k]->rpm=0;
  }
  TransmitRPM(pwheel);
  interrupts();
  
  Stopflag=0;
  delay(1000);
  
  initLSA(9600,LSAArray[0]->OePin);            //const int minControl = -255;      const int maxControl = 255;
  initLSA(9600,LSAArray[1]->OePin);
  initLSA(9600,LSAArray[2]->OePin);
  initLSA(9600,LSAArray[3]->OePin);
  
  PIDinit(0.3,0.0,0,0,-255,255,pLinegain[0]);
  PIDinit(0.7,2.0,0,0,-255,255,pLinegain[1]);
  PIDinit(0.7,2.0,0,0,-255,255,pLinegain[2]);
  PIDinit(0.4,0.0,0,0,-255,255,pLinegain[3]);
  PIDinit(0.8,0.0,0,0,-rpmmax,rpmmax,pAlignOmegagain);//Kp=0.67,Kd=0.7,Ki=0
  PIDinit(0.8,0.5,0,0,-rpmmax,rpmmax,pOmegagain);//Kp=0.67,Kd=0.7,Ki=0
  PIDinit(0.6,0.6,0.01,0,-35,35,pAligngain);
  PIDinit(0.9,1.1,0.02,0,-35,35,pAligngainperp);
  PIDinit(0.9,1.1,0.02,0,-35,35,pAligngain1);
  PIDinit(0.6,0.6,0.01,0,-35,35,pAligngainperp1);
  PIDinit(2.5,10,0,0,-rotateRPM,rotateRPM,pRotategain);

  pinMode(LSAArray[0]->JunctionPin,INPUT);
  pinMode(LSAArray[1]->JunctionPin,INPUT);
  pinMode(LSAArray[2]->JunctionPin,INPUT);
  pinMode(LSAArray[3]->JunctionPin,INPUT);

  digitalWrite(LSAArray[0]->JunctionPin,LOW);
  digitalWrite(LSAArray[1]->JunctionPin,LOW);
  digitalWrite(LSAArray[2]->JunctionPin,LOW);
  digitalWrite(LSAArray[3]->JunctionPin,LOW);
  
  initThrowing();
  initnewMech();
  
  clearJunction(LSAArray[0]->Address);
  clearJunction(LSAArray[1]->Address);
  clearJunction(LSAArray[2]->Address);
  clearJunction(LSAArray[3]->Address);

  posindex=0; 
  
//  pos[0]=0;
//  pos[1]=3;
//  dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex]; 
//  rdir = (enum activeLSA)abs((int)dir-3);
//  pdir = (enum activeLSA)(((int)dir+2)%4);   
//  ActiveLineSensor=dir;
//  ActiveOmegaSensor=rdir;
//  PerpendicularLineSensor= pdir;
  maxcolor=3;
  
  if(REDBOXSetup){
    while(abs(GetLSAReading(LSAArray[r]->OePin))>25){
      Serial.println("Setup0");
      calcRPM(0,LSAArray[f]->Theta,rpmmax/3,pwheel);
      TransmitRPM(pwheel);
      delay(100);            
    }
    while(abs(GetLSAReading(LSAArray[r]->OePin))<200){
      Serial.println("Setup1");
      calcRPM(0,LSAArray[f]->Theta,rpmmax/3,pwheel);
      TransmitRPM(pwheel);
      delay(100);             
    }
    while(abs(GetLSAReading(LSAArray[r]->OePin))>25){
      Serial.println("Setup2");
      calcRPM(0,LSAArray[f]->Theta,rpmmax/3,pwheel);
      TransmitRPM(pwheel);
      delay(100);            
    }
  } 
      // wdt_enable(WDTO_1S);
 Serial.println("Starting code!");
 pos[0]=0;
 pos[1]=0;
 LoadBot(); 
 }
      
void loop(){
    if(Serial.available()>0)if(Serial.read()=='a')(ThrowShuttleCock(5));
    transmit = false;
    //Serial.println("forward:"+String(GetLSAReading(LSAArray[l]->OePin))+String(digitalRead(LSAArray[l]->JunctionPin)));//+"right:"+String(GetLSAReading(LSAArray[r]->OePin))+String(LSAArray[r]->JunctionPin)+"left:"+String(GetLSAReading(LSAArray[l]->OePin))+String(LSAArray[l]->JunctionPin)+"forward:"+String(GetLSAReading(LSAArray[b]->OePin))+String(LSAArray[b]->JunctionPin));
    if(ActiveLineSensor==4)Stopflag=1;      
    if(Stopflag==0){
        if(digitalRead(LSAArray[ActiveLineSensor]->JunctionPin))
          {
            junctimer=millis();
            while(digitalRead(LSAArray[ActiveLineSensor]->JunctionPin))if(millis()-junctimer>2000)break;
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
                rpmmax/=3; 
              }
            }
         }
        else{
           Linecontrol=LineControl(LSAArray[ActiveLineSensor]->OePin,15,35,pLinegain[ActiveLineSensor]);
           if(!(digitalRead(LSAArray[ActiveLineSensor]->JunctionPin)||(digitalRead(LSAArray[ActiveOmegaSensor]->JunctionPin))))
           Omegacontrol=OmegaControl(LSAArray[ActiveLineSensor]->OePin,LSAArray[ActiveOmegaSensor]->OePin,40,pOmegagain);
           }
          Serial.println("LineControl:"+String(Linecontrol)+"OmegaControl:"+String(Omegacontrol)+"  "+String(posindex));
          if(arr[pos[0]][pos[1]][posindex]==4 && alignedFlag==0){
                if(abs(GetLSAReading(LSAArray[PerpendicularLineSensor]->OePin))<16){
                   for(int k =0;k<3;k++){
                     pwheel[k]->rpm=0;
                   }
                   TransmitRPM(pwheel);              
                   alignedFlag=1; 
                }
                else{
                    calcRPM(-Omegacontrol,LSAArray[ActiveLineSensor]->Theta-Linecontrol,rpmmax/3.5,pwheel);
                }
          }
        else if(arr[pos[0]][pos[1]][posindex]==4 && Rotateflag==1){                                                                                        //set align flag 0 before AND afterrotate 1 
          RotateBot(1,6);
          ToleranceOfAlignment=6;
          Rotateflag=-2;
          //alignedFlag=1;
        }
        else if(arr[pos[0]][pos[1]][posindex]==4 && Rotateflag==2){                                                                                       //set align flag 0 before AND afterrotate 1 
          RotateBot(0,6);
          ToleranceOfAlignment=5 ;
          Rotateflag=-1;
          //alignedFlag=1;
        }
        else if(arr[pos[0]][pos[1]][posindex]==4 && alignedFlag==1){
          
             alignCounter1+=alignBot(ActiveLineSensor,ToleranceOfAlignment); 
             
             if(Rotateflag==0){
                Rotateflag=1; 
                alignCounter1=0;
                //alignedFlag=0;
             }
             if(Rotateflag==-1 && cyclecomplete==0){
                alignCounter1=0;
                if(LoadFlag==0)
                  LoadBot();
                //delay after throw moving towards loading cycle completes here
             }
                 
             if(alignCounter1>=2){
                    alignCounter1=0;
                    if(Rotateflag==-2)
                    {
                          Rotateflag=2;
                          ThrowShuttleCock(pos[1]);
                          //Throwcomplete=1;
                          //alignedFlag=0;        
                          //throw here
                    } 
                    if(Rotateflag==-1 && cyclecomplete==1 )//&& Throwcomplete==1)
                    {
                          //commented out throw complete flag everywhere its redundant
                          digitalWrite(CamLed,HIGH);
                          chechIr(50);
                          //delay(1000);
                          int location;
                          
                          while(1){
                            location=CheckBall();
                            if (location!=0)
                            break;
                          }
                          digitalWrite(CamLed,LOW);
                          
                          if(location<=maxcolor){
                          NextThrowCycle(location);
                          }
                          else{
                          NextThrowCycle(maxcolor);
                          }
                          
                          maxcolor++;
                          if(maxcolor>5)maxcolor=5;
                          //delay(1000);
                          //wait for loading
                    }
            }
        }
        else if(arr[pos[0]][pos[1]][posindex+1]!=dir)
          calcRPM(-Omegacontrol,LSAArray[ActiveLineSensor]->Theta-Linecontrol,rpmmax/1.5,pwheel);
        else if(arr[pos[0]][pos[1]][posindex]!=4)
          calcRPM(-Omegacontrol,LSAArray[ActiveLineSensor]->Theta-Linecontrol,rpmmax,pwheel);
        if(Dirchange==1){
                    //if(checkbacksensor){
                        if(dir!=4)
                        if(abs(GetLSAReading(LSAArray[dir]->OePin))<=30){
                          ActiveLineSensor=dir;
                          ActiveOmegaSensor = rdir;        
                          //LSAforwardprev=0;
                          PerpendicularLineSensor= pdir;
                          //checkbacksensor=0;
                            }
                      //}
                      if(digitalRead(LSAArray[ActiveOmegaSensor]->JunctionPin)){
                       Dirchange=0;
                       rpmmax*=3;  
                      //checkbacksensor=1;
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
      if(pwheel[l]->rpm>RPMMAX)Stopflag=1;
      break;
      //if(pwheel[l]->prev_rpm!=pwheel[l]->rpm)
      //transmit = true;
    }
    if(CheckIrflag){
      if(!digitalRead(TriggerPin))
        Ircounter++;
      if(Ircounter>20){
        Ircounter=0;
        CheckIrflag=0;
        }
      else
        if(posindex==1){
        Ircounter=0;
        CheckIrflag=0;
        ReLoadBot();     
        }
    }
    
      //    if(true)
    TransmitRPM(pwheel);
      //delay(20);  
      //   wdt_reset();
} 


