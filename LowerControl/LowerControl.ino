#include <DueTimer.h>
//#include "digitalWriteFast.h"
#define Time 0.05
#define MaxPwm 255
#define GearRatio 0.87
#define desiredpwm(x) (x*255.0)/320.0  
int flag=0;
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
struct gain Motor[3];
struct gain *pMotorgain[3]={&Motor[0],&Motor[1],&Motor[2]};
//Struct for motors
struct motor{
  int pin1;
  int pin2;
  int pwmpin;
};
typedef struct motor MOTOR;

MOTOR MotorA={52,50,8};
MOTOR MotorB={48,46,7};
MOTOR MotorC={36,38,6};

MOTOR *motors[3]={&MotorA,&MotorB,&MotorC};


//struct for encoder and lower control systemX

int Dirflag=0;
//****************************************************************

struct encoder{
  int channelA;
  int channelB;
  long int Count;
  long int previousCount;
  int Rpm; 
  int Ppr;
};

typedef struct encoder Encoder;

//Encoder encoderA,encoderB,encoderC;
Encoder encoder1={34,32,0,0,0,200};
Encoder encoder2={28,26,0,0,0,200};
Encoder encoder3={24,22,0,0,0,200};
Encoder *encoderA=&encoder3;
Encoder *encoderB=&encoder2;
Encoder *encoderC=&encoder1;
//encoderC={};
float outsee[3]={0};
float Output[3];

void setup(){
  initEncoders();
  Serial.begin(9600);
  Timer1.attachInterrupt(Timerhandler);
  Timer1.start(100000/2);
  
  initMotor(motors[0]);
  initMotor(motors[1]);
  initMotor(motors[2]);
  
  PIDinit(0.038,0.015,0,0,0,255,pMotorgain[0]);
  
  PIDinit(0.038,0.015,0,0,0,255,pMotorgain[1]);
  
  PIDinit(0.038,0.015,0,0,0,255,pMotorgain[2]);

  pMotorgain[0]->required=0;
  pMotorgain[1]->required=0;
  pMotorgain[2]->required=0;
  Serial.println("================================");
  delay(5000);
  
}

void Timerhandler(){
  if(flag==1){
  encoderA->Rpm=((encoderA->Count-encoderA->previousCount)*60.0)/(Time*GearRatio*encoderA->Ppr);
  encoderB->Rpm=((encoderB->Count-encoderB->previousCount)*60.0)/(Time*GearRatio*encoderB->Ppr);
  encoderC->Rpm=((encoderC->Count-encoderC->previousCount)*60.0)/(Time*GearRatio*encoderC->Ppr);
  encoderA->previousCount=encoderA->Count;
  encoderB->previousCount=encoderB->Count;
  encoderC->previousCount=encoderC->Count;
  
  float temp[3];
  
  temp[0]=PID(encoderA->Rpm,pMotorgain[0]);
  temp[1]=PID(encoderB->Rpm,pMotorgain[1]);
  temp[2]=PID(encoderC->Rpm,pMotorgain[2]);
  //Serial.println("A: "+String(temp[0])+"B: "+String(temp[1])+"C: "+String(temp[2]));
  Output[0]+=temp[0];
  Output[1]+=temp[1];
  Output[2]+=temp[2];

  //outsee[0]=drivewheel(128,MaxPwm,motors[0]);
  //outsee[1]=drivewheel(128,MaxPwm,motors[1]);
  //outsee[2]=drivewheel(128,MaxPwm,motors[2]);
  
  outsee[0]=drivewheel(Output[0],MaxPwm,motors[0]);
  outsee[1]=drivewheel(Output[1],MaxPwm,motors[1]);
  outsee[2]=drivewheel(Output[2],MaxPwm,motors[2]);
  }
  else
  {
    for(int i=0;i<3;i++){
    digitalWrite(motors[i]->pin1,HIGH);
    digitalWrite(motors[i]->pin2,HIGH);
    }
  }

  
}
void loop(){
  char data;
  if(Serial.available()>0)
    data=Serial.read();
  if(data=='s')
    flag^=1;
  
  Serial.println("Rpm A:"+ String(encoderA->Rpm)+" Rpm B:"+ String(encoderB->Rpm)+" Rpm C:"+ String(encoderC->Rpm) + "Out: "+String(Output[0])+" "+String(Output[1])+" "+String(Output[2])+"outsee:"+String(outsee[0]));//+"Rpm a:"+ String(encoderA->Rpm));
}
void initEncoders(){
  pinMode(encoderA->channelA, INPUT);
  pinMode(encoderA->channelB, INPUT);
  
  pinMode(encoderB->channelA, INPUT);
  pinMode(encoderB->channelB, INPUT);
  
  pinMode(encoderC->channelA, INPUT);
  pinMode(encoderC->channelB, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(encoderA->channelA),TicksA,RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB->channelA),TicksB,RISING);
  attachInterrupt(digitalPinToInterrupt(encoderC->channelA),TicksC,RISING);
}


void TicksA(){
  if(digitalRead(encoderA->channelB))
  encoderA->Count+=1;
  else
  encoderA->Count-=1;
}

void TicksB(){
  if(digitalRead(encoderB->channelB))
  encoderB->Count+=1;
  else
  encoderB->Count-=1;
}

void TicksC(){
  if(digitalRead(encoderC->channelB))
  encoderC->Count+=1;
  else
  encoderC->Count-=1;
}


