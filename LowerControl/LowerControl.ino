#include <DueTimer.h>
//#include "digitalWriteFast.h"
#define Time 0.1
#define MaxPwm 255
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

MOTOR MotorA={52,50,13};
MOTOR MotorB={51,53,12};
MOTOR MotorC={48,49,11};

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
Encoder encoder1={22,24,0,0,0,1024};
Encoder encoder2={23,25,0,0,0,1024};
Encoder encoder3={26,27,0,0,0,1024};
Encoder *encoderA=&encoder1;
Encoder *encoderB=&encoder2;
Encoder *encoderC=&encoder3;
//encoderC={};
float outsee[3]={0};
float Output[3];

void setup(){
  initEncoders();
  Serial.begin(9600);
  Timer1.attachInterrupt(Timerhandler);
  Timer1.start(100000);
  
  initMotor(motors[0]);
  initMotor(motors[1]);
  initMotor(motors[2]);
  
  PIDinit(0.1,0,0,0,0,255,pMotorgain[0]);
  
  PIDinit(0.1,0,0,0,0,255,pMotorgain[1]);
  
  PIDinit(0.1,0,0,0,0,255,pMotorgain[2]);

  pMotorgain[0]->required=50;
  pMotorgain[2]->required=50;
  
}

void Timerhandler(){
  
  encoderA->Rpm=((encoderA->Count-encoderA->previousCount)*60.0)/(Time*12*encoderA->Ppr);
  encoderB->Rpm=((encoderB->Count-encoderB->previousCount)*60.0)/(Time*12*encoderB->Ppr);
  encoderC->Rpm=((encoderC->Count-encoderC->previousCount)*60.0)/(Time*12*encoderC->Ppr);
  encoderA->previousCount=encoderA->Count;
  encoderB->previousCount=encoderB->Count;
  encoderC->previousCount=encoderC->Count;
  
  float temp[3];
  
  temp[0]=PID(encoderA->Rpm,pMotorgain[0]);
  temp[1]=PID(encoderB->Rpm,pMotorgain[1]);
  temp[2]=PID(encoderC->Rpm,pMotorgain[2]);

  Output[0]+=temp[0];
  Output[1]+=temp[1];
  Output[2]+=temp[2];

  outsee[0]=drivewheel(Output[0],MaxPwm,motors[0]);
  outsee[1]=drivewheel(Output[1],MaxPwm,motors[1]);
  outsee[2]=drivewheel(Output[2],MaxPwm,motors[2]);
  
}
void loop(){
  Serial.println("Rpm A:"+ String(encoderA->Rpm)+" Rpm B:"+ String(pMotorgain[0]->required)+" Rpm C:"+ String(encoderC->Rpm) + "Out: "+String(Output[0])+" "+String(Output[1])+" "+String(Output[2])+"outsee:"+String(outsee[0]));//+"Rpm a:"+ String(encoderA->Rpm));
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


