#include <DueTimer.h>
#include <Wire.h>
#include <DuePWM.h>
//Spartaaaaaaaa 33:43
#define rpmmax 300
#define Time 0.1
#define MaxPwm 255
#define GearRatio 0.89
#define desiredpwm(x) (x*255.0)/320.0
#define PWM_FREQ1 2500
DuePWM pwm(PWM_FREQ1, 3000);

int RPM[3] = {0};
int flag = 1;
int zeroflag[3];

struct gain {
  float kd;
  float kp;
  float ki;
  float required;
  float Prevrequired;
  float maxControl;
  float minControl;
  float error;
  float previousError;
  float derivativeError;
  float integralError;
};
int j;
struct gain Motor[3];
struct gain *pMotorgain[3] = {&Motor[0], &Motor[1], &Motor[2]};

//Struct for motors
struct motor {
  int pin1;
  int pin2;
  int pwmpin;
};
typedef struct motor MOTOR;

//MOTOR MotorA = {52, 50, 8};
//MOTOR MotorB = {46, 48, 7};
//MOTOR MotorC = {36, 38, 6};

MOTOR MotorA = {29, 23, 3};
MOTOR MotorB = {27, 25, 4};
MOTOR MotorC = {39, 37, 5};

MOTOR *motors[3] = {&MotorA, &MotorB, &MotorC};


//struct for encoder and lower control systemX

int Dirflag = 0;
//****************************************************************

struct encoder {
  int channelA;
  int channelB;
  long int Count;
  long int previousCount;
  int Rpm;
  int Ppr;
};

typedef struct encoder Encoder;

//Encoder encoderA,encoderB,encoderC;
//Encoder encoder1 = {22, 24, 0, 0, 0, 200};
//Encoder encoder2 = {26, 28, 0, 0, 0, 200};
//Encoder encoder3 = {34, 32, 0, 0, 0, 200};

Encoder encoder1 = {43, 41, 0, 0, 0, 200};
Encoder encoder2 = {47, 45, 0, 0, 0, 200};
Encoder encoder3 = {49, 51, 0, 0, 0, 200};

Encoder *encoderA = &encoder1;
Encoder *encoderB = &encoder2;
Encoder *encoderC = &encoder3;
//encoderC={};
float outsee[3] = {0};
float Output[3];

void setup() {
  initEncoders();
  Serial.begin(9600);
  //Serial3.begin(115200);
  Timer1.attachInterrupt(Timerhandler);
  Timer1.start(1000000 * Time);
  Wire.begin(8);
  Wire.onReceive(getData);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  initMotor(motors[0]);
  initMotor(motors[1]);
  initMotor(motors[2]);
  pwm.setFreq1(PWM_FREQ1);
  pwm.pinFreq1( motors[0]->pwmpin );  // Pin 6 freq set to "pwm_freq1" on clock A
  pwm.pinFreq1( motors[1]->pwmpin );  // Pin 7 freq set to "pwm_freq2" on clock B
  pwm.pinFreq1( motors[2]->pwmpin );  // Pin 8 freq set to "pwm_freq2" on clock B

  PIDinit(1, 0.15, 0.00, 0, 0, 255, pMotorgain[0]); //.09,.03,0.0

  PIDinit(0.95, 0.15, 0.00, 0, 0, 255, pMotorgain[1]); //1,.015,0.001

  PIDinit(1, 0.15, 0.00, 0, 0, 255, pMotorgain[2]); //1,.03

  pMotorgain[0]->required = 0;
  pMotorgain[1]->required = 0;
  pMotorgain[2]->required = 0 ;

  Serial.println("================================");
  delay(1000);
  //analogWrite(6,100);
  flag = 1;
}



void loop() {
}

void Timerhandler() {
  if (flag == 1) {
    Serial.println("A:"+String(encoderA->Rpm)+"B:"+String(encoderB->Rpm)+"C:"+String(encoderC->Rpm));
    encoderA->Rpm = ((encoderA->Count - encoderA->previousCount) * 60.0) / (Time * GearRatio * encoderA->Ppr);
    encoderB->Rpm = ((encoderB->Count - encoderB->previousCount) * 60.0) / (Time * GearRatio * encoderB->Ppr); //Why do we need previous count....
    encoderC->Rpm = ((encoderC->Count - encoderC->previousCount) * 60.0) / (Time * GearRatio * encoderC->Ppr);
    encoderA->previousCount = encoderA->Count;
    encoderB->previousCount = encoderB->Count;
    encoderC->previousCount = encoderC->Count;
    float temp[3];

    if (PIDstatus(pMotorgain[0]))
    {
      temp[0] = PID(abs(encoderA->Rpm), pMotorgain[0]);
      Output[0] = temp[0];
    }
    else
    {
      Output[0] = 0;
    }
    if (PIDstatus(pMotorgain[1]))
    {
      temp[1] = PID(abs(encoderB->Rpm), pMotorgain[1]);
      Output[1] = temp[1];
    }
    else
    {
      Output[1] = 0;
    }
    if (PIDstatus(pMotorgain[2]))
    {
      temp[2] = PID(abs(encoderC->Rpm), pMotorgain[2]);
      Output[2] = temp[2];
    }
    else
    {
      Output[2] = 0;
    }

    for (j = 0; j < 3; j++) {
      if (pMotorgain[j]->required == 0) {
        Output[j] = 0;
      }
    }
    drivewheel1(Output[0], MaxPwm, motors[0], pMotorgain[0]);
    drivewheel1(Output[1], MaxPwm, motors[1], pMotorgain[1]);
    drivewheel1(Output[2], MaxPwm, motors[2], pMotorgain[2]);

  }
  else
  {
    for (int i = 0; i < 3; i++) {
      digitalWrite(motors[i]->pin1, HIGH);
      digitalWrite(motors[i]->pin2, HIGH);
    }
  }

}

void initEncoders() {
  pinMode(encoderA->channelA, INPUT);
  pinMode(encoderA->channelB, INPUT);

  pinMode(encoderB->channelA, INPUT);
  pinMode(encoderB->channelB, INPUT);

  pinMode(encoderC->channelA, INPUT);
  pinMode(encoderC->channelB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderA->channelA), TicksA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB->channelA), TicksB, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderC->channelA), TicksC, RISING);

}


void TicksA() {
  if (digitalRead(encoderA->channelB))
    encoderA->Count += 1;
  else
    encoderA->Count -= 1;
}

void TicksB() {
  if (digitalRead(encoderB->channelB))
    encoderB->Count += 1;
  else
    encoderB->Count -= 1;
}

void TicksC() {
  if (digitalRead(encoderC->channelB))
    encoderC->Count += 1;
  else
    encoderC->Count -= 1;
}
int x;
void getData(int howMany) {
  while (Wire.available())
  {
    for (int y = 0; y < 3; y++)
    {
      x = 1;
      RPM[y] = 0;
      for (int r = 0; r < 3; r++)
      {
        RPM[y] += (int)Wire.read() * x;
        x *= 10;
      }
      RPM[y] -= 400;

    }

    pMotorgain[0]->Prevrequired = pMotorgain[0]->required;
    pMotorgain[1]->Prevrequired = pMotorgain[1]->required;
    pMotorgain[2]->Prevrequired = pMotorgain[2]->required;
    pMotorgain[0]->required = RPM[0];
    pMotorgain[1]->required = RPM[1];
    pMotorgain[2]->required = RPM[2];
    if (digitalRead(13))
      digitalWrite(13, LOW);
    else
      digitalWrite(13,HIGH);

  }
}

