/*
 * TO DO:
 * Check if IMU giving readings- Timer interrupt to calc loop counter value, if same then rese
 * Limit ang_velocity?
 * 6-A,7-B,8-C
 * enum LSA
 */


#include "includes.h"

#define mode 2
//#define mode 1
//Mode 1 - Line following + IMU control
//Mode 0 - IMU control + Manual Driving


struct gain IMUgain, Linegain[3];
struct gain *pIMUgain = &IMUgain, *pLinegain[3] = {&Linegain[0],&Linegain[1],&Linegain[2]};
int flag = 0;
int theta;
wheel wheela = {0, 0, anglea, rpmmax, pinpwma, pinaa, pinab,0}, wheelb = {0, 0, angleb, rpmmax, pinpwmb, pinba, pinbb,0}, wheelc = {0, 0, anglec, rpmmax, pinpwmc, pinca, pincb,0};
wheels *pwheel[]={&wheela,&wheelb,&wheelc};
LSA08 LSA08a={0x01,270,3,0,35}, LSA08b={0x02,0,2,0,37}, LSA08c={0x03,180,18,0,39};
LSA08 * pLSA08[]={&LSA08a,&LSA08b,&LSA08c};
int Test[3]={6,1,0};
enum LSA{LSAa,LSAb,LSAc};
enum LSA ActiveLSA = LSAa;
float Linecontrol, IMUcontrol;
int ActiveSensor=0;
void setup() {
  Serial.begin(9600);
  //Serial2.begin(115200);
  Serial3.begin(9600);
  IMUinit();                //Initialise IMU
  SetOffset();              //Take initial readings for offset
  initDriving();
  initLSA(9600, pLSA08);            //const int minControl = -255;      const int maxControl = 255;
  //PIDinit(15,0,0,0,-255,255, pIMUgain);
  PIDinit(13,2,0,0,-255,255, pIMUgain);
  PIDinit(.5,0,0,0,-255,255,pLinegain[0]);
  PIDinit(.5,0,0,0,-255,255,pLinegain[1]);
  timer=millis();           //save ccurrent time in timer for gyro integration
  delay(20);
  counter=0;
  
}
///////////////////Set limit if >90

void loop() {
      if(pLSA08[ActiveSensor]->JunctionCount>Test[ActiveSensor]){
        Serial.println("hello");
        brakeWheels(pwheel);
        ActiveSensor^=1;
      }

      else 
      {
      float IMUcontrol=HeadControl(HeadTheta,pIMUgain);
      Serial.print("a");
      float Linecontrol=LineControl(pLSA08[ActiveSensor],17,35,pLinegain[ActiveSensor]);
      Serial.print("a");
      for(int j=0;j<2;j++)
      if(digitalRead(pLSA08[j]->JunctionPin)){
        while(digitalRead(pLSA08[j]->JunctionPin)){
        Serial.print("hello");
        }
        pLSA08[j]->JunctionCount=getJunction(pLSA08[j]->address);
      }
      calcRPM(IMUcontrol,pLSA08[ActiveSensor]->theta+Linecontrol,trans_velMax,pwheel);
      //calcRPM(Linecontrol,90,rpmmax,pwheel);
      Serial.println(" Head: "+String(IMUcontrol)+" Line: "+String(Linecontrol)+" CurrentYaw: "+ String(ToDeg(yaw))+" Junction1: "+String(pLSA08[ActiveSensor]->JunctionCount));
      if(mode==1){
      Serial2.flush();
       if(Serial2.available()>0){
       char data = Serial2.read();
       switch(data){
        case 'w':
          theta=270;
          break;
        case 'a':
          theta=180;
          break;
        case 's':
          theta=90;
          break;
        case 'd':
          theta=0;
          break;
        case 'c':
          CalibrateIMU(pIMUgain);
          break;
          }
          calcRPM(IMUcontrol,theta,trans_velMax,pwheel);
          startMotion(pwheel);
       }
       else
          brakeWheels(pwheel);
      }
      else if(mode==2){
       if(Serial.available()>0){
        Serial.print("a");
        String data = Serial.readString();
        Serial.print(data);
        if (data == "s")
          flag^=1;
        else if(data== "c")
            CalibrateIMU(pIMUgain);
        else
            theta=atoi(data.c_str());
        }
      if(flag==1){
        Serial.println(" Started ");
        startMotion(pwheel);
        }
      else if(flag==0){
        Serial.println(" Stopped ");
        brakeWheels(pwheel);
        }
      }
     }
}
