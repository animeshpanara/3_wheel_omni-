#include <Wire.h>

int current=0,desired=0,flag=0;
int ta1[]={5,6,7,8};
int ta2[]={1,2,3,4};
int ta1count=0, ta2count=0;
enum direction {forward, backward};
enum direction motordir;
int desiredpos, currentpos;
const int StrikerPin = 10;
const int liftRackPin = 8;
const int releaseRackPin=9;
const int ClutchWallPin = 11;
const int ReleaseClutchWallPin =13;
const int channelB = 4;
const int pulse = 2;
const int channelA = 3;
const int motorA=5,motorB=7,motorPWM=6;
const int rackSpeed=30;
const int rackSlowSpeed=50;
volatile bool dataReceived = false;
volatile char data;
int wallFlag =0 ;
int liftFlag=0;
int moveFlag=0;

void setup() {
  //Wire.begin(8);
  Serial.begin(9600);
 // Wire.onReceive(getData);
  initRackMovement();
  initRackLift();
  initWallClutch();
  initStriker();
}
int dirFlag=0;

void loop() {
      if(dataReceived){
      executeI2Creceived(data);
      dataReceived=false;
      Serial.println('a');
      }
      //Serial.println('a');
       if(moveFlag)
       moveRack(dirFlag);  
       
}



/* Striker */

void initStriker()
{
 pinMode(StrikerPin,OUTPUT);//for first pneumatic
 digitalWrite(StrikerPin, LOW); 
}
void strike()
{
  digitalWrite(StrikerPin,HIGH);
  delay(1000);
  digitalWrite(StrikerPin,LOW);
}



/* Wall Clutch */
void initWallClutch()
{
  pinMode(ClutchWallPin,OUTPUT);//for second pneumatic
  digitalWrite(ClutchWallPin, LOW);
}
void clutchWall()
{
  digitalWrite(ClutchWallPin,HIGH); 
  //delay(1000);
  //digitalWrite(ClutchWallPin,LOW);  
}
void unclutchWall()
{
  digitalWrite(ClutchWallPin,LOW); 
  //delay(1000);
  //digitalWrite(ReleaseClutchWallPin,LOW); 
  
}


/* Rack */
void initRackLift()
{
  pinMode(liftRackPin,OUTPUT);//for third pneumatic
  pinMode(releaseRackPin, OUTPUT);
  digitalWrite(liftRackPin, LOW);
  digitalWrite(releaseRackPin, LOW);
}
void liftRack()
{
  digitalWrite(liftRackPin,HIGH);
  delay(1000);
  digitalWrite(liftRackPin,LOW);

}

void releaseRack()
{
  digitalWrite(releaseRackPin,HIGH);
  delay(1000);
  digitalWrite(releaseRackPin,LOW);
}




/*Linear Encoder*/
void initRackMovement(){
  pinMode(channelA,INPUT);   //INPUT PULSE
  pinMode(pulse,INPUT);
  pinMode(channelB,INPUT);
  pinMode(motorA,OUTPUT);  //MOTOR DIRECTION
  pinMode(motorB,OUTPUT);
  pinMode(motorPWM,OUTPUT); 
  analogWrite(motorPWM,rackSpeed);  
  currentpos=0;
  attachInterrupt(digitalPinToInterrupt(pulse),changePosition,RISING);
  //attachInterrupt(digitalPinToInterrupt(channelA),checkDirection,RISING);
}

void moveRack(int dir){
  if(abs(currentpos-desiredpos)==1)
  { 
    analogWrite(motorPWM,rackSlowSpeed);
  }
  else
  {
    analogWrite(motorPWM,rackSpeed); 
  }   
  if(dir<0&&abs(currentpos)!=abs(desiredpos)){
    Serial.print("3");
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,LOW);
   }
  else if(dir>0&&abs(currentpos)!=abs(desiredpos)){
    Serial.print("2");
    digitalWrite(motorA,LOW);
    digitalWrite(motorB,HIGH);
  }
  if(abs(currentpos)>=abs(desiredpos)){
    analogWrite(motorPWM,255); 
    digitalWrite(motorA,HIGH);
    digitalWrite(motorB,HIGH);
    moveFlag=0;
  }
  
  current = (currentpos+1)/2;
  desired = (desiredpos+1)/2; 
  Serial.println("POS:"+String(currentpos)+"cur:"+String(desiredpos)+"des");
}





void executeI2Creceived(volatile char data){
  if(data=='l')
  {
    //Move Linear left
    Serial.println('l');
    desiredpos=1;
    currentpos=0;
    moveFlag=1;
    dirFlag=-1;
    
  }
    
  if(data=='r')
  {
    //Move linear right
    Serial.println('r');
    desiredpos=1;
    currentpos=0;
    moveFlag=1;
    dirFlag=1;
  }
  
  if(data=='s')
  {
    //Strike
    Serial.println('s');
    strike();
  }
  
  if(data=='w')
  {
    //Toggle wall clutch
    Serial.println('w');
    wallFlag^=1;
    if(wallFlag)
    clutchWall();
    else
    unclutchWall();  
  }
  
  if(data=='x')
  {
    //toggle rack lift
    Serial.println('x');
    liftFlag^=1;
    if(liftFlag)
    releaseRack();
    else
    liftRack();
    }
}

//
//void getData(int howMany){
//  dataReceived = true;
//  data = Wire.read();
//}

void serialEvent(){
  if(Serial.available())
  {
  dataReceived = true;
  data = Serial.read();
  }
}

void changePosition(){
//    if(motordir == forward){
//      currentpos+=1;
//      } 
//     else if(motordir == backward)
//     {
//      currentpos-=1; 
//     }
      currentpos+=1;
}

/*void checkDirection(){
  
  if(digitalRead(channelB)==HIGH)
  { 
    motordir = forward;
    Serial.println("-------------------------------------");
  }
  else{
    motordir = backward;
    Serial.println("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
  }
}
*/
