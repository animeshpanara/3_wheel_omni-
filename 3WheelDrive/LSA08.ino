#define SerialLSA Serial3
#include "includes.h"

void initLSA(int baud, LSA08 ** LSA){
  for(int i =0;i<3;i++){
    pinMode(LSA[i]->JunctionPin,INPUT);
    pinMode(LSA[i]->OutputEnable,OUTPUT);
  //pinMode(serialEn2,OUTPUT);
  //pinMode(serialEn3,OUTPUT);
    digitalWrite(LSA[i]->OutputEnable,HIGH);
//  digitalWrite(serialEn2,HIGH);
//  digitalWrite(serialEn3,HIGH);
//  sendCommand('X',0x00,add);
//  sendCommand('L',0x01,add);
//  sendCommand('A',0x01,add);
//  sendCommand('B',0x05,add);
//  sendCommand('S',90,add);
//  sendCommand('R',0x00,add);
//  sendCommand('D',0x02,add);
//  ChangeBaud(baud,add);
   clearJunction(LSA[i]);
  }
  
}


void sendCommand(char command, char data, LSA08 * LSA) {
  char checksum = LSA->address + command + data;  
  SerialLSA.write(LSA->address);
  SerialLSA.write(command);
  SerialLSA.write(data);
  SerialLSA.write(checksum);
}

void ChangeBaud(char baud, LSA08 * LSA)
{
  char command='R';
  char data;
  if(baud==9600) data=0;
  else if(baud==19200) data=1;
       else if(baud==38400) data=2;
            else if(baud==57600) data=3;
                 else if(baud==115200) data=4;
                      else if(baud==230400) data=5;
   sendCommand(command,data,LSA);
}

void clearJunction(LSA08 * LSA) 
{
  char address = LSA->address;
  char command = 'X';
  char data = 0x00;
  sendCommand(command,data,address);
}

int getJunction(LSA08 * LSA){
  char address = LSA->address;
  char command = 'X';
  char data = 0x01;
 sendCommand(command,data,address);

  while(SerialLSA.available() <= 0);
  return (int(SerialLSA.read()));
}
//change serial by serial2
byte GetByteOfLSA(LSA08 * LSA){                                            //Initially each and every serialLSAEnX(X=1,2,3) should be HIGH
  byte a=0;
  digitalWrite(LSA->OutputEnable,LOW);
  while(SerialLSA.available()<=0);
  a=SerialLSA.read();
  digitalWrite(LSA->OutputEnable,HIGH);
  return a;   
 }
float GetLSAReading(LSA08 * LSA){
  int LineReading = GetByteOfLSA(LSA->OutputEnable); 
  LineReading=35-LineReading;
  return LineReading;
}
float GetThetaofLSA(LSA08 * LSA){
   int LineReading = GetByteOfLSA(LSA->OutputEnable); 
   LineReading=35-LineReading;
 //  Serial.println(LineReading); 
   float scaledReading = (float)LSAlength*LineReading/70;
   float theta = atan(scaledReading/LSAdistance);
   return theta;
}
