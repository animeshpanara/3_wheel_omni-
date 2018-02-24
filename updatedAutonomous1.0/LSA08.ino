#define SerialLSA Serial2

void initLSA(int baud,int OutputEnable){
  pinMode(OutputEnable,OUTPUT);
  // pinMode(serialEn2,OUTPUT);
  // pinMode(serialEn3,OUTPUT);
  digitalWrite(OutputEnable,HIGH);
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
  //  clearJunction();
}


void sendCommand(char command, char data, char address){
  char checksum = address + command + data;  
  SerialLSA.write(address);
  SerialLSA.write(command);
  SerialLSA.write(data);
  SerialLSA.write(checksum);
}

void ChangeBaud(char baud, char add){
  char command='R';
  char data;
  if(baud==9600) data=0;
  else if(baud==19200) data=1;
       else if(baud==38400) data=2;
            else if(baud==57600) data=3;
                 else if(baud==115200) data=4;
                      else if(baud==230400) data=5;
   sendCommand(command,data,add);
}

void clearJunction(char add) {
  char address = add;
  char command = 'X';
  char data = 0x00;
  sendCommand(command,data,address);
}

int getJunction(char add){
  char address = add;
  char command = 'X';
  char data = 0x01;
  sendCommand(command,data,address);

  while(SerialLSA.available() <= 0);
  return (int(SerialLSA.read()));
}
//change serial by serial2
int GetByteOfLSA(int OutputEnable){                                            //Initially each and every serialLSAEnX(X=1,2,3) should be HIGH
  int a=0;
  digitalWrite(OutputEnable,LOW);
  while(SerialLSA.available()<=0);
  a=SerialLSA.read();
  digitalWrite(OutputEnable,HIGH);
  return a;   
}
 
float GetLSAReading(int OutputEnable){
  int LineReading = GetByteOfLSA(OutputEnable); 
  LineReading=35-LineReading;
  //delay(1); 
  return LineReading;
}

