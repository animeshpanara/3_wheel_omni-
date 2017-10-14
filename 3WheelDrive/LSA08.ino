void initLSA(){
  pinMode(serialEn1,OUTPUT);
  pinMode(serialEn2,OUTPUT);
  pinMode(serialEn3,OUTPUT);
  digitalWrite(serialEn1,HIGH);
  digitalWrite(serialEn2,HIGH);
  digitalWrite(serialEn3,HIGH);
  sendCommand('X',0x00,add);
  sendCommand('L',0x01,add);
  sendCommand('A',0x01,add);
  sendCommand('B',0x05,add);
  sendCommand('S',90,add);
  sendCommand('R',0x00,add);
  sendCommand('D',0x02,add);
  clearJunction();
}


void sendCommand(char command, char data, char address) {
  char checksum = address + command + data;  
  Serial.write(address);
  Serial.write(command);
  Serial.write(data);
  Serial.write(checksum);
}

void ChangeBaud(char baud, char add)
{
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

void clearJunction() 
{
  char address = 0x01;
  char command = 'X';
  char data = 0x00;
  sendCommand(command,data,address);
}



