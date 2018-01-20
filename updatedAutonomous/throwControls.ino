void ThrowShuttleCock(){
   digitalWrite(ThrowLed,HIGH);
   delay(1000);
   digitalWrite(ThrowLed,LOW);
   delay(1000/2);
   digitalWrite(ThrowPin,HIGH);
   delay(1000);
   digitalWrite(ThrowPin,LOW);
   //delay(1000);
}
void chechIr(){
  while(1){
    if(!digitalRead(TriggerPin))
      Ircounter++;
    else
      Ircounter=0;
    Serial.println("IRCounter:"+String(Ircounter));  
    if(Ircounter>50)
    {
      Ircounter=0;
      break;
    }
  }
}
int CheckBall(){
  PiSerial.write('a');
  while(PiSerial.available()<=0);
  char x=PiSerial.read();
  Serial.println("Ball detected:"+String(x));
  if(x=='a') return 5;
  else if(x=='b') return 4;
  else if(x=='r') return 3;
  else return 0;
}
void LoadBot(){
   int LoadPos;
   if(pos[1]==3)
       LoadPos=1;
   else if(pos[1]==5){
       LoadPos=1;
       //DACcounter=0;
       //dac.setVoltage(DACcounter,false);         
   }    
   else
       LoadPos=2;
   pos[0]=pos[1];
   pos[1]=LoadPos;
   posindex=0;
   dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex]; 
   rdir = (enum activeLSA)abs((int)dir-3);
   pdir = (enum activeLSA)(((int)dir+2)%4);   
   ActiveLineSensor=dir;
   ActiveOmegaSensor=rdir;
   LoadFlag=1;
   alignedFlag=0;
   cyclecomplete=1;
}

void NextThrowCycle(int posx){ 
    int LoadPos=2;
    pos[0]=pos[1];
    pos[1]=posx;
    if(pos[1]==3){
      DACcounter=645;
      }
    else if(pos[1]==4){
      DACcounter=910;
      }
    else if(pos[1]==5){
      DACcounter=1210;
      }
   dac.setVoltage(DACcounter, false);  
   posindex=0;
   dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex]; 
   rdir = (enum activeLSA)abs((int)dir-3);
   pdir = (enum activeLSA)(((int)dir+2)%4);   
   ActiveLineSensor=dir;
   ActiveOmegaSensor=rdir;
   alignedFlag=0;
   Dirchange=0;
   Rotateflag=0;
   ToleranceOfAlignment=10;
   Throwcomplete=0;
   LoadFlag=0;
   alignCounter=0;
   cyclecomplete=0;
}

