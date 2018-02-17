void  ThrowShuttleCock(int pos){
   if(TimedchechIr(20,2.5)){
       digitalWrite(ThrowLed,HIGH);
       delay(1000);
       digitalWrite(ThrowPin,HIGH);
       delay(1000);
       digitalWrite(ThrowPin,LOW);
       //delay(1000/2);
       digitalWrite(ThrowLed,LOW);
   }
       if(pos==5){
          int throwagain=newMech();
         if(throwagain==1){
             Serial.println("I am here");
             digitalWrite(ThrowLed,HIGH);
             delay(1000);
             digitalWrite(ThrowPin,HIGH);
             delay(1000);
             digitalWrite(ThrowPin,LOW);
             delay(1000/2);
             digitalWrite(ThrowLed,LOW);
             }
       }
   
   //delay(1000);
}
bool TimedchechIr(int cntr,float TimeInSec){
  long int timerIr=0;
  timerIr=millis();
  while(1){
    if(!digitalRead(TriggerPin))
      Ircounter++;
    else
      Ircounter=0;
    //Serial.println("IRCounter:"+String(Ircounter));  
    if(millis()-timerIr>TimeInSec*1000)
    {
      Ircounter=0;
      return 0;
      //break;
    }
    if(Ircounter>cntr)
    {
      Ircounter=0;
      return 1;
      //break;
    }
  }
}
void chechIr(int cntr){
  while(1){
    if(!digitalRead(TriggerPin))
      Ircounter++;
    else
      Ircounter=0;
    Serial.println("IRCounter:"+String(Ircounter));  
    if(Ircounter>cntr)
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
   if(pos[1]==3 || pos[1]==0)
       LoadPos=1;
   else if(pos[1]==5){
       LoadPos=2;
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
   Rotateflag=-1;
   cyclecomplete=1;
   ToleranceOfAlignment=6;
}

void NextThrowCycle(int posx){ 
    int LoadPos=2;
    pos[0]=pos[1];
    pos[1]=posx;
    if(pos[1]==3){
      DACcounter=dacTZ1;
      }
    else if(pos[1]==4){
      DACcounter=dacTZ2;
      }
    else if(pos[1]==5){
      DACcounter=dacTZ3;
      }
   dac.setVoltage(DACcounter, false);  
   
   posindex=0;
   
   dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex]; 
   rdir = (enum activeLSA)abs((int)dir-3);
   pdir = (enum activeLSA)(((int)dir+2)%4);   
   ActiveLineSensor=dir;
   ActiveOmegaSensor=rdir;
   
   LoadFlag=0;
   alignedFlag=0;
   Rotateflag=0;
   cyclecomplete=0;
   ToleranceOfAlignment=10;
   CheckIrflag=1;
   //Dirchange=0;
   //alignCounter=0;
   //Throwcomplete=0;
}
void ReLoadBot(){
   int LoadPos,temp=0;
   
   if(pos[0]==1&&pos[1]==3)
   temp=1;
   if(pos[0]==1&&pos[1]==4)
   temp=2;
   if(pos[0]==1&&pos[1]==5)
   temp=5;
   if(pos[0]==2&&pos[1]==3)
   temp=2;
   if(pos[0]==2&&pos[1]==4)
   temp=1;
   if(pos[0]==2&&pos[1]==5)
   temp=4;
   
   LoadPos=pos[0];    
   pos[0]=pos[1];
   pos[1]=LoadPos;
   posindex=temp;
   dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex]; 
   rdir = (enum activeLSA)abs((int)dir-3);
   pdir = (enum activeLSA)(((int)dir+2)%4);   
   ActiveLineSensor=dir;
   ActiveOmegaSensor=rdir;
   
   LoadFlag=1;
   alignedFlag=0;
   Rotateflag=-1;
   cyclecomplete=1;
   ToleranceOfAlignment=6;
   CheckIrflag=0;
}

