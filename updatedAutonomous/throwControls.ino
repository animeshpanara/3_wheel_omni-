void ThrowShuttleCock(){
   delay(1000);
   digitalWrite(ThrowPin,LOW);
   delay(1000);
   digitalWrite(ThrowPin,HIGH);
   delay(1000);
}

void LoadBot(){
   int LoadPos;
   if(pos[1]==3)
       LoadPos=1;
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
    if(pos[1]==4){
      digitalWrite(DAC_PinTZ2,LOW);
      digitalWrite(DAC_PinTZ3,HIGH);
    }
    else if(pos[1]==5){
      digitalWrite(DAC_PinTZ3,LOW);
      digitalWrite(DAC_PinTZ2,HIGH);
    }
    else{
     digitalWrite(DAC_PinTZ2,HIGH);
     digitalWrite(DAC_PinTZ2,HIGH);
   }
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

