int alignBot(int alignToleranceOmega, int alignTolerancePerp)
{
   float alignControlActive=LineControl(LSAArray[f]->OePin,40,36,pAligngain);
   float alignControlPerpendicular=LineControl(LSAArray[l]->OePin,40,36,pAligngainperp);
   float alignOmegaControl = OmegaControl(LSAArray[f]->OePin,LSAArray[b]->OePin,40,pAlignOmegagain);
   int theta = (((int)ToDeg((atan2(-alignControlPerpendicular,-alignControlActive)))+90));
   theta = (theta +LSAArray[f]->Theta)%360;
   float speed1 = (float)sqrt(pow(alignControlActive,2)+pow(alignControlPerpendicular,2))/(40*1.414);
   calcRPM(-alignOmegaControl,theta,speed1*alignrpm,pwheel);              
   if(abs(GetLSAReading(LSAArray[f]->OePin))<alignToleranceOmega && abs(GetLSAReading(LSAArray[l]->OePin))<alignTolerancePerp)
      alignCounter++;
   else
      alignCounter = 0;   
   Serial.println("Theta "+String(theta)+"Speed "+String(speed1)+"Perp " + String(alignControlPerpendicular)+ "Active "+String(alignControlActive));
   if(alignCounter>10)
      return 1;
   else
      return 0;
}
bool alignBot1(int x,float Tolerance)
{
   
   int Yaxis,Xaxis,Xaxis1,Yaxis1;
   switch(x){
   case 0:Yaxis=0;
          Xaxis=2;
          Yaxis1=3;
          Xaxis1=1;
          break;
   case 1:Yaxis=2;
          Xaxis=3;
          Yaxis1=1;
          Xaxis1=0;
          break;
   case 2:Yaxis=1;
          Xaxis=0;
          Yaxis1=2;
          Xaxis1=3;
          break;
   case 3:Yaxis=3;
          Xaxis=1;
          Yaxis1=0;
          Xaxis1=2;
          break;
   }
   if(timerStart)
   {
      alignTime = millis();
      timerStart=0;
   }  
   int maxBounds=40;
   float AlignControlActive=LineControl(LSAArray[Yaxis]->OePin,maxBounds,35,pAligngain);
   float AlignControlPerpendicular=LineControl(LSAArray[Xaxis]->OePin,maxBounds,35,pAligngainperp);
   float AlignControlActive1=LineControl(LSAArray[Yaxis1]->OePin,maxBounds,35,pAligngain1);
   float AlignControlPerpendicular1=LineControl(LSAArray[Xaxis1]->OePin,maxBounds,35,pAligngainperp1);
   
   float alignOmegaControl = OmegaControl(LSAArray[f]->OePin,LSAArray[b]->OePin,2*maxBounds,pAlignOmegagain);
   int theta;
   float speed1,speed2,speed3;
   if(abs(AlignControlActive)==maxBounds && abs(AlignControlActive1)==maxBounds && AlignControlActive==AlignControlActive1||abs(AlignControlPerpendicular)==maxBounds && abs(AlignControlPerpendicular1)==maxBounds && AlignControlPerpendicular==AlignControlPerpendicular1)
    {
        theta = ((int)ToDeg((atan2(-AlignControlPerpendicular,-AlignControlActive))) +90);
        speed1 = (float)sqrt(pow(AlignControlActive,2)+pow(AlignControlPerpendicular,2))/(maxBounds*1.414);
        speed3 = speed1;
    }
   else
    {
        theta = (((int)ToDeg((atan2(-AlignControlPerpendicular+AlignControlPerpendicular1/2.0,-AlignControlActive+AlignControlActive1/2.0))) +90));
        speed1 = (float)sqrt(pow(AlignControlActive,2)+pow(AlignControlPerpendicular,2))/(maxBounds*1.414);
        speed2 = (float)sqrt(pow(AlignControlActive1,2)+pow(AlignControlPerpendicular1,2))/(maxBounds*1.414);
        speed3 = (speed1+speed2)/2;
    }
    
   theta = (theta +LSAArray[Yaxis]->Theta)%360;
   calcRPM(-alignOmegaControl,theta,speed3*alignrpm,pwheel);   
              
   Serial.println("Theta "+String(theta)+"Speed "+String(speed3)+"Perp " + String(AlignControlPerpendicular)+ "Active "+String(AlignControlActive));
  //if(abs(AlignControlActive)<Tolerance && abs(AlignControlPerpendicular)<Tolerance)
   if(abs(GetLSAReading(LSAArray[f]->OePin))<Tolerance && abs(GetLSAReading(LSAArray[l]->OePin))<Tolerance && abs(GetLSAReading(LSAArray[r]->OePin))<(Tolerance+5) && abs(GetLSAReading(LSAArray[l]->OePin))<(Tolerance+5))    
      {
        alignCounter++;
      }
   else
      alignCounter = 0;
   
   if((millis()-alignTime)>12000)
   {    
      timerStart=1;
      calcRPM(0,0,0,pwheel);                  
      return 1; 
   }
       
  // Serial.println("Theta "+String(theta)+"Speed "+String(speed1)+"Perp " + String(alignControlPerpendicular)+ "Active "+String(alignControlActive));
   if(alignCounter>10)
   {
      pAligngain->integralError=0;
      pAligngain1->integralError=0;
      pAligngainperp->integralError=0;
      pAligngainperp1->integralError=0;
      timerStart = 1;
      calcRPM(0,0,0,pwheel);
      return 1;
   }
   else
      return 0;

}
void RotateBot(bool dir,int Tolerance)
{
    while(abs(GetLSAReading(LSAArray[f]->OePin))<35&&abs(GetLSAReading(LSAArray[l]->OePin))<35)
    {
      if(dir)
        calcRPM(100,0,0,pwheel);              
      else
        calcRPM(-100,0,0,pwheel);              
      TransmitRPM(pwheel);
    } 
   while((abs(GetLSAReading(LSAArray[f]->OePin))>Tolerance||abs(GetLSAReading(LSAArray[l]->OePin))>Tolerance))
   {
      if(dir)
        calcRPM(100,0,0,pwheel);              
      else
        calcRPM(-100,0,0,pwheel);              
      TransmitRPM(pwheel);
      Serial.println("Rotating by Dir: "+String(dir));
   }
}

//int rotateControl;

bool RotateBot1(bool dir,int Tolerance)
{
  Serial.print("Entering");
    while((abs(GetLSAReading(LSAArray[f]->OePin))<220))
    {
      Serial.println('a');
      if(dir)
        calcRPM(100,0,0,pwheel);              
      else
        calcRPM(-100,0,0,pwheel);              
      TransmitRPM(pwheel);
    }
   if(dir)
    LSArotateprev= -40;
   else
    LSArotateprev= 40;

   int maxBounds=40;
   int Yaxis,Xaxis,Xaxis1,Yaxis1;
    
   switch(ActiveLineSensor){
   case 0:Yaxis=0;
          Xaxis=2;
          Yaxis1=3;
          Xaxis1=1;
          break;
   case 1:Yaxis=2;
          Xaxis=3;
          Yaxis1=1;
          Xaxis1=0;
          break;
   case 2:Yaxis=1;
          Xaxis=0;
          Yaxis1=2;
          Xaxis1=3;
          break;
   case 3:Yaxis=3;
          Xaxis=1;
          Yaxis1=0;
          Xaxis1=2;
          break;
   }
    
   while((abs(GetLSAReading(LSAArray[f]->OePin))>Tolerance))
   {
      int rotateControl = RotateControl(LSAArray[f]->OePin ,pRotategain);
      calcRPM(rotateControl,0,0,pwheel);
      TransmitRPM(pwheel);
      Serial.println("Rotating by Dir: "+String(rotateControl));
      //updation of prev error of 4 sensors
      float AlignControlActive=LineControl(LSAArray[Yaxis]->OePin,maxBounds,35,pAligngain);
      float AlignControlPerpendicular=LineControl(LSAArray[Xaxis]->OePin,maxBounds,35,pAligngainperp);
      float AlignControlActive1=LineControl(LSAArray[Yaxis1]->OePin,maxBounds,35,pAligngain1);
      float AlignControlPerpendicular1=LineControl(LSAArray[Xaxis1]->OePin,maxBounds,35,pAligngainperp1);
   
   }
   
   Serial.println("Exiting");
      calcRPM(0,0,0,pwheel);
      TransmitRPM(pwheel);
   return true;
}


float RotateControl(int Serialforward , volatile gain *pgain)
{
   float LSArotate = -GetLSAReading(Serialforward);
   int rotatecontrol;
   
   if(abs(LSArotate) < 36){
    //do nothing
    }
    else{
     LSArotate = (float)(LSArotateprev)*40/abs(LSArotateprev);
    }
    if(LSArotate<41)
    rotatecontrol = PID(LSArotate, pgain);
    
    LSArotateprev = LSArotate;

    return rotatecontrol;
}

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
  else
  {
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

