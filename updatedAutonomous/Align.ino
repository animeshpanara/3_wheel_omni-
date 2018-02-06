bool alignBot(int x,float Tolerance){
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
   
   if(timerStart){
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
   
   if(abs(AlignControlActive)==maxBounds && abs(AlignControlActive1)==maxBounds && AlignControlActive==AlignControlActive1||abs(AlignControlPerpendicular)==maxBounds && abs(AlignControlPerpendicular1)==maxBounds && AlignControlPerpendicular==AlignControlPerpendicular1){
     theta = ((int)ToDeg((atan2(-AlignControlPerpendicular,-AlignControlActive))) +90);
     speed1 = (float)sqrt(pow(AlignControlActive,2)+pow(AlignControlPerpendicular,2))/(maxBounds*1.414);
     speed3 = speed1;
   }
   else{
     theta = (((int)ToDeg((atan2(-AlignControlPerpendicular+AlignControlPerpendicular1/2.0,-AlignControlActive+AlignControlActive1/2.0))) +90));
     speed1 = (float)sqrt(pow(AlignControlActive,2)+pow(AlignControlPerpendicular,2))/(maxBounds*1.414);
     speed2 = (float)sqrt(pow(AlignControlActive1,2)+pow(AlignControlPerpendicular1,2))/(maxBounds*1.414);
     speed3 = (speed1+speed2)/2;
   }
    
   theta = (theta +LSAArray[Yaxis]->Theta)%360;
   calcRPM(-alignOmegaControl,theta,speed3*alignrpm,pwheel);   
   if(abs(GetLSAReading(LSAArray[f]->OePin))<Tolerance && abs(GetLSAReading(LSAArray[b]->OePin))<Tolerance && abs(GetLSAReading(LSAArray[r]->OePin))<(Tolerance+5) && abs(GetLSAReading(LSAArray[l]->OePin))<(Tolerance+5))
     alignCounter++;
   else
     alignCounter = 0;
   
   if((millis()-alignTime)>10000){    
     alignCounter = 0;
     pAligngain->integralError=0;
     pAligngain1->integralError=0;
     pAligngainperp->integralError=0;
     pAligngainperp1->integralError=0;
     timerStart=1;
     calcRPM(0,0,0,pwheel); 
     TransmitRPM(pwheel);                 
     return 1; 
   }
  
   if(alignCounter>10){
     alignCounter = 0;
     pAligngain->integralError=0;
     pAligngain1->integralError=0;
     pAligngainperp->integralError=0;
     pAligngainperp1->integralError=0;
     timerStart = 1;
     calcRPM(0,0,0,pwheel);
     TransmitRPM(pwheel);
     return 1;
   }
   else
    return 0;
}


bool RotateBot(bool dir,int Tolerance)
{
  Serial.print("Entering");
  while((abs(GetLSAReading(LSAArray[f]->OePin))<220)){
    
    Serial.println('a');
    if(dir)
      calcRPM(rotateRPM,0,0,pwheel);              
    else
      calcRPM(-rotateRPM,0,0,pwheel);              
    TransmitRPM(pwheel);
    delay(100);
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
      
   while((abs(GetLSAReading(LSAArray[f]->OePin))>Tolerance)){
      int rotateControl = RotateControl(LSAArray[f]->OePin ,pRotategain);
      calcRPM(rotateControl,0,0,pwheel);
      TransmitRPM(pwheel);
      //Serial.println("Rotating by Dir: "+String(rotateControl));
      //updation of prev error of 4 sensors
      float AlignControlActive=LineControl(LSAArray[Yaxis]->OePin,maxBounds,35,pAligngain);
      float AlignControlPerpendicular=LineControl(LSAArray[Xaxis]->OePin,maxBounds,35,pAligngainperp);
      float AlignControlActive1=LineControl(LSAArray[Yaxis1]->OePin,maxBounds,35,pAligngain1);
      float AlignControlPerpendicular1=LineControl(LSAArray[Xaxis1]->OePin,maxBounds,35,pAligngainperp1);
      //delay(100);
   }
     
   calcRPM(0,0,0,pwheel);
   TransmitRPM(pwheel);
   return true;
}


