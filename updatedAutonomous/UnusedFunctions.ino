int alignBot1(int alignToleranceOmega, int alignTolerancePerp){
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
   if(alignCounter>10)
      return 1;
   else
      return 0;
}

void RotateBot1(bool dir,int Tolerance){
 while(abs(GetLSAReading(LSAArray[f]->OePin))<35&&abs(GetLSAReading(LSAArray[l]->OePin))<35){
      if(dir)
        calcRPM(100,0,0,pwheel);              
      else
        calcRPM(-100,0,0,pwheel);              
      TransmitRPM(pwheel);
    } 
   while((abs(GetLSAReading(LSAArray[f]->OePin))>Tolerance||abs(GetLSAReading(LSAArray[l]->OePin))>Tolerance)){
      if(dir)
        calcRPM(100,0,0,pwheel);              
      else
        calcRPM(-100,0,0,pwheel);              
      TransmitRPM(pwheel);
   }
}

