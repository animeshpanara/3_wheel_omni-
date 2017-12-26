int alignBot(int alignTolerance)
{
   float alignControlActive=LineControl(LSAArray[f]->OePin,25,35,pAligngain);
   float alignControlPerpendicular=LineControl(LSAArray[l]->OePin,25,35,pAligngainperp);
   int theta = (((int)ToDeg((atan2(-alignControlPerpendicular,-alignControlActive)))+90));
   theta = (theta +LSAArray[f]->Theta)%360;
   float speed1 = (float)sqrt(pow(alignControlActive,2)+pow(alignControlPerpendicular,2))/(25*1.414);
   calcRPM(-Omegacontrol,theta,speed1*alignrpm,pwheel);              
   if(abs(alignControlActive)<alignTolerance && abs(alignControlPerpendicular)<alignTolerance)
      alignCounter++;
   else
      alignCounter = 0;   
   Serial.println("Theta "+String(theta)+"Speed "+String(speed1)+"Perp " + String(alignControlPerpendicular)+ "Active "+String(alignControlActive));
   if(alignCounter>10)
      return 1;
   else
      return 0;
}

bool alignBot1(int x,int Tolerance)
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
   if(abs(GetLSAReading(LSAArray[f]->OePin))<Tolerance&&abs(GetLSAReading(LSAArray[l]->OePin))<Tolerance)
   {
    Serial.println("::::ALIGNED::::"); 
    return 1;
   }
   else
   {
   float AlignControlActive=LineControl(LSAArray[Yaxis]->OePin,25,35,pAligngain);
   float AlignControlPerpendicular=LineControl(LSAArray[Xaxis]->OePin,25,35,pAligngainperp);
   float AlignControlActive1=LineControl(LSAArray[Yaxis1]->OePin,25,35,pAligngain1);
   float AlignControlPerpendicular1=LineControl(LSAArray[Xaxis1]->OePin,25,35,pAligngainperp1);
   
   int theta = (((int)ToDeg((atan2(-AlignControlPerpendicular+AlignControlPerpendicular1/2.0,-AlignControlActive+AlignControlActive1/2.0))) +90));
   theta = (theta +LSAArray[Yaxis]->Theta)%360;
   float speed1 = (float)sqrt(pow(AlignControlActive,2)+pow(AlignControlPerpendicular,2))/(25*1.414);
   calcRPM(-Omegacontrol,theta,speed1*alignrpm,pwheel);              
   Serial.println("Theta "+String(theta)+"Speed "+String(speed1)+"Perp " + String(AlignControlPerpendicular)+ "Active "+String(AlignControlActive));
   return 0;
   }
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
  posindex=0;
  dir = (enum activeLSA)arr[pos[0]][pos[1]][posindex]; 
  rdir = (enum activeLSA)abs((int)dir-3);
  pdir = (enum activeLSA)(((int)dir+2)%4);   
  ActiveLineSensor=dir;
  ActiveOmegaSensor=rdir;
  alignedFlag=0;
  Dirchange=0;
  Rotateflag=0;
  ToleranceOfAlignment=12;
  Throwcomplete=0;
  LoadFlag=0;
  alignCounter=0;
  cyclecomplete=0;
}


