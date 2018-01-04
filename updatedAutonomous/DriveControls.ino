float LineThetaControl(int serialEn,int thetalimit,float PIDmaxbound,gain *pLinegain){
  float Linecontrol;
  float LineTheta = ToDeg(GetThetaofLSA(serialEn));
  if(abs(ToDeg(GetThetaofLSA(serialEn))) < PIDmaxbound){
  Linecontrol = PID(LineTheta,pLinegain);
  }
  else{
    Linecontrol = (float)(pLinegain->previousError)*thetalimit/abs(pLinegain->previousError);
  }
  return Linecontrol;
}


float LineControl(int serialEn,int thetalimit,float PIDmaxbound,gain *pLinegain){
  float Linecontrol;
  float Lineerror = GetLSAReading(serialEn);
  if(abs(Lineerror) < PIDmaxbound){
  Linecontrol = PID(Lineerror,pLinegain);
  }
  else{
      Linecontrol = (float)(pLinegain->previousError)*thetalimit/abs(pLinegain->previousError);
  }
  return Linecontrol;
}  

 
float OmegaControl(int Serialforward , int Serialback,int limit,gain *pgain)
{
  float Lineerror=0;
  float omegacontrol=0;
  float LSAforward = GetLSAReading(Serialforward);
  float LSAbackward = GetLSAReading(Serialback);
  
 if(abs(LSAforward) < 36){
  //do nothing
  }
  else{
      LSAforward = (float)(LSAforwardprev)*limit/abs(LSAforwardprev);
     }

  if(abs(LSAbackward) < 36){
  //do nothing
  }
  else{
      LSAbackward = (float)(LSAbackwardprev)*limit/abs(LSAbackwardprev);
      }
 Lineerror = LSAforward + LSAbackward;
  if(abs(Lineerror)<3){
    omegacontrol=0;
  }
  else if(abs(Lineerror) < 81){
    omegacontrol = PID(Lineerror,pgain);
   }
  LSAbackwardprev = LSAbackward;
  LSAforwardprev = LSAforward;

  return omegacontrol;
}

float RotateControl(int Serialforward , gain *pgain){
  
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
