
float HeadControl(float head,volatile gain* IMUgain){
  GetIMUReading();
  yawdeg=(int(ToDeg(yaw)+360))%360;    
  float OmegaControl = PID(yawdeg,IMUgain);
  return OmegaControl;
}

float CompassHeadControl(float head,volatile gain* Compassgain){
  GetCompassHeading();    
  float CompassControl = PID(CompassHeading,Compassgain);
  return CompassControl;
}

float LineThetaControl(int serialEn,int thetalimit,float PIDmaxbound,volatile gain *pLinegain){
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
float LineControl(int serialEn,int thetalimit,float PIDmaxbound,volatile gain *pLinegain){
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

float OmegaControl(int Serialforward , int Serialback,int limit,volatile gain *pgain)
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
