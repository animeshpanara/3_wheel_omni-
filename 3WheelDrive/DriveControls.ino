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
      float Linecontrol=thetalimit;
      float Lineerror = GetLSAReading(serialEn);
      if(abs(Lineerror) < PIDmaxbound){
      Linecontrol = PID(Lineerror,pLinegain);
      }
      else{
          Linecontrol = (float)(pLinegain->previousError)*thetalimit/abs(pLinegain->previousError);
      }
      return Linecontrol;
}

float HeadControl(float head,gain* IMUgain){
  GetIMUReading();    
  float IMUControl = PID(ToDeg(yaw),IMUgain);
  return IMUControl;
}

float CompassHeadControl(float head,gain* Compassgain){
  GetModifiedCompassHeading();    
  float CompassControl = PID(CompassHeading,Compassgain);
  return CompassControl;
}
