float LineControl(int serialEn,int thetalimit,float PIDmaxbound,gain *pLinegain){
      float LineTheta = ToDeg(GetThetaofLSA(serialEn));
      if(abs(ToDeg(GetThetaofLSA(serialEn))) < PIDmaxbound){
      Linecontrol = PID(LineTheta,pLinegain);
      }
      else{
        Linecontrol = (float)(pLinegain->previousError)*thetalimit/abs(pLinegain->previousError);
      }
      //Serial.print("Linecontrol: "+String(Linecontrol));
      return Linecontrol;
}
float HeadControl(float head,gain* IMUgain){
  GetIMUReading();    
  float IMUControl = PID(ToDeg(yaw),IMUgain);
  return IMUControl;
}

