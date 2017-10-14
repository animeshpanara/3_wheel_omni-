void initDriving(){
  pinMode(pinpwma, OUTPUT);
  pinMode(pinpwmb, OUTPUT);
  pinMode(pinpwmc, OUTPUT);
  pinMode(pinaa, OUTPUT);
  pinMode(pinab, OUTPUT);
  pinMode(pinba, OUTPUT);
  pinMode(pinbb, OUTPUT);
  pinMode(pinca, OUTPUT);
  pinMode(pincb, OUTPUT);
}

float setMotion(int omega, int angle, int transvel, wheel * whee) { 
    
}

double degtorad(float degree) {
    return (0.0174533 * degree); //0.0174533 rad is 1 degree
}


void startMotion(wheel ** whee){
  for(int i=0;i<3;i++)
  {
    int pwm = (float)whee[i]->rpm * 255/rpmmax;
    //Serial.println(pwm);
    if(pwm > 0){
        digitalWrite(whee[i]->pina, HIGH);
        digitalWrite(whee[i]->pinb, LOW);
      }
      else if(pwm < 0){
        digitalWrite(whee[i]->pina, LOW);
        digitalWrite(whee[i]->pinb, HIGH);
        //Negative direction
      }
      analogWrite(whee[i]->pinpwm, abs((int)pwm)/3);
  }
}

void brakeWheels(wheel ** whee){
  for(int i=0;i<3;i++){
  digitalWrite(whee[i]->pina,HIGH);
  digitalWrite(whee[i]->pinb,HIGH);
  }
}

void calcRPM(int omega, int angle, int transvel, wheel **whee){
  for(int i=0;i<3;i++){
    whee[i]->trans_rpm = transvel * sin(degtorad(whee[i]->angle - angle)); //RPMtyre=RPM*sin(wheel angle- vel angle)
    whee[i]->ang_rpm = omega * r;
    //float pwm = ((float)(whee->trans_rpm + whee->ang_rpm) / rpmmax) * 255;   
    whee[i]->rpm=whee[i]->trans_rpm+whee[i]->ang_rpm;
  }
  setScale(whee);
}

void setScale(wheel **whee){
  int maxm = abs(whee[0]->rpm);
  for(int i=1;i<3;i++){
    if(abs(whee[i]->rpm)>maxm){
      maxm=abs(whee[i]->rpm);
    }
  }
  if(maxm>=rpmmax){
    for(int i=0;i<3;i++){
      whee[i]->rpm = (float)whee[i]->rpm*rpmmax/maxm;
    }
  }
}

