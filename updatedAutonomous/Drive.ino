void initMotor(wheel * whee){
  pinMode(whee->pina,OUTPUT);
  pinMode(whee->pinb,OUTPUT);
  pinMode(whee->pinpwm,OUTPUT);
}

void initDriving(wheel ** whee){
  for(int i = 0; i<3; i++)
  initMotor(whee[i]);
}

void initThrowing(){
  pinMode(ThrowPin,OUTPUT);
  digitalWrite(ThrowPin,HIGH);
  pinMode(DAC_PinTZ2,OUTPUT);
  digitalWrite(DAC_PinTZ2,HIGH);
  pinMode(DAC_PinTZ3,OUTPUT);
  digitalWrite(DAC_PinTZ3,HIGH);
}

double degtorad(float degree) {
    return (0.0174533 * degree); //0.0174533 rad is 1 degree
}
double radtodeg(float rad) {
    return (57.3*rad);
}

void startMotion(wheel ** whee){
  for(int i=0;i<3;i++)
  {
    int pwm = (float)whee[i]->rpm * 255/rpmmax;
    if(pwm > 0){
        digitalWrite(whee[i]->pina, HIGH);
        digitalWrite(whee[i]->pinb, LOW);
      }
      else if(pwm < 0){
        digitalWrite(whee[i]->pina, LOW);
        digitalWrite(whee[i]->pinb, HIGH);
        //Negative direction
      }
      else
      {
        digitalWrite(whee[i]->pina, HIGH);
        digitalWrite(whee[i]->pinb, HIGH);
      }
      analogWrite(whee[i]->pinpwm, abs((int)pwm));
  }
}

void brakeWheels(wheel ** whee){
  for(int i=0;i<3;i++){
  digitalWrite(whee[i]->pina,HIGH);
  digitalWrite(whee[i]->pinb,HIGH);
  }
}

void calcRPM(int omega, int angle, int transvel, wheel **whee){
  int r = 1;
  for(int i=0;i<3;i++){
    whee[i]->trans_rpm = transvel * sin(degtorad(whee[i]->angle - angle)); //RPMtyre=RPM*sin(wheel angle- vel angle)
    whee[i]->ang_rpm = omega * r;
    whee[i]->prev_rpm = whee[i]->rpm;
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

