void initMotor(wheel * whee){
  pinMode(whee->pina,OUTPUT);
  pinMode(whee->pinb,OUTPUT);
  pinMode(whee->pinpwm,OUTPUT);
}

void initDriving(wheel ** whee){
  for(int i = 0; i<3; i++)
  initMotor(whee[i]);
}

float setMotion(int omega, int angle, int transvel, wheel * whee) { 
    
}

double degtorad(float degree) {
    return (0.0174533 * degree); //0.0174533 rad is 1 degree
}
double radtodeg(float rad) {
    return (57.3*rad); //0.0174533 rad is 1 degree
}

void startMotion(wheel ** whee){
  for(int i=0;i<3;i++)
  {
    int pwmval = (float)whee[i]->rpm * 1023.0/(float)rpmmax;
    
    if(pwmval > 0){
        digitalWrite(whee[i]->pina, HIGH);
        digitalWrite(whee[i]->pinb, LOW);
      }
      else if(pwmval < 0){
        digitalWrite(whee[i]->pina, LOW);
        digitalWrite(whee[i]->pinb, HIGH);
        //Negative direction
      }
      else
      {
        digitalWrite(whee[i]->pina, HIGH);
        digitalWrite(whee[i]->pinb, HIGH);
      }
      Timer3.pwm(whee[i]->pinpwm, abs((int)pwmval)/PWMfactor);
  }  
}

void brakeWheels(wheel ** whee){
  for(int i=0;i<3;i++){
  digitalWrite(whee[i]->pina, HIGH);
  digitalWrite(whee[i]->pinb, HIGH);
  }
}

void calcRPM(int omega, int angle, int transvel, wheel **whee){
  int r = 1;
if(passflag==1 && racklift == 1)//fastwithrack
{
 whee[0]->trans_rpm = -0.5584*transvel*1.825*cos(degtorad(angle)); //front wheel
 whee[0]->ang_rpm = omega*r;
 whee[1]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))-0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[1]->ang_rpm = omega*r;
 whee[2]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))+0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[2]->ang_rpm = omega*r;
}
else if(passflag==1 && racklift == 0)//fastwithoutrack
{
 whee[0]->trans_rpm = -0.5584*transvel*1.97*cos(degtorad(angle)); //front wheel
 whee[0]->ang_rpm = omega*r;
 whee[1]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))-0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[1]->ang_rpm = omega*r;
 whee[2]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))+0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[2]->ang_rpm = omega*r;
 
}
else if(passflag==0 && racklift == 0)//slowwithoutrack
{
 whee[0]->trans_rpm = -0.5584*transvel*2.2*cos(degtorad(angle)); //front wheel
 whee[0]->ang_rpm = omega*r;
 whee[1]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))-0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[1]->ang_rpm = omega*r;
 whee[2]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))+0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[2]->ang_rpm = omega*r; 
}
else if(passflag==0 && racklift == 1)//slowwithrack
{
 whee[0]->trans_rpm = -0.5584*transvel*1.98*cos(degtorad(angle)); //front wheel
 whee[0]->ang_rpm = omega*r;
 whee[1]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))-0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[1]->ang_rpm = omega*r;
 whee[2]->trans_rpm = 0.3123*transvel*cos(degtorad(angle))+0.7071*transvel*sin(degtorad(angle));//anticlockwise
 whee[2]->ang_rpm = omega*r;
}

 setScale(whee);
}


void setScale(wheel **whee){
  int maxm = abs(whee[0]->trans_rpm);
  for(int i=1;i<3;i++){
    if(abs(whee[i]->trans_rpm)>maxm){
      maxm=abs(whee[i]->trans_rpm);
    }
  }
  if(maxm!=0)
  {
    for(int i=0;i<3;i++){
      whee[i]->trans_rpm = (float)whee[i]->trans_rpm*rpmmax/maxm;
    }
  }
   for(int i=0;i<3;i++){
    whee[i]->rpm=whee[i]->trans_rpm+whee[i]->ang_rpm;
 }

}

