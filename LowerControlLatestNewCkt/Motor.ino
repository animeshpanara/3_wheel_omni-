void initMotor(MOTOR *motor){
  pinMode(motor->pin1,OUTPUT);
  pinMode(motor->pin2,OUTPUT);
  pinMode(motor->pwmpin,OUTPUT);
}
float drivewheel(float output,int maxval,MOTOR *motor){
  if(output>maxval){
    output=maxval;
  }
  if(output<-maxval){
    output=-maxval;
  }
  if(output>0)
  {
    digitalWrite(motor->pin1,HIGH);
    digitalWrite(motor->pin2,LOW);
  }
  else if(output<0)
  {
    output=-1*output;
    digitalWrite(motor->pin1,LOW);
    digitalWrite(motor->pin2,HIGH);
  }
  else{
    digitalWrite(motor->pin1,HIGH);
    digitalWrite(motor->pin2,HIGH);  
  }
  pwm.pinDuty( motor->pwmpin, output );
    
  //analogWrite(motor->pwmpin,output);
  return output;
}
void drivewheel1(float output,int maxval,MOTOR *motor,gain *pgain){
//  if(output>maxval){
//    output=maxval;
//  }
//  if(output<-maxval){
//    output=-maxval;
//  }
  if(pgain->required>0)
  {
    digitalWrite(motor->pin1,HIGH);
    digitalWrite(motor->pin2,LOW);
  }
  else if(pgain->required<0)
  {
    digitalWrite(motor->pin1,LOW);
    digitalWrite(motor->pin2,HIGH);
  }
  else{
    digitalWrite(motor->pin1,HIGH);
    digitalWrite(motor->pin2,HIGH);  
  }
  //pwm.pinDuty( motor->pwmpin, (int)output );
    
  analogWrite(motor->pwmpin,output);
  //return output;
}

