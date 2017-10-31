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
  else
  {
    output=-1*output;
    digitalWrite(motor->pin1,LOW);
    digitalWrite(motor->pin2,HIGH);
  }
  
  analogWrite(motor->pwmpin,output);
  return output;
}

