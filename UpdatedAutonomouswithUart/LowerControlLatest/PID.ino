 

void PIDinit(float kp, float kd, float ki,float required,float minControl, float maxControl, struct gain * gain) {
    gain->kp = kp;
    gain->kd = kd;
    gain->ki = ki;
    gain->required=required;
    gain->Prevrequired=0;
    gain->minControl = minControl;
    gain->maxControl = maxControl;
    gain->error=0;
    gain->previousError=1;
    gain->derivativeError=0;
    gain->integralError=0;
  }

float PID(float current, struct gain * gain) {
    gain->error = abs((int)gain->required) - current; 
    gain->derivativeError = gain->error - gain->previousError; 
    gain->previousError = gain->error;

    float control = gain->kp * gain->error + gain->ki * gain->integralError + gain->kd * gain->derivativeError;

    if (control > gain->maxControl)
      control = gain->maxControl;
    else if (control < gain->minControl)
      control = gain->minControl;
    else 
      gain->integralError += gain->error;
    return control;
}
int ScaleOp(int op){
  if(op>255)
  op=255;
  else if(op<-255)
  op=-255;
  return op;
}
bool PIDstatus(struct gain * gain){
  if(gain->Prevrequired>0&&gain->required>0)
  return 1;
  else if(gain->Prevrequired<0&&gain->required<0)
  return 1;
  else{
  gain->Prevrequired=gain->required;
  gain->integralError=0;
  
  return 0;
  }
}

