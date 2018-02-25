void PIDinit(float kp, float kd, float ki,float required,float minControl, float maxControl,struct gain * gain) {
    gain->kp = kp;
    gain->kd = kd;
    gain->ki = ki;
    gain->required=required;
    gain->minControl = minControl;
    gain->maxControl = maxControl;
    gain->error=0;
    gain->previousError=1;
    gain->derivativeError=0;
    gain->integralError=0;
}

int PID(float current, struct gain * gain) {
    gain->error = gain->required - current; 
    gain->derivativeError = gain->error - gain->previousError; 
    gain->previousError = gain->error;

    float control = gain->kp * gain->error + gain->ki* gain->integralError + gain->kd * gain->derivativeError;
    
    if (control > gain->maxControl)
      control = gain->maxControl;
    else if (control < gain->minControl)
      control = gain->minControl;
    else 
       gain->integralError += gain->error;   
    
    return control;
}

