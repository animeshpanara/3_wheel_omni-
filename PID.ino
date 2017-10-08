

void PIDinit(float kp, float kd, float ki, struct gain * gain) {
    gain->kp = kp;
    gain->kd = kd;
    gain->ki = ki;
  }

  int PID(float req, float current, struct gain * gain) {
    prop = req - current; //WHen car left, right sensors off, take right control positive
    integral += prop;
    derivative = prevprop - prop; ///*********************
    prevprop = prop;

    float control = gain->kp * prop + gain->ki * integral + gain->kd * derivative;

    if (control > maxControl)
      control = maxControl;
    if (control < minControl)
      control = minControl;
    return control;
  }

