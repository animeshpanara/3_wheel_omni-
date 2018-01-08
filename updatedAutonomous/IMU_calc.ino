
void IMUinit(){
  I2C_Init();
  Serial.println("Pololu MinIMU-9 + Arduino AHRS");
  delay(1500);
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  delay(20);
  
}

void GetIMUReading(){
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
      
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;



    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer

    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading
    }

    // Calculations...
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
    }
}
void SetIMUOffset(){
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }

  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  delay(2000);
}
void CalibrateIMU(gain* IMU){
  float avg=0;
    for(int i=0;i<50;i++)
      {
        Serial.println("Calibrating IMU Sensor");
        GetIMUReading(); 
        avg+=(int(ToDeg(yaw)+360)%360);
        delay(5);
      }
      avg=avg/50;
      IMU->required=avg;
      IMU->integralError=0;  
}


