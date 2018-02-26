#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

float ModuloFloat(float x,int y)
{
  float fractpart=-(int)x+x;
  int integralpart=(int)x;
  int modulo=integralpart%y;
  float Modulo=modulo+fractpart;
  return Modulo;
}

void CompassInit(){
  Wire.begin();
  LIS3MDL::vector<int16_t> running_min = {-57,  -5655,  +2030}, running_max = { +3737,  -3105,  +4672};
//min:    max: 

  if (!mag.init())
  {
    Serial.println("Failed to detect and initialize magnetometer!");
  }
  mag.enableDefault();

}

void GetCompassHeading(int n){
mag.read();
magn[0]=(int)mag.m.x-bias[0];
magn[1]=(int)mag.m.y-bias[1];
magn[2]=(int)mag.m.z-bias[2];
if(n==0)
prevcompassHeading =((ToDeg(atan2(magn[1],magn[0])))+180);
else
{
  compassHeading=((ToDeg(atan2(magn[1],magn[0])))+180);
    if(abs(compassHeading-prevcompassHeading)>180)
  {
      if(prevcompassHeading>350&&prevcompassHeading<360)
      {
        compassHeading+=360; 
      }
      if(prevcompassHeading>0&&prevcompassHeading<10)
      {
        compassHeading-=360;
      }
  }
  
 else
  {
    prevcompassHeading=compassHeading;
  }
 }
  }


void setOffsetCompass(gain *pgain){
  float sum;

  for(int i=0;i<20;i++){
    GetCompassHeading(2);
    sum+=compassHeading;
  }
  compassOffset=sum/20; 
  pgain->required=abs(compassOffset);
}
