void TransmitRPM(wheel ** whee)
{
  Wire.beginTransmission(8);
        for(int y =0;y<3;y++)
        {
         int temp = ((int)whee[y]->rpm+400);
           for(int r=0;r<3;r++)
        {
          Wire.write(temp%10);
          temp/=10;
        } 
        }
  Wire.endTransmission();
 }
 
void TransmitRPM_STM(wheel ** whee)
{
  int y1=0;
  char str[9];
  Wire.beginTransmission(8);
        for(int y =0;y<3;y++)
        {
         int temp = ((int)whee[y]->rpm+400);
         int x;
         for(int r=0;r<3;r++)
        {
           str[y1]=temp%10+'0';
           y1++;
           temp/=10;
        } 
        }
  Wire.write(str);
  Serial.println(str);      
  Wire.endTransmission();
 }
