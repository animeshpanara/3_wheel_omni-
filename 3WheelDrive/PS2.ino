#include <SPI.h>

#define manual 1



void SPI_send_array(char *a, char cmd_length)
{  
  int spi_send_array_i;
  for(spi_send_array_i=0; spi_send_array_i<cmd_length; spi_send_array_i++)
  {
    data_array[spi_send_array_i]= SPI.transfer(*a);
    a++;
  }
}

void initps2(){
  pinMode(slave,OUTPUT);
  SPI.begin();
  char  PS2_CONFIGMODE[5]= {0x01, 0x43, 0x00, 0x01, 0x00};
  char  PS2_ANALOGMODE[9]= {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
  char  PS2_SETUPMOTOR[9]= {0x01, 0x4D, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff};
  char  PS2_EXITCONFIG[9]= {0x01, 0x43, 0x00, 0x00, 0x5a, 0x5a, 0x5a, 0x5a, 0x5a};
  digitalWrite(slave,LOW);
  SPI_send_array(PS2_CONFIGMODE,5);   
  digitalWrite(slave,HIGH); // Driving Attention Low
  delay(1);
  delay(1);
  digitalWrite(slave,LOW);
  SPI_send_array(PS2_ANALOGMODE,9);   
  digitalWrite(slave,HIGH); 
  delay(1);
  delay(1);
  digitalWrite(slave,LOW);
  SPI_send_array(PS2_SETUPMOTOR,9);   
  digitalWrite(slave,HIGH);
  delay(1);
  delay(1);
  digitalWrite(slave,LOW);
  SPI_send_array(PS2_EXITCONFIG,9);
  digitalWrite(slave,HIGH);
  delay(1);
  delay(1);
  volatile char ps2_init_array[9]= {0x01, 0x42, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
  volatile int ps2_init_i;
  for(ps2_init_i=0; ps2_init_i<10; ps2_init_i++)
  {
    PS2_POLLBUTTON[ps2_init_i] = ps2_init_array[ps2_init_i];
  }
}
void rightPressed(){
       calcRPM(rpmmax,w1,r1*rpmmax,wheelp);
       Serial.println('R');
       buttonmove=true;
       calibratenext=true;
    
}
void rightReleased(){
}
void leftPressed(){
        calcRPM(-rpmmax,w1,r1*rpmmax,wheelp);
        buttonmove=true;
        calibratenext=true;
}
void leftReleased(){
}
void upPressed(){
}
void upReleased(){
}
void downPressed(){
}
void downReleased(){
}
void selPressed(){
}
void selReleased(){
}
void startPressed(){
  flag^=1;
}
void startReleased(){  
}
void trianglePressed(){
    CalibrateIMU(pIMUgain);
}
void triangleReleased(){
}
void circleReleased(){
}
void circlePressed(){
}
void crossPressed(){
    brakeWheels(wheelp); //Stop
}
void crossReleased(){
}
void squarePressed(){
  flag^=1;
}  
void squareReleased(){
}
void l1Pressed(){
}
void l1Released(){
}  
void r1Released(){
}
void r1Pressed(){
}  
void l2Released(){
}
void l2Pressed(){
}
void r2Released(){
}
void r2Pressed(){
}


int isPressed(uint8_t dataByte, uint8_t dataBit)
{
    return ((dataByte & (1 << dataBit)) ? 1 : 0);
}

void GetPS2value(){
    SPI.beginTransaction(settingA);
    scan_PS2();
    delay(2);
  
  x=~data_array[3];
  y=~data_array[4];
  rv=~data_array[6];
  lh=~data_array[7];
  lv=~data_array[8];
  rh=~data_array[9];
  r1=sqrt(abs(((127-lh)*(127-lh))/2+((lv-127)*(lv-127))/2));//
  w1=atan2((lv-127),(127-lh))*180/3.1415926;
  r2=sqrt(((127-rh)*(127-rh))/2+((rv-127)*(rv-127))/2);
  w2=atan2((rv-127),(127-rh))*180/3.1415926;
}
/*
X - Stop
R - Right Rotate
L- Left Rotate
Square - Toggle 
Start - Toggle motion
Triangle - Calibrate
*/

void PS2executePressed(){
    
    if (isPressed(x,up))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
    { 
      if(CurrButtonState[up_Button]==released)
        {
          upPressed();
          //flag^=1;
          CurrButtonState[up_Button]=pressed;
          Serial.println('U');
        }
    }
    else
    {
        if(CurrButtonState[up_Button]==pressed)
        upReleased();
        CurrButtonState[up_Button]=released;
     }
    
     if (isPressed(x,right))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
     {
      if(CurrButtonState[right_Button]==released)
        {
           rightPressed(); 
           CurrButtonState[right_Button]=pressed;
           Serial.println('R');
        }
     }
     else
     {
        if(CurrButtonState[right_Button]==pressed)
          rightReleased();
        CurrButtonState[right_Button]=released;
     }
    
       
     if (isPressed(x,left)) //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
     {
        if(CurrButtonState[left_Button]==released)
        {
           leftPressed(); 
           CurrButtonState[left_Button]=pressed;
           Serial.println('L');
        }
     }
     else
     {
        if(CurrButtonState[left_Button]==pressed)
          leftReleased();
        CurrButtonState[left_Button]=released;
     }
    
     if (isPressed(x,down)) //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
     {
      if(CurrButtonState[down_Button]==released)
        {
          downPressed();
          //flag^=1;
          CurrButtonState[down_Button]=pressed;
          Serial.println('D');
        }
     }
     else
     {
        if(CurrButtonState[down_Button]==pressed)
        downReleased();
        CurrButtonState[down_Button]=released;
     }
    
     if (isPressed(x,select))
     {  
        if(CurrButtonState[sel_Button]==released)
        {
          selPressed();
          //flag^=1;
          CurrButtonState[sel_Button]=pressed;
          Serial.println('E');
        }
    }
    else
    {
        if(CurrButtonState[sel_Button]==pressed)
        selReleased();
        CurrButtonState[sel_Button]=released;
    }
    
    if (isPressed(x,start))
    {
          if(CurrButtonState[start_Button]==released)
          {
            startPressed();
            //flag^=1;
            CurrButtonState[start_Button]=pressed;
            Serial.println('S');
          }
    }
    else
    {
        if(CurrButtonState[start_Button]==pressed)
        startReleased();
        CurrButtonState[start_Button]=released;
    }
    
    if (isPressed(y,triangle_up))
    {   
        if(CurrButtonState[triangle_Button]==released)
        {
            trianglePressed();
            //flag^=1;
            CurrButtonState[triangle_Button]=pressed;
            Serial.println('T');     
         }
     }  
     else
     {
        if(CurrButtonState[triangle_Button]==pressed)
        triangleReleased();
        CurrButtonState[triangle_Button]=released;
     }
    
     if (isPressed(y,circle_right))
     {
         if(CurrButtonState[circle_Button]==released)
         {
            circlePressed();
            //flag^=1;
            CurrButtonState[circle_Button]=pressed;
            Serial.println('C');     
          }
     }
     else
     {
        if(CurrButtonState[circle_Button]==pressed)
        circleReleased();
        CurrButtonState[circle_Button]=released;
     }
   
     if (isPressed(y,cross_down))
     {
        if(CurrButtonState[cross_Button]==released)
        {
            crossPressed();
            //flag^=1;
            CurrButtonState[cross_Button]=pressed;
            Serial.println('X');     
            }
     }
     else
     {
        if(CurrButtonState[cross_Button]==pressed)
        crossReleased();
        CurrButtonState[cross_Button]=released;
     }
     
        
    
     if (isPressed(y,square_left))
     {
      if(CurrButtonState[square_Button]==released)
      {
          squarePressed();
          //flag^=1;
          CurrButtonState[square_Button]=pressed;
           Serial.println('S');     
       }
     }
     else
     {
        if(CurrButtonState[square_Button]==pressed)
        squareReleased();
        CurrButtonState[square_Button]=released;
     }
  
     if (isPressed(y,leftFront1))
     {
          if(CurrButtonState[l1_Button]==released)
          {
          l1Pressed();
          //flag^=1;
          CurrButtonState[l1_Button]=pressed;
           Serial.println('F');   
          }
     }
     else
     {
        if(CurrButtonState[l1_Button]==pressed)
        l1Released();
        CurrButtonState[l1_Button]=released;
     }
     
     if (isPressed(y,leftFront2))
     {    
          if(CurrButtonState[l2_Button]==released)
          {
          l2Pressed();
          CurrButtonState[l2_Button]=pressed;
          Serial.println('G');
           }
    }
    else
    {
        if(CurrButtonState[l2_Button]==pressed)
        l2Released();
        CurrButtonState[l2_Button]=released;
    }
 
    if (isPressed(y,rightFront1))
    {
          if(CurrButtonState[r1_Button]==released)
          {
          r1Pressed();
          CurrButtonState[r1_Button]=pressed;
          Serial.println('H');
          }
    }
    else
    {
        if(CurrButtonState[r1_Button]==pressed)
        r1Released();
        CurrButtonState[r1_Button]=released;
    }
    
    if (isPressed(y,rightFront2))
    {
          if(CurrButtonState[r2_Button]==released)
          {
          r2Pressed();
          CurrButtonState[r2_Button]=pressed;
          Serial.println('I');
          }
    }
    else
    {
        if(CurrButtonState[r2_Button]==pressed)
        r2Released();
        CurrButtonState[r2_Button]=released;
    }
    
}

void scan_PS2(){
  digitalWrite(slave,LOW);  
  SPI_send_array(PS2_POLLBUTTON,9);
  digitalWrite(slave,HIGH);
  delay(2); 
}

