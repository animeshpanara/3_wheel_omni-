//#include <SPI.h>
//int Confidencelevel[5];
//enum {x,l1,l2,r1,r2};
//int xflag=0,l1flag=0,l2flag=0,r1flag=0,r2flag=0;
//int xhold=0,l1hold=0,l2hold=0,r1hold=0,r2hold=0;
//void scalePS2value(){
//      ySquare_leftdistance=(128-ps2x.Analog(PSS_LY))/128.0;
//      xSquare_leftdistance=(-ps2x.Analog(PSS_LX)+128)/128.0;
//      ySquare_rightdistance=(128-ps2x.Analog(PSS_RY))/ 128.0;
//      xSquare_rightdistance=(-ps2x.Analog(PSS_RX)+128)/128.0;
//      if(xSquare_leftdistance==0 && ySquare_leftdistance==0){
//      xCircle_leftdistance=0;
//      yCircle_leftdistance=0; 
//      }
//      else{
//      xCircle_leftdistance= xSquare_leftdistance *sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2)-(pow(xSquare_leftdistance,2)*pow(ySquare_leftdistance,2)))/sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2));
//      yCircle_leftdistance= ySquare_leftdistance *sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2)-(pow(xSquare_leftdistance,2)*pow(ySquare_leftdistance,2)))/sqrt(pow(xSquare_leftdistance,2)+pow(ySquare_leftdistance,2));
//      }
//      if(xSquare_rightdistance==0 && ySquare_rightdistance==0){
//      xCircle_rightdistance=0;
//      yCircle_rightdistance=0; 
//      }
//      else{
//      xCircle_rightdistance= xSquare_rightdistance *sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2)-(pow(xSquare_rightdistance,2)*pow(ySquare_rightdistance,2)))/sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2));
//      yCircle_rightdistance= ySquare_rightdistance *sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2)-(pow(xSquare_rightdistance,2)*pow(ySquare_rightdistance,2)))/sqrt(pow(xSquare_rightdistance,2)+pow(ySquare_rightdistance,2));
//      }
//      
//      LeftAnalogDistance=sqrt((xCircle_leftdistance)*(xCircle_leftdistance)+(yCircle_leftdistance)*(yCircle_leftdistance));
//      LeftAnalogTheta=atan2(yCircle_leftdistance,xCircle_leftdistance)*(180.0/3.14);
//      RightAnalogDistance=sqrt((xCircle_rightdistance)*(xCircle_rightdistance)+(yCircle_rightdistance)*(yCircle_rightdistance));
//      RightAnalogTheta=atan2(yCircle_rightdistance,xCircle_rightdistance)*(180.0/3.14);
//
////      Serial.println("LeftAnalogDistance: " + String(LeftAnalogDistance)+"       LeftAnalogTheta:"+String(LeftAnalogTheta));
////      Serial.println("RightAnalogDistance: " + String(RightAnalogDistance)+"       RightAnalogTheta:"+String(RightAnalogTheta));
//      
//  }
//
//
//void initPS2()
//{
//   error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
//  if(error == 0)
//    Serial.print("Found Controller, configured successful ");
//  else if(error == 1)
//    Serial.println("No controller found"); 
//  
//  type = ps2x.readType(); 
//  
//  switch(type) {
//    case 0:
//      Serial.print("Unknown Controller type found ");
//      break;
//    case 1:
//      Serial.print("DualShock Controller found ");
//      break;
//    case 2: 
//      Serial.print("GuitarHero Controller found ");
//      break;
//    case 3:
//      Serial.print("Wireless Sony DualShock Controller found ");
//      break;
//  }
//}
//
//
//
//
//
//void getPS2value()
//{ 
//  ps2x.read_gamepad(false,vibrate);          //read controller  
//}
//
//void rightPressed(){  
//  Wire.beginTransmission(8);
//  Wire.write('r');
//  Wire.endTransmission();
//}
//void rightReleased(){
//}
//void leftPressed(){
//  Wire.beginTransmission(8);
//  Wire.write('l');
//  Wire.endTransmission();
//} 
//void leftReleased(){
//  
//}
//void upPressed(){
//  Wire.beginTransmission(8);
//  Wire.write('s');
//  Wire.endTransmission();
//}
//void upReleased(){
//}
//void downPressed(){
//}
//void downReleased(){
//}
//void selPressed(){
//}
//void selReleased(){
//}
//void startPressed(){
//}
//void startReleased(){  
//}
//void trianglePressed(){
//    //CalibrateIMU(pIMUgain);
//}
//void triangleReleased(){
//}
//void circleReleased(){
//}
// void circlePressed(){
//}
//void crossPressed(){
//    brakeWheels(pwheel); //Stop
//}
//void crossReleased(){
//}
//void squarePressed(){
//  Stopflag^=1;
//}  
//void squareReleased(){
//}
//void l1Pressed(){
//}
//void l1Released(){
//  
//}  
//void r1Released(){
//}
//void r1Pressed(){
//}  
//void l2Released(){
//
//}
//void l2Pressed(){
//  Wire.beginTransmission(8);
//  Wire.write('x');
//  Wire.endTransmission();
//
//}
//void r2Released(){
//}
//void r2Pressed(){
//  Wire.beginTransmission(8);
//  Wire.write('w');
//  Wire.endTransmission();
//
//
//}
//void crossandr1pressed(){
//  //Go right
//semiAutonomousFlag^=1;
////LineAlign();
//if(pAlignmentSensor->val=255)
//pAlignmentSensor->Previousvalue=-255;
//}
//void crossandl1pressed(){
////Go left
//semiAutonomousFlag^=1;
//if(pAlignmentSensor->val=255)
//pAlignmentSensor->Previousvalue=255;
//
//}
//void crossandr2pressed(){
//
//}
//
//void crossandl2pressed(){
//}
//
//void PS2executePressed(){
//       if((ps2x.ButtonPressed(PSB_CROSS) || xflag==1)&&(r1hold!=1 && r2hold!=1 && l1hold!=1 && l2hold!=1)){
//        xflag=1;
//        xhold=1;
//        Confidencelevel[x]++;
//        if(Confidencelevel[x]>7){
//        crossPressed(); 
//        Serial.println("X");
//        Confidencelevel[x]=0;
//        xflag=0;
//        xhold=0;
//        return;
//       }
//       else if(ps2x.ButtonPressed(PSB_L1)){
//        crossandl1pressed();
//        Serial.println("XL1");
//        Confidencelevel[x]=0;
//        xflag=0;
//        xhold=0;
//        return;
//       }
//       else if(ps2x.ButtonPressed(PSB_R1)){
//        crossandr1pressed();
//        Serial.println("XR1");
//        Confidencelevel[x]=0;
//        xflag=0;
//        xhold=0;
//        return; 
//       }
//       else if(ps2x.ButtonPressed(PSB_L2)){
//        crossandl2pressed();
//        Serial.println("XL2");
//        Confidencelevel[x]=0;
//        xflag=0;
//        xhold=0;
//        return;
//       }
//       else if(ps2x.ButtonPressed(PSB_R2)){
//        crossandr2pressed();
//        Serial.println("XR2");
//        Confidencelevel[x]=0;
//        xflag=0;
//        xhold=0;
//        return;
//       }
//    }
//   if((ps2x.ButtonPressed(PSB_R1)||r1flag==1)&&xhold!=1)
//   {
//        r1hold=1;
//        r1flag=1;
//        Confidencelevel[r1]++;
//        if(Confidencelevel[r1]>9){
//        r1Pressed(); 
//        Serial.println("R1");
//        Confidencelevel[r1]=0;
//        r1flag=0;
//        r1hold=0;
//        return;
//       }
//       else if(ps2x.ButtonPressed(PSB_CROSS)){
//        crossandr1pressed();
//        Serial.println("R1X");
//        Confidencelevel[r1]=0;
//        r1flag=0;
//        r1hold=0;
//        return;
//       } 
//   }
//   if((ps2x.ButtonPressed(PSB_R2)||r2flag==1)&&xhold!=1)
//   {
//        r2hold=1;
//        r2flag=1;
//        Confidencelevel[r2]++;
//        if(Confidencelevel[r2]>9){
//        r2Pressed(); 
//        Serial.println("R2");
//        Confidencelevel[r2]=0;
//        r2flag=0;
//        r2hold=0;
//        return;
//       }
//       else if(ps2x.ButtonPressed(PSB_CROSS)){
//       crossandr2pressed();
//        Serial.println("R2X");
//        Confidencelevel[r2]=0;
//        r2flag=0;
//        r2hold=0;
//        return;
//       } 
//   }
//   if(ps2x.ButtonPressed(PSB_L2)||l2flag==1)
//   {
//        l2hold=1;   
//        l2flag=1;
//        Confidencelevel[l2]++;
//        if(Confidencelevel[l2]>9){
//        l2Pressed(); 
//        Serial.println("L2");
//        Confidencelevel[l2]=0;
//        l2flag=0;
//        l2hold=0;
//        return;
//       }
//       else if(ps2x.ButtonPressed(PSB_CROSS)){
//        crossandl2pressed();
//        Serial.println("L2X");
//        Confidencelevel[l2]=0;
//        l2flag=0;
//        l2hold=0;
//        return;
//       } 
//   }
//   if(ps2x.ButtonPressed(PSB_L1)||l1flag==1)
//   {
//        l1hold=1;   
//        l1flag=1;
//        Confidencelevel[l1]++;
//        if(Confidencelevel[l1]>9){
//        l1Pressed(); 
//        Serial.println("L1");
//        Confidencelevel[l1]=0;
//        l1flag=0;
//        l1hold=0;
//        return;
//       }
//       else if(ps2x.ButtonPressed(PSB_CROSS)){
//        crossandl1pressed();
//        Serial.println("L1X");
//        Confidencelevel[l1]=0;
//        l1flag=0;
//        l1hold=0;
//        return;
//       } 
//   }
//   /*if(ps2x.ButtonReleased(PSB_CROSS) && ps2x.ButtonReleased(PSB_L1))
//   {
//      //xcrossandl1pressed();
//      Serial.println("XL1R"); 
//   }*/
//    if (ps2x.ButtonPressed(PSB_PAD_UP))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//    { 
//          upPressed();
//          Serial.println('s');
//    }
////    if(ps2x.ButtonReleased(PSB_PAD_UP))
////    {
////        upReleased();
////      Serial.println("UR");
////    }
//    
//    if(ps2x.ButtonPressed(PSB_PAD_RIGHT))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//     {
//            rightPressed(); 
//           Serial.println('r');
//     }
////    if(ps2x.ButtonReleased(PSB_PAD_RIGHT))
////     {
////          Serial.println("RR");
////     
////          rightReleased();
////     }
////    
//    if(ps2x.ButtonPressed(PSB_PAD_LEFT))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//     {
//           leftPressed(); 
//           Serial.println('l');
//     }
////    if(ps2x.ButtonReleased(PSB_PAD_LEFT))
////     {
////         Serial.println("LR");
////          
////         leftReleased();
////     }
//    
//     if(ps2x.ButtonPressed(PSB_PAD_DOWN)) //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//     {
//          downPressed();
//          Serial.println('D');
//     }
//     
////     if(ps2x.ButtonReleased(PSB_PAD_DOWN)) //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
////     {
////         Serial.println("DR");
////     
////        downReleased();
////     }
//    
//     if(ps2x.ButtonPressed(PSB_SELECT))
//     {  
//          selPressed();
//          Serial.println('E');
//    }
////    if(ps2x.ButtonReleased(PSB_SELECT))
////    {
////          selReleased();
////         Serial.println("ER");
////     
////    }
//    
//    if(ps2x.ButtonPressed(PSB_START))
//    {  
//            startPressed();
//            Serial.println('S');
//     }
//    
////    if(ps2x.ButtonReleased(PSB_START)) 
////    {
////       Serial.println("SR");
////     
////        startReleased();
////    }
////    
//    if(ps2x.ButtonPressed(PSB_TRIANGLE))
//    {   
//           trianglePressed();
//            Serial.println('T');     
//    }  
//    
////    if(ps2x.ButtonReleased(PSB_TRIANGLE))
////    {
////       Serial.println("TR");
////     
////        triangleReleased();
////    }
//    
//    if(ps2x.ButtonPressed(PSB_CIRCLE))
//     {
//            circlePressed();
//            Serial.println('C'); 
//     }
//     
////     if(ps2x.ButtonReleased(PSB_CIRCLE))
////     {
////       Serial.println("CR");
////     
////       circleReleased();
////     }
//   
//     
////     if(ps2x.ButtonReleased(PSB_CROSS))
////     {
////       Serial.println("XR");
////        flag=0;
////        crossReleased();
////      }
//     
//        
//    
//     if(ps2x.ButtonPressed(PSB_SQUARE))
//     {
//          squarePressed();
//           Serial.println('S');     
//     }
//
////     if(ps2x.ButtonReleased(PSB_SQUARE))
////     {
////       Serial.println("SR");
////     
////        squareReleased();
////     }  
//  
//   
////     if(ps2x.ButtonReleased(PSB_L1))
////     {
////       Serial.println("FR");
////     
////        l1Released();
////      }
//     
//     /*if(ps2x.ButtonPressed(PSB_L2))
//     {    
//          l2Pressed();
//          Serial.println('x');
//    }*/
//
////    if(ps2x.ButtonReleased(PSB_L2))
////    {
////        l2Released();
////         Serial.println("GR");
////     
////    }
//// 
//    if(ps2x.ButtonPressed(PSB_R1))
//    {
//          r1Pressed();    
//          Serial.println('H');
//    }
//
////    if(ps2x.ButtonReleased(PSB_R1))
////    {
////         Serial.println("HR");
////         r1Released();
////    }
//    if(ps2x.ButtonPressed(PSB_R2))
//    {  
//          r2Pressed();
//          Serial.println('w');
//    }
//
////    if(ps2x.ButtonReleased(PSB_R2))
////    {
////       Serial.println("IR");
////        r2Released();
////    }
//   
//}
//

