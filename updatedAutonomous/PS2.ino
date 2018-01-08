//void scalePS2value(){
//      ySquare_leftdistance=(128-ps2x.Analog(PSS_LY))/128.0;
//      xSquare_leftdistance=(ps2x.Analog(PSS_LX)-128)/128.0;
//      ySquare_rightdistance=(128-ps2x.Analog(PSS_RY))/128.0;
//      xSquare_rightdistance=(ps2x.Analog(PSS_RX)-128)/128.0;
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
//}
//
//
//void initPS2()
//{
//   ControllerError = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
//  type = ps2x.readType(); 
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
//  case 3:
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
//    ps2x.read_gamepad();          //read controller  
//  }
//void getVibratevalue()  { //DualShock Controller
//    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
//    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
//  }
//
//void PS2executePressed(){
//    
//   
//    if (ps2x.ButtonPressed(PSB_PAD_UP))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//    { 
//          upPressed();
//          Serial.println('U');
//    }
//    if(ps2x.ButtonReleased(PSB_PAD_UP))
//    {
//        upReleased();
//      Serial.println("UR");
//    }
//    
//    if(ps2x.ButtonPressed(PSB_PAD_RIGHT))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//     {
//         rightPressed(); 
//           Serial.println('R');
//     }
//    if(ps2x.ButtonReleased(PSB_PAD_RIGHT))
//     {
//          Serial.println("RR");
//     
//          rightReleased();
//     }
//    
//    if(ps2x.ButtonPressed(PSB_PAD_LEFT))  //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//     {
//           leftPressed(); 
//           Serial.println('L');
//     }
//    if(ps2x.ButtonReleased(PSB_PAD_LEFT))
//     {
//         Serial.println("LR");
//          
//         leftReleased();
//     }
//    
//     if(ps2x.ButtonPressed(PSB_PAD_DOWN)) //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//     {
//          downPressed();
//          Serial.println('D');
//     }
//     
//     if(ps2x.ButtonReleased(PSB_PAD_DOWN)) //x refers to the array which will be in terms of 000x 0000 if switch corresponding to up is pressed and up refers to enum
//     {
//         Serial.println("DR");
//     
//        downReleased();
//     }
//    
//     if(ps2x.ButtonPressed(PSB_SELECT))
//     {  
//          selPressed();
//          Serial.println('E');
//    }
//    if(ps2x.ButtonReleased(PSB_SELECT))
//    {
//        selReleased();
//         Serial.println("ER");
//     
//    }
//    
//    if(ps2x.ButtonPressed(PSB_START))
//    {  
//        startPressed();
//            Serial.println('S');
//     }
//    
//    if(ps2x.ButtonReleased(PSB_START)) 
//    {
//       Serial.println("SR");
//     
//        startReleased();
//    }
//    
//    if(ps2x.ButtonPressed(PSB_TRIANGLE))
//    {   
//            trianglePressed();
//            Serial.println('T');     
//    }  
//    
//    if(ps2x.ButtonReleased(PSB_TRIANGLE))
//    {
//       Serial.println("TR");
//     
//        triangleReleased();
//    }
//    
//    if(ps2x.ButtonPressed(PSB_CIRCLE))
//     {
//            circlePressed();
//            Serial.println('C'); 
//     }
//     
//     if(ps2x.ButtonReleased(PSB_CIRCLE))
//     {
//       Serial.println("CR");
//     
//        circleReleased();
//     }
//   
//     if(ps2x.ButtonPressed(PSB_CROSS))
//     {
//            crossPressed();
//            Serial.println('X');     
//     }
//     
//     if(ps2x.ButtonReleased(PSB_CROSS))
//     {
//       Serial.println("XR");
//     
//        crossReleased();
//      }
//     
//        
//    
//     if(ps2x.ButtonPressed(PSB_SQUARE))
//     {
//          squarePressed();
//           Serial.println('S');     
//     }
//
//     if(ps2x.ButtonReleased(PSB_SQUARE))
//     {
//       Serial.println("SR");
//     
//        squareReleased();
//     }  
//  
//     if(ps2x.ButtonPressed(PSB_L1))
//     {
//          l1Pressed();
//           Serial.println('F');   
//     }
//
//     if(ps2x.ButtonReleased(PSB_L1))
//     {
//       Serial.println("FR");
//     
//        l1Released();
//      }
//     
//     if(ps2x.ButtonPressed(PSB_L2))
//     {    
//          l2Pressed();
//          Serial.println('G');
//    }
//
//    if(ps2x.ButtonReleased(PSB_L2))
//    {
//        l2Released();
//         Serial.println("GR");
//     
//    }
// 
//    if(ps2x.ButtonPressed(PSB_R1))
//    {
//          r1Pressed();    
//          Serial.println('H');
//    }
//
//    if(ps2x.ButtonReleased(PSB_R1))
//    {
//         Serial.println("HR");
//         r1Released();
//    }
//    if(ps2x.ButtonPressed(PSB_R2))
//    {
//          r2Pressed();
//          Serial.println('I');
//    }
//
//    if(ps2x.ButtonReleased(PSB_R2))
//    {
//       Serial.println("IR");
//      r2Released();
//    }
//    
//}
//void rightPressed(){
////       calcRPM(rpmmax,w1,distance_leftAnalog*rpmmax,wheelp);
////       Serial.println('R');
////       buttonmove=true;
////       calibratenext=true;
//    
//}
//void rightReleased(){
//}
//void leftPressed(){
////        calcRPM(-rpmmax,w1,distance_leftAnalog*rpmmax,wheelp);
////        buttonmove=true;
////        calibratenext=true;
//}
//void leftReleased(){
//}
//void upPressed(){
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
//  //flag^=1;
//}
//void startReleased(){  
//}
//void trianglePressed(){
//  CalibrateCompass(pCompassgain);
//}
//void triangleReleased(){
//}
//void circleReleased(){
//}
// void circlePressed(){
//  circleFlag^=1;
//}
//void crossPressed(){
//  //  brakeWheels(wheelp); //Stop
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
//}  
//void l2Released(){
//}
//void l2Pressed(){
//}
//void r2Released(){
//}
//void r2Pressed(){
//}
//
//void r1Pressed(){}
//void r1Released(){}
//

