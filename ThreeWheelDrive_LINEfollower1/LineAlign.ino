
// do for n=5 line sensor 
// PID TUNING WILL BE DIFFERENT OR SAME IN CASE IF PS2 INPUT IS 0 IN ALIGN NORMAL. SEE THE END OF THE CODE IN CALC RPM FUNCTION 3 CASES :
//WITH OUTPUTAS SPEED OR WITH LEFTANALOG DISTANCE SPEED. what will be the pid constants for these? same or different?????????????????????????????????   

// Check awhen align flag is to be set AND WHEN TO BE CLEARED

// see in the loop for the cases
// set range for pid output

void initLineAlign()   //mode=1 dark   mode=2  light
{   
    pAlignmentSensor->n=5;
    pAlignmentSensor->Previousvalue=1;
    for(int i=0;i<5;i++){
      pinMode(a[i],INPUT);
    }
    pinMode(lsscalPin,OUTPUT);
    digitalWrite(lsscalPin,HIGH);
}

void calibrateLSS(int Mode){
  
    digitalWrite(lsscalPin,LOW);
    delay(10);
    digitalWrite(lsscalPin,HIGH);
    
    for(int i=0;i<Mode;i++){
      delay(1500);
      digitalWrite(lsscalPin,LOW);
      delay(10);
      digitalWrite(lsscalPin,HIGH);
    }     
  
}


void lineAlign()
{
    pAlignmentSensor->sum=0;
    pAlignmentSensor->count=0;
    pAlignmentSensor->outputAS=0;
  

    for(int i=0;i<5;++i)
    { 
      if(!(digitalRead(a[i])))
      {
        pAlignmentSensor->count++;
        pAlignmentSensor->sum+=10*i;
      } 
    }
  
    if(pAlignmentSensor->count==0)
    pAlignmentSensor->val=(float)( pAlignmentSensor->Previousvalue)*255/abs( pAlignmentSensor->Previousvalue); 
    else
    pAlignmentSensor->val=(pAlignmentSensor->sum)/(pAlignmentSensor->count);


  
    pAlignmentSensor->val=pAlignmentSensor->val-20;
    int alignAngle;
    pAlignmentSensor->outputAS=LSScontrol(40, pAlignmentSensor->val,pAligngain);
    
    if(pAlignmentSensor->outputAS>0)
    alignAngle=0;//Go Left
  
    if(pAlignmentSensor->outputAS<0)
    alignAngle=180;//Go right
  
    //Serial.println(alignAngle);
    calcRPM(0,alignAngle,abs(pAlignmentSensor->outputAS)*rpmmax,pwheel);
    
    if(pAlignmentSensor->outputAS<=abs(0.2))
    lineFollowFlag=1;
    
     Serial.println("Value: "+ String( pAlignmentSensor->val)+ " RPM"+ String(rpmmax)+ "PID:" + String( pAlignmentSensor->outputAS));
     //Serial.println("**************************************");
}


float LSScontrol(int limit,float value,gain *pgain){
    if(abs(value)>20){
    value=(float)( pAlignmentSensor->Previousvalue)*limit/abs( pAlignmentSensor->Previousvalue); 
    }
    float Lineerror=value;
    pAlignmentSensor->Previousvalue=value;
    float control=PID(Lineerror,pgain);   

/////////////////////////chaNGED FROM PID(Lineerror,pAligngain)  to   PID  PID(Lineerror,pgain)
    control/=20;
    return control;

}




/////////////////////////////////////////////////////////////////////////////////////////////////
//for Line Following 
void AlignNormal(){

    pAlignmentSensor->sum=0;
    pAlignmentSensor->count=0;
    pAlignmentSensor->outputAS=0;
    
    for(int i=0;i<5;++i)
    { 
      if(!(digitalRead(a[i])))
      {
       pAlignmentSensor->count++;
       pAlignmentSensor->sum+=10*i;
      } 
    }
  
    if(pAlignmentSensor->count==0)
      pAlignmentSensor->val=(float)( pAlignmentSensor->Previousvalue)*255/abs( pAlignmentSensor->Previousvalue); 
    else
      pAlignmentSensor->val=(pAlignmentSensor->sum)/(pAlignmentSensor->count);
  
      pAlignmentSensor->val=pAlignmentSensor->val-25;
      pAlignmentSensor->outputAS=LSScontrol(40, pAlignmentSensor->val,pAligngainNormal);
  
    
      int alignNormalangle; //GET PS2 ANGLE
      alignNormalangle=atan2(Rpm,(pAlignmentSensor->outputAS)*rpmmax)*(180/3.14);  
      
      /*if(alignNormalangle>0){
        calcRPM(0,alignNormalangle,abs(rpm),pwheel); 
      }
  
      if(alignNormalangle<0){
        alignNormalangle=180-abs(alignNormalangle);
        calcRPM(0,alignNormalangle,abs(rpm),pwheel);
      }*/
      alignNormalangle=(alignNormalangle+360)%360;
      if(alignNormalangle!=0){
        calcRPM(0,alignNormalangle,abs(Rpm),pwheel); 
       }
       
      if(alignNormalangle==0){
        
        if(pAlignmentSensor->outputAS)
        alignNormalangle=0;
    
        else
        alignNormalangle=180;   
    
        calcRPM(0,alignNormalangle,abs(pAlignmentSensor->outputAS)*rpmmax,pwheel);
      }  
      Serial.println("angle"+String(alignNormalangle));
}




