#include <Adafruit_MCP4725.h>

  #include <EEPROM.h>
  //#include <Wire.h>
  //#include <Adafruit_MCP4725.h>
  #include <LiquidCrystal.h>
  Adafruit_MCP4725 dac;
  const int rs = 3, en = 4, d4 =8 , d5 =9 , d6 =10, d7 = 11;
  LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
  int released_zone=0;
  int pressedvoltage_increase=0;
  int releasedvoltage_increase=0;
  int pressedvoltage_decrease=0;
  int releasedvoltage_decrease=0;
  int pressed_zone=0;  
  int pressed=0;
  int zone_pin=2;
  int voltagepin_increase=5;
  int voltagepin_decrease=6;
  int ThrowPin = 7;
  int ThrowPin1 = 12;
  int ZoneSelect=-1;
  float VZ1=0,VZ2=0,VZ3=0;
  int BZ1=680,BZ2=850,BZ3=1100;
  int address = 0;
  byte value;
  int prev=0;
 // int address=0;
  void setup() {
    Serial.begin(9600);
    dac.begin(0x60);
    lcd.begin(16, 2);
    //EEPROM.put(address,BZ1);
    //EEPROM.put(address+2*sizeof(int),BZ3);
    //EEPROM.put(address+sizeof(int),BZ2);
    pinMode(zone_pin,INPUT_PULLUP);
    pinMode(voltagepin_increase,INPUT_PULLUP);// put your setup code here, to run once:
    pinMode(voltagepin_decrease,INPUT_PULLUP);
    pinMode(ThrowPin1,INPUT_PULLUP);
    pinMode(ThrowPin,OUTPUT);
    EEPROM.get(address,BZ1);
    EEPROM.get(address+sizeof(int),BZ2);
    EEPROM.get(address+2*sizeof(int),BZ3);
    lcd.print("STARTED");
    //Serial.println('a');
  }
  void loop() {
    //Serial.println('a');
    //value = EEPROM.read(address);
    //Serial.print(address);
    //Serial.println('a');
    //Serial.print(value, DEC);
    if(Serial.available()>0)
     {
      if(Serial.read()=='t'){
      digitalWrite(ThrowPin,HIGH);
      delay(1200);
      digitalWrite (ThrowPin,LOW); 
      lcd.print("Throw");
      delay(1000);
      lcd.setCursor(0,1); 
      lcd.print("                      ");
   }
     else
      digitalWrite(ThrowPin,LOW); 
     }
     
   
    if(digitalRead(zone_pin)==0)
    {
      pressed_zone++;
      if(pressed_zone>65)
      {
        if(pressed==0)
        {
        pressed_zone=0;
        ZoneSelect=(ZoneSelect+1)%3;
//          if(ZoneSelect<3)
//            ZoneSelect++;
//            else
//            ZoneSelect=1;
//          //else
            //ZoneSelect++;
         }
      pressed=1;
      }
    }
    
    if(digitalRead(zone_pin)==1)
    {
      
      released_zone++;
      if(released_zone>65)
      { 
        released_zone=0;
        pressed=0;   // button got released  
      }
    }
    if(digitalRead(ThrowPin1)==0)
    {
      pressedvoltage_increase++;
      
      if(pressedvoltage_increase>65)
      {
        pressedvoltage_increase=0;  
        digitalWrite(ThrowPin,HIGH);
        delay(1200);
        digitalWrite (ThrowPin,LOW);
        lcd.print("Throw");
        delay(1000);
        lcd.setCursor(0,1); 
        lcd.print("                      ");
     
      }
    }
    if(digitalRead(ThrowPin1)==1)
    {
      
      releasedvoltage_increase++;
      if(releasedvoltage_increase>65)
      { 
        releasedvoltage_increase=0;
        // button got released  
      }
    }    

    
    
    if(digitalRead(voltagepin_increase)==0)
    {
      pressedvoltage_increase++;
      
      if(pressedvoltage_increase>65)
      {
        pressedvoltage_increase=0;
          lcd.setCursor(0,1); 
          lcd.print("                 ");
   
          if(ZoneSelect==0)
          {
            BZ1=BZ1+5;
            EEPROM.put(address,BZ1);
          
            //BZ1 = (VZ1*4095)/5;
          }
          if(ZoneSelect==1)
          {
            BZ2=BZ2+5;
            EEPROM.put(address+sizeof(int),BZ2);
          
            //966
            //BZ2 = (VZ2*4095)/5;
          }
          
          if(ZoneSelect==2)
          {
            BZ3=BZ3+5;
            EEPROM.put(address+2*sizeof(int),BZ3);
          
            //1380
            //BZ3 = (VZ3*4095)/5;
          }
        
      }
    }
    
    
    if(digitalRead(voltagepin_increase)==1)
    {
      
      releasedvoltage_increase++;
      if(releasedvoltage_increase>65)
      { 
        releasedvoltage_increase=0;
          // button got released  
      }
    }    


    if(digitalRead(voltagepin_decrease)==0)
    {
      pressedvoltage_decrease++;
      
      if(pressedvoltage_decrease>65)
      {
        pressedvoltage_decrease=0;
          lcd.setCursor(0,1); 
          lcd.print("                 ");
   
          if(ZoneSelect==0)
          {  
            BZ1=BZ1-5;
            EEPROM.put(address,BZ1);
          }
          if(ZoneSelect==1)
          {
            BZ2=BZ2-5;
            EEPROM.put(address+sizeof(int),BZ2);
          }
          if(ZoneSelect==2)
          {
            BZ3=BZ3-5;
            EEPROM.put(address+2*sizeof(int),BZ3);
            //BZ3 = (VZ3*4095)/5;
          }
      }
    }
    
    if(digitalRead(voltagepin_decrease)==1)
    {
      
      releasedvoltage_decrease++;
      if(releasedvoltage_decrease>65)
      { 
        releasedvoltage_decrease=0;
          // button got released  
      }
    }    
    lcd.setCursor(0,1);
    //lcd.print("loop started");
    if(ZoneSelect==0)
    {
      lcd.print("TZ1:");
      dac.setVoltage(BZ1, false);
      lcd.setCursor(4,1);
      lcd.print(BZ1);
    }
    
    if(ZoneSelect==1)
   {
     lcd.print("TZ2:");
     dac.setVoltage(BZ2, false);
     lcd.setCursor(4,1);
     lcd.print(BZ2);
   }
   if(ZoneSelect==2)
   {
      lcd.print("TZ3:");
      dac.setVoltage(BZ3, false);
      lcd.setCursor(4,1);
      lcd.print(BZ3);
   }// put your main code here, to run repeatedly:
   if(prev!=ZoneSelect)
   {
    lcd.setCursor(0,1); 
    lcd.print("                 ");
   } 
   prev=ZoneSelect;  
}
//released is very wrong
