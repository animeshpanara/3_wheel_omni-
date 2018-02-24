void initnewMech(void)
{
  pinMode(Motordir1pin,OUTPUT);
  pinMode(Motordir2pin,OUTPUT);
  pinMode(PWMpin,OUTPUT);
  pinMode(reloadLiftpin,OUTPUT);
  pinMode(LimitLpin,INPUT_PULLUP);
  pinMode(LimitRpin,INPUT_PULLUP);
}
int newMech(){
  long int timerIr=0;
  int TempIr=0;
  analogWrite(PWMpin,MechanismRPM);
  delay(1500);
  while(digitalRead(LimitRpin)==HIGH){
  digitalWrite(Motordir1pin,LOW);
  digitalWrite(Motordir2pin,HIGH);   
  }
  digitalWrite(reloadLiftpin,HIGH);
  while(digitalRead(LimitLpin)==HIGH){
  digitalWrite(Motordir1pin,HIGH);
  digitalWrite(Motordir2pin,LOW);   
  }
  digitalWrite(reloadLiftpin,LOW);
  digitalWrite(Motordir1pin,HIGH);
  digitalWrite(Motordir2pin,HIGH);
  delay(500);
  digitalWrite(reloadLiftpin,HIGH);
  delay(100);
  analogWrite(PWMpin,MechanismRPM);
  
  while(digitalRead(LimitRpin)==HIGH){
  digitalWrite(Motordir1pin,LOW);
  digitalWrite(Motordir2pin,HIGH);   
  }
  digitalWrite(reloadLiftpin,LOW);
  digitalWrite(Motordir1pin,HIGH);
  digitalWrite(Motordir2pin,HIGH);
  timerIr=millis();
  while(1){
    if(millis()-timerIr>5000)
      return 0;
    //Serial.println(digitalRead(TriggerPin));  
    if(!digitalRead(TriggerPin))
      TempIr++;
    else
      TempIr=0;
    if(TempIr>100){
      Serial.println("::Throw again::");
      return 1;
    }  
  }
}

