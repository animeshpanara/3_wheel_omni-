//void initPosEncoder(encoder * encoderA){
//  Serial.begin(9600);
//  pinMode(encoderA->channelA, INPUT);
//  pinMode(encoderA->channelB, INPUT);
//  attachInterrupt(digitalPinToInterrupt(encoderA->channelA), *(encoderA->Tickfunction), RISING);
//}
//
//
//void TicksA(){
//  if(digitalRead(pencoderA->channelB))
//  pencoderA->count+=1;
//  else
//  pencoderA->count-=1;
//}
//
//
//double getDistance(encoder * pencoderA){
//      return ((float)pencoderA->count/pencoderA->ppr)*pencoderA->diameter*pi;
//}
//
//void adjustVertical(float distance, gain * Adjustgain, encoder * encoderA){
//    float currentDistance = getDistance(encoderA);
//    Adjustgain->required = distance;
//    float adjustControl = PID(currentDistance,Adjustgain);
//    if(adjustControl==0)
//    Linearflag ^=1;
//    calcRPM(-OmegaControl, 90, adjustControl, pwheel);
//}
