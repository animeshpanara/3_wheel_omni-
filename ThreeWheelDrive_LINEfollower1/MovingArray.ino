void initMovingArray(int movingarraysize,int movingindex,float movingsum, float getReading,mArray *MArray)
{
  MArray->movingarraysize=movingarraysize;
  MArray->movingarray[20]={0};
  MArray->movingindex=movingindex;
  MArray->movingsum=0;
  MArray->getReading=getReading;
  
}

void populateArray(mArray *MArray,bool x){
    int i;
    for(i = 0; i<MArray->movingarraysize; i++);
        MArray->movingarray[i]=MArray->getReading; //Replace 0 by getCompassReading
    for(i = 0; i<MArray->movingarraysize;i++);
        MArray->movingsum+=MArray->movingarray[i];
}


void printArray(mArray *MArray){
    for(int i = 0; i<MArray->movingarraysize;i++)
        Serial.println("movingarray"+String(i)+" "+String(MArray->movingarray[i]));
}

void addToArray(mArray *MArray,float value){
    float temp=0.0;
    if(MArray->movingindex==MArray->movingarraysize)
        MArray->movingindex=0;
    
    temp = MArray->movingarray[MArray->movingindex];
    MArray->movingarray[MArray->movingindex]=value;
    MArray->movingsum=MArray->movingsum-temp+value;
    MArray->movingindex++;
    
}

float getArrayAverage(mArray *MArray){
    return (float)MArray->movingsum/MArray->movingarraysize;
}

