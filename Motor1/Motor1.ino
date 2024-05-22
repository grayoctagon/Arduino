
int inputDir=A0;
int inputPot=A1;
int motorA=5;
int motorB=6;

long pDirRaw;
long pPot;

long pDir1=0;
long pDir2=0;



void setup() {
  Serial.begin(115200);
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);
}

void loop() {
  readVals();
  applyVals();
  delay(10);
}





void readVals(){
  pDirRaw=analogRead(inputDir);
  pPot=analogRead(inputPot);
  long pDirC=map(pDirRaw,28,993,-511,511);


  pDir1=0;
  pDir2=0;

  int dir=1;
  if(pDirC<0){
    dir=-1;
    pDirC=pDirC*-1;
  }
  if(pDirC<5)pDirC=0;
  //pDirC=map(pDirC,5,260,0,255);



  //speed multiplikator
  float mult=map(pPot,23,1023,0,1000)/50.0;
  if(mult<1)mult=1;

  pDirC=mult*pDirC*.5;

  if(pDirC<0)pDirC=0;
  if(pDirC>255)pDirC=255;



  
  if(dir<0){
    pDir1=pDirC;
  }else{
    pDir2=pDirC;
  }



  Serial.println("f0:0,f1024:1024,f20:20,dirRaw:"+String(pDirRaw)+
      ",dirC:"+String(pDirC)+
      ",dir1:"+String(pDir1)+
      ",dir2:"+String(pDir2)+
      ",pot:"+String(pPot)+
      ",mult:"+String(mult));
}
void applyVals(){
  analogWrite(motorA,pDir1);
  analogWrite(motorB,pDir2);
}


