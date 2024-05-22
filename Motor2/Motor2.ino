
#include "AS5600.h"
#include "Wire.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

AS5600 as5600;   //  use default Wire

int inputDir=A0;
int inputPot=A1;
int inputSwitch=8;
int motorA=5;
int motorB=6;

long pDirRaw;
long pPot;

long pDir1=0;
long pDir2=0;

int32_t position;


int32_t topVal=90000;
int32_t target=topVal;
int32_t graceZone=12000;
int32_t haltZone=500;


void setup() {
  Serial.begin(115200);
  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);

  pinMode(inputSwitch, INPUT_PULLUP);


  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  Serial.println(as5600.getAddress());
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  delay(1000);
}
long lastPrint=0;
long runs=0;
int printsPerSecond=30;

void loop() {
  readPosition();
  //readVals();


  /* //auto position
  if(position>=topVal){
    target=0;
  }
  if(position<=0){
    target=topVal;
  }
  */
  
  if(digitalRead(inputSwitch)==HIGH){
    target=topVal;
  }else{
    target=0;
  }



  int speed=0;
  int maxSpeed=255;
  long difference1=diff(0,position);
  long difference2=diff(topVal,position);
  long difference3=diff(target,position);
  long difference=min(difference1,difference2);
  if(difference<graceZone){
    maxSpeed=maxSpeed*(difference*1.0/graceZone);
  }
  if(maxSpeed<20){
    maxSpeed=20;
  }
  if(difference3<haltZone){
    maxSpeed=0;
  }

  if(target<position)
    speed=-1*maxSpeed;
  else
    speed=maxSpeed;


  pDir1=0;
  pDir2=0;
  if(speed>0){
    pDir1=speed;
  }else{
    pDir2=-1*speed;
  }
  applyVals();

  runs++;
  if(millis()-lastPrint>(1000.0/printsPerSecond)){
    lastPrint=millis();
    Serial.println("f-:-25500,f+:"+String(topVal)+
        ",speed:"+String(speed*100)+
        ",position:"+String(position)+
        ",runs/s:"+String(runs*printsPerSecond)+
        "");

    runs=0;
  }
}

long diff(long a, long b){
  return max(a,b)-min(a,b);
}


/*
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
      ",POS:"+String(position)+
      ",dir1:"+String(pDir1)+
      ",dir2:"+String(pDir2)+
      ",pot:"+String(pPot)+
      ",mult:"+String(mult));
}
*/
void applyVals(){
  analogWrite(motorA,pDir1);
  analogWrite(motorB,pDir2);
}

void readPosition(){
  position=as5600.getCumulativePosition();
}

