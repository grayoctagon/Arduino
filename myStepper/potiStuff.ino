int myPin;
long lastIn=0;
float lastCalc=0;

long lastPrint=0;
int timeBetweenPrint=1000/12;
long lastRefresh=0;
int timeBetweenRefresh=1000/100;

void setupPoti(int potiPin) {
  pinMode(potiPin, INPUT);
}

void loopPoti() {
  long now=millis();
  if(now-timeBetweenRefresh>lastRefresh){
    loopRefreshPoti();
    lastRefresh=now;
  }
  if(now-timeBetweenPrint>lastPrint){
    loopPrintPoti();
    lastPrint=now;
  }
}
void loopRefreshPoti() {
  lastIn=analogRead(myPin);
  lastCalc=(lastIn-512);
  if(lastCalc<50&&lastCalc>-50){
    lastCalc=0;
  }else{
    lastCalc+=(lastCalc>0?1:-1)*0;
    lastCalc=lastCalc/2;
  }
  setSpeed(lastCalc);
}
void loopPrintPoti() {
  Serial.println("rpm: "+String(lastCalc));
}
