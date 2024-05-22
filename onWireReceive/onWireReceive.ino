#include <OneWire.h>


const byte rxPin = 2;
const byte txPin = 3;

OneWire myWire(rxPin);

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  int in=myWire.read();
  bool done =false;
  while(in!=255){
    done=true;
    Serial.print((in));
    Serial.print(" \t");
    in=myWire.read();
  }
  if(done)
  Serial.println(" ");
}



