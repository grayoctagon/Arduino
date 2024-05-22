#include <SoftwareSerial.h>

//https://www.electronicwings.com/arduino/ir-communication-using-arduino-uno

#define cr_pin 9
//#define baud 300
//#define baud 1200
//#define baud 9600
#define baud 19200
//#define baud 38400
//#define baud 57600  //works mostly with transistor

//#define baud 74880 //works mostly with transistor, a bit worse without

//#define baud 115200 //does mostly not work with transistor or without

const byte rxPin = 2;
const byte txPin = 3;

SoftwareSerial mySerial (rxPin, txPin);

void setup() {
  Serial.begin(baud);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  mySerial.begin(baud);

  //tone(cr_pin, 38000);	/* For modulation at 38kHz */
}


bool doSerial(){
  if(mySerial.available()<=0)return false;
      int c=mySerial.read();
      if(c!=-1){
        Serial.write(c);
        return true;
      }
      return false;
}
long count =0;
void loop() {
  /* */
  mySerial.println("msg "+String(count)+ " "+String(analogRead(A0)));
  while(doSerial());
  delay(500);
  count++;
  if(count>999)count=0;
  //*/
  /*
  if(mySerial.available())
  Serial.print((char)mySerial.read());
  */
}
