
#include <AccelStepper.h>
// https://www.arduino.cc/reference/en/libraries/accelstepper/
// https://github.com/waspinator/AccelStepper


#define pinStep1step 3
#define pinStep1dir 4
#define pinPotentiometer0 A0

AccelStepper stepper1(1, pinStep1step, pinStep1dir);
//AccelStepper stepper2(AccelStepper::FULL4WIRE, 8, 9, 10, 11);


void setup() {
  delay(10);
  Serial.begin(115200);
  delay(10);

  stepper1.setMaxSpeed(1400);
  stepper1.setAcceleration(1500);
  stepper1.moveTo(1000);
}

long lastPrint=0;
int potiPos=0;

void loop() {
  //stepper1.runToNewPosition(000);
  //stepper1.runToNewPosition(200);
  stepper1.run();
  if(stepper1.distanceToGo()==0){
    stepper1.moveTo(-1* stepper1.currentPosition());
  }
  if(lastPrint+(1000/5)<millis()){
    potiPos=analogRead(pinPotentiometer0);
    //stepper1.setAcceleration(4*potiPos+0.1);
    //stepper1.setMaxSpeed(4*potiPos+0.1);

    Serial.println("min:-1000,max:1000,StepperPos:"+String(stepper1.currentPosition())+",speed:"+String(stepper1.speed())+",poti4:"+String(4*potiPos));
    lastPrint=millis();
  }
  //delay(1000);
}
