

int pinStep=-1;
int pinDir=-1;

long step_delay=1;
long last_step_time=0;

int number_of_steps=200;


int stepperDirection=0;



void setupStepper(int pStep, int pDir) {
  pinStep=pStep;
  pinDir=pDir;
  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);
  setSpeed(60);
}


void loopStepper() {
 long now=micros();
 if (now - last_step_time >= step_delay) {
    last_step_time = now;
    if(stepperDirection!=0){
      if(stepperDirection>0){
          digitalWrite(pinDir, LOW);
      }else{
          digitalWrite(pinDir, HIGH);
      }
      digitalWrite(pinStep, LOW);
      digitalWrite(pinStep, HIGH);
    }
  }
}

void setSpeed(float refsPerMinute)
{
  if(refsPerMinute==0){
    stepperDirection=0;
    step_delay=60*1000L*5;
  }else{
    if(refsPerMinute<0){
      refsPerMinute=refsPerMinute*-1;
      stepperDirection=-1;
    }else{
      stepperDirection=1;
    }
    step_delay = 60L * 1000L * 1000L / number_of_steps / refsPerMinute;
  }
}
