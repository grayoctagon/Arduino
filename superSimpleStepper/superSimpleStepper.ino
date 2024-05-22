int pinStep=3;
int pinDir=4;
void setup() {
  pinMode(pinStep, OUTPUT);
  pinMode(pinDir, OUTPUT);
}
long last_step_time=0;
long step_delay=1500;//=201rpm
void loop() {
 long now=micros();
 if (now - last_step_time >= step_delay) {
    last_step_time = now;
      digitalWrite(pinDir, HIGH);
      digitalWrite(pinStep, LOW);
      digitalWrite(pinStep, HIGH);
 }
}
