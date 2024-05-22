void setup() {
  Serial.begin(115200);

  setupPoti(A0);
  setupStepper(3,4);

}

void loop() {
  loopStepper();
  loopPoti();
}
