
int outPin=6;

void setup() {
  Serial.begin(115200);
  //Serial.begin(9600);
  Serial.setTimeout(10);
  pinMode(outPin,OUTPUT);

  //Code for Available PWM frequency for D5 & D6: https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/?utm_content=cmp-true

  //TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz   
  
  //TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz

  //TCCR0B = TCCR0B & B11111000 | B00000011; // for PWM frequency of 976.56 Hz (The DEFAULT)

  //TCCR0B = TCCR0B & B11111000 | B00000100; // for PWM frequency of 244.14 Hz

  //TCCR0B = TCCR0B & B11111000 | B00000101; // for PWM frequency of 61.04 Hz
}

void loop() {
  int val=analogRead(A0);
  
  if(val<=100){
    val=100;
  }
  //else   val=val*50;
  if(val>=1000){
    val=1000;
  }
  val=map(val,100,1000,0,255);// 
  //val=(val-100)/(1000/255.0);//900/255
  Serial.println("writing "+String(val));

  analogWrite(outPin,val);
  delay(100);
}
