
int outPin1=5;
int outPin2=6;

void setup() {
  Serial.begin(115200);
  //Serial.begin(9600);
  Serial.setTimeout(10);
  pinMode(outPin1,OUTPUT);
  pinMode(outPin2,OUTPUT);
  
  //partialy based on https://gist.github.com/Crypto69/fe1a8a319056cae923c3ee3a39b0fbae

  //Code for Available PWM frequency for D5 & D6: https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/?utm_content=cmp-true

  //TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz   
  
  //TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz

  //TCCR0B = TCCR0B & B11111000 | B00000011; // for PWM frequency of 976.56 Hz (The DEFAULT)

  //TCCR0B = TCCR0B & B11111000 | B00000100; // for PWM frequency of 244.14 Hz

  //TCCR0B = TCCR0B & B11111000 | B00000101; // for PWM frequency of 61.04 Hz
}

void loop() {
  int val=analogRead(A0);
  
  int mval=val;
  int dir=1;
  if(val>=512){
    dir=-1;
    mval=val-512;
  }else{
    mval=512-val;
  }
  mval=mval-30;
  if(mval<0)mval=0;
  if(mval>450)mval=450;
  mval=map(mval,0,450,0,255);// 

  Serial.println("mval:"+String(mval)+",mi:0,ma:255");

  if(dir>0){
    analogWrite(outPin1,0);
    analogWrite(outPin2,mval);
  }else{
    analogWrite(outPin1,mval);
    analogWrite(outPin2,0);
  }

  //val=(val-100)/(1000/255.0);//900/255
  //Serial.println("writing "+String(val));
  delay(1000/30);
}
