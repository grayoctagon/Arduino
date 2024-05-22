#include <OneWire.h>


const byte rxPin = 2;
const byte txPin = 3;

OneWire myWire(txPin);

void setup() {
  Serial.begin(9600);
  
}

long count =0;
void loop() {
  doWrite("msg "+String(count)+ " "+String(analogRead(A0)) );
  Serial.println("msg "+String(count)+ " "+String(analogRead(A0)));
  delay(1000);
  count++;
}

void doWrite(String msg1){
  int str_len = msg1.length() + 1; 

  // Prepare the character array (the buffer) 
  char char_array[str_len];

  // Copy it over 
  msg1.toCharArray(char_array, str_len);
  for(int i =0; i < msg1.length(); i++ ) {
    char c = msg1[i];
    myWire.write(c);
  }
}


