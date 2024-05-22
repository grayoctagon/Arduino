/*
  copy pasta comands:
  setM;0,0,0
  setM;0,0,255
  setReadAnal;0,-1
  setReadAnal;0,1
  setReadAnal;0,7
  setAnalLimit;0,0,0
  setAnalLimit;0,0,5
*/

//PWM Pins: 3,5,6,9,10,11,

#define ADC_SCALE 1023.0
#define VREF 5.0
#define ACS712_zero 512

short pins=6;
int motorPins[]={3,5,6,9,10,11};
int motorVals[]={0,0,0,0,0,0};

short sensors=8;
int sensorPins[]={A0,A1,A2,A3,A4,A5,A6,A7};
int sensorValsRead[]={0,0,0,0,0,0,0,0};
int sensorValsLimit[]={1,10,10,10,10,10,10,10};
int sensorValsSend[]={-500,-500,-500,-500,-500,-500,-500,-500};

int sensorPin = A0;
int sensorValue = 0;
String inStr;
int readAnalogSensorsUntil=1;

long runs=0;
long runst=0;
long lastRuns=millis();

void setup() {
  Serial.begin(115200);
  //Serial.begin(9600);
  Serial.setTimeout(10);
}

void doSerial(){
  if(!Serial.available())return;
  inStr = Serial.readString();
  while(-1!=inStr.indexOf("\n")){
    String line=inStr.substring(0,inStr.indexOf("\n"));
    line.trim();
    inStr=inStr.substring(inStr.indexOf("\n")+1);
    lineSerial(line);
  }
}
void lineSerial(String line){
  if(line.indexOf("setM;")==0){
    line=line.substring(String("setM;").length());
    int ack=line.substring(0,line.indexOf(",")).toInt();
    line=line.substring(line.indexOf(",")+1);
    int mnum=line.substring(0,line.indexOf(",")).toInt();
    int val=line.substring(line.indexOf(",")+1).toInt();
    motorVals[mnum]=val;
    applyMotors();
    Serial.println("ACK"+String(ack));
  }else
  if(line.indexOf("setReadAnal;")==0){
    line=line.substring(String("setReadAnal;").length());
    int ack=line.substring(0,line.indexOf(",")).toInt();
    line=line.substring(line.indexOf(",")+1);
    int readIt=line.substring(line.indexOf(",")+1).toInt();
    readAnalogSensorsUntil=readIt;
    Serial.println("ACK"+String(ack));
  }else
  if(line.indexOf("setAnalLimit;")==0){
    line=line.substring(String("setAnalLimit;").length());
    int ack=line.substring(0,line.indexOf(",")).toInt();
    line=line.substring(line.indexOf(",")+1);
    int mnum=line.substring(0,line.indexOf(",")).toInt();
    int val=line.substring(line.indexOf(",")+1).toInt();
    sensorValsLimit[mnum]=val;
    applyMotors();
    Serial.println("ACK"+String(ack));
  }
}
void applyMotors(){
  for (short i = 0; i < pins; i++) {
    analogWrite(motorPins[i], motorVals[i]);
  }
}
bool readAnaloSenor(int i){
  sensorValsRead[i]=analogRead(sensorPins[i]);
  return sensorValsRead[i];
}
void readAnalogs(){
  boolean sendSensors=false;
  for (short i = 0; i < sensors && i <= readAnalogSensorsUntil; i++) {
    readAnaloSenor(i);
    
    if(i==0){
      float z=getCurrentDC_ACS712(sensorValsRead[0]);
      z=z*10;
      sensorValsRead[0]=z;
    }
    if(sensorValsLimit[i]<diff(sensorValsRead[i],sensorValsSend[i])){
      sendSensors=true;
    }

  }
  if(sendSensors){
    String sendText="SensorValues,";
    for (short i = 0; i < sensors; i++) {
      sensorValsSend[i]=sensorValsRead[i];
      sendText+=String(sensorValsRead[i])+(",");
    }
    Serial.println(sendText);
  }
}
int diff(int a, int b){
  if(a<b){
    return b-a;
  }else{
    return a-b;
  }  
}

float getCurrentDC_ACS712(int val) {
//source: https://github.com/rkoptev/ACS712-arduino
  /*
			sensitivity = 0.185; // ACS712_05B
			sensitivity = 0.100; // ACS712_20A
			sensitivity = 0.066; // ACS712_30A
  */
  float sensitivity = 0.066; // ACS712_30A
	float I = ((float)val-ACS712_zero) / ADC_SCALE * VREF / sensitivity;
  if(I<0)return -I;
	return I;
}

void checkRuns(){
  runs++;
  if(runs>999){
    runs=0;
    runst++;
    long t=millis();
    if(lastRuns+3000<t){
      Serial.println(String(runst)+"k runs/3s");
      runst=0;
      lastRuns=t;
    }
  }
}

void loop() {
  checkRuns();

  doSerial();
  if(readAnalogSensorsUntil>-1){
    readAnalogs();
  }

  
/*
  sensorValue = analogRead(sensorPin);
  int sensorValue2=map(sensorValue, 0, 1023, 0, 510);
  //sensorValue=sensorValue/2;
  //if(sensorValue<0)sensorValue=0;
  //if(sensorValue>511)sensorValue=511;
  int o1=0;
  int o2=0;
  int val=sensorValue2-255;
  if(val>0){
    o1=val;
  }else{
    o2=val*-1;
  }
  
  motorVals[0]=o1;
  motorVals[1]=o2;

  if(false)
  Serial.println(
    String("Analog: ")+
    String(sensorValue)+
    String(", calc: ")+
    String(sensorValue2)+
    String(", o1: ")+
    String(o1)+
    String(", o2: ")+
    String(o2)
  );*/


  /*
    delay(30);
  }
  */
}
