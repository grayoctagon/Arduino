
#include "AS5600.h"
#include "Wire.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
/**

*/

AS5600 as5600;   //  use default Wire
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int inputDir=A0;
//int inputPot=A1;
int inputAmpere=A7;
int inputSwitch=8;
int inputEndSwitch=3;

int motorA=5;
int motorB=6;
int beeper=2;

long pDirRaw;
//long pPot;

long pDir1=0;
long pDir2=0;

int32_t position=0;


int32_t topVal=81920;//80cm = 800*(4096/40) = 81920
int32_t target=topVal;
int32_t graceZone=15360;//15cm = 150*(4096/40) = 15360
int32_t haltZone=50;//to avoid oscilation

long ampsSum=0;
long ampsNum=0;
long ampsMin=1024;
long ampsMax=0;

long lastTarget=target;
long lastTargetStart=0;
long lastTargetEnd=0;
bool lastTargetreached=false;

long now=0;

void setup() {

  pinMode(beeper, OUTPUT);
  digitalWrite(beeper, HIGH);
  delay(100);
  digitalWrite(beeper, LOW);
  delay(100);

  Serial.begin(115200);

  pinMode(motorA, OUTPUT);
  pinMode(motorB, OUTPUT);

  pinMode(inputAmpere, INPUT);
  pinMode(inputSwitch, INPUT_PULLUP);
  pinMode(inputEndSwitch, INPUT_PULLUP);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display();
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  delay(100);
  //testdrawchar();
  delay(900);
  testdrawbitmap();
  delay(500);
  printText("starting AS5600",false);

  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.
  Serial.println(as5600.getAddress());
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  
  delay(100);
  as5600.getCumulativePosition();
  as5600.resetPosition();

  printText("AS5600 done ",true);
  //
  delay(800);
  digitalWrite(beeper, HIGH);
  delay(100);
  digitalWrite(beeper, LOW);
  delay(100);
}
long lastPrint=0;
long runs=0;
int printsPerSecond=2;

int speed=0;
long updateDispStart=0;
long updateDispEnd=0;

float movingSpeed=0;
float movingSpeedMax=0;
float movingSpeedSum=0;
int movingSpeedNum=0;

int skeppedDisplay=0;

long manualOverwrite=-1;

void loop() {
  
  now=millis();

  readPosition();
  readAmps();
  //loopBeep();

  readEndswitch();

  readManualInput();
  if(manualOverwrite!=-1){
    return;
  }


  /* //auto position
  if(position>=topVal){
    target=0;
  }
  if(position<=0){
    target=topVal;
  }
  */
  

  if(digitalRead(inputSwitch)==HIGH){
    target=0;
  }else{
    target=topVal;
  }
  if(lastTarget!=target){
    lastTarget=target;
    lastTargetStart=millis();
    lastTargetEnd=lastTargetStart;
    lastTargetreached=false;
  }


  speed=0;
  int maxSpeed=255;
  long difference1=diff(0,position);
  long difference2=diff(topVal,position);
  long difference3=diff(target,position);
  long difference=min(difference1,difference2);

  bool skipDisplay=false;

  if(difference<graceZone){
    maxSpeed=maxSpeed*(difference*1.0/graceZone);
  }
  if(maxSpeed<20){
    maxSpeed=20;
    skipDisplay=true;
  }
  if(difference3<haltZone){
    maxSpeed=0;
    skipDisplay=false;
    movingSpeedMax=0;
    movingSpeedSum=0;
    movingSpeedNum=0;
    lastTargetreached=true;
  }
    if(!lastTargetreached)
      lastTargetEnd=millis();

  

  if(target<position)
    speed=-1*maxSpeed;
  else
    speed=maxSpeed;


  pDir1=0;
  pDir2=0;
  if(speed>0){
    pDir1=speed;
  }else{
    pDir2=-1*speed;
  }
  applyVals();

  runs++;
  if(millis()-lastPrint>(1000.0/printsPerSecond)){
    lastPrint=millis();
    Serial.println("f-:-25500,f+:"+String(topVal)+
        ",speed:"+String(speed*100)+
        ",position:"+String(position)+
        ",runs/s:"+String(runs*printsPerSecond)+
        "");
    //if(!skipDisplay || skeppedDisplay>5){
    if(movingSpeed<0.2){
      updateDisplay();
      //skeppedDisplay=0;
    }else{
      //skeppedDisplay++;
    }
    runs=0;
  }
}

long diff(long a, long b){
  return max(a,b)-min(a,b);
}

void updateDisplay() {
  long runtime=updateDispEnd-updateDispStart;
  updateDispStart=millis();

  
  display.clearDisplay();

  display.setCursor(0, 0);     // Start at top-left corner

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  /*for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }*/
  display.println("ta:"+String(target)+" ti:"+String((lastTargetEnd-lastTargetStart)/1000.0)+"s");
  display.println("speed:"+String(speed));
  display.println("position:"+String(position/102.4));
  display.println("runs:"+String(runs));
  display.println("disp.refresh:"+String(runtime));
  display.println("amps:"+String(getAmpsMin())+"-"+String(getAmpsMax()) );
  display.println("ampsAVG:"+String(getAmpsAVG()));
  display.println(String(movingSpeed)+"m/s  ~"+String(movingSpeedSum/movingSpeedNum)+"  "+String(movingSpeedMax));

  

  display.display();
  display.flush();
  resetAmps();

  //delay(5);
  updateDispEnd=millis();
}

void printText(String text, bool clear){
  if(clear)
    display.clearDisplay();
  display.setCursor(0, 0);
  display.println(text);
  display.display();
}


double getAmpsMin(){
  if(ampsMin==1024)return 0;
  double d= (ampsMin-512)/1024.0*30;
  if(d<0)return -1*d;
  return d;
}
double getAmpsMax(){
  double d=(ampsMax-512)/1024.0*30;
  if(d<0)return -1*d;
  return d;
}
double getAmpsAVG(){
  double avg=1.0;avg=ampsSum/(avg*ampsNum);
  double d= (avg-512)/1024.0*30;
  if(d<0)return -1*d;
  return d;
}
void readAmps(){
  int a=analogRead(inputAmpere);
  ampsSum+=a;
  ampsNum++;
  if(diff(a,512)<diff(ampsMin,512))ampsMin=a;
  if(diff(a,512)>diff(ampsMax,512))ampsMax=a;
}
void resetAmps(){
  ampsSum=0;
  ampsNum=0;
  ampsMin=0;
  ampsMax=512;
}

void applyVals(){
  analogWrite(motorA,pDir1);
  analogWrite(motorB,pDir2);
}

void readPosition(){
  position=as5600.getCumulativePosition();

  movingSpeed= (as5600.getAngularSpeed(AS5600_MODE_RPM)/60*20*2)/1000;//20teeth
  if(movingSpeed<0)movingSpeed=movingSpeed*-1;
  if(movingSpeed>movingSpeedMax)movingSpeedMax=movingSpeed;
  movingSpeedSum+=movingSpeed;
  movingSpeedNum++;
}


void readEndswitch(){
  if(digitalRead(inputEndSwitch)==LOW){
    
    digitalWrite(beeper, HIGH);
    delay(100);
    digitalWrite(beeper, LOW);
    delay(100);
    
    as5600.getCumulativePosition();
    as5600.resetCumulativePosition(-100);
  }
}



void readManualInput(){	
  pDirRaw=analogRead(inputDir);	
  long center=502;
  long pDirC=pDirRaw-center;
  int dir=1;	
  if(pDirC<0){	
    dir=-1;	
    pDirC=pDirC*-1;
  }
  if(pDirC<0)pDirC=0;	
  if(pDirC>500)pDirC=500;
  pDirC=map(pDirC,0,500,0,255);
  

  if(pDirC<10)pDirC=0;

  //map(pDirRaw,28,993,-511,511);	


  long mpDir1=0;
  long mpDir2=0;
  if(dir>0){	
    mpDir1=pDirC;	
  }else{	
    mpDir2=pDirC;	
  }
  
  if(pDirC==0){
    if(manualOverwrite!=-1){
      pDir1=0;
      pDir2=0;
      applyVals();
      if(manualOverwrite+5000<now){
        manualOverwrite=-1;
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("manual overwrite\nendet");
        display.display();
        display.flush();
      }
    }
  }else{
    manualOverwrite=now;
    pDir1=mpDir1;
    pDir2=mpDir2;
    applyVals();
  }
  if(manualOverwrite!=-1 && (movingSpeed < 0.2) ){
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("MANUAL OVERWRITE");
    display.println("MANUAL OVERWRITE");
    
    display.print("timeout: ");
    display.println(manualOverwrite+5000-now);

    display.print("moving: ");
    display.println(pDirC*dir);

    display.print(movingSpeed);
    display.println("m/s ");

    display.print("pos: ");
    display.println(position);
    

    display.println("MANUAL OVERWRITE");
    display.println("MANUAL OVERWRITE");
    display.display();

  }else if( (movingSpeed >= 0.2) ){
    Serial.println(movingSpeed);
  }
  /** /
  Serial.println("f0:0,f1024:1024,f20:20,dirRaw:"+String(pDirRaw)+	
      ",dirC:"+String(pDirC)+	
      ",POS:"+String(position)+	
      ",dir1:"+String(pDir1)+	
      ",dir2:"+String(pDir2)+	
      ",pot:"+String(pPot)+	
      ",mult:"+String(mult));/**/
      
}	












long beepoff=-1;
void doBeep(long time){
    digitalWrite(beeper, HIGH);
  beepoff=now+time;
}
void loopBeep(){
  if(beepoff==-1)return;
  if(beepoff<now){
    digitalWrite(beeper, LOW);
  }
}













void testdrawchar(void) {
  display.clearDisplay();

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  // Not all the characters will fit on the display. This is normal.
  // Library will draw what it can and the rest will be clipped.
  for(int16_t i=0; i<256; i++) {
    if(i == '\n') display.write(' ');
    else          display.write(i);
  }

  display.display();
}

//https://javl.github.io/image2cpp/





//#define LOGO_HEIGHT   60
//#define LOGO_WIDTH    60
#define LOGO_HEIGHT   22
#define LOGO_WIDTH    22

static const unsigned char PROGMEM logo_bmp[] =
{ 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xfe, 0x00, 0x03, 0xff, 0x00, 0x07, 0xff, 0x80, 0x0f, 
	0xff, 0xc0, 0x1f, 0xff, 0xe0, 0x3f, 0xff, 0xf0, 0x3f, 0xff, 0xf0, 0x3f, 0xff, 0xf0, 0x3f, 0xff, 
	0xf0, 0x3f, 0xff, 0xf0, 0x3f, 0xff, 0xf0, 0x3f, 0xff, 0xf0, 0x3f, 0xff, 0xf0, 0x1f, 0xff, 0xe0, 
	0x0f, 0xff, 0xc0, 0x07, 0xff, 0x80, 0x03, 0xff, 0x00, 0x01, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00
  /*
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x0F,0xFF,0xFF,0x00,0x00,0x00, // ....................####################........................
  0x00,0x00,0x1F,0xFF,0xFF,0x80,0x00,0x00, // ...................######################.......................
  0x00,0x00,0x3F,0xFF,0xFF,0xC0,0x00,0x00, // ..................########################......................
  0x00,0x00,0x7F,0xFF,0xFF,0xE0,0x00,0x00, // .................##########################.....................
  0x00,0x00,0xFF,0xFF,0xFF,0xF0,0x00,0x00, // ................############################....................
  0x00,0x01,0xFF,0xFF,0xFF,0xF8,0x00,0x00, // ...............##############################...................
  0x00,0x03,0xFF,0xFF,0xFF,0xFC,0x00,0x00, // ..............################################..................
  0x00,0x07,0xFF,0xFF,0xFF,0xFE,0x00,0x00, // .............##################################.................
  0x00,0x0F,0xFF,0xFF,0xFF,0xFF,0x00,0x00, // ............####################################................
  0x00,0x1F,0xFF,0xFF,0xFF,0xFF,0x80,0x00, // ...........######################################...............
  0x00,0x3F,0xFF,0xFF,0xFF,0xFF,0xC0,0x00, // ..........########################################..............
  0x00,0x7F,0xFF,0xFF,0xFF,0xFF,0xE0,0x00, // .........##########################################.............
  0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xF0,0x00, // ........############################################............
  0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8,0x00, // .......##############################################...........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x03,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC,0x00, // ......################################################..........
  0x01,0xFF,0xFF,0xFF,0xFF,0xFF,0xF8,0x00, // .......##############################################...........
  0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xF0,0x00, // ........############################################............
  0x00,0x7F,0xFF,0xFF,0xFF,0xFF,0xE0,0x00, // .........##########################################.............
  0x00,0x3F,0xFF,0xFF,0xFF,0xFF,0xC0,0x00, // ..........########################################..............
  0x00,0x1F,0xFF,0xFF,0xFF,0xFF,0x80,0x00, // ...........######################################...............
  0x00,0x0F,0xFF,0xFF,0xFF,0xFF,0x00,0x00, // ............####################################................
  0x00,0x07,0xFF,0xFF,0xFF,0xFE,0x00,0x00, // .............##################################.................
  0x00,0x03,0xFF,0xFF,0xFF,0xFC,0x00,0x00, // ..............################################..................
  0x00,0x01,0xFF,0xFF,0xFF,0xF8,0x00,0x00, // ...............##############################...................
  0x00,0x00,0xFF,0xFF,0xFF,0xF0,0x00,0x00, // ................############################....................
  0x00,0x00,0x7F,0xFF,0xFF,0xE0,0x00,0x00, // .................##########################.....................
  0x00,0x00,0x3F,0xFF,0xFF,0xC0,0x00,0x00, // ..................########################......................
  0x00,0x00,0x1F,0xFF,0xFF,0x80,0x00,0x00, // ...................######################.......................
  0x00,0x00,0x0F,0xFF,0xFF,0x00,0x00,0x00, // ....................####################........................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // ................................................................
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00  // ................................................................
  /**/
  
  /*0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 */
  };

void testdrawbitmap(void) {
  display.clearDisplay();

  display.drawBitmap(
    (display.width()  - LOGO_WIDTH ) / 2,
    (display.height() - LOGO_HEIGHT) / 2,
    logo_bmp, LOGO_WIDTH, LOGO_HEIGHT, 1);
  display.display();
  delay(1000);
}
