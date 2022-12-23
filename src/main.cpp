#include <Arduino.h>
#include "uFire_SHT20.h"
#include "PinChangeInterrupt.h"

//Encoders and LED hardware pins
#define noOfCannels 2
const byte SW [noOfCannels] = {4, A0};
const byte CLK[noOfCannels] = {7, A1};
const byte DTA[noOfCannels] = {8, A2};
const byte PWM_1[noOfCannels] = {5, 9};
const byte PWM_2[noOfCannels] = {6, 10};
const byte LEDPin [noOfCannels] = {20, 21};
//encoders variables
volatile int deg [noOfCannels]= {0, 0};
volatile unsigned long push [noOfCannels] {0, 0};
volatile unsigned long release[noOfCannels] = {0,0};
// encoder timmings
#define longPressTime 1500
#define dooubleClickTreshold 400
//dimmer LED control variables
int lum[noOfCannels]= {50, 50};
int temp[noOfCannels]= {75, 75};
byte targetPWM_1 [noOfCannels] = {0, 0};
byte targetPWM_2 [noOfCannels] = {0, 0};
byte actPWM_1 [noOfCannels] = {0, 0};
byte actPWM_2 [noOfCannels] = {0, 0};
bool initialized = false;
#define PWMStep 3

bool on[noOfCannels]= {false, false};
const byte brightnessCurve[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 14, 17, 21, 26, 32, 39, 48, 59, 73, 89, 110, 135, 166, 204, 255};


//DHT specific
#define TEMPSENSOROFFSET 1.3       //(*0.1)=temperature sensor offset to calibrate specific sensor
#define DHT_UPDATE_INTERVAL 300   // (seconds) time interval to force temp/hum. message
unsigned long DHT_lastMillis = 0;
uFire_SHT20 sht20;


//MySensors specific
// Enable debug prints to serial monitor
//#define MY_DEBUG
#define MY_RF24_CHANNEL 118
#define MY_TRANSPORT_WAIT_READY_MS 5000
#define MY_RADIO_RF24
#define MY_RF24_CE_PIN A3
#define MY_RF24_CS_PIN 2
#define MY_NODE_ID 10 
#define SKETCH_NAME "Double CCT LED"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "4"
#define MY_REPEATER_FEATURE enable
#define MY_DISABLED_SERIAL enable
#include <MySensors.h>
const byte light_ID [noOfCannels] = {3, 5};
const byte lightTemp_ID [noOfCannels] = {31, 51};
#define DHT_HUM_ID 41
#define DHT_TEMP_ID 42
MyMessage msgState[noOfCannels];
MyMessage msgDimmer[noOfCannels];
MyMessage msgLigthtTempState[noOfCannels];
MyMessage msgLigthtTemp[noOfCannels];
MyMessage msgHum(DHT_HUM_ID, V_HUM);
MyMessage msgTemp(DHT_TEMP_ID, V_TEMP);



//encoder interrupts routines
void encod_A(){
  deg[0]+=(digitalRead(DTA[0])!=digitalRead(CLK[0]))*2-1;
//  Serial.print(deg1);
}

void butt_A(){
  if (!digitalRead(SW[0]))
    push[0]=millis();
  else
    if (push[0])
      release[0]=millis();  
}

void encod_B(){
  deg[1]+=(digitalRead(DTA[1])!=digitalRead(CLK[1]))*2-1;
}

void butt_B(){
  if (!digitalRead(SW[1]))
    push[1]=millis();
  else
    if (push[1])
      release[1]=millis();  
}

//calculate target PWM values and send message to gate 
void setPWM(byte channel ) {
  targetPWM_1[channel] = on[channel]*brightnessCurve[lum[channel]/4]*min(100-temp[channel], 50)/50;
  targetPWM_2[channel] = on[channel]*brightnessCurve[lum[channel]/4]*min(temp[channel], 50)/50;
  if (on[channel]) send(msgState[channel].set(on[channel]));   //if on, send on message first, then dimmer
  send(msgDimmer[channel].set(lum[channel]));
  send(msgLigthtTemp[channel].set(temp[channel]));
  if (!on[channel])
  {
     send(msgState[channel].set(on[channel]));   //if off, send dimmer message first, then off
     send(msgLigthtTempState[channel].set(on[channel]));
  }  
   
 
  //  saveState(A_ID, A_lum);
  //  saveState(B_ID, B_lum);

}

//blink control leds
void LEDBlink (int blinkMillis, byte repeat) {
  for(byte j=0; j<repeat; j++) {
    for(byte i=0; i<noOfCannels; i++) {
      pinMode(LEDPin[i], OUTPUT);
      digitalWrite(LEDPin[i], HIGH);
      wait(blinkMillis);
      digitalWrite(LEDPin[i], LOW);
      pinMode(LEDPin[i], INPUT);
    }
    wait(blinkMillis);
  }
}

//set hardware before radio transmission initialization
void before() {
  for(byte i=0; i<noOfCannels; i++) {
   pinMode(SW[i], INPUT_PULLUP);
   pinMode(CLK[i], INPUT_PULLUP);
   pinMode(DTA[i], INPUT_PULLUP);
   pinMode(PWM_1[i], OUTPUT);
   pinMode(PWM_2[i], OUTPUT);
   digitalWrite(PWM_1[i], LOW);
   digitalWrite(PWM_2[i], LOW);
   pinMode(LEDPin[i], INPUT);
   msgState[i].sensor = light_ID[i];                                   // initialize messages
   msgState[i].type = V_LIGHT;
   msgDimmer[i].sensor = light_ID[i];                                  
   msgDimmer[i].type = V_DIMMER;
   msgLigthtTempState[i].sensor = lightTemp_ID[i];
   msgLigthtTempState[i].type = V_LIGHT;
   msgLigthtTemp[i].sensor = lightTemp_ID[i];                                  
   msgLigthtTemp[i].type = V_DIMMER;
  }
}


void setup() {
// put your setup code here, to run once:
  
//  Serial.begin(115200);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CLK[0]), encod_A, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(CLK[1]), encod_B, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SW[0]), butt_A, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(SW[1]), butt_B, CHANGE);
  LEDBlink(100, 3);
  sht20.begin();
  DHT_lastMillis=millis();
  for(byte i=0; i<noOfCannels; i++)
   setPWM(i);
  //A_lum=loadState(A_ID);
  //B_lum=loadState(B_ID);
}

void presentation(){
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

  // Register devices to sensor_node (they will be created as child devices)
  for(byte i=0; i<noOfCannels; i++) {
    present(light_ID[i], S_DIMMER);
    present(lightTemp_ID[i], S_DIMMER);
  }
  present(DHT_HUM_ID, S_HUM);
  present(DHT_TEMP_ID, S_TEMP);
}


void receive(const MyMessage &message) {
  for(byte i=0; i<noOfCannels; i++){
    if (message.getType()==V_LIGHT)
    {
      // Change on/off state
      if (message.getSensor()==light_ID[i])
       on[i]=message.getBool();
    }
    if (message.getType()==V_DIMMER)
    {
      // Change brightness or color temp.
      int dimvalue= atoi( message.data );
      if (message.getSensor()==light_ID[i])
       lum[i]=dimvalue;
      if (message.getSensor()==lightTemp_ID[i])
       temp[i]=dimvalue;
    }
   targetPWM_1[i] = on[i]*brightnessCurve[lum[i]/4]*min(100-temp[i], 50)/50;
   targetPWM_2[i] = on[i]*brightnessCurve[lum[i]/4]*min(temp[i], 50)/50;
   }
}


void loop() {
if (!initialized)
{
  for(byte i=0; i<noOfCannels; i++){
    send(msgState[i].set(on[i]));
    send(msgDimmer[i].set(lum[i]));
    send(msgLigthtTempState[i].set(on[i]));
    send(msgLigthtTemp[i].set(temp[i])); 
    initialized=true;
  }  
}

 unsigned long curMillis=millis();
 //check if it is time to read and send data from DHT
 if((curMillis-DHT_lastMillis)/1000 >= DHT_UPDATE_INTERVAL) 
 {
    float temp=sht20.temperature();
    DHT_lastMillis=curMillis;
    send(msgTemp.set(temp-TEMPSENSOROFFSET,1));
    temp=sht20.humidity();
    send(msgHum.set(temp,1));
 }  
 // loop thru all channels 
 for (byte i=0; i<noOfCannels; i++) {                     
   //check encoders 
    if(push[i]){                                  
      //long press, toggle all channels on/off
      if (curMillis-push[i]>longPressTime) {      
        on[i]=!on[i];
        for (byte j=0; j<noOfCannels; j++) {              
          on[j]=on[i];
          setPWM(j);
        }
        push[i]=0;
      }
      else
      if(release[i]){
        //double click, set all channels equal to channel "i"
        if (release[i]<push[i]){              
         for (byte j=0; j<noOfCannels; j++) {              
           lum[j]=lum[i];
           temp[j]=temp[i];
           setPWM(j);
         }
         release[i]=0;
         push[i]=0;
        }
        else 
          //single click, toggle channel on/off
          if (curMillis-release[i]>dooubleClickTreshold) {    
            on[i]=!on[i];
            release[i]=0;
            push[i]=0;
            setPWM(i);
         }
      }
    }
    //check if rotator has been turned
    if (deg[i]){                               
      if (on[i]) {
        //with push
        if (!digitalRead(SW[i])){              
          temp[i]=constrain(temp[i]+deg[i]*4, 1, 100);
          push[i]=0;
        }  
        //without push
        else                                    
          lum[i]=constrain(lum[i]+deg[i]*4, 1, 100);
        setPWM(i);
      }
      deg[i]=0;    
    }
    //flawless set hardware PWM to target values
    int delta=targetPWM_1[i]-actPWM_1[i];
    if (delta!=0) {
      delta=(delta<PWMStep*(-1)?(PWMStep*(-1)):(delta)>PWMStep?(PWMStep):(delta));
      actPWM_1[i]+=delta;
      analogWrite(PWM_1[i], actPWM_1[i]);
    } 
    delta=targetPWM_2[i]-actPWM_2[i];
     if (delta!=0) {
      delta=(delta<PWMStep*(-1)?(PWMStep*(-1)):(delta)>PWMStep?(PWMStep):(delta));
      actPWM_2[i]+=delta;
      analogWrite(PWM_2[i], actPWM_2[i]);
    }
  }
  wait(10);
}
