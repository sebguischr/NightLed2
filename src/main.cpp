#include <Arduino.h>
#include <FastLED.h>
#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <EEPROM.h>

#define EEPROM_SIZE 4

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C

#define ledpin 12
#define calibrationpin 27

FASTLED_USING_NAMESPACE
#define DATA_PIN    14
//#define CLK_PIN   4
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    170
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          80  
#define FRAMES_PER_SECOND  120


uint16_t ref=0;

uint16_t ref_off_low=0;
uint16_t ref_off_high=0;

uint16_t ref_on_low=0;
uint16_t ref_on_high=0;
int nbison=0;

bool on=false;
bool touch_on=false;

bool calibration=true;
bool calibration_off=true;

SSD1306AsciiWire oled;

bool ison(){
  uint16_t value=touchRead(T4);
  if( value<=ref_on_high && value>=ref_on_low) return true;
  else return false; 
}

bool isoff(){
  uint16_t value=touchRead(T4);
  if( value<=ref_off_high && value>=ref_off_low) return true;
  else return false; 
}

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(500); } }

void allumageled(){
  for (int n=0;n<NUM_LEDS;n++){
    leds[n]=CRGB::DarkCyan;
    FastLED.show();
    delay(10);            
  }
}

void extinctionled(){
  while((int)leds[0]>0){
    fadeall();
    FastLED.show();
    delay(10);
  }
}

void calibrate();

void setup() {
  Serial.begin(9600);
  delay(1000); // give me time to bring up serial monitor
  Serial.println("NightLed");
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  
  pinMode (ledpin, OUTPUT);
  pinMode (calibrationpin, INPUT);
  Wire.begin();
  Wire.setClock(400000L);

  oled.begin(&SH1106_128x64, I2C_ADDRESS);
  

  oled.setFont(System5x7);
  oled.clear();
  oled.set2X();
  oled.println("NightLed");
  delay(2000);

  EEPROM.begin(EEPROM_SIZE);

  ref_off_low=EEPROM.read(0);
  ref_off_high=EEPROM.read(1);
  ref_on_low=EEPROM.read(2);
  ref_on_high=EEPROM.read(3);

  if(digitalRead(calibrationpin)) calibrate();
  
  FastLED.clear();
  FastLED.show();
  oled.clear();
  oled.set1X();
  oled.println("Valeur lue:");
  //oled.println(value);
  oled.print("Off: ");
  oled.print(ref_off_low);
  oled.print(" - ");
  oled.println(ref_off_high);
  oled.print("On: ");
  oled.print(ref_on_low);
  oled.print(" - ");
  oled.println(ref_on_high);

}


void calibrate(){
  
  
  uint16_t value=touchRead(T4);
  int tour=0;
  
  while(calibration){

    if (calibration_off){
      oled.clear();
      oled.println();
      oled.set1X();
      oled.println("Calibration Off...");        
      Serial.println("Calibration Off");
      for (int n=0;n<20;n++){
          leds[n]=CRGB::White;
      }
      FastLED.show();
      tour=0;
      while(tour<20){
        leds[20-tour]=CRGB::Black;
        FastLED.show();
        value=touchRead(T4);          
        Serial.println(value);
        oled.set2X();
        oled.setCursor(40,4);
        oled.print(value);
        if(tour==0){
          ref_off_low=value;
          ref_off_high=value;
        }          
        if(value<ref_off_low) ref_off_low=value;
        if(value>ref_off_high) ref_off_high=value;
        tour++;
        delay(100);
      }
      oled.clear();
      oled.println();
      oled.set1X();
      oled.println("Fin Calibration Off");
      oled.print("(");
      oled.print(ref_off_low);
      oled.print(",");
      oled.print(ref_off_high);
      oled.println(")");
      Serial.print("Fin Calibration Off(");
      Serial.print(ref_off_low);
      Serial.print(",");
      Serial.print(ref_off_high);
      Serial.println(")");
      calibration_off=false;
      delay(5000);
      
    }
    else{
      
      FastLED.clear();
      for (int n=0;n<20;n++){
        leds[n]=CRGB::Blue;
        
      }
      FastLED.show();
      oled.clear();
      oled.println();
      oled.set1X();
      oled.println("Calibration On");
      Serial.println("Pret pour Calibration on");
      delay(3000);
      tour=0;
      while(tour<20){
        leds[tour]=CRGB::Black;
        FastLED.show();
        value=touchRead(T4);          
        Serial.println(value);
        oled.set2X();
        oled.setCursor(40,4);
        oled.print(value);
        if(tour==0){
          ref_on_low=value;
          ref_on_high=value;
        }          
        if(value<ref_on_low) ref_on_low=value;
        if(value>ref_on_high) ref_on_high=value;
        tour++;
        delay(100);
      }
      oled.clear();
      oled.println();
      oled.set1X();
      oled.println("Fin Calibration On");
      oled.print("(");
      oled.print(ref_on_low);
      oled.print(",");
      oled.print(ref_on_high);
      oled.println(")");

      Serial.print("Fin Calibration On(");
      Serial.print(ref_on_low);
      Serial.print(",");
      Serial.print(ref_on_high);
      Serial.println(")");
      delay(5000);

      if(ref_on_high>=ref_off_low){
        oled.clear();
        oled.println();
        oled.set1X();
        oled.println("Erreur Calibration");
        Serial.println("Erreur Calibration. Nouvelle Calibration"); 
        calibration_off=true;
        ref_off_low=0;
        ref_off_high=0;
        ref_on_low=0;
        ref_on_high=0;
        for (int n=0;n<20;n++){
          leds[n]=CRGB::Red;            
        }
        
        FastLED.show();
      }
      else{
        oled.clear();
        oled.println();
        oled.set1X();
        oled.println("Fin Calibration");
        Serial.println("Fin Calibration"); 
        calibration=false;
        for (int n=0;n<20;n++){
          leds[n]=CRGB::Green;
          
        }
        FastLED.show();

        EEPROM.write(0, ref_off_low);
        EEPROM.write(1, ref_off_high);
        EEPROM.write(2,ref_on_low);
        EEPROM.write(3,ref_on_high);
        EEPROM.commit();
      }
      delay(5000);
      
    }
  }     
      
}

void loop() {
  uint16_t value=touchRead(T4);
  
  
  Serial.print("Valeur lue:");
  Serial.println(value);
  oled.setCursor(66,0);      
  oled.print(value);

  if(touch_on){
    if (isoff()) {
      touch_on=false;
      //delay(1000);
    }
  }
  else{
    
    if (ison()){
      if(nbison>=3){
      
      
          if(on ){
            oled.setCursor(0,5);
            oled.println("Extinction");
            Serial.println("Extinction");
            digitalWrite(ledpin,0);
            extinctionled();
            on=false;  
          }
          else{
            digitalWrite(ledpin,1);
            Serial.println("Allumage");
            oled.setCursor(0,5);
            oled.println("Allumage");
            allumageled();
            on=true;
          }
          //delay(1000);
          touch_on=true;
          oled.setCursor(0,5);
          oled.println("            ");
      }
      else{
        nbison++;
      }
    }
    else{
      nbison=0;
      delay(20);
    }
  }     
    
}



