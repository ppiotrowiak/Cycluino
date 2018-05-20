/*
 * Generator impulsów do testowania Cycluino
 * Część pracy inżynierskiej
 * Politechnika Warszawska
 * Informatyka Stosowana
 * Wydział Elektryczny
 * Autor: Przemyslaw Piotrowiak
 */
#include "Encoder.h" // Wlasna biblioteka dla enkodera
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*setting up display */
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

/* sprawdzenie ustawien wyswietlacza i biblioteki */
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

 byte cadencePin = 8 ; 
 byte speedPin = 10; 
 bool cadencePinState = false;
 bool speedPinState = false;

 unsigned long unitDistance = 2073; // dystans jednego obrotu kola
 
 volatile bool speedSelected = false;
 volatile bool modeChanged = true;
 
 unsigned long lastTimeCadence;
 unsigned int cadenceInterval;
 unsigned long currentCadenceMillis;

 unsigned long lastTimeSpeed;
 unsigned long speedInterval;
 unsigned long currentSpeedMillis;

 // ustawienie pinów enkodera
const byte pushButton = 7; // przycisk
const byte dt = 5; // kanal 1
const byte clk = 6; // kanal 2

// utworz instancje enkodera
Encoder enCadence(dt, clk, pushButton, 240);
Encoder enSpeed(dt, clk, pushButton, 400);

void setup() {
  pinMode(cadencePin, OUTPUT);
  pinMode(speedPin, OUTPUT);
  cadenceInterval = 1000;  
  speedInterval = 500;  
  lastTimeCadence = 0;
  lastTimeSpeed = 0;

  //ustawienie przerwan dla enkodera
  pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(7), onButtonLow, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64
  display.clearDisplay();
  display.display();
}

void loop() {  
  currentCadenceMillis = millis();
  if ((unsigned long)(currentCadenceMillis - lastTimeCadence) >= cadenceInterval)
  {
    lastTimeCadence = currentCadenceMillis;
    digitalWrite(cadencePin, cadencePinState);  
    cadencePinState = !cadencePinState;
  }
  currentSpeedMillis = millis();
  if ((unsigned long)(currentSpeedMillis - lastTimeSpeed) >= speedInterval)
  {
    lastTimeSpeed = currentSpeedMillis;
    digitalWrite(speedPin, speedPinState);  
    speedPinState = !speedPinState;
  }

  // rysowanie podkreslenia
  if (modeChanged) drawUnderlines();

  // sprawdzenie pozycji enkodera i aktualizacja interwałów
  if (speedSelected)
  { 
    enSpeed.HasRotated();          
    if (enSpeed.AskPosChange())
    {   
      int enkoderPosition = enSpeed.GetPosition(); 
      enSpeed.AckPosChange();   
      if (enkoderPosition < 1) {enkoderPosition = 1;}
      speedInterval = (60000 / enkoderPosition) / 2;
      display.setCursor(0,10);
      display.setTextColor(WHITE, BLACK);
      display.setTextSize(2);
      display.print("       ");
      display.setCursor(0,10);
      display.print(enkoderPosition);
      display.print("(");
      display.print((enkoderPosition * unitDistance * 60) / 1000);      // metrow na godzine
      display.print(")");
      display.display();         
    };
  }
  else
  {
    enCadence.HasRotated();
    if (enCadence.AskPosChange()) updateCadence();
  }
  delay(1);
}

void updateCadence()
{      
  int enkoderPosition = enCadence.GetPosition(); 
  enCadence.AckPosChange();   
  if (enkoderPosition < 1) {enkoderPosition = 1;}
  cadenceInterval = (60000 / enkoderPosition) / 2;
  
  display.setCursor(0,40);
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(2);
  display.print("       ");
  display.setCursor(0,40);
  display.print(enkoderPosition);
  display.print("(");
  display.print(cadenceInterval*2);      
  display.print(")");
  display.display();                   
};


void onButtonLow() {        
  speedSelected = !speedSelected;
  modeChanged = true;
}

void drawUnderlines()
{
  if (speedSelected)
  {
    display.drawFastHLine(0, 30, 128, WHITE);
    display.drawFastHLine(0, 62, 128, BLACK);
    display.display();
  }
  else
  {
    display.drawFastHLine(0, 30, 128, BLACK);
    display.drawFastHLine(0, 62, 128, WHITE);
    display.display();
  }
  modeChanged = false;
}

