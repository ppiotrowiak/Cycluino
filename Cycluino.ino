/*
 * Cycluino - Komputer rowerowy w oparciu o mikrokontroler Arduino - Praca inżynierska
 * Politechnika Warszawska
 * Informatyka Stosowana
 * Wydział Elektryczny
 * Autor: Przemyslaw Piotrowiak
 * Wersja dla Arduino Mega2560
 */
 
//Dolacz konieczne biblioteki
#include <Wire.h> // Komunikacja przez protokół I2C / TWI
#include <SPI.h> // Do komunikacji z urządzeniami przez interfejs SPI (Serial Peripheral Interface)
#include <Adafruit_GFX.h>  // Biblioteka graficzna stworzona przez Adafruit
#include <Adafruit_ST7735.h> // Biblioteka dla kontrolera ekranu TFT stworzona przez Adafruit
#include <SD.h> // Biblioteka do obsługi karty SD
#include <Adafruit_Sensor.h> // Biblioteka dla obsługi sensorów stworzona przez Adafruit
#include <Adafruit_BMP280.h> // Biblioteka do obsługi sensora ciśnienia atmosferycznego i temperatury
#include <OneWire.h> // Biblioteka do obsługi interfejsu OneWire
#include <DS18B20.h> // Biblioteka do obsługi termometru Dallas Semiconductor
#include <HMC5883L.h> // Biblioteka do obsługi magnetometru

#include "Encoder.h" // Wlasna biblioteka dla enkodera

// Definiuj i inicjuj czujnik cisnienia i temperatury
Adafruit_BMP280 bme;

// Początek ustawień wyświetlacza
// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     53
#define TFT_RST    6  // you can also connect this to the Arduino reset
                      // in which case, set this #define pin to 0!
#define TFT_DC     7 // Data command for tft

#define SD_CS    8  // Chip select line for SD card

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//ustawienia cycluino
volatile word steps;//load the variable from RAM and not from a storage register
volatile unsigned long speedTimes[2]; //load the variable from RAM and not from a storage register
volatile unsigned long cadenceTimes[2];
unsigned long wheelRotationInterval;
unsigned long lastTime;
unsigned long lastTimeCadence;

const unsigned int circ = 2073; //dystans w mm jaki pokonuje kolo w 1 obrocie zwiazane ze srednica kola. UWAGA zrobic funkcje przeliczajaca z cali w momencie uruchomienia programu
const unsigned long distFact = 1000;
const unsigned long hour = 3600000;
unsigned long speedFactor = 0;
volatile unsigned int speed = 0; //max value 65,535 (2^16) - 1)
volatile unsigned int cadence = 0;

//settings for the display refresh
unsigned long screenRefreshInterval = 1000; //screen
unsigned long screenRefreshLast = 0;
int screenNo = 0;

settings for the encoder
byte pushButton = 44;
byte dt = 42;
byte clk = 40;
 
void setup() {
  // Uruchom port szeregowy
  Serial.begin(9600);
  Serial.print("Hello! ST7735 TFT Test");

  // Inicjalizacja wyswietlacza
  // Use this initializer if you're using a 1.8" TFT
  //tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  

  Serial.println("Initialized");

  //tft.fillScreen(ST7735_BLACK);
  //showSplash(ST7735_WHITE);

  
  // Uruchom czujnik cisnienia i temperatury 
  if (!bme.begin())
  {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}

void loop() {

// kod testowy sensorow
// barometr-termometr bmp 280
    Serial.print("---- GY BMP 280 ----------------\n");
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100); // 100 Pa = 1 millibar
    Serial.println(" mb");
    Serial.print("Approx altitude = ");
    Serial.print(bme.readAltitude(997.25));
    Serial.println(" m");
    Serial.print("--------------------------------\n\n");


delay(2000);
}

void showSplash(uint16_t color)
{
    tft.setCursor(0, 0);
    tft.setTextColor(color);
    tft.setTextWrap(true);
    tft.print("Splash screen");
}


