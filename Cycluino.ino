/*
 * Cycluino - Komputer rowerowy w oparciu o mikrokontroler Arduino - Praca inżynierska
 * Politechnika Warszawska
 * Informatyka Stosowana
 * Wydział Elektryczny
 * Autor: Przemyslaw Piotrowiak
 * Wersja dla Creoqode Mini Mega (kompatybilne z Arduino Mega2560)
 */
 
//Dolacz konieczne biblioteki
#include <avr/pgmspace.h> // biblioteka pozwalajaca przechowywac zmienne w pamieci Flash
#include <Wire.h> // Komunikacja przez protokół I2C / TWI
#include <SPI.h> // Do komunikacji z urządzeniami przez interfejs SPI (Serial Peripheral Interface)
#include <OneWire.h> // Biblioteka do obsługi interfejsu OneWire
#include <Adafruit_GFX.h>  // Biblioteka graficzna (Adafruit)
#include <Adafruit_ST7735.h> // Biblioteka dla kontrolera ekranu TFT stworzona przez Adafruit
#include <Adafruit_Sensor.h> // Biblioteka dla obsługi sensorów stworzona przez Adafruit
#include <Adafruit_BMP280.h> // Biblioteka do obsługi sensora ciśnienia atmosferycznego i temperatury
#include <SD.h> // Biblioteka do obsługi karty SD
#include <HMC5883L.h> // // biblioteka do obsługi magnetometru
#include <DS3231.h> // https://github.com/jarzebski/Arduino-DS3231 do obsługi zegara 
#include <DS18B20.h> // Biblioteka do obsługi termometru Dallas Semiconductor
//#include <Adafruit_HMC5883_U.h> // Biblioteka do obsługi magnetometru
#include "Encoder.h" // Wlasna biblioteka dla enkodera

// Początek ustawień wyświetlacza
#define TFT_CS     53 // linia Chip Select dla wyświetlacza (wybór aktywnego urządzenia Slave SPI)
#define TFT_RST    6  // Wymuszenie resetu wyświetlacza
#define TFT_DC     7 // Linia Data Command
#define SD_CS    8  // Linia Chip Select dla czytnika kart SD

#define BUFFPIXEL 20 // bufor pikseli dla ?

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//ustawienia cycluino
volatile word steps;//load the variable from RAM and not from a storage register
volatile word rotations; // number of times the pedal shaft rotates
volatile unsigned long speedTimes[2]; // tablica do przechowywania ostatnich dwoch czasow wystapienia przerwania
volatile unsigned long cadenceTimes[2];
unsigned long wheelRotationInterval;
unsigned long lastTimeSpeed; //
unsigned long lastTimeCadence;

const unsigned int circ = 2073; //dystans w mm jaki pokonuje kolo w 1 obrocie zwiazane ze srednica kola. UWAGA zrobic funkcje przeliczajaca z cali w momencie uruchomienia programu
const unsigned long distFact = 1000;
const unsigned long hourMs = 3600000; // ilosc milisekund w godzinie
unsigned long speedFactor = 0; // ?
volatile unsigned int speed = 0; //max value 65,535 (2^16) - 1)
volatile unsigned int cadence = 0;

unsigned int speed_last = 1;
unsigned int cadence_last = 1;

// parametry odswiezania ekranu
const unsigned long screenRefreshInterval = 1000; // odswiezanie ekrany interwal w milisekundach
unsigned long screenRefreshLast = 0;
volatile int screenNo = 0; // domyslny numer ekranu
int screenNoLast = 0;

// odswiezanie sensorow
const unsigned long sensorsRefreshInterval = 1000; // w milisekundach
unsigned long sensorsRefreshLast = 0;

// dane srodowiskowe z sensorow
float temp1 = 0.0; // temperatura z barometru
float temp2 = 0.0; // temperatura z zegara ds3231
float atmPressure = 0.0;
float altitude = 0.0;
int analogVoltage = 0;
float voltage = 0.0;
float headingDegrees = 0.0;
int analogCharging = 0;


// ustawienie pinów enkodera
const byte pushButton = 30;
const byte dt = 28;
const byte clk = 26;

// ustawienie pinów dla bluetooth xm15
const byte btOnOffPin = 37;
const byte btStatePin = 35;
byte btOnOffLast;

// string zawierający dane do wysłania przez Bluetooth
String btMessage;

// Inicjalizuj Enkoder
Encoder en1(dt, clk, pushButton, 5);

// Definiuj i inicjuj czujnik cisnienia i temperatury
Adafruit_BMP280 bme;

// Definiuj i inicjalizuj magnetometr
HMC5883L compass;
//Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Zegar i czas
DS3231 clock;
RTCDateTime clock_dt;
String timeLast;

void setup() {    
  // ustawienie pinow dla czujnikow halla/kontaktronow
  // predkosc
  pinMode(19, INPUT_PULLUP); // przerwanie int.4 na pinie 19 
  attachInterrupt(4, onSpeed, RISING);//przerwanie wywolane przy wzroscie sygnalu
  speedFactor = circ*hourMs/distFact; //wstępne obliczenia

  // kadencja
  pinMode(2, INPUT_PULLUP); // ustaw rezystor podciagajacy na pinie 2, przerwanie na tym pinie to int.0
  attachInterrupt(0, onCadence, RISING); 
  
  // Port serial 2 dla komunikacji z modulem bluetooth
  Serial2.begin(9600);
  
  // ustaw pin stanu bluetooth
  pinMode(btStatePin, INPUT);
  pinMode(btOnOffPin, OUTPUT);
  digitalWrite(btOnOffPin, HIGH);
  //digitalWrite(btOnOffPin, LOW);
  btOnOffLast = LOW;
  
  // Uruchom port szeregowy dla diagnostyki
  Serial.begin(9600);
  
  // Inicjalizacja DS2131
  clock.begin();

  // Inicjalizacja HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Nie odnaleziono HMC5883L, sprawdz polaczenie!");
    delay(500);
  }
  // Ustawienie zakresu pomiarowego
  compass.setRange(HMC5883L_RANGE_1_3GA);
  // Ciagly pomiar: HMC5883L_CONTINOUS (domyslny)
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  // Ustawienie czestotliwosci pomiarow
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  // Liczba usrednionych probek
  compass.setSamples(HMC5883L_SAMPLES_4);

  // Ustawiamy date i czas z kompilacji szkicu
  //clock.setDateTime(__DATE__,__TIME__); 

  // Inicjalizacja wyswietlacza
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setRotation(2); // obrot orientacji ekranu o 180 stopni

  Serial.println("TFT Initialized");

  tft.fillScreen(ST7735_BLACK); //ustaw tlo na czarne
    // start karty SD
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("failed!");
    return;
  }
  Serial.println("OK!");

  // pokaz splash screen po wlaczeniu zasilania
  bmpDraw("splash.bmp", 0, 0);
  // wait 3 seconds
  delay(3000);
  tft.fillScreen(ST7735_BLACK); 
  
  // Uruchom czujnik cisnienia i temperatury 
  if (!bme.begin())
  {  
    Serial.println("Nie mozna znalezc sensora bmp280, sprawdz polaczenia!");
    while (1);
  }
}

void loop() {
 
  unsigned long currentMillis = millis();
  // odswiez predkosc 
  if ((millis() - speedTimes[0]) < 2000)
  {
    wheelRotationInterval = speedTimes[0] - speedTimes[1];
    speed = speedFactor/wheelRotationInterval;
  }
  else 
  {
    speed = 0;
  } 

  //obliczenia kadencji
  if (millis() - cadenceTimes[0] < 3500) // ostatnie przejscie przez czujnik nie pozniej niz 3.5 sekundy wczesniej
  {
    cadence = 60000 / (cadenceTimes[0] - cadenceTimes[1]);
  }
  else 
  {
    cadence = 0;
  }

  // odswiez dane z czujnikow i przycisku
  unsigned long sensorsCurrentMillis = millis();
  if((unsigned long)(sensorsCurrentMillis - sensorsRefreshLast) >= sensorsRefreshInterval)
  {
    sensorsRefreshLast = sensorsCurrentMillis;
    temp1 = bme.readTemperature();
    temp2 = clock.readTemperature();
    atmPressure = bme.readPressure();
    altitude = bme.readAltitude(1008);
    analogVoltage = analogRead(A0);
    analogCharging = analogRead(A1);
    
    voltage = analogVoltage * 0.004483; // kompensacja dla podlaczonej diody
    clock_dt = clock.getDateTime();
    headingDegrees = compassHeading(compass);

    if (en1.IsButtonPressed() && screenNo == 2)
    {
      Serial.println("BluetoothSwitch");
      switchBluetooth();
    }

    
    if (HIGH == digitalRead(btOnOffPin))
    {
    btMessage = String(temp1) + "," + String(temp2) + "," + String(atmPressure/100) + "," + String(voltage) + "," + String(speed/1000) + 
        "," + String(headingDegrees) + ";";
      Serial2.print(btMessage);
      // testowo wyslij dane 
    }
  }
  
  // sprawdzenie pozycji enkodera
  if (en1.AskPosChange())
  {
    Serial.println(en1.GetPosition());
    screenNo = en1.GetPosition();
    en1.AckPosChange();   
    screenRefresh(screenNo);
  };
  en1.HasRotated(); // enkoder sprawdza czy zostal obrocony
  
  
  //odswiezanie ekranu
  unsigned long screenCurrentMillis = millis();
  if((unsigned long)(screenCurrentMillis - screenRefreshLast) >= screenRefreshInterval)
  {
    screenRefreshLast = screenCurrentMillis;
    screenRefresh(screenNo);
  }
  
  delay(1);// male opoznienie dla poprawy stabilnosci systemu
}




