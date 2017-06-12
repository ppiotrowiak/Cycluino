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
//#include <Adafruit_HMC5883_U.h> // Biblioteka do obsługi magnetometru
#include <HMC5883L.h> // 
#include <DS3231.h> // https://github.com/jarzebski/Arduino-DS3231 do obsługi zegara 
#include "Encoder.h" // Wlasna biblioteka dla enkodera

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

//settings for the display refresh
const unsigned long screenRefreshInterval = 1000; // odswiezanie ekrany interwal w milisekundach
unsigned long screenRefreshLast = 0;
volatile int screenNo = 0;

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


// ustawienie pinów enkodera
const byte pushButton = 30;
const byte dt = 28;
const byte clk = 26;

// ustawienie pinów dla bluetooth xm15
const byte btOnOffPin = 37;
const byte btStatePin = 35;

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

 
void setup() {  
  // ustawienie pinow dla czujnikow halla/kontaktronow
  // predkosc
  pinMode(19, INPUT_PULLUP); // przerwanie int.4 na pinie 19 
  attachInterrupt(4, onSpeed, RISING);//przerwanie wywolane przy wzroscie sygnalu
  speedFactor = circ*hourMs/distFact; //wstępne obliczenia

  // kadencja
  pinMode(2, INPUT_PULLUP); // ustaw rezystor podciagajacy na pinie 2, przerwanie na tym pinie to int.0
  attachInterrupt(0, onCadence, RISING); 
  
  // Port serial 3 dla komunikacji z modulem bluetooth
  Serial2.begin(9600);
  
  // ustaw pin stanu bluetooth
  pinMode(btStatePin, INPUT);
  pinMode(btOnOffPin, OUTPUT);
  digitalWrite(btOnOffPin, HIGH);
  
  // Uruchom port szeregowy
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
  clock.setDateTime(__DATE__,__TIME__); 

  // Inicjalizacja wyswietlacza
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setRotation(2); // obrot orientacji ekranu o 180 stopni

  Serial.println("TFT Initialized");

  tft.fillScreen(ST7735_BLACK); //ustaw tlo na czarne
  showSplash(ST7735_WHITE);
  
  // Uruchom czujnik cisnienia i temperatury 
  if (!bme.begin())
  {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
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

  //calculation of cadence
  if (millis() - cadenceTimes[0] < 3500) // ostatnie przejscie przez czujnik nie pozniej niz 3.5 sekundy wczesniej
  {
    cadence = 60000 / (cadenceTimes[0] - cadenceTimes[1]);
  }
  else 
  {
    cadence = 0;
  }

  // odswiez dane z czujnikow
  unsigned long sensorsCurrentMillis = millis();
  if((unsigned long)(sensorsCurrentMillis - sensorsRefreshLast) >= sensorsRefreshInterval)
  {
    temp1 = bme.readTemperature();
    temp2 = clock.readTemperature();
    atmPressure = bme.readPressure();
    altitude = bme.readAltitude(1008);
    analogVoltage = analogRead(A0);
    voltage = analogVoltage * 0.004483; // kompensacja dla podlaczonej diody
    clock_dt = clock.getDateTime();
    headingDegrees = compassHeading(compass);

    // testowo wyslij dane 
    Serial2.print(temp1);
    Serial2.print(",");
    Serial2.print(temp2);
    Serial2.print(",");
    Serial2.print(atmPressure/100);
    Serial2.print(" hPa");
    Serial2.print(",");
    Serial2.print(voltage);
    Serial2.print(" V");
    Serial2.print(",");
    Serial2.print(speed / 1000);
    Serial2.print(" km/h");
    Serial2.print(headingDegrees);
    Serial2.print("deg");
    Serial2.print(";");
    
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
  
  delay(1);//for stability
}

void screenRefresh(int screen)
{  
    if (screen < 0) {screen = screen*(-1);}

  Serial.print("Display refresh: ");
  Serial.println(screen);   
  
  switch (screen)
  {
    case 0:
      showScreen_0();
      break;    
    case 1:
      showScreen_1();
      break;
      
  }
  
}

void showSplash(uint16_t color)
{
    tft.setCursor(0, 0);
    tft.setTextColor(color);
    tft.setTextWrap(true);
    tft.print("Starting Cycluino");
}

void showScreen_0()
{
  showStatusBar();
  
  tft.setCursor(0, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextWrap(true);
  tft.print("test");
}

void showScreen_1()
{
  showStatusBar();
  tft.setCursor(0, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextWrap(true);
  tft.println(screenNo);
  tft.print("temp1: ");
  tft.println(temp1);
  tft.print("temp2: ");
  tft.println(temp2);
  tft.print("pressure: ");
  tft.println(atmPressure);
  tft.print("altitude: ");
  tft.println(altitude);
  tft.print("Voltage :");
  tft.println(analogVoltage);
  tft.print("voltage: ");
  tft.println(voltage);
  tft.print("Bluetooth pin: ");
  tft.println(digitalRead(btStatePin));
  tft.print("Date: ");
  tft.println(clock.dateFormat("d-m-Y", clock_dt));
  tft.print("Time: ");
  tft.println(clock.dateFormat("H:i:s", clock_dt));
  tft.print("HeadingDeg: ");
  tft.println(headingDegrees);
  tft.print("Speed: ");
  tft.println(speed);
  tft.print("Cadence: ");
  tft.println(cadence);  
}

void showStatusBar()
{
  tft.fillScreen(ST7735_BLACK);  
  tft.setCursor(0,0);
  tft.setTextColor(ST7735_WHITE);
  tft.println(clock.dateFormat("H:i:s", clock_dt));
}
void showTemperature(uint16_t color)
{
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0,0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.println("Temperatura (bmp): ");
  tft.print(bme.readTemperature());  
}

void showPressure(uint16_t color)
{
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(0,10);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.println("Cisnienie (bmp): ");
  tft.print(bme.readPressure() / 100); // 100 Pa = 1 millibar
}   

float compassHeading(HMC5883L compass)
{
  // Pobranie wektorow znormalizowanych
  Vector norm = compass.readNormalize();

  // Obliczenie kierunku (rad)
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Ustawienie kate deklinacji dla Sosnowca 5'1E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  //float declinationAngle = (5.0 + (1.0 / 60.0)) / (180 /M_PI);
  float declinationAngle = 0.08756;

  // Korekta katow
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Zamiana radianow na stopnie
  float headingDegrees = heading * 180/M_PI;

  return headingDegrees;
}

void onSpeed()
{
  //static unsigned long lastTime;
  unsigned long timeNow = millis(); //unsigned long 32 bits  range from 0 to 4,294,967,295 (2^32 - 1)
  if (timeNow - speedTimes[0] < 200) // eliminacja zaklocen
    return;
    
    speedTimes[1] = speedTimes[0];
    speedTimes[0] = timeNow;   
    
    steps++;      
}

void onCadence()
{
  unsigned long timeNow = millis();
  if (timeNow - cadenceTimes[0] < 200) //debouncing
  return;

  cadenceTimes[1] = cadenceTimes[0];
  cadenceTimes[0] = timeNow;

  rotations++;  
}


