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
#include <DHT.h> // biblioteka do abslugi higrometru DHT-11
#include "Encoder.h" // Wlasna biblioteka dla enkodera

// Początek ustawień wyświetlacza
#define TFT_CS     53 // linia Chip Select dla wyświetlacza (wybór aktywnego urządzenia Slave SPI)
#define TFT_RST    6  // Wymuszenie resetu wyświetlacza
#define TFT_DC     7 // Linia Data Command
#define SD_CS    8  // Linia Chip Select dla czytnika kart SD
#define BUFFPIXEL 20 // bufor pikseli dla obrazka splash
#define DHT11_PIN 32 // pin higrometru DHT-11

// Inicjowanie wyswietlacza tft tak by uzywal sprzetowych pinow SPI
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

//ustawienia cycluino
byte speedPin = 3;
byte cadencePin = 2;
volatile word steps;// slowo kluczowe volatile wymusza uzycie zmiennej z pamieci zamiast kopii z rejestru
volatile word rotations; // ilosc obrotow korbowodu
volatile unsigned long speedTimes[2]; // tablica do przechowywania ostatnich dwoch czasow wystapienia przerwania z czujnika predkosci
volatile unsigned long cadenceTimes[2]; // tablica do przechowywania ostatnich dwoch czasow wystapienia przerwania z czujnika kadencji
unsigned long wheelRotationInterval;
unsigned long lastTimeSpeed; //
unsigned long lastTimeSpeedCalc;
unsigned long lastTimeCadence;
unsigned long lastTimeCadenceCalc;
const unsigned long circ = 2073; //dystans w mm jaki pokonuje kolo w 1 obrocie zwiazane ze srednica kola. UWAGA zrobic funkcje przeliczajaca z cali w momencie uruchomienia programu
const unsigned long hourMs = 3600000; // ilosc milisekund w godzinie
unsigned long speedFactor = 0; // 
volatile unsigned long speed = 0; 
volatile unsigned int cadenceFull = 0;
volatile unsigned int cadenceFraction = 0;
String cadence;

unsigned int speed_last = 1;
unsigned int cadence_last = 1;

// parametry odswiezania ekranu
const unsigned long screenRefreshInterval = 1000; // odswiezanie ekranu interwal w milisekundach
unsigned long screenRefreshLast = 0; // moment ostatniego odswiezenia ekranu
volatile int screenNo = 0; // domyslny numer ekranu
int screenNoLast = 0; // numer ostatnio odswiezanego ekranu

// odswiezanie sensorow
const unsigned long sensorsRefreshInterval = 1000; // w milisekundach
unsigned long sensorsRefreshLast = 0;

// dane srodowiskowe z sensorow
float temp1 = 0.0; // temperatura z barometru
float temp2 = 0.0; // temperatura z zegara ds3231
float temp3 = 0.0; // temperatura z DHT-11 w C
float atmPressure = 0.0; // cisnienie atmosferyczne
float altitude = 0.0; // wysokosc npm
float humidity = 0.0; // wilgotnosc %
int analogVoltage = 0; // napiecie na baterii
float voltage = 0.0;
float headingDegrees = 0.0; // odchylenie osi kompasu od polnocy w katach
int analogCharging = 0; // napiecie ladowania

// ustawienie pinów enkodera
const byte pushButton = 30; // przycisk
const byte dt = 28; // kanal 1
const byte clk = 26; // kanal 2

// ustawienie pinów dla bluetooth xm15
const byte btOnOffPin = 37; // wlaczenie/wylaczenie modulu
const byte btStatePin = 35; // odczyt stanu modulu 
byte btOnOffLast; 

// string zawierający dane do wysłania przez Bluetooth
String btMessage;

// Inicjalizuj Enkoder
Encoder en1(dt, clk, pushButton, 6);

// Definiuj czujnik cisnienia i temperatury
Adafruit_BMP280 bme;

// Definiuj magnetometr
HMC5883L compass;

// Definiuj higrometr
DHT dht;

// Zegar i czas
DS3231 clock;
RTCDateTime clock_dt;
String timeLast;

// dla diagnostyki
bool ReportToSerial = false;

void setup() {    
  // ustawienie pinow dla czujnikow halla
  // predkosc
  pinMode(speedPin, INPUT_PULLUP); // przerwanie int.1 na pinie 3 
  attachInterrupt(1, onSpeed, RISING);//przerwanie wywolane przy wzroscie sygnalu
  speedFactor = circ*(hourMs / 1000); //wstępne obliczenia

  // kadencja
  pinMode(cadencePin, INPUT_PULLUP); // ustaw rezystor podciagajacy na pinie 2, przerwanie na tym pinie to int.0
  attachInterrupt(0, onCadence, RISING); 
  
  // Port serial 2 dla komunikacji z modulem bluetooth
  Serial2.begin(9600);
  
  // ustaw pin stanu bluetooth
  pinMode(btStatePin, INPUT);
  pinMode(btOnOffPin, OUTPUT);
  digitalWrite(btOnOffPin, HIGH);
  btOnOffLast = LOW;
  
  // Uruchom port szeregowy dla diagnostyki
  Serial.begin(9600);  
  
  // Inicjalizacja DS2131 (zegar)
  clock.begin();

  // Uruchom czujnik cisnienia i temperatury 
  if (!bme.begin())
  {  
    if (ReportToSerial) Serial.println("Nie mozna znalezc sensora bmp280, sprawdz polaczenia!");
    while (1);
  }
  // Uruchom higrometr
  dht.setup(DHT11_PIN);
  
  // Inicjalizacja HMC5883L
  if (ReportToSerial) Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    if (ReportToSerial) Serial.println("Nie odnaleziono HMC5883L, sprawdz polaczenie!");
    delay(500);
  }
  // Ustawienie zakresu pomiarowego magnetometru
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
  tft.initR(INITR_BLACKTAB);   
  tft.setRotation(2); // obrot orientacji ekranu o 180 stopni dla uzyskania trybu portretowego

  if (ReportToSerial) Serial.println("TFT Initialized");

  tft.fillScreen(ST7735_BLACK); //ustaw tlo wyswietlacza na czarne
    // start karty SD
  if (ReportToSerial) Serial.print("Inicjalizacja karty SD...");
  if (!SD.begin(SD_CS)) {
    if (ReportToSerial) Serial.println("Inicjalizacja karty SD sie nie powiodla.");
    return;
  }
  Serial.println("OK!");

  lastTimeSpeedCalc = lastTimeCadenceCalc = millis(); 
  // pokaz splash screen po wlaczeniu zasilania
  bmpDraw("splash.bmp", 0, 0);
  // wstrzymaj program na 3 sekundy
  delay(3000);
  tft.fillScreen(ST7735_BLACK); 
}

void loop() { 
  unsigned long currentMillis = millis();
  // odswiez predkosc 
  if (((millis() - speedTimes[0]) < 4000) && (currentMillis - lastTimeSpeedCalc) > 100) //jesli odswiezano mniej niz 2 sek temu odswiez po raz kolejny, ale nie czesciej niz 0.1sekundy
  {
    lastTimeSpeedCalc = currentMillis;   
    wheelRotationInterval = speedTimes[0] - speedTimes[1];
    speed = speedFactor/wheelRotationInterval;    
  }
  else 
  {
    if (millis() - speedTimes[0] > 4000)
    {
      speed = 0;
    }
  } 

  //obliczenia kadencji
  if ((millis() - cadenceTimes[0] < 3500) && ((currentMillis - lastTimeCadenceCalc) > 100 )) // ostatnie przejscie przez czujnik nie pozniej niz 3.5 sekundy wczesniej
  {
    lastTimeCadenceCalc = currentMillis;
    cadenceFull = 60000 / (cadenceTimes[0] - cadenceTimes[1]);
    cadenceFraction = 60000 % (cadenceTimes[0] - cadenceTimes[1]);
    cadence = String(cadenceFull) + "." + String(cadenceFraction/1000);
  }
  else 
  {
    if (millis() - cadenceTimes[0] > 3500)
    {
      cadenceFull = cadenceFraction = 0;
      cadence = "0.0";
    }
  }

  // odswiez dane z czujnikow i przycisku
  unsigned long sensorsCurrentMillis = millis();
  if((unsigned long)(sensorsCurrentMillis - sensorsRefreshLast) >= sensorsRefreshInterval)
  {
    sensorsRefreshLast = sensorsCurrentMillis;
    temp1 = bme.readTemperature();
    temp2 = clock.readTemperature();
    temp3 = dht.getTemperature();
    atmPressure = bme.readPressure();
    altitude = bme.readAltitude(1018); 
    humidity = dht.getHumidity();
    analogVoltage = analogRead(A0);
    analogCharging = analogRead(A1);
    
    voltage = analogVoltage * 0.004483; // kompensacja dla podlaczonej diody
    clock_dt = clock.getDateTime();
    headingDegrees = compassHeading(compass);

    if (en1.IsButtonPressed() && screenNo == 4) // jesli jestesmy na ekranie bluetooth i przycisk wcisniety zmien wlacz/wylacz modul 
    {
      if (ReportToSerial) Serial.println("BluetoothSwitch");
      switchBluetooth();
    }
    
    if (HIGH == digitalRead(btOnOffPin))
    {
      btMessage = 
      String(temp1) + " C," + 
      String(temp2) + " C," + 
      String(temp3) + " C," +
      String(atmPressure/100) + " hPa," + 
      String(cadenceFull) + " rpm," +
      String(speed/1000) + " km/h," + 
      String(atmPressure/100) + " hPa" +
      String(headingDegrees) + " head;";
    }
  }
  
  // sprawdzenie pozycji enkodera
  if (en1.AskPosChange())
  {
    if (ReportToSerial) Serial.println(en1.GetPosition());
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




