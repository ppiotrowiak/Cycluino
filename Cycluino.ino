/*
 * Cycluino - Komputer rowerowy w oparciu o mikrokontroler Arduino - Praca inżynierska
 * Politechnika Warszawska
 * Informatyka Stosowana
 * Wydział Elektryczny
 * Autor: Przemyslaw Piotrowiak
 * Wersja dla Arduino Mega2560
 */
 
//Dolacz konieczne biblioteki
#include <avr/pgmspace.h> // biblioteka pozwalajaca przechowywac zmienne w pamieci Flash
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

#define BUFFPIXEL 20 // bufor pikseli dla 

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

// bitmapy dla wyswietlacza
const unsigned char btLogo [] PROGMEM = {
  // 'bluetooth'
  B00111000, 
  B00111100, 
  B00110110, 
  B00110011, 
  B10110001, 
  B11110011, 
  B00110110, 
  B00111100, 
  B00110110, 
  B11110011, 
  B10110001, 
  B00110011, 
  B00110110,
  B00111100, 
  B00111000  
};

const unsigned char bat0perc [] PROGMEM = {
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B11111100, B00000000, B00000000, B00000011,
  B11111100, B00000000, B00000000, B00000011,
  B11000000, B00000000, B00000000, B00000011,
  B11000000, B00000000, B00000000, B00000011,
  B11000000, B00000000, B00000000, B00000011,
  B11000000, B00000000, B00000000, B00000011,
  B11111100, B00000000, B00000000, B00000011,
  B11111100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111
};

const unsigned char bat25perc [] PROGMEM = {
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B11111100, B00000000, B00000001, B11110011,
  B11111100, B00000000, B00000001, B11110011,
  B11000000, B00000000, B00000001, B11110011,
  B11000000, B00000000, B00000001, B11110011,
  B11000000, B00000000, B00000001, B11110011,
  B11000000, B00000000, B00000001, B11110011,
  B11111100, B00000000, B00000001, B11110011,
  B11111100, B00000000, B00000001, B11110011,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111
};

const unsigned char bat50perc [] PROGMEM = {
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B11111100, B00000000, B00111111, B11110011,
  B11111100, B00000000, B00111111, B11110011,
  B11000000, B00000000, B00111111, B11110011,
  B11000000, B00000000, B00111111, B11110011,
  B11000000, B00000000, B00111111, B11110011,
  B11000000, B00000000, B00111111, B11110011,
  B11111100, B00000000, B00111111, B11110011,
  B11111100, B00000000, B00111111, B11110011,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111
};

const unsigned char bat75perc [] PROGMEM = {
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B11111100, B00000111, B11111111, B11110011,
  B11111100, B00000111, B11111111, B11110011,
  B11000000, B00000111, B11111111, B11110011,
  B11000000, B00000111, B11111111, B11110011,
  B11000000, B00000111, B11111111, B11110011,
  B11000000, B00000111, B11111111, B11110011,
  B11111100, B00000111, B11111111, B11110011,
  B11111100, B00000111, B11111111, B11110011,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111
};

const unsigned char bat100perc [] PROGMEM = {
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B11111100, B11111111, B11111111, B11110011,
  B11111100, B11111111, B11111111, B11110011,
  B11000000, B11111111, B11111111, B11110011,
  B11000000, B11111111, B11111111, B11110011,
  B11000000, B11111111, B11111111, B11110011,
  B11000000, B11111111, B11111111, B11110011,
  B11111100, B11111111, B11111111, B11110011,
  B11111100, B11111111, B11111111, B11110011,
  B00001100, B00000000, B00000000, B00000011,
  B00001100, B00000000, B00000000, B00000011,
  B00001111, B11111111, B11111111, B11111111,
  B00001111, B11111111, B11111111, B11111111
};

const unsigned char batCharging [] PROGMEM = {
  B00000011, B00000000, B11100000, B00000000,
  B00000011, B10000000, B11110000, B00000000,
  B00000011, B11000000, B11111000, B00000000,
  B00000011, B11100000, B11111100, B00000000,
  B00000001, B11110000, B11111110, B00000000,
  B00000000, B01111000, B11111111, B00000000,
  B00000000, B00111111, B11000111, B10000000,
  B00000000, B00011111, B11000011, B11000000,
  B00000000, B00001111, B11000001, B11100000,
  B00000000, B00000111, B11000000, B11110000,
  B00000000, B00000011, B11000000, B01111000,
  B00000000, B00000001, B11000000, B00011100
};

const unsigned char bmHeart [] PROGMEM = {
  B00001111, B11000000, B11111100, B00000000, //1
  B00011111, B11100001, B11111110, B00000000, //2
  B00111111, B11110011, B11111111, B00000000, //3
  B01111111, B11110011, B11111111, B10000000, //4
  B11111111, B11111111, B11111111, B11000000, //5
  B11111111, B11111111, B11111111, B11000000, //6
  B11111111, B11111111, B11111111, B11000000, //7
  B11111111, B11111111, B11111111, B11000000, //8
  B11111111, B11111111, B11111111, B11000000, //9
  B11111111, B11111111, B11111111, B11000000, //10
  B01111111, B11111111, B11111111, B10000000, //11
  B00111111, B11111111, B11111111, B00000000, //12
  B00011111, B11111111, B11111110, B00000000, //13
  B00001111, B11111111, B11111100, B00000000, //14
  B00000111, B11111111, B11111000, B00000000, //15
  B00000011, B11111111, B11110000, B00000000, //16
  B00000001, B11111111, B11100000, B00000000, //17
  B00000000, B11111111, B11000000, B00000000, //18
  B00000000, B01111111, B10000000, B00000000, //19
  B00000000, B00111111, B00000000, B00000000, //20
  B00000000, B00011110, B00000000, B00000000,  //21
  B00000000, B00001100, B00000000, B00000000  //22
};
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

    if ((digitalRead(pushButton) == LOW) && screenNo == 2)
    {
      Serial.println("BluetoothSwitch");
      switchBluetooth();
    }
    

    // testowo wyslij dane 
//    Serial2.print(temp1);
//    Serial2.print(",");
//    Serial2.print(temp2);
//    Serial2.print(",");
//    Serial2.print(atmPressure/100);
//    Serial2.print(" hPa");
//    Serial2.print(",");
//    Serial2.print(voltage);
//    Serial2.print(" V");
//    Serial2.print(",");
//    Serial2.print(speed / 1000);
//    Serial2.print(" km/h");
//    Serial2.print(headingDegrees);
//    Serial2.print("deg");
//    Serial2.print(";");
    
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
    case 2:
      showBTscreen();
      break;  
    default:
      showScreen_0();
      break;      
  }  
  screenNoLast = screenNo;
}

//void showSplash(uint16_t color)
//{
//    tft.setCursor(0, 0);
//    tft.setTextColor(color);
//    tft.setTextWrap(true);
//    tft.print("Starting Cycluino");
//}

void showScreen_0()
{
  showStatusBar();
  
  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(4);

  // formatuj wskaznik predkosci w zaleznosci od wartosci
  if ((speed/1000) > 9)
  {
    tft.setCursor(3, 40);// dla predkosci km 10-99
  }
  else 
  {
    tft.setCursor(25, 40);// dla predkosci km 0-9
  }
  tft.print(speed/1000);
  tft.setCursor(40, 40);
  tft.print(".");
  tft.setCursor(57,40);
  int reminder = speed % 1000;
  String rem = String(reminder, DEC);
  if (rem.length() == 1)
  {
    rem = rem + "0";
  }
  else if (rem.length() > 2)
  {
    rem = rem.substring(0,2);
  }
  tft.print(rem);
  tft.setCursor(100, 60);
  tft.setTextSize(1);
  tft.print("km/h");  

  tft.setTextSize(4);
  tft.setCursor(45,80);
  tft.print(cadence);
  tft.setTextSize(1);
  tft.setCursor(20,100);  
  tft.print("rpm");
  tft.drawBitmap(0, 138,bmHeart, 32, 22, ST7735_RED);
}

void showBTscreen()
{
  showStatusBar();

  tft.setTextColor(ST7735_WHITE);
  tft.setTextWrap(true);
  tft.setTextSize(2);
  tft.setCursor(10,60);
  tft.println("Bluetooth");
  tft.fillRect(40, 80, 35, 14, ST7735_BLUE);
  tft.setCursor(40,80);
  if (HIGH == digitalRead(btOnOffPin))
  {
   tft.print("ON"); 
  }
  else
  {
    tft.print("OFF");
  }
}
void showScreen_1()
{
  showStatusBar();
  tft.setCursor(0, 20);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(1);
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
  tft.print("Charging");
  tft.println(analogCharging);
  tft.print("A2");
  tft.println(analogRead(A2));
}

void showStatusBar()
{ 
  // rysuj czas tylko jesli inny niz ostatnio
  if (timeLast != clock.dateFormat("H:i", clock_dt))
  {
    tft.fillRect(0, 0, 60, 15, ST7735_RED);
    timeLast =  clock.dateFormat("H:i", clock_dt);
    tft.setCursor(0,0);
    tft.setTextSize(2);
    tft.setTextColor(ST7735_WHITE);
    tft.println(timeLast);
  }

  // rysuj symbol bluetooth tylko jesli status zmienil sie
  if (btOnOffLast != digitalRead(btOnOffPin))
  {
    tft.fillRect(70, 0, 9, 15, ST7735_WHITE);
    // wyswietl logo bluetooth tylko jesli bluetooth jest on (nie oznacza to ze jest polaczenie!)
    if (HIGH == digitalRead(btOnOffPin)) 
    {
      tft.drawBitmap(70, 0, btLogo, 8, 15, ST7735_BLUE);
    }
    btOnOffLast = digitalRead(btOnOffPin);
  }  

  tft.fillRect(96, 0, 128, 16, ST7735_BLACK);
  // wyswietl symbole baterii w zaleznosci od 
  if (voltage < 2.8)
  {
    tft.drawBitmap(96, 0,bat0perc, 32, 16, ST7735_RED);
  }
  else if (voltage < 3.1)
  {
    tft.drawBitmap(96, 0,bat25perc, 32, 16, ST7735_YELLOW);
  }
  else if (voltage < 3.4)
  {
    tft.drawBitmap(96, 0,bat50perc, 32, 16, ST7735_YELLOW);
  }
  else if (voltage < 3.6)
  {
      tft.drawBitmap(96, 0,bat75perc, 32, 16, ST7735_GREEN);
  }
  else 
  {
    tft.drawBitmap(96, 0,bat100perc, 32, 16, ST7735_GREEN);
  }
   //wyswietl symbol ladowania lub pelnego naladowania
   // wskaznik pelnego naladowania na diodzie ladowarki
  if (analogRead(A2) > 800)
    {
    tft.setCursor(102,4);
    tft.setTextColor(ST7735_RED);
    tft.setTextSize(1);
    tft.print("100%");    
  } 
  else if (analogCharging > 200) 
  {
    tft.drawBitmap(96, 2,batCharging, 32, 12, ST7735_WHITE);
  }

  //kasuj reszte ekranu tylko jesli zmiana ekranu
  if (screenNo != screenNoLast)
  {
    tft.fillRect(0, 17, 128, 160, ST7735_BLACK);
  }
}

void showTemperature(uint16_t color)
{  
  tft.setCursor(0,0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.println("Temperatura (bmp): ");
  tft.print(bme.readTemperature());  
}

void showPressure(uint16_t color)
{  
  tft.setCursor(0,10);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.println("Cisnienie (bmp): ");
  tft.print(bme.readPressure() / 100); // 100 Pa = 1 millibar
}   

float compassHeading(HMC5883L compass) // kod pochodzi z przykladu ze strony http://www.jarzebski.pl/
{
  // Pobranie wektorow znormalizowanych
  Vector norm = compass.readNormalize();

  // Obliczenie kierunku (rad)
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Ustawienie kata deklinacji dla Sosnowca 5'1E (positive)
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

// ladowanie obrazka. Kod pochodzi z przykladu spitftbitmap z biblioteki Adafruit_ST7735

void bmpDraw(char *filename, uint8_t x, uint8_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft.width()) || (y >= tft.height())) return;

  Serial.println();
  Serial.print("Loading image '");
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print("File not found");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print("File size: "); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print("Image Offset: "); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print("Header size: "); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print("Bit Depth: "); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print("Image size: ");
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
        Serial.print("Loaded in ");
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println("BMP format not recognized.");
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

void switchBluetooth()
{      
      if (digitalRead(btOnOffPin) == HIGH)     
      {
        Serial.println("DigitalWriteLow");
        digitalWrite(btOnOffPin, LOW);
        screenRefresh(2);
      }
      else 
      {
        Serial.println("DigitalWriteHigh");
        digitalWrite(btOnOffPin, HIGH);
        screenRefresh(2);
      }
}

