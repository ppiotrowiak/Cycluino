#include "Arduino.h"
#include "TFTController.h"


//void TFTController::init(Adafruit_SSD1306* Oled1)
//{
//    _Oled1 = Oled1;
//     // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
//    _Oled1->begin(SSD1306_SWITCHCAPVCC, 0x3C); 
//    _Oled1->clearDisplay();
//    _Oled1->setCursor(10, 10);
//    _Oled1->setTextColor(WHITE);
//    _Oled1->setTextSize(3);
//    _Oled1->print("Cycluinor");
//    _Oled1->display();
//    delay(2000);
//    _Oled1->clearDisplay();
//    _Oled1->display();
//}

//void TFTController::showSpeedAndCadence(unsigned int speed, unsigned int cadence)
//{

//  //show speed on the oled  
//  _Oled1->clearDisplay();
//  _Oled1->setTextColor(WHITE);
//  _Oled1->setTextSize(3);
//  if ((speed/1000) > 9)
//  {
//    _Oled1->setCursor(0, 10);//for speed km 10-99
//  }
//  else 
//  {
//    _Oled1->setCursor(18, 10);//for speed km0-9
//  }
//  _Oled1->print(speed/1000);
//  _Oled1->setCursor(30, 10);
//  _Oled1->print(".");
//  _Oled1->setCursor(45,10);
//  int reminder = speed % 1000;
//  String rem = String(reminder, DEC);
//  if (rem.length() == 1)
//  {
//    rem = rem + "0";
//  }
//  else if (rem.length() > 2)
//  {
//    rem = rem.substring(0,2);
//  }
//  _Oled1->print(rem);
//  _Oled1->setCursor(80, 24);
//  _Oled1->setTextSize(1);
//  _Oled1->print("km/h");  
//
//  //display cadence
//  _Oled1->setTextSize(3);
//  _Oled1->setCursor(45,40);
//  _Oled1->print(cadence);
//  _Oled1->setTextSize(1);
//  _Oled1->setCursor(80,54);  
//  _Oled1->print("rpm");
//  _Oled1->display();
//}
