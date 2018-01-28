void screenRefresh(int screen)
{  
    if (screen < 0) 
    {
      screen = screen*(-1);
    }

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
      showScreen_2();
      break;
    case 3:
      showScreen_3();
      break;  
    case 4:
      showScreen_4();      
      break;       
    case 5:      
      showBTscreen();
      break;
    default:
      showScreen_0();
      break;      
  }  
  screenNoLast = screenNo;
}


void showScreen_0()
{
  showStatusBar();
 
  tft.fillRect(3,40, 97, 28, ST7735_BLACK);
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
  tft.setCursor(103, 60);
  tft.setTextSize(1);
  tft.print("km/h");  
  
  tft.fillRect(44, 80, 60, 28, ST7735_BLACK);
  tft.setTextSize(4);
  tft.setCursor(45,80);
  tft.print(cadence);
  tft.setTextSize(1);
  tft.setCursor(20,100);  
  tft.print("rpm");
  tft.drawBitmap(0, 138,bmHeart, 32, 22, ST7735_BLACK);
  speed_last = speed;
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
  tft.print("humidity: ");
  tft.println(humidity);
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

void showScreen_2() // temperatury
{
  showStatusBar();
  tft.fillRect(0,20, 159, 127, ST7735_BLACK);
  tft.setTextColor(ST7735_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(0,20);
  tft.println("Temp1:");
  tft.setTextColor(ST7735_WHITE);
  tft.println(temp1);
  tft.setTextColor(ST7735_YELLOW);
  tft.println("Temp2:");
  tft.setTextColor(ST7735_WHITE);
  tft.println(temp2);
}

void showScreen_3() // cisnienie i wysokosc
{
  showStatusBar();

  tft.fillRect(0,20,159,127,ST7735_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0,20);
  tft.setTextColor(ST7735_YELLOW);
  tft.println("Pressure:");
  tft.setTextColor(ST7735_WHITE);
  tft.println(atmPressure);
  tft.setTextColor(ST7735_YELLOW);
  tft.println("Altitude:");
  tft.setTextColor(ST7735_WHITE);
  tft.println(altitude);
}

void showScreen_4() // 
{
  showStatusBar();

  tft.fillRect(0,20,159,127,ST7735_BLACK);
  tft.setCursor(0,20);
  tft.setTextSize(2);
  tft.setTextColor(ST7735_YELLOW);
  tft.println("Heading");
  tft.setTextColor(ST7735_WHITE);
  tft.println(headingDegrees);
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
        screenRefresh(5);
      }
      else 
      {
        Serial.println("DigitalWriteHigh");
        digitalWrite(btOnOffPin, HIGH);
        screenRefresh(5);
      }
}
