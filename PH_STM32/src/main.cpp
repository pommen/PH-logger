#include <Arduino.h>
#include <STM32Sleep.h>
#include <RTClock.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"
#include <TinyGPS++.h>
/* //#include <SoftwareSerial.h>

static const int RXPin = PB3, TXPin = PA12;
static const uint32_t GPSBaud = 9600; */
//SoftwareSerial ss(RXPin, TXPin);
float gpsLAT = 0.000000;
float gpsLONG = 0.000000;

// 0X3C+SA0 - 0x3C or 0x3D
#define I2C_ADDRESS 0x3C
#define SD_CS_PIN PA4
File dataFile;
SdFat SD;
SSD1306AsciiWire oled;
RTClock rt(RTCSEL_LSE);
// The TinyGPS++ object
TinyGPSPlus gps;

long int alarmDelay = 10; //this number +1 sec is the sleep time

//protos:
static void smartDelay(unsigned long ms);
void gpsTest();
void startGPS();
void displayInfo();
void setup()
{
     // ss.begin(GPSBaud);
      Serial3.begin(9600);

      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, LOW);
      Wire.begin();
      Wire.setClock(400000L);
      oled.begin(&Adafruit128x32, I2C_ADDRESS);
      oled.setFont(Adafruit5x7);
      oled.println("Starting");

      if (!SD.begin(SD_CS_PIN, SPI_FULL_SPEED))
      {
            oled.clear();
            oled.set2X();
            oled.println(F("  FAIL!!"));
            oled.set1X();
            oled.println(F("Card failed"));
            oled.println(F("or not present"));
            delay(100000); // don't do anything more:
            return;
      }
      // Serial.print(F("card initialized."));
      oled.println(F("card initialized."));

      int cardSize = SD.card()->cardSize();
      if (cardSize == 0)
      {
            oled.println("cardSize failed");
            return;
      }
      oled.println("card initialized.");

      //adc_disable_all();
      //setGPIOModeToAllPins(GPIO_INPUT_ANALOG);
      //gpsTest();
      startGPS();
      dataFile.print(F("https://www.google.no/maps/search/"));
      dataFile.print(gpsLAT);
      dataFile.print('+');
      dataFile.println(gpsLONG);
      dataFile.println();
      delay(200);
      oled.clear();
      // first row
      //oled.println("set1X test");
}
int i = 0;
String dataString = "";
void gpsTest()
{
      while (1)
      {
            // This sketch displays information every time a new sentence is correctly encoded.
            while (Serial3.available() > 0)
                  if (gps.encode(Serial3.read()))
                        displayInfo();

            if (millis() > 5000 && gps.charsProcessed() < 10)
            {
                  Serial.println(F("No GPS detected: check wiring."));
                  while (true)
                        ;
            }
      }
}
void displayInfo()
{
      oled.clear();
      // This sketch displays information every time a new sentence is correctly encoded.
      if (gps.time.isValid())
      {
            if (gps.time.hour() < 10)
                  oled.print(F("0"));
            oled.print(gps.time.hour());
            oled.print(F(":"));
            if (gps.time.minute() < 10)
                  oled.print(F("0"));
            oled.print(gps.time.minute());
            oled.print(F(":"));
            if (gps.time.second() < 10)
                  oled.print(F("0"));
            oled.print(gps.time.second());

            if (millis() > 5000 && gps.charsProcessed() < 10)
            {
                  oled.println(F("No GPS detected: check wiring."));
                  while (true)
                        ;
            }
      }
}
void loop()
{
      oled.clear();

      dataString += i;
      digitalWrite(LED_BUILTIN, HIGH);
      rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE, RCC_PLLMUL_9); // 72MHz  => 48 mA  -- datasheet value           => between 40 and 41mA

      dataFile = SD.open("datalog.txt", FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile)
      {
            dataFile.seek(dataFile.size());
            dataFile.println(dataString);
            dataFile.close();
            // print to the serial port too:
            oled.println(dataString);
      }
      // if the file isn't open, pop up an error:
      else
      {
            oled.println("error opening datalog.txt");
      }
      delay(100);

      digitalWrite(LED_BUILTIN, LOW);
      oled.println(i);
      i++;
      sleepAndWakeUp(STOP, &rt, alarmDelay);
}

void startGPS()
{
      unsigned int numSats = 1;
      delay(250);

      oled.clear();
      oled.set1X();
      oled.println("Starting GPS");
      oled.println("Searching for GPS");
      oled.print("GPS Found: ");
      oled.print(numSats);
      //    int i=0;

      while (numSats <= 5)
      {

            //  Serial.print("numSats:");
            //Serial.println(numSats);

            oled.set2X();
            oled.setCol(i);
            oled.setRow(3);
            oled.print("   ");
            i++;
            oled.setCol(i);
            oled.print("-->");
            oled.set1X();
            oled.setCursor(70, 4);
            oled.print(gps.charsProcessed());

            smartDelay(1000);
            numSats = gps.satellites.value();
            oled.setCursor(70, 3);
            oled.print(gps.satellites.value());
            if (gps.satellites.value() != numSats)
            {
            }

            if (millis() > 30000 && gps.charsProcessed() < 10)
            {
                  //Serial.println(F("No GPS detected: check wiring."));
                  oled.clear();
                  oled.print("No GPS found");
                  while (1)
                  {
                  }
            }
      }
      gpsLAT = gps.location.lat(), 6;
      gpsLONG = gps.location.lng(), 6;
      oled.print("GPS is valid: ");
      oled.println(gps.location.isValid());
}
static void smartDelay(unsigned long ms)
{
      unsigned long start = millis();
      do
      {
            while (Serial3.available())
                  gps.encode(Serial3.read());
      } while (millis() - start < ms);
}
