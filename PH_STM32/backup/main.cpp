/*
i2c adresses
   OLED          0x3C DEC:60
  
   PH            0x63 DEC:99
   RTC           0x68 DEC:104?
   EEPROM        0x50 DEC:80?



#include "RTClock.h"
RTClock rt (RTCSEL_LSE); // initialise
uint32 tt;

// Define the Base address of the RTC  registers (battery backed up CMOS Ram), so we can use them for config of touch screen and other calibration.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.

#define BKP_REG_BASE   (uint32_t *)(0x40006C00 +0x04)


 */

#include <Arduino.h>
#include <STM32Sleep.h>
#include <RTClock.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <U8x8lib.h>
#include <TinyGPS++.h>
//Objects:
File dataFile;
SdFat SD;
RTClock rt(RTCSEL_LSE);
TinyGPSPlus gps;
U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(/* reset=*/U8X8_PIN_NONE, /* clock=*/PB6, /* data=*/PB7);

//Pins:
const int SD_CS_PIN = PA4; //CS till SD kort
const int GPSPower = PB1;

//Globals:
long int alarmDelay = 3; //this number +1 sec is the sleep time
boolean bootUpDone = 0;
const boolean debugMSG = 0; //send debug msg to display
String dataString = "";

uint32_t CurrentTime =0;;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//#define I2C_ADDRESS 0x3C //oled I"2 address
double gpsLAT = 0.000000;
double gpsLONG = 0.000000;
char fileName[] = "Logger1.csv";
int PHaddress = 99;

//protos:
static void smartDelay(unsigned long ms);
void gpsTest();
void startGPS();
void displayInfo();
void SDBoilerPlate();
void reInitOled();
void initSD();
void sleep();
void EZOStatus(); //asking EZO vcc and reason for last restart.
void bootUp();    //check that things are booted up and functioning.
void syncGPS();   //sync to gps time

//My files:

#include <PH.h>  //Atlas PH stuff
#include <GPS.h> //GPS stuff

void setup()
{
      adc_disable_all();
      // setGPIOModeToAllPins(GPIO_INPUT_ANALOG);

      Serial3.begin(9600);

      pinMode(LED_BUILTIN, OUTPUT);
      pinMode(GPSPower, OUTPUT);
      // digitalWrite(LED_BUILTIN, LOW);
      Wire.begin();
      Wire.setClock(400000L);

      oled.begin();
      oled.setPowerSave(0);
      // oled.setFont(u8x8_font_chroma48medium8_r);
      oled.setFont(u8x8_font_amstrad_cpc_extended_f);
      oled.drawUTF8(0, 0, "Starting");
      // oled.println("Starting");

      //adc_disable_all();
      //setGPIOModeToAllPins(GPIO_INPUT_ANALOG);
      //gpsTest();

      //oled.setInverseFont(0);

      //   nvic_sys_reset(); //reset mcu

      bootUp();

      if (bootUpDone == 1)
            SDBoilerPlate();
      delay(200);
}
void bootUp()
{
      EZOStatus();
      initSD();
      startGPS();
      syncGPS();
      bootUpDone = 1;
}

void loop()
{
      rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE, RCC_PLLMUL_9); // 72MHz  => 48 mA  -- datasheet value           => between 40 and 41mA
      float temp = phMeasure();
      oled.print(temp);
      temp = 0;
      delay(500);
      oled.clear();
      //1529192293

      /*   oled.println(" tt: ");
            oled.println(tt);
            oled.println(" gps: ");
            oled.print(CurrentTime); */
      //  oled.print("Date:  ");
      oled.print(rt.day());
      oled.print("- ");
      oled.print(rt.month());
      oled.print("  ");
      oled.println(rt.year() + 1970);
      oled.print(rt.hour());
      oled.print(" : ");
      oled.print(rt.minute());
      oled.print(" : ");
      oled.println(rt.second());
     /*  oled.print(" day: ");
      oled.println(rt.day()); */
      //  tm.day = 2;
      oled.println("time: ");
      oled.println(CurrentTime);
      while (1)
      {
      }
      delay(500);
      //you can also SET the rtc time with rtc.setTime(unixTimestamp) where unixTimestamp is a unix timestamp

      //go back to sleep
      sleep();
}
void SDBoilerPlate()
{
      // Writing header
      oled.clear();
      oled.println(F("Logging"));
      oled.println(F("Starting"));

      dataFile = SD.open(fileName, FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile)
      {
            dataFile.seek(dataFile.size());

            // RtcDateTime now = Rtc.GetDateTime();
            //Boilderplate for .CSV file
            /*  dataFile.print(F("Logging started: "));
            dataFile.print(now.Month(), DEC);
            dataFile.print('/');
            dataFile.print(now.Day(), DEC);
            dataFile.print(F(" ("));
            dataFile.print(daysOfTheWeek[now.DayOfWeek()]);
            dataFile.print(F(") "));
            dataFile.print(now.Hour(), DEC);
            dataFile.print(':');
            dataFile.print(now.Minute(), DEC);
            dataFile.print(';');
            dataFile.print(';');
            dataFile.print(';');
            dataFile.print(';');
            dataFile.print(';'); */
            dataFile.print("Link to logger position: ");
            dataFile.print(';');
            dataFile.print("https://www.google.no/maps/search/");
            dataFile.print(gps.location.lat(), 6);
            dataFile.print('+');
            dataFile.print(gps.location.lng(), 6);
            dataFile.println();

            ///Print legend to file
            dataFile.print(F("Month"));
            dataFile.print('/');
            dataFile.print(F("Day"));
            dataFile.print('(');
            dataFile.print(F("Weekday"));
            dataFile.print(')');
            dataFile.print(F("Hour:Minute"));
            dataFile.print(';');
            dataFile.print(F("PH"));
            dataFile.print(';');
            dataFile.print(F("Vann Temp"));
            dataFile.print(';');
            dataFile.print(F("Luft Temp"));
            dataFile.print(';');
            dataFile.print("Batt Volt");
            dataFile.print(';');
            dataFile.println("freeMemory()");
            dataFile.close();
            delay(2000);
      }
      else
      {
            oled.print("Cant open datafile");
            while (1)
            {
            }
      }
}
void initSD() //Ini SD
{
      if (!SD.begin(SD_CS_PIN, SPI_FULL_SPEED))
      {
            oled.clear();
            oled.println(F("  FAIL!!"));
            oled.println(F("Card failed"));
            oled.println(F("or not present"));
            delay(5000); // don't do anything more:
            nvic_sys_reset();

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
}

void startGPS()
{
      digitalWrite(GPSPower, HIGH);
      unsigned int numSats = 1;
      delay(250);

      oled.clear();
      oled.drawString(0, 0, "Starting GPS");
     // oled.drawString(0, 1, "Searching for GPS");
      oled.drawString(0, 2, "GPS Found: ");

      //    int i=0;

      while (numSats <= 3)
      {

            smartDelay(1000);
            numSats = gps.satellites.value();
            oled.setCursor(11, 2);
            oled.println(gps.satellites.value());
            oled.print(gps.charsProcessed());

            if (millis() > 30000 && gps.charsProcessed() < 10)
            {
                  oled.clear();
                  oled.drawString(0, 0, "No GPS found");

                  while (1)
                  {
                  }
            }
      }
      gpsLAT = (gps.location.lat(), 6);
      gpsLONG = (gps.location.lng(), 6);

      oled.print("GPS is valid: ");
      // oled.println(gps.location.isValid());
      digitalWrite(GPSPower, LOW);
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
void syncGPS()
{
      if (gps.date.isValid() && gps.time.isValid() && gps.time.age() < 2000)
      {
            CurrentTime = (31556926 * (gps.date.year() - 1970));
            CurrentTime += (2629743 * gps.date.month());
            CurrentTime += (86400 * gps.date.day());
            CurrentTime += (3600 * gps.time.hour());
            CurrentTime += (60 * gps.time.minute());
            CurrentTime += gps.time.second();
            rt.setTime(CurrentTime);

            //   RTC.set(1529192293);
      }
}
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

void reInitOled()
{
      oled.setPowerSave(0);
      oled.begin();
      oled.clear();
      oled.home();
}

void EZOStatus() //asking EZO vcc and reason for last restart.
{

      if (debugMSG == 1)
      {
            oled.clear();
            oled.println("ez0status: ");
            delay(500);
      }

      Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
      Wire.write("Status");              //transmit the command to sleep.
      Wire.endTransmission();            //end the I2C data transmission.

      delay(300);
      Wire.requestFrom(PHaddress, 15); //call the circuit and request 20 bytes (this may be more than we need)
      byte code = Wire.read();         //the first byte is the response code, we read this separately.
      char EZOStatus[12] = {};
      int j = 0;
      while (Wire.available())
      {
            //are there bytes to receive.
            in_char = Wire.read();  //receive a byte.
            EZOStatus[j] = in_char; //load this byte into our array.
            j++;                    //incur the counter for the array element.
            if (in_char == 0)
            {                             //if we see that we have been sent a null command.
                  j = 0;                  //reset the counter i to 0.
                  Wire.endTransmission(); //end the I2C data transmission.
                  break;                  //exit the while loop.
            }
      }

      if (debugMSG == 1)
      {
            /* code */
            String temp = EZOStatus;
            oled.println(temp);
            oled.print(sizeof(temp));
            delay(500);
      }
      /*
Restart codes
 
P powered off
S software reset
B brown out
W watchdog
U unknown

*/
}

void sleep()
{

      if (debugMSG == 1)
      {
            oled.clear();
            oled.print("Sleeping");
            delay(800);
            oled.clear();
      }

      oled.setPowerSave(1);

      //sleep EZO PH:
      Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
      Wire.write("sleep");               //transmit the command to sleep.
      Wire.endTransmission();            //end the I2C data transmission.
      delay(50);
      digitalWrite(LED_BUILTIN, HIGH);
      sleepAndWakeUp(STOP, &rt, alarmDelay); //sleep for "alarmDelay" secs - 1
      digitalWrite(LED_BUILTIN, LOW);
      reInitOled(); //Get OLED back from sleep.

      if (debugMSG == 1)
      {
            /* code */
            oled.print("Awake");
      }

      EZOStatus();
      //sleepAndWakeUp(STANDBY, &rt, alarmDelay); //this will reboot
}

/*
useful stuff:

http://stm32duino.com/viewtopic.php?f=3&t=658&p=11094&hilit=sleep_deep#p11094

void systemHardReset(void) {
  // Perform a system reset, see ../libmaple/nvic.c for the method
  nvic_sys_reset();
  // We will never get any further than this.

}

*/