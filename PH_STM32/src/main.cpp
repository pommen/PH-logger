/*
i2c adresses
   OLED          0x3C DEC:60
  
   PH            0x63 DEC:99
   EEPROM        0x50 DEC:80 ?



*********************
OM den inte startar så kolla att RTC oscillatorn verklagen fugerar!

************


#ifdef debugMSG
Serial.print("Some debug stuff follows");
// More debug code...
#endif
 */

#include <Arduino.h>
#include <STM32Sleep.h>
#include <RTClock.h>
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <U8x8lib.h>
#include <TinyGPS++.h>

//#include "FreeStack.h"

//Objects:
File dataFile;
SdFat SD(1);
SdFat sd2(2);
RTClock rt(RTCSEL_LSE);
TinyGPSPlus gps;
U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(/* reset=*/U8X8_PIN_NONE, /* clock=*/PB6, /* data=*/PB7);

//time_t tt1;
//Pins:
const uint8_t SD2_CS = PB12;   // chip select for sd2
const uint8_t SD_CS_PIN = PA4; //CS till SD kort
const int GPSPower = PA8;
const int loggingTypePin = PA9; //connected to a flipswitch that idecates what kind of logging we want.
const int powerOnPin = PB5;
const int LEDPlus = PA3;
const int LEDGreen = PA0;
const int LEDRed = PA2;
const int LEDBlue = PA1;
const int BTN = PB9;

//Globals:
//long int alarmDelay = (60 * 10) - 1; //this number +1 sec is the sleep time
long int alarmDelay = 10; //this number +1 sec is the sleep time

uint32_t CurrentTime = 0;
//char weekday1[][7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"}; // 0,1,2,3,4,5,6

//*******************************************************************************************************
//#define debugMSG 1 //uncomment this to get deMSG to OLED. Spammy and uses more space
//*******************************************************************************************************

String dataString;
String fileName;
uint8_t loggingType = 0; //0=logging 1=SpotChecking
long int gpsLAT;
long int gpsLONG;

boolean bootUpDone = 0;
int PHaddress = 99;

float vcc = 0.00;
float Battvolt = 0.00;

//protos:
//void gpsTest();                           //OLD test rutine
void startGPS(); //boot GPS up and get fix
//void displayInfo();                       //OLD GPS rutine
static void smartDelay(unsigned long ms);                               //parse GPS while waiting
void SDBoilerPlate();                                                   //Print the boiler plate to SD card. when starting a new log
void reInitOled();                                                      //Powerup OLED disp after sleepmode
int initSD();                                                           //Init SD
void sleep();                                                           //Powersave mode
void EZOStatus();                                                       //asking EZO vcc and reason for last restart.
void bootUp();                                                          //check that things are booted up and functioning.
void syncGPS();                                                         //sync to gps time
void systemHardReset(void);                                             //reset MCU
int PHLED(int on);                                                      //toggle the LED on or off the EZO sensor board
void PHSleep();                                                         //Sets the EZO in sleepmode. send i2c to wake up
void LoggingtypeAndFileNames();                                         //Here we find ou what kind of logging we should do and generate filenames for them.
void spotcheckLogging();                                                //Do a check, log it to SD, sound a Buzer and poer off
void longTimeLogging();                                                 //Logg every n'th minute and go to sleep
void writeToFile(float airTemp, float waterTemp, float ph, float Volt); //puts the log results in the SD's funtion to contain error checking between the cards?
float getTemperatures(int external);                                    //1 for external temp 0 for internal
void PHCal();                                                           //3 point calibration rutine for EZO PH
void PHCheckCal();                                                      //Check if we are calibrated

//My files:

#include <PH.h>     //Atlas PH stuff
#include <GPS.h>    //GPS stuff
#include <SDcard.h> //SD card stuff
#include <temperatures.h>

void setup()
{
      adc_disable_all(); //disable all SDC to save some power
      // setGPIOModeToAllPins(GPIO_INPUT_ANALOG);

      Serial1.begin(9600); //UART till GPS PA10

      pinMode(LED_BUILTIN, OUTPUT);
      pinMode(GPSPower, OUTPUT);
      pinMode(loggingTypePin, INPUT);
      pinMode(powerOnPin, OUTPUT);
      pinMode(LEDPlus, OUTPUT);
      pinMode(LEDRed, OUTPUT_OPEN_DRAIN);
      pinMode(LEDGreen, OUTPUT_OPEN_DRAIN);
      pinMode(LEDBlue, OUTPUT_OPEN_DRAIN);
      pinMode(BTN, INPUT_PULLDOWN);

      digitalWrite(powerOnPin, HIGH); //denna slår på hela balletten, sätt låg on du vill sänga av loggern. slås förs på med en brytare.
      digitalWrite(GPSPower, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(LEDPlus, HIGH);
      digitalWrite(LEDGreen, LOW);

      Wire.begin();
      Wire.setClock(400000L);

      oled.begin();
      oled.setPowerSave(0);
      // oled.setFont(u8x8_font_chroma48medium8_r);
      oled.setFont(u8x8_font_amstrad_cpc_extended_f);
      oled.setFlipMode(1);
      oled.drawUTF8(0, 0, "Starting");

      digitalWrite(LEDPlus, LOW);
      digitalWrite(LEDGreen, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      // PHCheckCal();

      //oled.setInverseFont(0);

      //   nvic_sys_reset(); //reset mcu
      //  PHCal(); //uncomment to run the PH calibration sequence.
      bootUp();

      if (bootUpDone == 1)
            SDBoilerPlate();
      delay(200);

      digitalWrite(GPSPower, LOW);     //this should be the last ting we do in setup
      digitalWrite(LED_BUILTIN, HIGH); //HIGH = LED off.
}
void bootUp()
{
      PHSleep(); //sleeping, to save power while we'ew booting up
      int SDOK = 0;
      while (SDOK == 0)
      {
            SDOK = initSD();
      }
      if (digitalRead(loggingTypePin))
      {

            digitalWrite(GPSPower, HIGH);
            uint timeout = millis();
            oled.clear();
            while (millis() - timeout < 30000)
            {
                  oled.setCursor(0, 0);
                  oled.print("PH: ");
                  oled.println(sortPH());
                  oled.print("VannTemp: ");
                  oled.println(getTemperatures(1));
                  oled.print("LuftTemp: ");
                  oled.println(getTemperatures(0));
                  smartDelay(1200);
            }
      }

      digitalWrite(LED_BUILTIN, HIGH);
      startGPS();
      syncGPS();
      LoggingtypeAndFileNames();
#ifdef debugMSG
      int temp = 1;
      temp = PHLED(temp);
#else
      int temp = 0;

      temp = PHLED(temp);
#endif

      EZOStatus();
      bootUpDone = 1;
}

void loop()
{
      if (loggingType == 1)
      {
            spotcheckLogging();
      }
      else
            longTimeLogging();

      float temp = phMeasure();
      oled.print(temp);
      temp = 0;
      delay(500);
      oled.clear();

      //go back to sleep
      sleep();
}

void longTimeLogging()
{
      while (1)
      {
            // float phtemp = sortPH(10);
            //float airTemp = getAirTemp();
            //float waterTemp = getWaterTemp();
            //float Volt=getBattVolt();
            // writeToFile( airTemp, waterTemp, float ph, Volt);
            writeToFile(getTemperatures(0), getTemperatures(1), sortPH(), 4.15);

            sleep();
      }
}

void spotcheckLogging()
{
      //month;date;hour;minute;airTemp;waterTemp;PH;batteryVolts
      writeToFile(getTemperatures(0), getTemperatures(1), sortPH(), 4.15);
      //Buzzer
      //Shutdown --HOW??
      oled.clear();
      oled.println(millis());
      oled.println("Done!");

      for (size_t i = 0; i < 4; i++)
      {
            digitalWrite(LEDGreen, LOW);
            delay(250);
            digitalWrite(LEDGreen, HIGH);
      }

      digitalWrite(powerOnPin, LOW); //stänger av loggern.
      while (1)
      {
            /* code */
      }
}

void LoggingtypeAndFileNames()
{
      //      loggingType = digitalRead(loggingTypePin);
      loggingType = 1;

      if (loggingType == 1) //Sportchecks are chosen
      {
            fileName = "Spotcheck.csv";    //this will be the filename
            if (!sd2.chdir("/Spotchecks")) //go into the correct folder
            {
                  oled.print("Dir2-1");
                  delay(500);
                  systemHardReset();
            }
            if (!SD.chdir("/Spotchecks"))
            { //on both SD cards
                  oled.print("Dir2-1");
                  delay(500);
                  systemHardReset();
            }
      }
      if (loggingType == 0)
      {
            dataString = (rt.year() + 1970);

            if (rt.month() < 10)
            {
                  String temp = "0";
                  temp += rt.month();
                  dataString += temp;
            }
            else
                  dataString += rt.month();

            if (rt.day() < 10)
            {
                  String temp = "0";
                  temp += rt.day();
                  dataString += temp;
            }
            else
                  dataString += (rt.day());

            //  dataString += ('.');
            // dataString += ('.');
            dataString += ('-');
            if (rt.hour() < 10)
            {
                  String temp = "0";
                  temp += rt.hour();
                  dataString += temp;
            }
            else
                  dataString += (rt.hour());
            if (rt.minute() < 10)
            {
                  String temp = "0";
                  temp += rt.minute();
                  dataString += temp;
            }
            else
                  dataString += (rt.minute());
            dataString += ".csv";

            fileName = dataString;
            oled.clear();
            oled.print(fileName);
            delay(1000);

            if (!sd2.chdir("/Logging")) //go into the correct folder
            {
                  oled.print("Dir2-1");
                  delay(500);
                  systemHardReset();
            }
            if (!SD.chdir("/Logging"))
            { //on both SD cards
                  oled.print("Dir2-1");
                  delay(500);
                  systemHardReset();
            }
      }
}

void reInitOled()
{
      oled.setPowerSave(0);
      oled.begin();
      oled.clear();
      oled.home();
      oled.setFlipMode(1);
}

void sleep()
{

#ifdef debugMSG
      oled.clear();
      oled.print("Sleeping");
      delay(800);
      oled.clear();
#endif

      oled.setPowerSave(1);

      PHSleep(); //Sets the EZO in sleepmode. send i2c to wake up

      digitalWrite(LED_BUILTIN, HIGH);
      ///Going to sleep
      sleepAndWakeUp(STOP, &rt, alarmDelay); //sleep for "alarmDelay" secs - 1

      //Woken up from sleep
      rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE, RCC_PLLMUL_9); // 72MHz  => 48 mA  -- datasheet value  => between 40 and 41mA
      digitalWrite(GPSPower, LOW);                                //this should be the last ting we do in setup

      reInitOled(); //starting OLED again
      digitalWrite(LED_BUILTIN, LOW);

#ifdef debugMSG
      oled.print("Awake");
#endif
      //Wake up PH
      phMeasure();
      // EZOStatus();
      //sleepAndWakeUp(STANDBY, &rt, alarmDelay); //this will reboot
}
void systemHardReset(void)
{
      // Perform a system reset, see ../libmaple/nvic.c for the method
      nvic_sys_reset();
      // We will never get any further than this.
}

/*
useful stuff:

http://stm32duino.com/viewtopic.php?f=3&t=658&p=11094&hilit=sleep_deep#p11094



      oled.print(rt.day());
      oled.print("- ");
      oled.print(rt.month());
      oled.print("  ");
      oled.println(rt.year() + 1970);
      oled.print(rt.hour());
      oled.print(":");
      oled.print(rt.minute());
      oled.print(":");
      oled.println(rt.second());

      oled.println("time: ");
      oled.println(CurrentTime);
      while (1)
      {
      }
*/
