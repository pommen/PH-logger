/*
i2c adresses
   OLED          0x3C DEC:60
  
   PH            0x63 DEC:99
   EEPROM        0x50 DEC:80 ?




// Define the Base address of the RTC  registers (battery backed up CMOS Ram), so we can use them for config of touch screen and other calibration.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.


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
#include <Adafruit_ADS1015.h>
#include <math.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//#include "FreeStack.h"
#define ONE_WIRE_BUS PB1

//Objects:
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
File dataFile;
SdFat SD(1);
SdFat sd2(2);
RTClock rt(RTCSEL_LSE);
TinyGPSPlus gps;
U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(/* reset=*/U8X8_PIN_NONE, /* clock=*/PB6, /* data=*/PB7);
OneWire oneWire(ONE_WIRE_BUS);       // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.

//time_t tt1;
//Pins:
const uint8_t SD2_CS = PB12;   // chip select for sd2
const uint8_t SD_CS_PIN = PA4; //CS till SD kort
const int GPSPower = PB1;
const int PHPower = PA10;
const int loggingTypePin = PA9; //connected to a flipswitch that idecates what kind of logging we want.
//Globals:
long int alarmDelay = 10; //this number +1 sec is the sleep time
uint32_t CurrentTime = 0;
char weekday1[][7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"}; // 0,1,2,3,4,5,6

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

//protos:
//void gpsTest();                           //OLD test rutine
void startGPS(); //boot GPS up and get fix
//void displayInfo();                       //OLD GPS rutine
static void smartDelay(unsigned long ms);                               //parse GPS while waiting
void SDBoilerPlate();                                                   //Print the boiler plate to SD card. when starting a new log
void reInitOled();                                                      //Powerup OLED disp after sleepmode
void initSD();                                                          //Init SD
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
float getAirTemp();                                                     //samples and feeds back the air temp
float getWaterTemp();                                                   //samples the and feeds back the water temp (Thermistor, or Ktype?)
float getBattVolt();                                                    //Gets the battery voltage (ADS1115?)
//My files:

#include <PH.h>     //Atlas PH stuff
#include <GPS.h>    //GPS stuff
#include <SDcard.h> //SD card stuff
#include <temperatures.h>

void setup()
{
      adc_disable_all(); //disable all SDC to save some power
      // setGPIOModeToAllPins(GPIO_INPUT_ANALOG);

      Serial3.begin(9600); //UART till GPS

      pinMode(LED_BUILTIN, OUTPUT);
      pinMode(GPSPower, OUTPUT);
      pinMode(PHPower, OUTPUT);
      pinMode(loggingTypePin, INPUT);
      digitalWrite(GPSPower, LOW);
      digitalWrite(PHPower, LOW);

      // digitalWrite(LED_BUILTIN, LOW);
      Wire.begin();
      Wire.setClock(400000L);

      oled.begin();
      oled.setPowerSave(0);
      // oled.setFont(u8x8_font_chroma48medium8_r);
      oled.setFont(u8x8_font_amstrad_cpc_extended_f);
      oled.setFlipMode(1);
      oled.drawUTF8(0, 0, "Starting");
      // oled.println("Starting");
      ads.begin();
      ads.setGain(GAIN_TWOTHIRDS);
      sensors.begin();
      adc_disable_all();
      //setGPIOModeToAllPins(GPIO_INPUT_ANALOG);
      //gpsTest();

      //oled.setInverseFont(0);

      //   nvic_sys_reset(); //reset mcu

      bootUp();

      if (bootUpDone == 1)
            SDBoilerPlate();
      delay(200);

      digitalWrite(GPSPower, LOW); //this should be the last ting we do in setup
}
void bootUp()
{
      initSD();
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

            sleep();
      }
}

void spotcheckLogging()
{
      //  writeToFile(float airTemp, float waterTemp, sortPH(10), float Volt);
      writeToFile(dallasTemp(), Thermistor(), sortPH(), 4.15);
      //Buzzer
      //Shutdown --HOW??
      oled.clear();
      oled.print("DONE!");
      while(1){
            /* code */
      }
      
}

void LoggingtypeAndFileNames()
{
      loggingType = digitalRead(loggingTypePin);

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

      reInitOled(); //starting OLED again
      digitalWrite(LED_BUILTIN, LOW);
#ifdef debugMSG

      /* code */
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
