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
#include "FreeStack.h"




//Objects:
File dataFile;
SdFat SD;
RTClock rt(RTCSEL_LSE);
TinyGPSPlus gps;
U8X8_SSD1306_128X32_UNIVISION_HW_I2C oled(/* reset=*/U8X8_PIN_NONE, /* clock=*/PB6, /* data=*/PB7);

//Pins:
const int SD_CS_PIN = PA4; //CS till SD kort
const int GPSPower = PB1;
const int PHPower = PA9;
//Globals:
long int alarmDelay = 3; //this number +1 sec is the sleep time
boolean bootUpDone = 0;
//*******************************************************************************************************
//#define debugMSG 1 //uncomment this to get deMSG to OLED. Spammy and uses more space
//*******************************************************************************************************
String dataString = "";
uint32_t CurrentTime = 0;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//#define I2C_ADDRESS 0x3C //oled I"2 address

double gpsLAT = 0.000000;
double gpsLONG = 0.000000;
char fileName[] = "Logger1.csv";
int PHaddress = 99;

//protos:
static void smartDelay(unsigned long ms); //parse GPS while waiting
//void gpsTest();                           //OLD test rutine
void startGPS(); //boot GPS up and get fix
//void displayInfo();                       //OLD GPS rutine
void SDBoilerPlate();       //Print the boiler plate to SD card. when starting a new log
void reInitOled();          //Powerup OLED disp after sleepmode
void initSD();              //Init SD
void sleep();               //Powersave mode
void EZOStatus();           //asking EZO vcc and reason for last restart.
void bootUp();              //check that things are booted up and functioning.
void syncGPS();             //sync to gps time
void systemHardReset(void); //reset MCU
int PHLED(int on);          //toggle the LED on or off the EZO sensor board
void PHSleep();             //Sets the EZO in sleepmode. send i2c to wake up
//My files:

#include <PH.h>     //Atlas PH stuff
#include <GPS.h>    //GPS stuff
#include <SDcard.h> //SD card stuff

void setup()
{
      adc_disable_all(); //disable all SDC to save some power
      // setGPIOModeToAllPins(GPIO_INPUT_ANALOG);

      Serial3.begin(9600); //UART till GPS

      pinMode(LED_BUILTIN, OUTPUT);
      pinMode(GPSPower, OUTPUT);
      pinMode(PHPower, OUTPUT);
      digitalWrite(GPSPower, LOW);
      digitalWrite(PHPower, LOW);

      // digitalWrite(LED_BUILTIN, LOW);
      Wire.begin();
      Wire.setClock(400000L);

      oled.begin();
      oled.setPowerSave(0);
      // oled.setFont(u8x8_font_chroma48medium8_r);
      oled.setFont(u8x8_font_amstrad_cpc_extended_f);
      oled.drawUTF8(0, 0, "Starting");
      // oled.println("Starting");

      adc_disable_all();
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
      initSD();
      startGPS();
      syncGPS();
#ifdef debugMSG
      int temp = 1;
      temp = PHLED(temp);
#else
      int temp = 0;

      temp = PHLED(temp);
#endif
      oled.print("LED state: ");
      oled.print(temp);
      delay(2000);
      EZOStatus();
      bootUpDone = 1;
}

void loop()
{
      float temp = phMeasure();
      oled.print(temp);
      temp = 0;
      delay(500);
      oled.clear();

     //go back to sleep
      sleep();
}

void reInitOled()
{
      oled.setPowerSave(0);
      oled.begin();
      oled.clear();
      oled.home();
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
      rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE, RCC_PLLMUL_9); // 72MHz  => 48 mA  -- datasheet value           => between 40 and 41mA

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
