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
<<<<<<< HEAD:old/summer17.cpp
        //  wait();
        Serial.begin(9600);    //enable serial port.
        Wire.begin();           //enable I2C port.
        Rtc.Begin();
        ads.begin();
        ads.setGain(GAIN_TWOTHIRDS);

        pinMode(interruptPin, INPUT);
        pinMode(GPSOn, OUTPUT);
        pinMode(ledR, OUTPUT);
        pinMode(ledB, OUTPUT);
        pinMode(ledG, OUTPUT);
        pinMode(lineDrvOn, OUTPUT);
        LEDstart();
        LED(0,0,0);
        digitalWrite(lineDrvOn, LOW);
        Serial.println(F("Starting"));
        if (!SD.begin(SD_CS_PIN)) {
                //oled.println(F("Card failed, or not present"));
                // don't do anything more:
                while (1==1) {
                        LED(255,0,0);
                        delay(200);
                        LED(0,0,0);
                        delay(4000);

                }
                return;
        }
        digitalWrite(lineDrvOn, LOW);

        Rtc.Enable32kHzPin(false);
        Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmTwo);


        RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
        //Rtc.SetDateTime(compiled);
        if (!Rtc.IsDateTimeValid())
        {
                // Common Cuases:
                //    1) first time you ran and the device wasn't running yet
                //    2) the battery on the device is low or even missing

                Serial.println("RTC lost confidence in the DateTime!");

                // following line sets the RTC to the date & time this sketch was compiled
                // it will also reset the valid flag internally unless the Rtc device is
                // having an issue

                Rtc.SetDateTime(compiled);
        }

        if (!Rtc.GetIsRunning())
        {
                Serial.println("RTC was not actively running, starting now");
                Rtc.SetIsRunning(true);
        }
        RtcDateTime now = Rtc.GetDateTime();

        if (now < compiled)
        {
                Serial.println("RTC is older than compile time!  (Updating DateTime)");
                Rtc.SetDateTime(compiled);
        }

        // Alarm 2 set to trigger at the top of the minute
        DS3231AlarmTwo alarm2(
                0,
                0,
                0,
                DS3231AlarmTwoControl_MinutesMatch);
        //DS3231AlarmTwoControl_OncePerMinute);
        Rtc.SetAlarmTwo(alarm2);
/*
   DS3231AlarmTwoControl_OncePerMinute - triggers every minute
   DS3231AlarmTwoControl_MinutesMatch - triggers once an hour when the minute matched
 */


        // throw away any old alarm state before we ran
        Rtc.LatchAlarmsTriggeredFlags();
        digitalWrite(lineDrvOn, LOW);
        delay(250);
        if (!SD.exists(fileName)) {
                Serial.println("New log. Starting from the beginning");
                startGPS();
                //date();
                Starting();
        }
        else{
                digitalWrite(lineDrvOn, LOW);
                delay(250);
                File dataFile = SD.open(fileName, FILE_WRITE);

                // if the file is available, write to it:
                if (dataFile) {
                        digitalWrite(lineDrvOn, LOW);
                        delay(320);
                        Serial.println(" This is a restart");
                        dataFile.println(F("Restarted"));
                        dataFile.close();
                        delay(2000);
                        digitalWrite(lineDrvOn, HIGH);
                }
        }

        Wire.beginTransmission(PHaddress);       //call the circuit by its ID number.
        Wire.write("l,0");        //transmit something to wake up
        Wire.endTransmission();          //end the I2C data transmission.


}
void wakeUp(){
        detachInterrupt(digitalPinToInterrupt(interruptPin));



}
void LEDstart(){

        int k=0;
        int j=0;
        for (int i=0; i <= 255; i++) {
                /* code */
                LED(i, j, k);
                delay(5);

        }
        for ( j=0; j <= 255; j++) {
                /* code */
                LED(i, j, k);
                delay(5);

        }
        for (k=0; k <= 255; k++) {
                /* code */
                LED(i, j, k);
                delay(5);

        }
}

void loop() {                   //the main loop.
//First we log, then we sleep. nothing fancy.
        logging();

        //oled.clear();
        //oled.print("nattinatt");

        delay(100); // Make sure  Serial finishes
        wait();

}

void LED(int red, int green, int blue){
        red = 255 - red;
        green = 255 - green;
        blue = 255 - blue;
        analogWrite(ledR, red);
        analogWrite(ledG, green);
        analogWrite(ledB, blue);
/*
   (255, 0, 0);  // red
   (0, 255, 0);  // green
   (0, 0, 255);  // blue
   (255, 255, 0);// yellow
   (80, 0, 80);  // purple
   (0, 255, 255);// aqua
 */
}

void startGPS(){
        unsigned int numSats=1;
        delay(250);
        digitalWrite(GPSOn, HIGH);
        digitalWrite(lineDrvOn, LOW);
        //oled.clear();
        delay(100);
        //oled.set1X();
        //oled.println("Starting GPS");
        //oled.println("Searching for GPS");
        //oled.print("GPS Found: ");
        //  //oled.print(numSats);
        //    int i=0;

        while (numSats <= 5) {
                if (state == true && numSats>0) {
                        unsigned int blink=0;
                        while(blink != numSats) {
                                LED(255, 255, 0); // yellow
                                delay(100);
                                LED(0,0,0);
                                blink++;
                        }
                }


                else if (state == true ) LED(255, 255, 0);   // yellow



                else LED(0, 0, 0);


                smartDelay(1000);
                state=!state;
                Serial.print("numSats:");
                Serial.println(numSats);


                ////oled.set2X();
                //oled.setCol(i);
                //oled.setRow(3);
                //oled.print("   ");
                //  i++;
                //oled.setCol(i);
                //oled.print("-->");
                ////oled.set1X();
                //if (i==127) i =0;

                if (gps.satellites.value() != numSats) {
                        //oled.setCursor(70, 2);
                        Serial.print(gps.satellites.value());

                }

                numSats=gps.satellites.value();

                if (millis() > 30000 && gps.charsProcessed() < 10)
                {
                        Serial.println(F("No GPS detected: check wiring."));
                        //oled.clear();
                        //oled.print("No GPS found");
                        while (1==1) {
                                LED(255,0,0);
                                delay(200);
                                LED(0,0,0);
                                delay(200);

                                LED(255,0,0);
                                delay(200);
                                LED(0,0,0);

                                delay(4000);

                        }
                        while(true) ;

                }
        }
        gpsLAT = gps.location.lat(),6;
        gpsLONG = gps.location.lng(),6;
        Serial.print("GPS is valid: ");
        Serial.println(gps.location.isValid());
        LED(0, 0, 0);
        for (int i=0; i<10; i++) {
                LED  (0, 255, 0);// green
                delay(100);
                LED(0,0,0);
                delay(100);
        }
        LED(0, 0, 0);
        digitalWrite(GPSOn, LOW);
        digitalWrite(lineDrvOn, HIGH);
        int gotGPS=(gps.location.lat(),6);

        if (gotGPS ==0) {
                wdt_enable( WDTO_15MS);
                while(true) ;
        }


=======
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
>>>>>>> c46ced79111a05dfebd25fcf27a6380ac9ed2c54:src/main.cpp
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
