/*

   PH logger
   May -17
   by Peter R
   Wake up and check if its time to log values. IF yes, run logging function, else go back to sleep.


   Goal is >month battery time on a lipo.

   Perhaps an strech goal is external battery.


   This unit might be suitable with adafruits feather and logger board.

   The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

   I2c lines to  RTC, ADC, PH sensor and display
   //This code will output data to the Arduino serial monitor. Type commands into the Arduino serial monitor to control the EZO pH Circuit in I2C mode. temperature from thermistor and 15bit dac

   Air temperature from RTC and water



 */




#include <Arduino.h>
#include <SPI.h>
#include "SdFat.h"
#include <Wire.h>
#include <Time.h>
#include <DS3231_Simple.h>
#include <avr/power.h>
#include "LowPower.h"
#include <Adafruit_ADS1015.h>
#include <math.h>
//#include "SSD1306Ascii.h"
//#include "SSD1306AsciiAvrI2c.h"
#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
#include <MemoryFree.h>
//My functions so that the GCC code checker dosent freak out
void Starting();
void logging();
void wait();
int battVolt();
float Thermistor();
void GetGPSTime();
void startGPS();
void Wakeup();
static void smartDelay(unsigned long ms);
void date();
void tempComp();
/*i2c adresses
   OLED          0x3C DEC:60
   ADC           0x48 DEC:72
   PH            0x63 DEC:99
   RTC           0x68 DEC:104
   EEPROM        0x50 DEC:80
 */
#define PHaddress 99               //default I2C ID number for EZO pH Circuit.
#define OLED_ADDRESS 0x3C
static const int RXPin = 5, TXPin = 6;
static const uint32_t GPSBaud = 9600;


//SD card CS
#define SD_CS_PIN 4
SdFat SD;
File dataFile;
String fileName= "Logger1.csv";

//OLED stuff
//SSD1306AsciiAvrI2c oled;
//adc stuff
Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//GPS stuff:
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
//SoftwareSerial ss(RXPin, TXPin);

const int GPSOn = 7;
const int lineDrvOn =8;

//Temperature vars:
float multiplier = 0.1875F;       /* ADS1115  @ +/- 6.144V gain (16-bit results) */
float IseriesResistor=0.000;
float VoltOverseriesResistance=0.000;
float VoltOverThermistor=0.000;
float ThmeristorResistans =0.00;





//PH Stuff:
char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer = 0; //we need to know how many characters have been received.
byte code = 0;                   //used to hold the I2C response code.
char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit.
byte in_char = 0;                //used as a 1 byte buffer to store in bound bytes from the pH Circuit.
byte i = 0;                      //counter used for ph_data array.
int PH_wait_time = 1800;                //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
float ph_float;                  //float var used to hold the float value of the pH.

long result=0; //Batt voltage

//RTC stuff
DS3231_Simple Clock;
char daysOfTheWeek[7][12] = {"Friday", "Saturday", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday"};
String nu ="";
int prevDay =0;
int prevHour =0;
int prevMinute=0;
const int interruptPin = 3;
boolean state =true;
const int led = 13;


void setup()                    //hardware initialization.
{
       Serial.begin(9600);     //enable serial port.
        Wire.begin();           //enable I2C port.
        Clock.begin();
        //ss.begin(GPSBaud);
        //oled.begin(&Adafruit128x32, OLED_ADDRESS);


        Clock.disableAlarms();  // First we will disable any existing alarms


        //oled.setFont(System5x7);
        //oled.clear();
        //oled.set2X();
        //oled.print(F("Woohooo"));
        //oled.print(F("Im up and running!"));
        pinMode(interruptPin, INPUT);
        pinMode(GPSOn, OUTPUT);
        pinMode(lineDrvOn, OUTPUT);
        digitalWrite(lineDrvOn, HIGH);

/*  //Serial.println("Getting single-ended readings from AIN0..3");
   //Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

   // The ADC input range (or gain) can be changed via the following
   // functions, but be careful never to exceed VDD +0.3V max, or to
   // exceed the upper and lower limits if you adjust the input range!
   // Setting these values incorrectly may destroy your ADC!
   //                                                                ADS1015  ADS1115
   //                                                                -------  -------
   // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
   // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
   // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
   // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
   // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
   // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
 */
        ads.begin();
        ads.setGain(GAIN_TWOTHIRDS);

        if (!SD.begin(SD_CS_PIN)) {
                //oled.println(F("Card failed, or not present"));
                // don't do anything more:
                return;
        }

      //  if (SD.exists("fileName")) {

        startGPS();
        GetGPSTime();
        //date();

        Starting();
/*
   digitalWrite(PHon, LOW);
   //Serial.print(F("Temperature Luft = "));
   //Serial.println(luftTemp.temperature());
   //Serial.print(F("Temperature vatten = "));
   //Serial.println(vattenTemp.temperature());
 */
}



void loop() {                   //the main loop.
//First we log, then we sleep. nothing fancy.
        logging();
        wait();

}
void date(){

    //    DateTime now = Clock.read();


      //Serial.print(now.Year,DEC);
      //Serial.print('/');
      //Serial.print(now.Month, DEC);
      //Serial.print('/');
      //Serial.print(now.Day, DEC);
      //Serial.print(" (");
      //Serial.print(daysOfTheWeek[now.Dow]);
      //Serial.print(") ");
      //Serial.print(now.Hour, DEC);
      //Serial.print(':');
      //Serial.print(now.Minute, DEC);
      //Serial.print(':');
      //Serial.print(now.Second, DEC);
      //Serial.println();
      //Serial.println();



        while (1==1) {
                /* code */
        }
}

void startGPS(){
  unsigned int numSats=1;

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

        while (numSats <= 4) {
                smartDelay(1000);
              Serial.print("numSats:");
                Serial.println(numSats);

                //Serial.println(millis() - runningTime);

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
                        //Serial.println(F("No GPS detected: check wiring."));
                        //oled.clear();
                        //oled.print("No GPS found");
                        while(true) ;

                }
        }
        digitalWrite(GPSOn, LOW);
        digitalWrite(lineDrvOn, HIGH);

}
static void smartDelay(unsigned long ms)
{
        unsigned long start = millis();
        do
        {
                while (Serial.available())
                        gps.encode(Serial.read());
        } while (millis() - start < ms);
}

void GetGPSTime() {
        DateTime MyTimestamp;

        if (gps.date.isValid())
        {
                MyTimestamp.Day    = gps.date.day();
                MyTimestamp.Month  = gps.date.month();
                MyTimestamp.Year   = gps.date.year();
        }

        if (gps.time.isValid()) {


                // Load it with the date and time you want to set, for example
                //   Saturday the 3rd of October 2020 at 14:17 and 33 Seconds...

                MyTimestamp.Hour   = gps.time.hour() +2;
                MyTimestamp.Minute = gps.time.minute();
                MyTimestamp.Second = gps.time.second();
                // Then write it to the clock
                Clock.write(MyTimestamp);
        }
        //Serial.println("Got GPS time!");
        //Serial.print(gps.satellites.value());
        //oled.clear();
        //oled.print("Set GPS Time");


        //Serial.print("Hour: ");   //Serial.println(MyTimestamp.Hour +2);
        //Serial.print("Minute: "); //Serial.println(MyTimestamp.Minute);
        //Serial.print("Second: "); //Serial.println(MyTimestamp.Second);
        //Serial.print("Year: ");   //Serial.println(MyTimestamp.Year);
        //Serial.print("Month: ");  //Serial.println(MyTimestamp.Month);
        //Serial.print("Day: ");    //Serial.println(MyTimestamp.Day);

}


void Starting()
{
///Writing header
        //oled.clear();
        //oled.set2X();
        //oled.println(F("Logging"));
        //oled.println(F("Starting"));
        //oled.set1X();
        digitalWrite(lineDrvOn, LOW);

        File dataFile = SD.open(fileName, FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile) {

                DateTime now = Clock.read();
//Boilderplate for .CSV file
                dataFile.print(F("Logging started: "));
                dataFile.print(now.Month, DEC);
                dataFile.print('/');
                dataFile.print(now.Day, DEC);
                dataFile.print(F(" ("));
                dataFile.print(daysOfTheWeek[now.Dow]);
                dataFile.print(F(") "));
                dataFile.print(now.Hour, DEC);
                dataFile.print(':');
                dataFile.print(now.Minute, DEC);
                dataFile.print(';');
                dataFile.print(F("Link to logger position: "));
                dataFile.print(';');
                dataFile.print(F("https://www.google.no/maps/search/"));
                dataFile.print(gps.location.lat(),6);
                dataFile.print('+');
                dataFile.println(gps.location.lng(),6);



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
                dataFile.print(F("Batt Volt"));
                dataFile.print(';');
                dataFile.println(F("freeMemory()"));
                dataFile.close();
                digitalWrite(lineDrvOn, HIGH);

        }

}


void tempComp(int temp){

                /* code */
String sendstring="t,";
 sendstring=+temp;
//Serial.print("sendstring: ");
Serial.println(sendstring);
                Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
                Wire.write("t,");
                Wire.write(sendstring.c_str());  //transmit the command to get enter temp comp mode
                Wire.endTransmission();    //end the I2C data transmission.
                delay(320); //delay to proess commands



                Wire.requestFrom(PHaddress, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
                code = Wire.read();     //the first byte is the response code, we read this separately.

                switch (code) {         //switch case based on what the response code is.
                case 1:                 //decimal 1.
                       //Serial.println("Success"); //means the command was successful.
                        break;            //exits the switch case.

                case 2:                  //decimal 2.
                       //Serial.println("Failed"); //means the command has failed.
                        break;             //exits the switch case.

                case 254:                //decimal 254.
                       //Serial.println("Pending"); //means the command has not yet been finished calculating.
                        break;             //exits the switch case.

                case 255:                //decimal 255.
                       //Serial.println("No Data"); //means there is no further data to send.
                        break;           //exits the switch case.
                }





                while (Wire.available()) { //are there bytes to receive.
                        in_char = Wire.read(); //receive a byte.
                        ph_data[i] = in_char; //load this byte into our array.
                        i += 1;            //incur the counter for the array element.
                        if (in_char == 0) { //if we see that we have been sent a null command.
                                i = 0;     //reset the counter i to 0.
                                Wire.endTransmission(); //end the I2C data transmission.
                                break;     //exit the while loop.
                        }
                }

               //Serial.println(ph_data);  //print the data.

                        /* code */


                        Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
                        Wire.write("t,?");  //transmit the command to get enter temp comp mode
                        Wire.endTransmission();    //end the I2C data transmission.
                        delay(320); //delay to proess commands



                        Wire.requestFrom(PHaddress, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
                        code = Wire.read();     //the first byte is the response code, we read this separately.

                        switch (code) {         //switch case based on what the response code is.
                        case 1:                 //decimal 1.
                               //Serial.println("Success"); //means the command was successful.
                                break;            //exits the switch case.

                        case 2:                  //decimal 2.
                               //Serial.println("Failed"); //means the command has failed.
                                break;             //exits the switch case.

                        case 254:                //decimal 254.
                               //Serial.println("Pending"); //means the command has not yet been finished calculating.
                                break;             //exits the switch case.

                        case 255:                //decimal 255.
                               //Serial.println("No Data"); //means there is no further data to send.
                                break;           //exits the switch case.
                        }





                        while (Wire.available()) { //are there bytes to receive.
                                in_char = Wire.read(); //receive a byte.
                                ph_data[i] = in_char; //load this byte into our array.
                                i += 1;            //incur the counter for the array element.
                                if (in_char == 0) { //if we see that we have been sent a null command.
                                        i = 0;     //reset the counter i to 0.
                                        Wire.endTransmission(); //end the I2C data transmission.
                                        break;     //exit the while loop.
                                }
                        }

                       //Serial.println(ph_data);  //print the data.
        }





void logging(){
        PH_wait_time = 1800;
        delay(500); // to stbilize the boost converter.
        float tmpvatten = Thermistor();
        float tmpluft =Clock.getTemperatureFloat();
        tempComp(tmpvatten); //temperatur korregering till ph mätaren.
        int buf[20]; //buffer for read analog
        for(int i=0; i<20; i++) //Get 10 sample value from the sensor for smooth the value
        {
                Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
                Wire.write("r");  //transmit the command to get a single measurement
                Wire.endTransmission();    //end the I2C data transmission.
                delay(1800);
                Wire.requestFrom(PHaddress, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
                code = Wire.read();       //the first byte is the response code, we read this separately.

                switch (code) {
                //switch case based on what the response code is.
                case 1:                   //decimal 1.
                        //oled.clear();
                        //oled.print(F("Success ")); //means the command was successful.
                        //oled.println(i);
                        break;              //exits the switch case.

                case 2:                    //decimal 2.
                        //Serial.println("Failed"); //means the command has failed.
                        break;               //exits the switch case.

                case 254:                  //decimal 254.
                        //Serial.println("Pending"); //means the command has not yet been finished calculating.
                        break;               //exits the switch case.

                case 255:                  //decimal 255.
                        //Serial.println("No Data"); //means there is no further data to send.
                        break;             //exits the switch case.
                }
                int j =0;
                while (Wire.available()) {
                        //are there bytes to receive.
                        in_char = Wire.read(); //receive a byte.
                        ph_data[j] = in_char; //load this byte into our array.
                        j++;              //incur the counter for the array element.
                        if (in_char == 0) {  //if we see that we have been sent a null command.
                                j = 0;       //reset the counter i to 0.
                                Wire.endTransmission(); //end the I2C data transmission.
                                break;       //exit the while loop.
                        }
                }

                ph_float=atof(ph_data);
                //oled.println(ph_float);
                buf[i]=ph_float;
        }


        for(int i=0; i<19; i++) //sort the analog from small to large
        {
                for(int j=i+1; j<20; j++)
                {
                        if(buf[i]>buf[j])
                        {
                                int temp=buf[i];
                                buf[i]=buf[j];
                                buf[j]=temp;
                        }
                }
                ////Serial.println(buf[i]);

        }
        int avgValue=0;
        for(int i=4; i<16; i++) {//take the average value of 6 center sample
                avgValue+=buf[i];
                //  //Serial.println(i);
        }



        //Serial.print(F(" pH:"));
        //Serial.print(ph_float,2);
        //Serial.println(F(" "));
        //Serial.println(tmpluft);
        //Serial.println(tmpvatten);
        float dispV=(battVolt()/1000.00);

        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        digitalWrite(lineDrvOn, LOW);

        File dataFile = SD.open(fileName, FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile) {

                DateTime now = Clock.read();
                dataFile.print(now.Month, DEC);
                dataFile.print('/');
                dataFile.print(now.Day, DEC);
                dataFile.print(F(" ("));
                dataFile.print(daysOfTheWeek[now.Dow]);
                dataFile.print(F(") "));
                dataFile.print(now.Hour, DEC);
                dataFile.print(':');
                dataFile.print(now.Minute, DEC);
                dataFile.print(';');

                dataFile.print(ph_float);
                dataFile.print(';');
                dataFile.print(tmpvatten, 2);
                dataFile.print(';');
                dataFile.print(tmpluft, 2);
                dataFile.print(';');
                dataFile.print(dispV);
                dataFile.print(';');
                dataFile.println(freeMemory());


                //Serial.print(F("Temperature Luft = "));
                //Serial.println(tmpluft);
                //Serial.print(F("Temperature vatten = "));
                //Serial.println(tmpvatten);

                dataFile.close();
                digitalWrite(lineDrvOn, HIGH);

        }

}


int battVolt(){

        // Read 1.1V reference against AVcc
        // set the reference to Vcc and the measurement to the internal 1.1V reference
          #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
          #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
        ADMUX = _BV(MUX5) | _BV(MUX0);
          #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        ADMUX = _BV(MUX3) | _BV(MUX2);
          #else
        ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
          #endif

        delay(2);   // Wait for Vref to settle
        ADCSRA |= _BV(ADSC);   // Start conversion
        while (bit_is_set(ADCSRA,ADSC)) ;  // measuring

        uint8_t low  = ADCL;   // must read ADCL first - it then locks ADCH
        uint8_t high = ADCH;   // unlocks both

        result = (high<<8) | low;

        result = 1125300L / result;   // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
        return result;


}


float Thermistor() {

        VoltOverseriesResistance  = (ads.readADC_Differential_0_1() * multiplier);
        VoltOverThermistor = (ads.readADC_Differential_2_3()*multiplier);

        IseriesResistor = VoltOverseriesResistance/9732;
        long Resistance = VoltOverThermistor/IseriesResistor;
        //Serial.print("Themristor Resistans: ");
        //Serial.println(Resistance);
        float Temp; // Dual-Purpose variable to save space.
        Temp = log(Resistance);         // Saving the Log(resistance) so not to calculate  it 4 times later
        Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
        Temp = Temp - 273.15; // Convert Kelvin to Celsius
        return Temp;                                // Return the Temperature
}

void wakeUp(){


}
void wait() {

        Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
        Wire.write("sleep");  //transmit the command to sleep.
        Wire.endTransmission();    //end the I2C data transmission.
        Clock.setAlarm(DS3231_Simple::ALARM_EVERY_MINUTE);
        //Serial.println(F("nattinatt"));
        //oled.clear();
        //oled.print("nattinatt");
        attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp, LOW);

        delay(100); // Make sure // Serial finishes
        /*    power_spi_disable(); // SPI
            power_usart0_disable();// Serial (USART)
            power_timer0_disable();// Timer 0
            power_timer1_disable();// Timer 1
            power_timer2_disable();// Timer 2
            power_twi_disable(); // TWI (I2C)
           power_adc_disable(); // ADC converter
         */
        power_twi_disable(); // TWI (I2C)

        //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);


        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        //LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
        //  SPI_OFF, USART0_OFF, TWI_OFF);
        detachInterrupt(digitalPinToInterrupt(interruptPin));
        digitalWrite(led, state);
        state=!state;
        delay(100);
        power_twi_enable();         // TWI (I2C)
        Wire.beginTransmission(PHaddress);       //call the circuit by its ID number.
        Wire.write("r");        //transmit something to wake up
        Wire.endTransmission();          //end the I2C data transmission.

        Clock.disableAlarms();          // First we will disable any existing alarms
        //Serial.println(F("Awake!"));
        //oled.clear();
        //oled.print("Awake! :D ");


}

/*

   Name: powerDown
 * Description: Putting microcontroller into power down state. This is
 *			   the lowest current consumption state. Use this together with
 *			   external pin interrupt to wake up through external event
 *			   triggering (example: RTC clockout pin, SD card detect pin).
 *
 * Argument   Description
 * =========    ===========
 * 1. period     Duration of low power mode. Use SLEEP_FOREVER to use other wake
 *				up resource:
 *				(a) SLEEP_15MS - 15 ms sleep
 *				(b) SLEEP_30MS - 30 ms sleep
 *				(c) SLEEP_60MS - 60 ms sleep
 *				(d) SLEEP_120MS - 120 ms sleep
 *				(e) SLEEP_250MS - 250 ms sleep
 *				(f) SLEEP_500MS - 500 ms sleep
 *				(g) SLEEP_1S - 1 s sleep
 *				(h) SLEEP_2S - 2 s sleep
 *				(i) SLEEP_4S - 4 s sleep
 *				(j) SLEEP_8S - 8 s sleep
 *				(k) SLEEP_FOREVER - Sleep without waking up through WDT
 *
 * 2. adc		ADC module disable control. Turning off the ADC module is
 *				basically removing the purpose of this low power mode.
 *				(a) ADC_OFF - Turn off ADC module
 *				(b) ADC_ON - Leave ADC module in its default state
 *
 * 3. bod		Brown Out Detector (BOD) module disable control:
 *				(a) BOD_OFF - Turn off BOD module
 *				(b) BOD_ON - Leave BOD module in its default state
 *









   The macros power_all_disable() and power_all_enable() modify the PRR register as appropriate for different processors.

   Enabling:


   power_adc_enable(); // ADC converter
   power_spi_enable(); // SPI
   power_usart0_enable(); // Serial (USART)
   power_timer0_enable(); // Timer 0
   power_timer1_enable(); // Timer 1
   power_timer2_enable(); // Timer 2
   power_twi_enable(); // TWI (I2C)


   Disabling:


   power_adc_disable(); // ADC converter
   power_spi_disable(); // SPI
   power_usart0_disable();// Serial (USART)
   power_timer0_disable();// Timer 0
   power_timer1_disable();// Timer 1
   power_timer2_disable();// Timer 2
   power_twi_disable(); // TWI (I2C)
 */







/*
   Thermisor connections:
 *
 *                  Analog pin 0
 *                        |
 *    5V |-----/\/\/\-----+-----/\/\/\-----| GND
 *
 *               ^                ^
 *        10K thermistor     10K resistor


 */


/*
   if (Serial.available() > 0) {                                           //if data is holding in the serial buffer
   received_from_computer =//Serial.readBytesUntil(13, computerdata, 20); //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
   computerdata[received_from_computer] = 0;                             //stop the buffer from transmitting leftovers or garbage.
   computerdata[0] = tolower(computerdata[0]);                           //we make sure the first char in the string is lower case.
   if (computerdata[0] == 'c' || computerdata[0] == 'r')PH_wait_time = 1800;    //if a command has been sent to calibrate or take a reading we wait 1800ms so that the circuit has time to take the reading.
   else PH_wait_time = 300;                                                     //if any other command has been sent we wait only 300ms.


   Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
   Wire.write(computerdata);        //transmit the command that was sent through the serial port.
   Wire.endTransmission();          //end the I2C data transmission.

   if (strcmp(computerdata, "sleep") != 0) {  //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                                             //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the pH circuit.



   delay(PH_wait_time);                    //wait the correct amount of time for the circuit to complete its instruction.


   //Serial.println(ph_data);          //print the data.
   }
   }
 */
/* There are 2 different alarms possible in the chip, the first alarm has a
 * resolution of seconds, the second a resolution of Minutes.
 *
 * Which alarm gets modified by setAlarm() depends on the type of alarm
 * you want.
 *
 * The following alarm types are available.
 *
 *  (Alarm 1)
 *    ALARM_EVERY_SECOND  (Timestamp not required)
 *    ALARM_MATCH_SECOND
 *    ALARM_MATCH_SECOND_Minute
 *    ALARM_MATCH_SECOND_Minute_Hour
 *    ALARM_MATCH_SECOND_Minute_Hour_DATE
 *    ALARM_MATCH_SECOND_Minute_Hour_DOW
 *
 * (Alarm 2)
 *    ALARM_EVERY_Minute (Timestamp not required)
 *    ALARM_MATCH_Minute
 *    ALARM_MATCH_Minute_Hour
 *    ALARM_MATCH_Minute_Hour_DATE
 *    ALARM_MATCH_Minute_Hour_DOW
 *
 * (Alarm 2)
 *    ALARM_HourLY   (on the Minute of the supplied timestamp *)
 *    ALARM_DAILY    (on the Hour and Minute *)
 *    ALARM_WEEKLY   (on the Hour and Minute and day-of-week *)
 *    ALARM_MONTHLY  (on the Hour and Minute and day-of-month *)
 *
 *  * If set without a timestamp, the current timestamp is used.
 */
