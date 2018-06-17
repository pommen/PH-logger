//#include <main.cpp>
#include <Wire.h>
#include <Arduino.h>

//PH Stuff:
char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer = 0; //we need to know how many characters have been received.
byte code = 0;                   //used to hold the I2C response code.
char ph_data[20];                //we make a 20 byte character array to hold incoming data from the pH circuit.
byte in_char = 0;                //used as a 1 byte buffer to store in bound bytes from the pH Circuit.
byte i = 0;                      //counter used for ph_data array.
int PH_wait_time = 1800;         //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
float ph_float;                  //float var used to hold the float value of the pH.

float phMeasure() //polls the atlas sensor for PH and takes avrages. Retuns ph (0-14) float or error code.
{
    int PH_wait_time = 1800;

    //tempComp(tmpvatten); //temperatur korregering till ph mätaren.
    int buf[20]; //buffer for read analog

    // state = !state;
    Wire.beginTransmission(PHaddress); //call the circuit by its ID number.
    Wire.write('r');                   //transmit the command to get a single measurement
    Wire.endTransmission();            //end the I2C data transmission.

    delay(PH_wait_time);

    Wire.requestFrom(PHaddress, 4); //call the circuit and request 15 bytes (this may be more than we need)
    code = Wire.read();              //the first byte is the response code, we read this separately.

    switch (code)
    {
    //switch case based on what the response code is.
    case 1: //decimal 1.
        //oled.clear();
        //oled.print(F("Success ")); //means the command was successful.
        //oled.println(i);
        break; //exits the switch case.

    case 2:
        if (debugMSG == 1) //if debug is enabled
        {

            oled.clear();
            oled.drawString(0, 0, "EZO Fail ");
            delay(2000);
        } //decimal 2.
        return 102;
        break; //exits the switch case.

    case 254:              //decimal 254.
        if (debugMSG == 1) //if debug is enabled
        {

            oled.clear();
            oled.drawString(0, 0, "Pending "); //means the command has not yet been finished calculating.
            delay(2000);
        }
        return 254;
        break; //exits the switch case.

    case 255: //decimal 255.

        if (debugMSG == 1) //if debug is enabled
        {

            oled.clear();
            oled.drawString(0, 0, "No Data "); //means there is no further data to send.
            delay(2000);
        }
        return 255;
        break; //exits the switch case.
    }
    int j = 0;
    while (Wire.available())
    {
        //are there bytes to receive.
        in_char = Wire.read(); //receive a byte.
        ph_data[j] = in_char;  //load this byte into our array.
        j++;                   //incur the counter for the array element.
        if (in_char == 0)
        {                           //if we see that we have been sent a null command.
            j = 0;                  //reset the counter i to 0.
            Wire.endTransmission(); //end the I2C data transmission.
            break;                  //exit the while loop.
        }
    }
    ph_float = atof(ph_data);
    if (debugMSG == 1)
    {

        oled.clear();
        oled.println("sizeof ph_Data: ");
        oled.println(sizeof(ph_data));
        oled.println("sizeof ph_float: ");
        oled.println(sizeof(ph_float));
        delay(5000);
    }
    //oled.println(ph_float);

    return ph_float;
}

float sortPH(int samples) //this function sorts samples and avreges them
{
    samples = samples - 1;

    if (samples < 3) //return 0 if are to sort less than three samples
    {
        return 0;
    }

    if (debugMSG == 1) //if debug is enabled
    {

        oled.clear();
        //oled.setInverseFont(1);
        oled.drawUTF8(0, 0, "Getting PH");
    }

    float buf[20] = {};
    for (size_t i = 0; i < samples; i++) //get (int samples) from the atles EZO and store them in buf[]
    {
        buf[i] = phMeasure();

        if (debugMSG == 1) //if debug is enabled
        {

            /* code */
            oled.setCursor(2, 2);
            oled.print(phMeasure());
            oled.setCursor(2, 3);
            oled.print(i);
        }
    }

    for (int i = 0; i < samples; i++) //sort the analog from small to large
    {
        for (int j = i + 1; j < 20; j++)
        {
            if (buf[i] > buf[j])
            {
                float temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }

    float avgValue = 0.0;

    //get the center 1/3 of sampes if we take 3 or more

    int onethird = samples / 3;

    for (int i = onethird; i < onethird * 2; i++) //take the average value of the center samples
    {
        avgValue += buf[i];
        if (debugMSG == 1) //if debug is enabled
        {

            oled.clear();
            oled.setCursor(2, 2);
            oled.drawString(0, 0, "avgValue: ");

            oled.print(avgValue);
            oled.setCursor(2, 3);
            oled.print(i);
        }
    }
    ph_float = avgValue / onethird;
    if (debugMSG == 1) //if debug is enabled
    {

        /* code */
        oled.drawString(0, 0, "avgValue: ");
        oled.print(avgValue);
        oled.drawString(0, 1, "onethird: ");
        oled.print(onethird);
        oled.drawString(0, 2, "ph_float: ");
        oled.print(ph_float);
        delay(2000);
    }
    return ph_float;
}

/*from atlas 




//**THIS CODE WILL WORK ON ANY ARDUINO**
//This code was written to be easy to understand.
//Modify this code as you see fit.
//This code will output data to the Arduino serial monitor.
//Type commands into the Arduino serial monitor to control the pH circuit.

//An Arduino UNO was used to test this code.
//This code was written in the Arduino 1.8.5 IDE
//This code was last tested 1/2018



#include <Wire.h>                //enable I2C.
#define address 99               //default I2C ID number for EZO pH Circuit.



char computerdata[20];           //we make a 20 byte character array to hold incoming data from a pc/mac/other.
byte received_from_computer = 0; //we need to know how many characters have been received.
byte code = 0;                   //used to hold the I2C response code.
char ph_data[20];                //we make a 20-byte character array to hold incoming data from the pH circuit.
byte in_char = 0;                //used as a 1 byte buffer to store inbound bytes from the pH Circuit.
byte i = 0;                      //counter used for ph_data array.
int time_ = 900;                 //used to change the delay needed depending on the command sent to the EZO Class pH Circuit.
float ph_float;                  //float var used to hold the float value of the pH.


void setup()                    //hardware initialization.
{
  Serial.begin(9600);           //enable serial port.
  Wire.begin();                 //enable I2C port.
}



void loop() {                                                             //the main loop.

  if (Serial.available() > 0) {                                           //if data is holding in the serial buffer
    received_from_computer = Serial.readBytesUntil(13, computerdata, 20); //we read the data sent from the serial monitor(pc/mac/other) until we see a <CR>. We also count how many characters have been received.
    computerdata[received_from_computer] = 0;                             //stop the buffer from transmitting leftovers or garbage.
    computerdata[0] = tolower(computerdata[0]);                           //we make sure the first char in the string is lower case.
    if (computerdata[0] == 'c' || computerdata[0] == 'r')time_ = 900;     //if a command has been sent to calibrate or take a reading we wait 1800ms so that the circuit has time to take the reading.
    else time_ = 300;                                                     //if any other command has been sent we wait only 300ms.


    Wire.beginTransmission(address); //call the circuit by its ID number.
    Wire.write(computerdata);        //transmit the command that was sent through the serial port.
    Wire.endTransmission();          //end the I2C data transmission.

    if (strcmp(computerdata, "sleep") != 0) {  //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                                               //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the pH circuit.



    delay(time_);                     //wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom(address, 20, 1); //call the circuit and request 20 bytes (this may be more than we need)
    code = Wire.read();               //the first byte is the response code, we read this separately.

    switch (code) {                  //switch case based on what the response code is.
      case 1:                        //decimal 1.
        Serial.println("Success");   //means the command was successful.
        break;                       //exits the switch case.

      case 2:                        //decimal 2.
        Serial.println("Failed");    //means the command has failed.
        break;                       //exits the switch case.

      case 254:                      //decimal 254.
        Serial.println("Pending");   //means the command has not yet been finished calculating.
        break;                       //exits the switch case.

      case 255:                      //decimal 255.
        Serial.println("No Data");   //means there is no further data to send.
        break;                       //exits the switch case.
    }





    while (Wire.available()) {         //are there bytes to receive.
      in_char = Wire.read();           //receive a byte.
      ph_data[i] = in_char;            //load this byte into our array.
      i += 1;                          //incur the counter for the array element.
      if (in_char == 0) {              //if we see that we have been sent a null command.
        i = 0;                         //reset the counter i to 0.
        Wire.endTransmission();        //end the I2C data transmission.
        break;                         //exit the while loop.
      }
    }

    Serial.println(ph_data);          //print the data.
  }
}
  //Uncomment this section if you want to take the pH value and convert it into floating point number.
  //ph_float=atof(ph_data);
}



*/