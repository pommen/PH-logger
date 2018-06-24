
void SDBoilerPlate()
{

    //Make sure we have our two directories.

    //  Writing header
    oled.clear();
    oled.println(F("Logging"));
    oled.println(F("Starting"));

    dataFile = SD.open(fileName, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {

        dataFile.seek(dataFile.size());

        //Boilderplate for .CSV file
        dataFile.print(F("Logging started: "));
        dataFile.print(rt.month());
        dataFile.print('/');
        dataFile.print(rt.day());
        dataFile.print(';');

        dataFile.print(rt.hour());
        dataFile.print(F(':'));
        dataFile.print(rt.minute());
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print("Link to logger position: ");
        dataFile.print(';');
        dataFile.print("https://www.google.no/maps/search/");

        double floatTemp = gpsLAT / 1000000; //blir nu 60.123456
        long int intTemp = floatTemp;        //blir nu 60
        dataFile.print(intTemp);             //skriver 60
        dataFile.print('.');                 //decimal
        intTemp = gpsLAT - (intTemp * 1000000);
        dataFile.print(intTemp);

        dataFile.print('+');

        floatTemp = gpsLONG / 1000000; //blir nu 60.123456
        intTemp = floatTemp;           //blir nu 60
        dataFile.print(intTemp);       //skriver 60
        dataFile.print('.');           //decimal
        intTemp = gpsLONG - (intTemp * 1000000);
        dataFile.print(intTemp);

        dataFile.println();
        dataFile.print("backup logger position: ");
        dataFile.print(';');
        dataFile.print("https://www.google.no/maps/search/");

        dataFile.print(gpsLAT);
        dataFile.print('+');
        dataFile.print(gpsLONG);
        dataFile.print(';');
        dataFile.println(';');

        ///Print legend to file
        dataFile.print(F("Month"));
        dataFile.print('/');
        dataFile.print(F("Day"));
        dataFile.print(';');
        dataFile.print(F("Hour:Minute"));
        dataFile.print(';');
        dataFile.print(F("Vann Temp"));
        dataFile.print(';');
        dataFile.print(F("Luft Temp"));
        dataFile.print(';');
        dataFile.print(F("PH"));
        dataFile.print(';');
        dataFile.println("Batt Volt");

        dataFile.close();
        delay(800);
    }
    else
    {
        oled.clear();
        oled.print("Error1");
        delay(500);
        systemHardReset();

        while (1)
        {
        }
    }

    dataFile = sd2.open(fileName, FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {
        dataFile.seek(dataFile.size());

        //Boilderplate for .CSV file
        dataFile.print(F("Logging started: "));
        dataFile.print(rt.month());
        dataFile.print('/');
        dataFile.print(rt.day());
        dataFile.print(';');

        dataFile.print(rt.hour());
        dataFile.print(F(':'));
        dataFile.print(rt.minute());
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print(';');
        dataFile.print("Link to logger position: ");
        dataFile.print(';');
        dataFile.print("https://www.google.no/maps/search/");

        double floatTemp = gpsLAT / 1000000; //blir nu 60.123456
        long int intTemp = floatTemp;        //blir nu 60
        dataFile.print(intTemp);             //skriver 60
        dataFile.print('.');                 //decimal
        intTemp = gpsLAT - (intTemp * 1000000);
        dataFile.print(intTemp);

        dataFile.print('+');

        floatTemp = gpsLONG / 1000000; //blir nu 60.123456
        intTemp = floatTemp;           //blir nu 60
        dataFile.print(intTemp);       //skriver 60
        dataFile.print('.');           //decimal
        intTemp = gpsLONG - (intTemp * 1000000);
        dataFile.print(intTemp);

        dataFile.println();
        dataFile.print("backup logger position: ");
        dataFile.print(';');
        dataFile.print("https://www.google.no/maps/search/");

        dataFile.print(gpsLAT);
        dataFile.print('+');
        dataFile.print(gpsLONG);
        dataFile.print(';');
        dataFile.println(';');

        ///Print legend to file
        dataFile.print(F("Month"));
        dataFile.print('/');
        dataFile.print(F("Day"));
        dataFile.print(';');
        dataFile.print(F("Hour:Minute"));
        dataFile.print(';');
        dataFile.print(F("Vann Temp"));
        dataFile.print(';');
        dataFile.print(F("Luft Temp"));
        dataFile.print(';');
        dataFile.print(F("PH"));
        dataFile.print(';');
        dataFile.println("Batt Volt");

        dataFile.close();
        delay(800);
    }
    else
    {
        oled.clear();

        oled.print("Error2");
        delay(500);
        systemHardReset();

        while (1)
        {
        }
    }
}


void initSD() //Init SD
{
    if (!SD.begin(SD_CS_PIN, SPI_HALF_SPEED /*SPI_FULL_SPEED*/))
    {
        oled.clear();
        oled.println(F("  FAIL!!"));
        oled.println(F("Card1 failed"));
        oled.println(F("or not present"));
        delay(1000); // don't do anything more:
        systemHardReset();
        return;
    }
    oled.println(F("card1 Init."));
    if (!sd2.begin(SD2_CS, SPI_HALF_SPEED /*SPI_FULL_SPEED*/))
    {
        oled.clear();
        oled.println(F("  FAIL!!"));
        oled.println(F("Backup failed"));
        oled.println(F("or not present"));
        delay(1000); // don't do anything more:
        systemHardReset();
        return;
    }
    oled.println(F("backup Init."));

    // Serial.print(F("card initialized."));

    int cardSize = SD.card()->cardSize();
    if (cardSize == 0)
    {
        oled.println("cardSize1 failed");
        delay(500);
        systemHardReset();

        return;
    }
    oled.println("card1 init");

    cardSize = sd2.card()->cardSize();
    if (cardSize == 0)
    {
        oled.println("backup failed");
        delay(500);
        systemHardReset();

        return;
    }
    oled.println("backup init");

    if (!sd2.exists("/Spotchecks"))
    {
        if (!sd2.mkdir("/Spotchecks"))
        {
            oled.clear();
            oled.println(F("MkDir 2-1"));
            delay(500);
            systemHardReset();
        }
    }
    if (!sd2.exists("/Logging"))
    {
        if (!sd2.mkdir("/Logging"))
        {
            oled.clear();
            oled.println(F("MkDir 2-2"));
        }
    }

    if (!SD.exists("/Spotchecks"))
    {
        if (!SD.mkdir("/Spotchecks"))
        {
            oled.clear();
            oled.println(F("MkDir 1-1"));
            delay(500);
            systemHardReset();
        }
    }
    if (!SD.exists("/Logging"))
    {
        if (!SD.mkdir("/Logging"))
        {
            oled.clear();
            oled.println(F("MkDir 1-2"));
        }
    }
}

//month;date;hour;minute;airTemp;waterTemp;PH;batteryVolts
void writeToFile(float airTemp, float waterTemp, float ph, float Volt)
{

    String SDCard[] = {"SD", "sd2"};

    for (size_t i = 0; i < 2; i++)
    {

        if (i == 0)
        {
            dataFile = SD.open(fileName, FILE_WRITE);
            /* code */
        }
        else
            dataFile = sd2.open(fileName, FILE_WRITE);

        // if the file is available, write to it:
        if (dataFile)
        {

            dataFile.seek(dataFile.size());

            //Boilderplate for .CSV file
            if (rt.month() < 10)
            {
                String temp = "0";
                temp += rt.month();
                dataFile.print(temp);
            }
            else
                dataFile.print(rt.month());

            dataFile.print(F("/"));

            if (rt.day() < 10)
            {
                String temp = "0";
                temp += rt.day();
                dataString += temp;
                dataFile.print(temp);
            }
            else
                dataFile.print(rt.day());

            dataFile.print(F(";"));

            if (rt.hour() < 10)
            {
                String temp = "0";
                temp += rt.hour();
                dataString += temp;
                dataFile.print(temp);
            }
            else
                dataFile.print(rt.hour());

            dataFile.print(F(":"));
            if (rt.minute() < 10)

            {
                String temp = "0";
                temp += rt.minute();
                dataString += temp;
                dataFile.print(temp);
            }
            else
                dataFile.print(rt.minute());

            dataFile.print(F(";"));
            dataFile.print(waterTemp);
            dataFile.print(F(";"));
            dataFile.print(airTemp);
            dataFile.print(F(";"));
            dataFile.print(ph);
            dataFile.print(F(";"));
            dataFile.println(Volt);

            dataFile.close();
        }
        else
        {
            oled.clear();

            oled.print("Error ");
            oled.print(SDCard[i]);

            delay(500);
            systemHardReset();

            while (1)
            {
            }
        }
    }
}

/*
   Serial.print("Date:  ");
    Serial.print(rtclock.day());
    Serial.print("- ");
    Serial.print(rtclock.month());
    Serial.print("  ");
    Serial.print(rtclock.year() + 1970);
    Serial.print("  ");
    Serial.print(weekday1[rtclock.weekday()]);
    Serial.print("  Time: ");
    Serial.print(rtclock.hour());
    Serial.print(" : ");
    Serial.print(rtclock.minute());
    Serial.print(" : ");
    Serial.println(rtclock.second());
    */
