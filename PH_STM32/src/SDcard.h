
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
void initSD() //Init SD
{
    if (!SD.begin(SD_CS_PIN, SPI_HALF_SPEED /*SPI_FULL_SPEED*/))
    {
        oled.clear();
        oled.println(F("  FAIL!!"));
        oled.println(F("Card failed"));
        oled.println(F("or not present"));
        delay(1000); // don't do anything more:
        systemHardReset();
        return;
    }
    // Serial.print(F("card initialized."));
    oled.println(F("card initialized."));

    int cardSize = SD.card()->cardSize();
    if (cardSize == 0)
    {
        oled.println("cardSize failed");
        delay(500);
        systemHardReset();

        return;
    }
    oled.println("card initialized.");
}
