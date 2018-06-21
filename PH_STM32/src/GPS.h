
void syncGPS()
{
      if (gps.date.isValid() && gps.time.isValid() && gps.time.age() < 2000)
      {

            int daysInMonths[]{0, 31, 28, 31, 30, 31, 30, 31, 031, 30, 31, 30, 31};
            int totalDays = 0;
            CurrentTime = (1 * gps.date.month() + 1);

            for (int i = 0; i < CurrentTime; i++)

            {
                  totalDays += daysInMonths[i];
            }

            CurrentTime = (31556926 * (gps.date.year() - 1970));
            CurrentTime += (86400 * totalDays);
            CurrentTime += (86400 * gps.date.day()) - 86400; //visar datum+1 annars
            CurrentTime += (3600 * (gps.time.hour() + 11));  //+11 pga timezone?
            CurrentTime += (60 * gps.time.minute());
            CurrentTime += gps.time.second();
            rt.setTime(CurrentTime);
      }

      /*
January - 31 days
February - 28 days in a common year and 29 days in leap years
March - 31 days
April - 30 days
May - 31 days
June - 30 days
July - 31 days
August - 31 days
September - 30 days
October - 31 days
November - 30 days
December - 31 days
*/
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
      gpsLAT = gps.location.lat() * 1000000;
      gpsLONG = gps.location.lng() * 1000000;
      /*     oled.clear();
      oled.println(gpsLAT);
      oled.println(gpsLONG);

      delay(50000); */
      if (gpsLAT < 1.000000)
            systemHardReset();
      if (gpsLONG < 1.000000)
            systemHardReset();

      // oled.print("GPS is valid: ");
      // oled.println(gps.location.isValid());
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