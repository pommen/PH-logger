

float Thermistor()
{
    float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
    float IseriesResistor = 0.000;
    float VoltOverseriesResistance = 0.000;
    float VoltOverThermistor = 0.000;
    float ThmeristorResistans = 0.00;

    VoltOverseriesResistance = (ads.readADC_Differential_0_1() * multiplier);
    VoltOverThermistor = (ads.readADC_Differential_2_3() * multiplier);

    IseriesResistor = VoltOverseriesResistance / 9995;
    long Resistance = VoltOverThermistor / IseriesResistor;
    // Serial.print("Themristor Resistans: ");
    // Serial.println(Resistance);
    float Temp;             // Dual-Purpose variable to save space.
    Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
    Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
    Temp = Temp - 273.15; // Convert Kelvin to Celsius
    return Temp;          // Return the Temperature
}

float dallasTemp()
{

    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus

#ifdef debugMSG
    oled.clear();
    oled.print("Req temps...");

#endif
    sensors.requestTemperatures(); // Send the command to get temperatures

#ifdef debugMSG
    oled.clear();
    // oled.print("Sleeping");
    oled.println("DONE");
    delay(1000);
    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    oled.println("Temperature : ");
    oled.println(sensors.getTempCByIndex(0));
#endif

    return sensors.getTempCByIndex(0);
}