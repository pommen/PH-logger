

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

float sortThermistor() //this function sorts samples and avreges them
{
    int samples = 6;

    float buf[6] = {};
    for (size_t i = 0; i < samples; i++) //get (int samples) from the atles EZO and store them in buf[]
    {
        buf[i] = Thermistor();
        delay(200);
        if (buf[i] > 100 ||
            buf[i] < 1) //if we there was a error retuned,or low value,  dont save it.
        {
            i--;
        }
    }

    for (int i = 0; i < samples; i++) //sort the analog from small to large
    {
        for (int j = i + 1; j < samples; j++)
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
    }
    avgValue = avgValue / onethird;
    return avgValue;
}

float Thermistor()
{
    

    //*************************
    int seriesResistor = 10000; //10Kohm
    float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */
    float IseriesResistor = 0.000;
    float VoltOverseriesResistance = 0.000;
    float VoltOverThermistor = 0.000;
    //float ThmeristorResistans = 0.00;

    VoltOverseriesResistance = (ads.readADC_Differential_0_1() * multiplier); //dropp över den fasta resistorn
    delay(400);
    VoltOverThermistor = (ads.readADC_Differential_2_3() * multiplier);

    IseriesResistor = VoltOverseriesResistance / seriesResistor;
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

/*
 * Inputs ADC Value from Thermistor and outputs Temperature in Celsius
 *  requires: include <math.h>
 * Utilizes the Steinhart-Hart Thermistor Equation:
 *    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]3}
 *    where A = 0.001129148, B = 0.000234125 and C = 8.76741E-08
 *
 * These coefficients seem to work fairly universally, which is a bit of a 
 * surprise. 
 *
 * Schematic:
 *   [Ground] -- [10k-pad-resistor] -- | -- [thermistor] --[Vcc (5 or 3.3v)]
 *                                               |
 *                                          Analog Pin 0
 *
 * In case it isn't obvious (as it wasn't to me until I thought about it), the analog ports
 * measure the voltage between 0v -> Vcc which for an Arduino is a nominal 5v, but for (say) 
 * a JeeNode, is a nominal 3.3v.
 *
 * The resistance calculation uses the ratio of the two resistors, so the voltage
 * specified above is really only required for the debugging that is commented out below
 *
 * Resistance = PadResistor * (1024/ADC -1)  
 *
 * I have used this successfully with some CH Pipe Sensors (http://www.atcsemitec.co.uk/pdfdocs/ch.pdf)
 * which be obtained from http://www.rapidonline.co.uk.
 *
 */
