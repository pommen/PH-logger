

const int addr = 0x18;

byte internalTemp = 0;
byte extTempMSB = 0;
byte extTempLSB = 0;
byte status = 0;
int tempComp = 2; //This shold be calibrated, and not har set.
float extTemp = 0.000;

float getTemperatures(int external) //this function sorts samples and avreges them
{
    float sendback = 0.00;
    if (external == 0)
    {
        //get the internal temp
        //00                      Not Applicable          Local Temperature Value                          1000 0000 (0x80) (−128C)
        Wire.beginTransmission(addr);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(50);

        Wire.requestFrom(addr, 1);
        internalTemp = Wire.read();
        delay(50);
        sendback = internalTemp;
    }

    if (external == 1)
    {

        //01                       Not Applicable         Remote Temperature Value High Byte               1000 0000 (0x80) (−128C)
        Wire.beginTransmission(addr);
        Wire.write(0x01);
        Wire.endTransmission();

        Wire.requestFrom(addr, 1);
        extTempMSB = Wire.read();
        delay(100);

        //10                      Not Applicable          Remote Temperature Value Low Byte                0000 0000
        Wire.beginTransmission(addr);
        Wire.write(0x10);
        Wire.endTransmission();

        Wire.requestFrom(addr, 1);
        extTempLSB = Wire.read();

        extTempLSB = extTempLSB / 32;
        extTemp = extTempLSB * 0.125;
        extTemp += extTempMSB;
        sendback = extTemp + tempComp;
    }
    // extTemp += extTempLSB * 0.125;

    /*

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

    */

    return sendback;
}

void temperatureCal()
{
    int internalTemp=getTemperatures(0);
    float externalTemp = getTemperatures(1);


}
/*

Table 7. LIST OF ADM1023 REGISTERS

Read Address (Hex)       Write Address (Hex)         Name                                       Power-on Default

Not Applicable          Not Applicable          Address Pointer                                     Undefined
00                      Not Applicable          Local Temperature Value                          1000 0000 (0x80) (−128C)
01                       Not Applicable         Remote Temperature Value High Byte               1000 0000 (0x80) (−128C)
02                      Not Applicable          Status                                           Undefined
03                      09                      Configuration                                    0000 0000 (0x00)
04                      0A                      Conversion Rate                                  0000 0010 (0x02)
05                      0B                      Local Temperature High Limit                     0111 1111 (0x7F) (+127C)
06                      0C                      Local Temperature Low Limit                      1100 1001 (0xC9) (−55C)
07                      0D                      Remote Temperature High Limit High Byte          0111 1111 (0x7F) (+127C)
08                      0E                      Remote Temperature Low Limit High Byte           1100 1001 (0xC9) (−55C)
Not Applicable          0F (Note 1) One-shot
10                      Not Applicable          Remote Temperature Value Low Byte                0000 0000
11                      11                      Remote Temperature Offset High Byte              0000 0000
12                      12                      Remote Temperature Offset Low Byte               0000 0000
13                      13                      Remote Temperature High Limit Low Byte           0000 0000
14                      14                      Remote Temperature Low Limit Low Byte            0000 0000
19                      Not Applicable          Reserved                                         0000 0000
20                      21                      Reserved                                         Undefined
FE                      Not Applicable          Manufacturer Device ID                           0100 0001 (0x41)
FF                      Not Applicable          Die Revision Code                                0011 xxxx (0x3x)


1. Writing to Address 0F causes the ADM1023 to perform a single measurement. It is not a data register as such; thus, it does not matter what
data is written to it.

*/