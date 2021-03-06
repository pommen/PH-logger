EESchema Schematic File Version 4
LIBS:Board-cache
EELAYER 29 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Peters:EZO_BNC U?
U 1 1 5D0732DA
P 10300 4500
AR Path="/5D0732DA" Ref="U?"  Part="1" 
AR Path="/5D06A739/5D0732DA" Ref="U22"  Part="1" 
F 0 "U22" H 9900 4900 60  0000 L CNN
F 1 "EZO_BNC" H 9800 4400 60  0000 L CNN
F 2 "Libs:EZO_BNC" H 10300 4500 60  0001 C CNN
F 3 "" H 10300 4500 60  0001 C CNN
	1    10300 4500
	1    0    0    -1  
$EndComp
$Comp
L Peters:EZ_PH U?
U 1 1 5D0732E0
P 8600 4300
AR Path="/5D0732E0" Ref="U?"  Part="1" 
AR Path="/5D06A739/5D0732E0" Ref="U21"  Part="1" 
F 0 "U21" H 8875 4575 50  0000 C CNN
F 1 "EZ_PH" H 8875 4484 50  0000 C CNN
F 2 "Libs:atlas-scientific.ezo.ph" H 8900 4650 50  0001 C CNN
F 3 "https://www.atlas-scientific.com/_files/_datasheets/_circuit/pH_EZO_Datasheet.pdf" H 9150 4800 50  0001 C CNN
	1    8600 4300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5D0732E8
P 9150 3650
AR Path="/5D0732E8" Ref="#PWR?"  Part="1" 
AR Path="/5D06A739/5D0732E8" Ref="#PWR0216"  Part="1" 
F 0 "#PWR0216" H 9150 3500 50  0001 C CNN
F 1 "VCC" H 9167 3823 50  0000 C CNN
F 2 "" H 9150 3650 50  0001 C CNN
F 3 "" H 9150 3650 50  0001 C CNN
	1    9150 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D0732EE
P 8400 4550
AR Path="/5D0732EE" Ref="#PWR?"  Part="1" 
AR Path="/5D06A739/5D0732EE" Ref="#PWR0217"  Part="1" 
F 0 "#PWR0217" H 8400 4300 50  0001 C CNN
F 1 "GND" H 8405 4377 50  0000 C CNN
F 2 "" H 8400 4550 50  0001 C CNN
F 3 "" H 8400 4550 50  0001 C CNN
	1    8400 4550
	1    0    0    -1  
$EndComp
$Comp
L Board-rescue:CP1_Small-device-Main-rescue C?
U 1 1 5D0732F4
P 9150 3800
AR Path="/5D0732F4" Ref="C?"  Part="1" 
AR Path="/5D06A739/5D0732F4" Ref="C42"  Part="1" 
F 0 "C42" H 9241 3846 50  0000 L CNN
F 1 "10u" H 9250 3750 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_4x5.3" H 9150 3800 50  0001 C CNN
F 3 "~" H 9150 3800 50  0001 C CNN
	1    9150 3800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5D0732FA
P 9150 3950
AR Path="/5D0732FA" Ref="#PWR?"  Part="1" 
AR Path="/5D06A739/5D0732FA" Ref="#PWR0218"  Part="1" 
F 0 "#PWR0218" H 9150 3700 50  0001 C CNN
F 1 "GND" H 9155 3777 50  0000 C CNN
F 2 "" H 9150 3950 50  0001 C CNN
F 3 "" H 9150 3950 50  0001 C CNN
	1    9150 3950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5D073300
P 8400 4250
AR Path="/5D073300" Ref="#PWR?"  Part="1" 
AR Path="/5D06A739/5D073300" Ref="#PWR0219"  Part="1" 
F 0 "#PWR0219" H 8400 4100 50  0001 C CNN
F 1 "VCC" H 8417 4423 50  0000 C CNN
F 2 "" H 8400 4250 50  0001 C CNN
F 3 "" H 8400 4250 50  0001 C CNN
	1    8400 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 3950 9150 3900
Wire Wire Line
	9150 3650 9150 3700
$Comp
L Board-rescue:C_Small-device-Main-rescue C?
U 1 1 5D073308
P 8950 3800
AR Path="/5D073308" Ref="C?"  Part="1" 
AR Path="/5D06A739/5D073308" Ref="C41"  Part="1" 
F 0 "C41" H 8750 3900 50  0000 L CNN
F 1 "100n" H 8700 3800 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8950 3800 50  0001 C CNN
F 3 "~" H 8950 3800 50  0001 C CNN
	1    8950 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 3700 8950 3650
Wire Wire Line
	8950 3650 9150 3650
Connection ~ 9150 3650
Wire Wire Line
	8950 3900 8950 3950
Wire Wire Line
	8950 3950 9150 3950
Connection ~ 9150 3950
Wire Notes Line
	8000 3100 10250 3100
Wire Notes Line
	10250 3100 10250 5200
Wire Notes Line
	10250 5200 8000 5200
Wire Notes Line
	8000 5200 8000 3100
Text Notes 9700 3250 0    50   ~ 10
PH sensor
Wire Wire Line
	9600 4350 9550 4350
Wire Wire Line
	9600 4450 9550 4450
$Comp
L power:GND #PWR?
U 1 1 5D07331B
P 9600 4200
AR Path="/5D07331B" Ref="#PWR?"  Part="1" 
AR Path="/5D06A739/5D07331B" Ref="#PWR0220"  Part="1" 
F 0 "#PWR0220" H 9600 3950 50  0001 C CNN
F 1 "GND" H 9605 4027 50  0000 C CNN
F 2 "" H 9600 4200 50  0001 C CNN
F 3 "" H 9600 4200 50  0001 C CNN
	1    9600 4200
	-1   0    0    1   
$EndComp
Wire Wire Line
	9600 4250 9600 4200
$Comp
L Sensor:DHT11 U12
U 1 1 5D0B5719
P 8600 1950
F 0 "U12" H 8356 1996 50  0000 R CNN
F 1 "DHT11" H 8356 1905 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 8600 1550 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 8750 2200 50  0001 C CNN
	1    8600 1950
	1    0    0    -1  
$EndComp
Text HLabel 8400 4350 0    50   Input ~ 0
I2C1_SDA
Text HLabel 8400 4450 0    50   Input ~ 0
I2C1_SCL
$Comp
L power:VCC #PWR?
U 1 1 5C9383A7
P 8600 1500
F 0 "#PWR?" H 8600 1350 50  0001 C CNN
F 1 "VCC" H 8617 1673 50  0000 C CNN
F 2 "" H 8600 1500 50  0001 C CNN
F 3 "" H 8600 1500 50  0001 C CNN
	1    8600 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C938D5D
P 8600 2350
F 0 "#PWR?" H 8600 2100 50  0001 C CNN
F 1 "GND" H 8605 2177 50  0000 C CNN
F 2 "" H 8600 2350 50  0001 C CNN
F 3 "" H 8600 2350 50  0001 C CNN
	1    8600 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 2350 8600 2250
Wire Wire Line
	8600 1500 8600 1650
$Comp
L Device:C_Small C?
U 1 1 5C9395C3
P 8050 1650
F 0 "C?" H 8142 1696 50  0000 L CNN
F 1 "100n" H 8142 1605 50  0000 L CNN
F 2 "" H 8050 1650 50  0001 C CNN
F 3 "~" H 8050 1650 50  0001 C CNN
	1    8050 1650
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C939E02
P 8050 1550
F 0 "#PWR?" H 8050 1400 50  0001 C CNN
F 1 "VCC" H 8067 1723 50  0000 C CNN
F 2 "" H 8050 1550 50  0001 C CNN
F 3 "" H 8050 1550 50  0001 C CNN
	1    8050 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C93A366
P 8050 1750
F 0 "#PWR?" H 8050 1500 50  0001 C CNN
F 1 "GND" H 8055 1577 50  0000 C CNN
F 2 "" H 8050 1750 50  0001 C CNN
F 3 "" H 8050 1750 50  0001 C CNN
	1    8050 1750
	1    0    0    -1  
$EndComp
Text HLabel 8900 1950 2    50   Output ~ 0
1WireDHT
Wire Notes Line
	9300 1300 9300 2650
Wire Notes Line
	9300 2650 7850 2650
Wire Notes Line
	7850 2650 7850 1250
Wire Notes Line
	7850 1250 9300 1250
Text Notes 8500 1200 0    50   ~ 0
Mousture and temp
$Comp
L Peters:MAX6575 U?
U 1 1 5C944929
P 5850 1700
F 0 "U?" H 5800 1825 50  0000 C CNN
F 1 "MAX6575" H 5800 1734 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6_Handsoldering" H 5850 1700 50  0001 C CNN
F 3 "http://192.168.0.102/api/part_attachments/1162/getFile" H 5850 1700 50  0001 C CNN
	1    5850 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C9472F7
P 6000 1250
F 0 "C?" H 6092 1296 50  0000 L CNN
F 1 "100n" H 6092 1205 50  0000 L CNN
F 2 "" H 6000 1250 50  0001 C CNN
F 3 "~" H 6000 1250 50  0001 C CNN
	1    6000 1250
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C947301
P 6000 1150
F 0 "#PWR?" H 6000 1000 50  0001 C CNN
F 1 "VCC" H 6017 1323 50  0000 C CNN
F 2 "" H 6000 1150 50  0001 C CNN
F 3 "" H 6000 1150 50  0001 C CNN
	1    6000 1150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C94730B
P 6000 1350
F 0 "#PWR?" H 6000 1100 50  0001 C CNN
F 1 "GND" H 6005 1177 50  0000 C CNN
F 2 "" H 6000 1350 50  0001 C CNN
F 3 "" H 6000 1350 50  0001 C CNN
	1    6000 1350
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C949649
P 5350 1800
F 0 "#PWR?" H 5350 1650 50  0001 C CNN
F 1 "VCC" H 5367 1973 50  0000 C CNN
F 2 "" H 5350 1800 50  0001 C CNN
F 3 "" H 5350 1800 50  0001 C CNN
	1    5350 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C949D8E
P 5350 2000
F 0 "#PWR?" H 5350 1750 50  0001 C CNN
F 1 "GND" H 5355 1827 50  0000 C CNN
F 2 "" H 5350 2000 50  0001 C CNN
F 3 "" H 5350 2000 50  0001 C CNN
	1    5350 2000
	1    0    0    -1  
$EndComp
Text HLabel 6400 1800 2    50   Input ~ 0
MAX6475L_IO
$Comp
L power:VCC #PWR?
U 1 1 5C94AC42
P 6250 2000
F 0 "#PWR?" H 6250 1850 50  0001 C CNN
F 1 "VCC" V 6267 2128 50  0000 L CNN
F 2 "" H 6250 2000 50  0001 C CNN
F 3 "" H 6250 2000 50  0001 C CNN
	1    6250 2000
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C94C745
P 6250 1900
F 0 "#PWR?" H 6250 1750 50  0001 C CNN
F 1 "VCC" V 6267 2028 50  0000 L CNN
F 2 "" H 6250 1900 50  0001 C CNN
F 3 "" H 6250 1900 50  0001 C CNN
	1    6250 1900
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 5C94CA60
P 6350 1500
F 0 "R?" H 6420 1546 50  0000 L CNN
F 1 "10K" H 6420 1455 50  0000 L CNN
F 2 "" V 6280 1500 50  0001 C CNN
F 3 "~" H 6350 1500 50  0001 C CNN
	1    6350 1500
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C94D271
P 6350 1300
F 0 "#PWR?" H 6350 1150 50  0001 C CNN
F 1 "VCC" H 6367 1473 50  0000 C CNN
F 2 "" H 6350 1300 50  0001 C CNN
F 3 "" H 6350 1300 50  0001 C CNN
	1    6350 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 1350 6350 1300
Wire Wire Line
	6250 1800 6350 1800
Wire Wire Line
	6350 1650 6350 1800
Connection ~ 6350 1800
Wire Wire Line
	6350 1800 6400 1800
Wire Notes Line
	5150 800  7050 800 
Wire Notes Line
	7050 800  7050 4000
Wire Notes Line
	7050 4000 5150 4000
Wire Notes Line
	5150 4000 5150 800 
Text Notes 6550 900  0    50   ~ 0
Temperature
$Comp
L Peters:SHT21 U?
U 1 1 5C9549D9
P 5800 3400
F 0 "U?" H 5800 3675 50  0000 C CNN
F 1 "SHT21" H 5800 3584 50  0000 C CNN
F 2 "Package_DFN_QFN:DFN-6-1EP_3x3mm_P1mm_EP1.5x2.4mm" H 5800 3400 50  0001 C CNN
F 3 "http://192.168.0.102/api/part_attachments/1295/getFile" H 5800 3400 50  0001 C CNN
	1    5800 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C9554C4
P 6100 2900
F 0 "C?" H 6192 2946 50  0000 L CNN
F 1 "100n" H 6192 2855 50  0000 L CNN
F 2 "" H 6100 2900 50  0001 C CNN
F 3 "~" H 6100 2900 50  0001 C CNN
	1    6100 2900
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C9554CE
P 6100 2800
F 0 "#PWR?" H 6100 2650 50  0001 C CNN
F 1 "VCC" H 6117 2973 50  0000 C CNN
F 2 "" H 6100 2800 50  0001 C CNN
F 3 "" H 6100 2800 50  0001 C CNN
	1    6100 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C9554D8
P 6100 3000
F 0 "#PWR?" H 6100 2750 50  0001 C CNN
F 1 "GND" H 6105 2827 50  0000 C CNN
F 2 "" H 6100 3000 50  0001 C CNN
F 3 "" H 6100 3000 50  0001 C CNN
	1    6100 3000
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C957A72
P 5400 3350
F 0 "#PWR?" H 5400 3200 50  0001 C CNN
F 1 "VCC" H 5417 3523 50  0000 C CNN
F 2 "" H 5400 3350 50  0001 C CNN
F 3 "" H 5400 3350 50  0001 C CNN
	1    5400 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C958498
P 5400 3500
F 0 "#PWR?" H 5400 3250 50  0001 C CNN
F 1 "GND" H 5405 3327 50  0000 C CNN
F 2 "" H 5400 3500 50  0001 C CNN
F 3 "" H 5400 3500 50  0001 C CNN
	1    5400 3500
	1    0    0    -1  
$EndComp
Text HLabel 6200 3350 2    50   Input ~ 0
I2C1_SDA
Text HLabel 6200 3500 2    50   Input ~ 0
I2C1_SCL
Wire Notes Line
	5050 5200 6600 5200
Wire Notes Line
	6600 5200 6600 6500
Wire Notes Line
	6600 6500 5050 6500
Wire Notes Line
	5050 6500 5050 5200
Text Notes 5650 5300 0    50   ~ 0
Digital pressure sensor
Wire Wire Line
	5200 5600 5200 5550
Connection ~ 5200 5600
Wire Wire Line
	5250 5600 5200 5600
Wire Wire Line
	5200 5750 5200 5600
Wire Wire Line
	5250 5750 5200 5750
$Comp
L power:GND #PWR?
U 1 1 5C963582
P 5250 5900
F 0 "#PWR?" H 5250 5650 50  0001 C CNN
F 1 "GND" H 5255 5727 50  0000 C CNN
F 2 "" H 5250 5900 50  0001 C CNN
F 3 "" H 5250 5900 50  0001 C CNN
	1    5250 5900
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C962E9E
P 5200 5550
F 0 "#PWR?" H 5200 5400 50  0001 C CNN
F 1 "VCC" H 5217 5723 50  0000 C CNN
F 2 "" H 5200 5550 50  0001 C CNN
F 3 "" H 5200 5550 50  0001 C CNN
	1    5200 5550
	1    0    0    -1  
$EndComp
Text HLabel 6150 5650 2    50   Input ~ 0
I2C1_SDA
Text HLabel 6150 5800 2    50   Input ~ 0
I2C1_SCL
$Comp
L Peters:BMP180 U?
U 1 1 5C9603F8
P 5700 5750
F 0 "U?" H 5700 6125 50  0000 C CNN
F 1 "BMP180" H 5700 6034 50  0000 C CNN
F 2 "" H 5700 6100 50  0001 C CNN
F 3 "http://192.168.0.102/api/part_attachments/1274/getFile" H 5700 6100 50  0001 C CNN
	1    5700 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C9611A0
P 6250 6300
F 0 "#PWR?" H 6250 6050 50  0001 C CNN
F 1 "GND" H 6255 6127 50  0000 C CNN
F 2 "" H 6250 6300 50  0001 C CNN
F 3 "" H 6250 6300 50  0001 C CNN
	1    6250 6300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C961196
P 6250 6100
F 0 "#PWR?" H 6250 5950 50  0001 C CNN
F 1 "VCC" H 6267 6273 50  0000 C CNN
F 2 "" H 6250 6100 50  0001 C CNN
F 3 "" H 6250 6100 50  0001 C CNN
	1    6250 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C96118C
P 6250 6200
F 0 "C?" H 6342 6246 50  0000 L CNN
F 1 "100n" H 6342 6155 50  0000 L CNN
F 2 "" H 6250 6200 50  0001 C CNN
F 3 "~" H 6250 6200 50  0001 C CNN
	1    6250 6200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
