EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
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
L power:VCC #PWR01
U 1 1 5932C624
P 4600 3100
F 0 "#PWR01" H 4600 2950 50  0001 C CNN
F 1 "VCC" H 4600 3250 50  0000 C CNN
F 2 "" H 4600 3100 50  0001 C CNN
F 3 "" H 4600 3100 50  0001 C CNN
	1    4600 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5932C638
P 3200 3100
F 0 "#PWR02" H 3200 2850 50  0001 C CNN
F 1 "GND" H 3200 2950 50  0000 C CNN
F 2 "" H 3200 3100 50  0001 C CNN
F 3 "" H 3200 3100 50  0001 C CNN
	1    3200 3100
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C3
U 1 1 5932C64C
P 1950 1750
F 0 "C3" H 1960 1820 50  0000 L CNN
F 1 "0,1u" H 1960 1670 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1950 1750 50  0001 C CNN
F 3 "" H 1950 1750 50  0001 C CNN
	1    1950 1750
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:DS3231 U4
U 1 1 5932CBF4
P 7550 1850
F 0 "U4" H 7400 2650 60  0000 C CNN
F 1 "DS3231" H 7350 1500 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-16W_7.5x10.3mm_Pitch1.27mm" H 7650 1850 60  0001 C CNN
F 3 "" H 7650 1850 60  0001 C CNN
	1    7550 1850
	1    0    0    -1  
$EndComp
$Comp
L Peters:SD-BREAKOUT D1
U 1 1 5932D399
P 5400 6250
F 0 "D1" H 5500 6750 60  0000 C CNN
F 1 "SD-BREAKOUT" H 5550 5750 60  0000 C CNN
F 2 "Libs:SD-breakout" H 5400 6250 60  0001 C CNN
F 3 "" H 5400 6250 60  0001 C CNN
	1    5400 6250
	1    0    0    -1  
$EndComp
$Comp
L Peters:74HCT541 U1
U 1 1 5932D416
P 3050 6500
F 0 "U1" H 3400 7400 60  0000 C CNN
F 1 "74HCT541" H 3400 6200 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-20W_7.5x12.8mm_Pitch1.27mm" H 3050 6500 60  0001 C CNN
F 3 "" H 3050 6500 60  0001 C CNN
	1    3050 6500
	1    0    0    -1  
$EndComp
$Comp
L Peters:pro-mini-deek U2
U 1 1 5932D487
P 3750 2950
F 0 "U2" H 3850 1100 60  0000 C CNN
F 1 "pro-mini-deek" H 4650 4600 60  0000 C CNN
F 2 "Libs:Pro-mini-deek-noheaders" H 4150 2950 60  0001 C CNN
F 3 "" H 4150 2950 60  0001 C CNN
	1    3750 2950
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C4
U 1 1 5932D4D6
P 2150 1750
F 0 "C4" H 2160 1820 50  0000 L CNN
F 1 "0,1u" H 2160 1670 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2150 1750 50  0001 C CNN
F 3 "" H 2150 1750 50  0001 C CNN
	1    2150 1750
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:Screw_Terminal_1x04 J3
U 1 1 5932D57D
P 10150 3300
F 0 "J3" H 10150 3750 50  0000 C TNN
F 1 "PH_PROBE" V 10000 3300 50  0000 C TNN
F 2 "Connectors_Phoenix:PhoenixContact_MC-G_04x5.08mm_Angled" H 10150 2875 50  0001 C CNN
F 3 "" H 10125 3500 50  0001 C CNN
	1    10150 3300
	-1   0    0    1   
$EndComp
$Comp
L device:Battery_Cell BT1
U 1 1 5932D5D8
P 8400 1800
F 0 "BT1" H 8500 1900 50  0000 L CNN
F 1 "Battery_Cell" H 8500 1800 50  0000 L CNN
F 2 "Battery_Holders:Keystone_1058_1x2032-CoinCell" V 8400 1860 50  0001 C CNN
F 3 "" V 8400 1860 50  0001 C CNN
	1    8400 1800
	1    0    0    -1  
$EndComp
$Comp
L device:CP C7
U 1 1 5932D635
P 2850 1750
F 0 "C7" H 2875 1850 50  0000 L CNN
F 1 "220u" H 2875 1650 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D10.0mm_P3.80mm" H 2888 1600 50  0001 C CNN
F 3 "" H 2850 1750 50  0001 C CNN
	1    2850 1750
	1    0    0    -1  
$EndComp
$Comp
L linear:LM321 U3
U 1 1 5932D71E
P 7100 5650
F 0 "U3" H 7100 5950 50  0000 L CNN
F 1 "LM321" H 7100 5850 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 7100 5500 50  0001 L CNN
F 3 "" H 7100 5650 50  0001 C CNN
	1    7100 5650
	1    0    0    -1  
$EndComp
$Comp
L Peters:3-pin-BoostConv U5
U 1 1 5932D815
P 7950 5550
F 0 "U5" H 8250 5550 60  0000 C CNN
F 1 "3-pin-BoostConv" H 8000 5800 60  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03_Pitch2.54mm" H 8150 5500 60  0001 C CNN
F 3 "" H 8150 5500 60  0001 C CNN
	1    7950 5550
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:Screw_Terminal_1x02 J1
U 1 1 5932DA92
P 1050 1900
F 0 "J1" H 1050 2150 50  0000 C TNN
F 1 "BATTERI CON" V 900 1900 50  0000 C TNN
F 2 "Connectors_Phoenix:PhoenixContact_MCV-G_02x3.50mm_Vertical" H 1050 1675 50  0001 C CNN
F 3 "" H 1025 1900 50  0001 C CNN
	1    1050 1900
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:CONN_01X04 J4
U 1 1 5932DCB2
P 10500 5600
F 0 "J4" H 10500 5850 50  0000 C CNN
F 1 "GPS" V 10600 5600 50  0000 C CNN
F 2 "Connectors_Phoenix:PhoenixContact_MCV-G_04x3.50mm_Vertical" H 10500 5600 50  0001 C CNN
F 3 "" H 10500 5600 50  0001 C CNN
	1    10500 5600
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR03
U 1 1 5932E87F
P 6450 1350
F 0 "#PWR03" H 6450 1200 50  0001 C CNN
F 1 "VCC" H 6450 1500 50  0000 C CNN
F 2 "" H 6450 1350 50  0001 C CNN
F 3 "" H 6450 1350 50  0001 C CNN
	1    6450 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5932E8B4
P 7900 1550
F 0 "#PWR04" H 7900 1300 50  0001 C CNN
F 1 "GND" H 7900 1400 50  0000 C CNN
F 2 "" H 7900 1550 50  0001 C CNN
F 3 "" H 7900 1550 50  0001 C CNN
	1    7900 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5932E967
P 8400 1900
F 0 "#PWR05" H 8400 1650 50  0001 C CNN
F 1 "GND" H 8400 1750 50  0000 C CNN
F 2 "" H 8400 1900 50  0001 C CNN
F 3 "" H 8400 1900 50  0001 C CNN
	1    8400 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 1350 6600 1350
Wire Wire Line
	7900 1450 8400 1450
Wire Wire Line
	8400 1450 8400 1600
Text GLabel 7900 1250 2    60   Input ~ 0
SCL
Text GLabel 7900 1350 2    60   Input ~ 0
SDA
Text GLabel 4950 3350 2    60   Input ~ 0
SDA
Text GLabel 5300 3300 2    60   Input ~ 0
SCL
Text GLabel 9950 3200 0    60   Input ~ 0
SDA
Text GLabel 9950 3000 0    60   Input ~ 0
SCL
$Comp
L power:GND #PWR06
U 1 1 5932FC04
P 9950 3600
F 0 "#PWR06" H 9950 3350 50  0001 C CNN
F 1 "GND" H 9950 3450 50  0000 C CNN
F 2 "" H 9950 3600 50  0001 C CNN
F 3 "" H 9950 3600 50  0001 C CNN
	1    9950 3600
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR07
U 1 1 5932FC39
P 9600 3400
F 0 "#PWR07" H 9600 3250 50  0001 C CNN
F 1 "VCC" H 9600 3550 50  0000 C CNN
F 2 "" H 9600 3400 50  0001 C CNN
F 3 "" H 9600 3400 50  0001 C CNN
	1    9600 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 3400 9950 3400
$Comp
L power:GND #PWR08
U 1 1 5933012A
P 3000 6700
F 0 "#PWR08" H 3000 6450 50  0001 C CNN
F 1 "GND" H 3000 6550 50  0000 C CNN
F 2 "" H 3000 6700 50  0001 C CNN
F 3 "" H 3000 6700 50  0001 C CNN
	1    3000 6700
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR09
U 1 1 5933015F
P 4950 6000
F 0 "#PWR09" H 4950 5850 50  0001 C CNN
F 1 "VCC" H 4950 6150 50  0000 C CNN
F 2 "" H 4950 6000 50  0001 C CNN
F 3 "" H 4950 6000 50  0001 C CNN
	1    4950 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 59330274
P 5200 6700
F 0 "#PWR010" H 5200 6450 50  0001 C CNN
F 1 "GND" H 5200 6550 50  0000 C CNN
F 2 "" H 5200 6700 50  0001 C CNN
F 3 "" H 5200 6700 50  0001 C CNN
	1    5200 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 6000 5200 6000
Wire Wire Line
	5200 6600 5200 6700
$Comp
L power:GND #PWR011
U 1 1 5933033A
P 5200 5900
F 0 "#PWR011" H 5200 5650 50  0001 C CNN
F 1 "GND" H 5200 5750 50  0000 C CNN
F 2 "" H 5200 5900 50  0001 C CNN
F 3 "" H 5200 5900 50  0001 C CNN
	1    5200 5900
	-1   0    0    1   
$EndComp
Text GLabel 5200 6500 0    60   Input ~ 0
SD_MISO
Text GLabel 3800 6400 2    60   Input ~ 0
SD_MOSI
Text GLabel 5200 6400 0    60   Input ~ 0
SD_SCK
Text GLabel 5200 6200 0    60   Input ~ 0
SD_CS
Text GLabel 10000 5650 0    60   Input ~ 0
TX_UART
Text GLabel 10000 5550 0    60   Input ~ 0
RX_UART
$Comp
L power:GND #PWR012
U 1 1 5933189E
P 10300 5750
F 0 "#PWR012" H 10300 5500 50  0001 C CNN
F 1 "GND" H 10300 5600 50  0000 C CNN
F 2 "" H 10300 5750 50  0001 C CNN
F 3 "" H 10300 5750 50  0001 C CNN
	1    10300 5750
	1    0    0    -1  
$EndComp
Text GLabel 3000 5900 0    60   Input ~ 0
TX_UART
Text GLabel 3800 6100 2    60   Input ~ 0
RX_UART
$Comp
L power:GND #PWR013
U 1 1 593323AA
P 4600 2800
F 0 "#PWR013" H 4600 2550 50  0001 C CNN
F 1 "GND" H 4600 2650 50  0000 C CNN
F 2 "" H 4600 2800 50  0001 C CNN
F 3 "" H 4600 2800 50  0001 C CNN
	1    4600 2800
	1    0    0    -1  
$EndComp
Text GLabel 3200 3700 0    60   Input ~ 0
SD_ON
Text GLabel 3000 5800 0    60   Input ~ 0
SD_ON
$Comp
L power:VCC #PWR014
U 1 1 59332621
P 3800 5800
F 0 "#PWR014" H 3800 5650 50  0001 C CNN
F 1 "VCC" H 3800 5950 50  0000 C CNN
F 2 "" H 3800 5800 50  0001 C CNN
F 3 "" H 3800 5800 50  0001 C CNN
	1    3800 5800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 59332D87
P 1250 2000
F 0 "#PWR015" H 1250 1750 50  0001 C CNN
F 1 "GND" H 1250 1850 50  0000 C CNN
F 2 "" H 1250 2000 50  0001 C CNN
F 3 "" H 1250 2000 50  0001 C CNN
	1    1250 2000
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR016
U 1 1 59332E73
P 1250 1800
F 0 "#PWR016" H 1250 1650 50  0001 C CNN
F 1 "VCC" H 1250 1950 50  0000 C CNN
F 2 "" H 1250 1800 50  0001 C CNN
F 3 "" H 1250 1800 50  0001 C CNN
	1    1250 1800
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C5
U 1 1 593331A0
P 2350 1750
F 0 "C5" H 2360 1820 50  0000 L CNN
F 1 "0,1u" H 2360 1670 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2350 1750 50  0001 C CNN
F 3 "" H 2350 1750 50  0001 C CNN
	1    2350 1750
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C2
U 1 1 593331E4
P 9300 5600
F 0 "C2" H 9310 5670 50  0000 L CNN
F 1 "1u" H 9310 5520 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 9300 5600 50  0001 C CNN
F 3 "" H 9300 5600 50  0001 C CNN
	1    9300 5600
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C6
U 1 1 5933322B
P 2550 1750
F 0 "C6" H 2560 1820 50  0000 L CNN
F 1 "0,1u" H 2560 1670 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2550 1750 50  0001 C CNN
F 3 "" H 2550 1750 50  0001 C CNN
	1    2550 1750
	1    0    0    -1  
$EndComp
$Comp
L device:C_Small C1
U 1 1 59333275
P 1600 1750
F 0 "C1" H 1610 1820 50  0000 L CNN
F 1 "1u" H 1610 1670 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 1600 1750 50  0001 C CNN
F 3 "" H 1600 1750 50  0001 C CNN
	1    1600 1750
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR017
U 1 1 593332C4
P 2150 1650
F 0 "#PWR017" H 2150 1500 50  0001 C CNN
F 1 "VCC" H 2150 1800 50  0000 C CNN
F 2 "" H 2150 1650 50  0001 C CNN
F 3 "" H 2150 1650 50  0001 C CNN
	1    2150 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR018
U 1 1 59333308
P 2150 1850
F 0 "#PWR018" H 2150 1600 50  0001 C CNN
F 1 "GND" H 2150 1700 50  0000 C CNN
F 2 "" H 2150 1850 50  0001 C CNN
F 3 "" H 2150 1850 50  0001 C CNN
	1    2150 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 1650 2550 1600
Wire Wire Line
	2550 1600 2850 1600
Wire Wire Line
	2850 1900 2550 1900
Wire Wire Line
	2550 1900 2550 1850
Connection ~ 1950 1850
Connection ~ 2350 1850
Connection ~ 2150 1850
Connection ~ 2150 1650
Connection ~ 2350 1650
Connection ~ 1950 1650
Text GLabel 3000 6100 0    60   Input ~ 0
SD_MISO
Text GLabel 3800 6500 2    60   Input ~ 0
SD_CS
Text GLabel 3800 6300 2    60   Input ~ 0
SD_SCK
Text GLabel 3800 6000 2    60   Input ~ 0
TX_UART_MCU
Text GLabel 3000 6000 0    60   Input ~ 0
RX_UART_MCU
Text GLabel 3200 2650 0    60   Input ~ 0
RX_UART_MCU
Text GLabel 3200 2800 0    60   Input ~ 0
TX_UART_MCU
Text GLabel 3800 6200 2    60   Input ~ 0
MCU_MISO
Text GLabel 3000 6300 0    60   Input ~ 0
MCU_MOSI
Text GLabel 3000 6200 0    60   Input ~ 0
MCU_SCK
Text GLabel 3000 6400 0    60   Input ~ 0
MCU_CS
Text GLabel 3200 4000 0    60   Input ~ 0
GPS_ON
Text GLabel 4600 4000 2    60   Input ~ 0
MCU_MISO
Text GLabel 3200 3550 0    60   Input ~ 0
MCU_CS
Text GLabel 4600 3850 2    60   Input ~ 0
MCU_SCK
Text GLabel 4600 4150 2    60   Input ~ 0
MCU_MOSI
Text GLabel 6500 1650 0    60   Input ~ 0
INT
Wire Wire Line
	6500 1650 6600 1650
Wire Wire Line
	6800 1650 6800 1450
Wire Wire Line
	6800 1450 6850 1450
$Comp
L device:R R3
U 1 1 59338033
P 6600 1500
F 0 "R3" V 6680 1500 50  0000 C CNN
F 1 "10K" V 6600 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6530 1500 50  0001 C CNN
F 3 "" H 6600 1500 50  0001 C CNN
	1    6600 1500
	1    0    0    -1  
$EndComp
Connection ~ 6600 1650
Connection ~ 6600 1350
$Comp
L power:VCC #PWR019
U 1 1 593397B0
P 7800 5850
F 0 "#PWR019" H 7800 5700 50  0001 C CNN
F 1 "VCC" H 7800 6000 50  0000 C CNN
F 2 "" H 7800 5850 50  0001 C CNN
F 3 "" H 7800 5850 50  0001 C CNN
	1    7800 5850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR020
U 1 1 593397F7
P 7000 5950
F 0 "#PWR020" H 7000 5700 50  0001 C CNN
F 1 "GND" H 7000 5800 50  0000 C CNN
F 2 "" H 7000 5950 50  0001 C CNN
F 3 "" H 7000 5950 50  0001 C CNN
	1    7000 5950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR021
U 1 1 5933983E
P 7000 5350
F 0 "#PWR021" H 7000 5200 50  0001 C CNN
F 1 "VCC" H 7000 5500 50  0000 C CNN
F 2 "" H 7000 5350 50  0001 C CNN
F 3 "" H 7000 5350 50  0001 C CNN
	1    7000 5350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 59339ADD
P 7950 5850
F 0 "#PWR022" H 7950 5600 50  0001 C CNN
F 1 "GND" H 7950 5700 50  0000 C CNN
F 2 "" H 7950 5850 50  0001 C CNN
F 3 "" H 7950 5850 50  0001 C CNN
	1    7950 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 5850 8800 5850
Wire Wire Line
	7400 5400 7400 5650
Wire Wire Line
	7400 6250 6800 6250
Wire Wire Line
	6800 6250 6800 5750
Text GLabel 6100 5450 0    60   Input ~ 0
GPS_ON
$Comp
L phloggerV2-rescue:CONN_01X01 J2
U 1 1 5933A18A
P 7400 5200
F 0 "J2" H 7400 5300 50  0000 C CNN
F 1 "boost_conv_ena" V 7500 5200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x01_Pitch2.54mm" H 7400 5200 50  0001 C CNN
F 3 "" H 7400 5200 50  0001 C CNN
	1    7400 5200
	0    -1   -1   0   
$EndComp
Connection ~ 7400 5650
$Comp
L device:R R1
U 1 1 5933ACA9
P 5400 2750
F 0 "R1" V 5480 2750 50  0000 C CNN
F 1 "4k7" V 5400 2750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5330 2750 50  0001 C CNN
F 3 "" H 5400 2750 50  0001 C CNN
	1    5400 2750
	1    0    0    -1  
$EndComp
$Comp
L device:R R2
U 1 1 5933AD43
P 5650 2750
F 0 "R2" V 5730 2750 50  0000 C CNN
F 1 "4k7" V 5650 2750 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 5580 2750 50  0001 C CNN
F 3 "" H 5650 2750 50  0001 C CNN
	1    5650 2750
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR023
U 1 1 5933ADA6
P 5500 2450
F 0 "#PWR023" H 5500 2300 50  0001 C CNN
F 1 "VCC" H 5500 2600 50  0000 C CNN
F 2 "" H 5500 2450 50  0001 C CNN
F 3 "" H 5500 2450 50  0001 C CNN
	1    5500 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 2600 5500 2600
Wire Wire Line
	5500 2450 5500 2600
Connection ~ 5500 2600
Wire Wire Line
	4600 3350 4750 3350
Wire Wire Line
	4600 3300 4850 3300
Wire Wire Line
	4750 2900 4750 3350
Connection ~ 4750 3350
Wire Wire Line
	5650 3000 5650 2900
Wire Wire Line
	4850 3000 4850 3300
Connection ~ 4850 3300
Text GLabel 3200 3400 0    60   Input ~ 0
INT
Text GLabel 5200 6300 0    60   Input ~ 0
SD_MOSI
$Comp
L power:GND #PWR024
U 1 1 59343C0A
P 6050 5900
F 0 "#PWR024" H 6050 5650 50  0001 C CNN
F 1 "GND" H 6050 5750 50  0000 C CNN
F 2 "" H 6050 5900 50  0001 C CNN
F 3 "" H 6050 5900 50  0001 C CNN
	1    6050 5900
	-1   0    0    1   
$EndComp
$Comp
L power:VCC #PWR025
U 1 1 59343C5D
P 6050 6000
F 0 "#PWR025" H 6050 5850 50  0001 C CNN
F 1 "VCC" H 6050 6150 50  0000 C CNN
F 2 "" H 6050 6000 50  0001 C CNN
F 3 "" H 6050 6000 50  0001 C CNN
	1    6050 6000
	1    0    0    -1  
$EndComp
Text GLabel 6050 6200 2    60   Input ~ 0
SD_CS
Text GLabel 6050 6300 2    60   Input ~ 0
SD_MOSI
Text GLabel 6050 6400 2    60   Input ~ 0
SD_SCK
Text GLabel 6050 6500 2    60   Input ~ 0
SD_MISO
$Comp
L power:GND #PWR026
U 1 1 593440A8
P 6050 6600
F 0 "#PWR026" H 6050 6350 50  0001 C CNN
F 1 "GND" H 6050 6450 50  0000 C CNN
F 2 "" H 6050 6600 50  0001 C CNN
F 3 "" H 6050 6600 50  0001 C CNN
	1    6050 6600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 59342824
P 4600 5900
F 0 "#PWR027" H 4600 5650 50  0001 C CNN
F 1 "GND" H 4600 5750 50  0000 C CNN
F 2 "" H 4600 5900 50  0001 C CNN
F 3 "" H 4600 5900 50  0001 C CNN
	1    4600 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 5900 4600 5900
$Comp
L device:R R4
U 1 1 593DAFBB
P 6450 5600
F 0 "R4" V 6530 5600 50  0000 C CNN
F 1 "10k" V 6450 5600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 6380 5600 50  0001 C CNN
F 3 "" H 6450 5600 50  0001 C CNN
	1    6450 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 5450 6450 5450
Wire Wire Line
	6800 5450 6800 5550
Connection ~ 6450 5450
$Comp
L power:GND #PWR028
U 1 1 593DB483
P 6450 5750
F 0 "#PWR028" H 6450 5500 50  0001 C CNN
F 1 "GND" H 6450 5600 50  0000 C CNN
F 2 "" H 6450 5750 50  0001 C CNN
F 3 "" H 6450 5750 50  0001 C CNN
	1    6450 5750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 593DD712
P 9300 5700
F 0 "#PWR029" H 9300 5450 50  0001 C CNN
F 1 "GND" H 9300 5550 50  0000 C CNN
F 2 "" H 9300 5700 50  0001 C CNN
F 3 "" H 9300 5700 50  0001 C CNN
	1    9300 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 5850 8800 5450
Wire Wire Line
	8800 5450 9300 5450
Wire Wire Line
	9300 5500 9300 5450
Connection ~ 9300 5450
$Comp
L phloggerV2-rescue:TEST TP8
U 1 1 59441B60
P 8750 3850
F 0 "TP8" H 8828 3990 50  0000 L CNN
F 1 "TEST" H 8828 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 8750 3850 50  0001 C CNN
F 3 "" H 8750 3850 50  0001 C CNN
	1    8750 3850
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:TEST TP7
U 1 1 594421F1
P 8350 3850
F 0 "TP7" H 8428 3990 50  0000 L CNN
F 1 "TEST" H 8428 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 8350 3850 50  0001 C CNN
F 3 "" H 8350 3850 50  0001 C CNN
	1    8350 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 5550 10300 5550
$Comp
L phloggerV2-rescue:TEST TP6
U 1 1 59442AAE
P 8000 3850
F 0 "TP6" H 8078 3990 50  0000 L CNN
F 1 "TEST" H 8078 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 8000 3850 50  0001 C CNN
F 3 "" H 8000 3850 50  0001 C CNN
	1    8000 3850
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:TEST TP5
U 1 1 594437D3
P 7650 3850
F 0 "TP5" H 7728 3990 50  0000 L CNN
F 1 "TEST" H 7728 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 7650 3850 50  0001 C CNN
F 3 "" H 7650 3850 50  0001 C CNN
	1    7650 3850
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:TEST TP1
U 1 1 59443E5D
P 6450 3850
F 0 "TP1" H 6528 3990 50  0000 L CNN
F 1 "TEST" H 6528 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 6450 3850 50  0001 C CNN
F 3 "" H 6450 3850 50  0001 C CNN
	1    6450 3850
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:TEST TP2
U 1 1 594450E6
P 6700 3850
F 0 "TP2" H 6778 3990 50  0000 L CNN
F 1 "TEST" H 6778 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 6700 3850 50  0001 C CNN
F 3 "" H 6700 3850 50  0001 C CNN
	1    6700 3850
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:TEST TP3
U 1 1 59445134
P 7000 3850
F 0 "TP3" H 7078 3990 50  0000 L CNN
F 1 "TEST" H 7078 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 7000 3850 50  0001 C CNN
F 3 "" H 7000 3850 50  0001 C CNN
	1    7000 3850
	1    0    0    -1  
$EndComp
$Comp
L phloggerV2-rescue:TEST TP4
U 1 1 59445186
P 7300 3850
F 0 "TP4" H 7378 3990 50  0000 L CNN
F 1 "TEST" H 7378 3899 50  0000 L CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x01_Pitch2.54mm" H 7300 3850 50  0001 C CNN
F 3 "" H 7300 3850 50  0001 C CNN
	1    7300 3850
	1    0    0    -1  
$EndComp
Text GLabel 6450 3850 3    60   Input ~ 0
SD_CS
Text GLabel 6700 3850 3    60   Input ~ 0
SD_MOSI
Text GLabel 7000 3850 3    60   Input ~ 0
SD_SCK
Text GLabel 7300 3850 3    60   Input ~ 0
SD_MISO
Text GLabel 7650 3850 3    60   Input ~ 0
SCL
Text GLabel 8000 3850 3    60   Input ~ 0
SDA
Text GLabel 8750 3850 3    60   Input ~ 0
TX_UART
Text GLabel 8350 3850 3    60   Input ~ 0
RX_UART
Wire Wire Line
	1950 1850 2150 1850
Wire Wire Line
	2350 1850 2550 1850
Wire Wire Line
	2150 1850 2350 1850
Wire Wire Line
	2150 1650 2350 1650
Wire Wire Line
	2350 1650 2550 1650
Wire Wire Line
	1950 1650 2150 1650
Wire Wire Line
	6600 1650 6800 1650
Wire Wire Line
	6600 1350 6850 1350
Wire Wire Line
	7400 5650 7400 6250
Wire Wire Line
	5500 2600 5650 2600
Wire Wire Line
	4750 3350 4950 3350
Wire Wire Line
	4850 3300 5300 3300
Wire Wire Line
	6450 5450 6800 5450
Wire Wire Line
	9300 5450 10300 5450
Wire Wire Line
	1600 1650 1950 1650
Wire Wire Line
	1600 1850 1950 1850
Wire Wire Line
	4750 2900 5400 2900
Wire Wire Line
	4850 3000 5650 3000
Wire Wire Line
	10000 5650 10300 5650
$EndSCHEMATC
