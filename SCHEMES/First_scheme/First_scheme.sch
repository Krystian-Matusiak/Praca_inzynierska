EESchema Schematic File Version 4
EELAYER 30 0
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
L MCU_ST_STM32F3:STM32F303K8Tx U2
U 1 1 60E409DE
P 7500 5150
F 0 "U2" H 7450 4061 50  0000 C CNN
F 1 "STM32F303K8Tx" H 7450 3970 50  0000 C CNN
F 2 "Package_QFP:LQFP-32_7x7mm_P0.8mm" H 7000 4250 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00092070.pdf" H 7500 5150 50  0001 C CNN
	1    7500 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 60E424BD
P 1900 1450
F 0 "R1" H 1970 1496 50  0000 L CNN
F 1 "1k" H 1970 1405 50  0000 L CNN
F 2 "" V 1830 1450 50  0001 C CNN
F 3 "~" H 1900 1450 50  0001 C CNN
	1    1900 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60E42BAC
P 4750 3750
F 0 "R2" H 4820 3796 50  0000 L CNN
F 1 "1k" H 4820 3705 50  0000 L CNN
F 2 "" V 4680 3750 50  0001 C CNN
F 3 "~" H 4750 3750 50  0001 C CNN
	1    4750 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 60E42D44
P 5200 1650
F 0 "R3" H 5270 1696 50  0000 L CNN
F 1 "1k" H 5270 1605 50  0000 L CNN
F 2 "" V 5130 1650 50  0001 C CNN
F 3 "~" H 5200 1650 50  0001 C CNN
	1    5200 1650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 60E43427
P 2250 3950
F 0 "R4" H 2320 3996 50  0000 L CNN
F 1 "1k" H 2320 3905 50  0000 L CNN
F 2 "" V 2180 3950 50  0001 C CNN
F 3 "~" H 2250 3950 50  0001 C CNN
	1    2250 3950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 60E43504
P 2250 6100
F 0 "R5" H 2320 6146 50  0000 L CNN
F 1 "1k" H 2320 6055 50  0000 L CNN
F 2 "" V 2180 6100 50  0001 C CNN
F 3 "~" H 2250 6100 50  0001 C CNN
	1    2250 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 60E43860
P 4800 6100
F 0 "R6" H 4870 6146 50  0000 L CNN
F 1 "1k" H 4870 6055 50  0000 L CNN
F 2 "" V 4730 6100 50  0001 C CNN
F 3 "~" H 4800 6100 50  0001 C CNN
	1    4800 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D_Kit
U 1 1 60E43FD0
P 1900 1150
F 0 "D_Kit" V 1939 1032 50  0000 R CNN
F 1 "LED" V 1848 1032 50  0000 R CNN
F 2 "" H 1900 1150 50  0001 C CNN
F 3 "~" H 1900 1150 50  0001 C CNN
	1    1900 1150
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D_Bath
U 1 1 60E45409
P 4750 3450
F 0 "D_Bath" V 4789 3332 50  0000 R CNN
F 1 "LED" V 4698 3332 50  0000 R CNN
F 2 "" H 4750 3450 50  0001 C CNN
F 3 "~" H 4750 3450 50  0001 C CNN
	1    4750 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D_Liv
U 1 1 60E46CE5
P 5200 1350
F 0 "D_Liv" V 5239 1232 50  0000 R CNN
F 1 "LED" V 5148 1232 50  0000 R CNN
F 2 "" H 5200 1350 50  0001 C CNN
F 3 "~" H 5200 1350 50  0001 C CNN
	1    5200 1350
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D_Hall
U 1 1 60E46F73
P 2250 3650
F 0 "D_Hall" V 2289 3532 50  0000 R CNN
F 1 "LED" V 2198 3532 50  0000 R CNN
F 2 "" H 2250 3650 50  0001 C CNN
F 3 "~" H 2250 3650 50  0001 C CNN
	1    2250 3650
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D_Room1
U 1 1 60E471D6
P 2250 5800
F 0 "D_Room1" V 2289 5682 50  0000 R CNN
F 1 "LED" V 2198 5682 50  0000 R CNN
F 2 "" H 2250 5800 50  0001 C CNN
F 3 "~" H 2250 5800 50  0001 C CNN
	1    2250 5800
	0    -1   -1   0   
$EndComp
$Comp
L Device:LED D_Room2
U 1 1 60E47442
P 4800 5800
F 0 "D_Room2" V 4839 5682 50  0000 R CNN
F 1 "LED" V 4748 5682 50  0000 R CNN
F 2 "" H 4800 5800 50  0001 C CNN
F 3 "~" H 4800 5800 50  0001 C CNN
	1    4800 5800
	0    -1   -1   0   
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E4BD20
P 1900 1600
F 0 "#PWR?" H 1900 1350 50  0001 C CNN
F 1 "Earth" H 1900 1450 50  0001 C CNN
F 2 "" H 1900 1600 50  0001 C CNN
F 3 "~" H 1900 1600 50  0001 C CNN
	1    1900 1600
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E4C438
P 4750 3900
F 0 "#PWR?" H 4750 3650 50  0001 C CNN
F 1 "Earth" H 4750 3750 50  0001 C CNN
F 2 "" H 4750 3900 50  0001 C CNN
F 3 "~" H 4750 3900 50  0001 C CNN
	1    4750 3900
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E4CDB0
P 5200 1800
F 0 "#PWR?" H 5200 1550 50  0001 C CNN
F 1 "Earth" H 5200 1650 50  0001 C CNN
F 2 "" H 5200 1800 50  0001 C CNN
F 3 "~" H 5200 1800 50  0001 C CNN
	1    5200 1800
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E4D118
P 2250 4100
F 0 "#PWR?" H 2250 3850 50  0001 C CNN
F 1 "Earth" H 2250 3950 50  0001 C CNN
F 2 "" H 2250 4100 50  0001 C CNN
F 3 "~" H 2250 4100 50  0001 C CNN
	1    2250 4100
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E4D19E
P 2250 6250
F 0 "#PWR?" H 2250 6000 50  0001 C CNN
F 1 "Earth" H 2250 6100 50  0001 C CNN
F 2 "" H 2250 6250 50  0001 C CNN
F 3 "~" H 2250 6250 50  0001 C CNN
	1    2250 6250
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E4D3CE
P 4800 6250
F 0 "#PWR?" H 4800 6000 50  0001 C CNN
F 1 "Earth" H 4800 6100 50  0001 C CNN
F 2 "" H 4800 6250 50  0001 C CNN
F 3 "~" H 4800 6250 50  0001 C CNN
	1    4800 6250
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DPST_x2 SW_Kit
U 1 1 60E50FD4
P 1750 2050
F 0 "SW_Kit" H 1750 2285 50  0000 C CNN
F 1 " " H 1750 2194 50  0000 C CNN
F 2 "" H 1750 2050 50  0001 C CNN
F 3 "~" H 1750 2050 50  0001 C CNN
	1    1750 2050
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E52686
P 1950 2150
F 0 "#PWR?" H 1950 1900 50  0001 C CNN
F 1 "Earth" H 1950 2000 50  0001 C CNN
F 2 "" H 1950 2150 50  0001 C CNN
F 3 "~" H 1950 2150 50  0001 C CNN
	1    1950 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 2150 1950 2050
$Comp
L Switch:SW_DPST_x2 SW_Liv
U 1 1 60E54E86
P 5100 2250
F 0 "SW_Liv" H 5100 2485 50  0000 C CNN
F 1 " " H 5100 2394 50  0000 C CNN
F 2 "" H 5100 2250 50  0001 C CNN
F 3 "~" H 5100 2250 50  0001 C CNN
	1    5100 2250
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E54E8C
P 5300 2350
F 0 "#PWR?" H 5300 2100 50  0001 C CNN
F 1 "Earth" H 5300 2200 50  0001 C CNN
F 2 "" H 5300 2350 50  0001 C CNN
F 3 "~" H 5300 2350 50  0001 C CNN
	1    5300 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2350 5300 2250
$Comp
L Switch:SW_DPST_x2 SW_Hall
U 1 1 60E54E93
P 2500 4550
F 0 "SW_Hall" H 2500 4785 50  0000 C CNN
F 1 " " H 2500 4694 50  0000 C CNN
F 2 "" H 2500 4550 50  0001 C CNN
F 3 "~" H 2500 4550 50  0001 C CNN
	1    2500 4550
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E54E99
P 2700 4650
F 0 "#PWR?" H 2700 4400 50  0001 C CNN
F 1 "Earth" H 2700 4500 50  0001 C CNN
F 2 "" H 2700 4650 50  0001 C CNN
F 3 "~" H 2700 4650 50  0001 C CNN
	1    2700 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 4650 2700 4550
$Comp
L Switch:SW_DPST_x2 SW_Room1
U 1 1 60E55E94
P 2350 6700
F 0 "SW_Room1" H 2350 6935 50  0000 C CNN
F 1 " " H 2350 6844 50  0000 C CNN
F 2 "" H 2350 6700 50  0001 C CNN
F 3 "~" H 2350 6700 50  0001 C CNN
	1    2350 6700
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E55E9A
P 2550 6800
F 0 "#PWR?" H 2550 6550 50  0001 C CNN
F 1 "Earth" H 2550 6650 50  0001 C CNN
F 2 "" H 2550 6800 50  0001 C CNN
F 3 "~" H 2550 6800 50  0001 C CNN
	1    2550 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 6800 2550 6700
$Comp
L Switch:SW_DPST_x2 SW_Room2
U 1 1 60E55EA1
P 5000 6700
F 0 "SW_Room2" H 5000 6935 50  0000 C CNN
F 1 " " H 5000 6844 50  0000 C CNN
F 2 "" H 5000 6700 50  0001 C CNN
F 3 "~" H 5000 6700 50  0001 C CNN
	1    5000 6700
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60E55EA7
P 5200 6800
F 0 "#PWR?" H 5200 6550 50  0001 C CNN
F 1 "Earth" H 5200 6650 50  0001 C CNN
F 2 "" H 5200 6800 50  0001 C CNN
F 3 "~" H 5200 6800 50  0001 C CNN
	1    5200 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 6800 5200 6700
$Comp
L MCU_ST_STM32F4:STM32F411VETx U1
U 1 1 60E58027
P 9950 3550
F 0 "U1" H 9950 761 50  0000 C CNN
F 1 "STM32F411VETx" H 9950 670 50  0000 C CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 9250 1050 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00115249.pdf" H 9950 3550 50  0001 C CNN
	1    9950 3550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Raspberry_Pi_2_3 J?
U 1 1 60E65637
P 7550 2500
F 0 "J?" H 7550 3981 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 7550 3890 50  0000 C CNN
F 2 "" H 7550 2500 50  0001 C CNN
F 3 "https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3bplus_1p0_reduced.pdf" H 7550 2500 50  0001 C CNN
	1    7550 2500
	1    0    0    -1  
$EndComp
Text GLabel 10850 5550 2    50   Input ~ 0
testtest
Text GLabel 8000 5550 2    50   Input ~ 0
testtest
Text GLabel 9050 5550 0    50   Input ~ 0
testtest
Wire Notes Line
	6350 500  11200 500 
Wire Notes Line
	11200 500  11200 6500
Wire Notes Line
	11200 6500 6350 6500
$Comp
L Device:R_POT P_Hum
U 1 1 60EB3039
P 5350 3750
F 0 "P_Hum" H 5280 3796 50  0000 R CNN
F 1 "10k" H 5280 3705 50  0000 R CNN
F 2 "" H 5350 3750 50  0001 C CNN
F 3 "~" H 5350 3750 50  0001 C CNN
	1    5350 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2.2
U 1 1 60EB423A
P 5350 3450
F 0 "R2.2" H 5420 3496 50  0000 L CNN
F 1 "10k" H 5420 3405 50  0000 L CNN
F 2 "" V 5280 3450 50  0001 C CNN
F 3 "~" H 5350 3450 50  0001 C CNN
	1    5350 3450
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60EB475E
P 5350 3900
F 0 "#PWR?" H 5350 3650 50  0001 C CNN
F 1 "Earth" H 5350 3750 50  0001 C CNN
F 2 "" H 5350 3900 50  0001 C CNN
F 3 "~" H 5350 3900 50  0001 C CNN
	1    5350 3900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60EB665B
P 5350 3300
F 0 "#PWR?" H 5350 3150 50  0001 C CNN
F 1 "+3.3V" H 5365 3473 50  0000 C CNN
F 2 "" H 5350 3300 50  0001 C CNN
F 3 "" H 5350 3300 50  0001 C CNN
	1    5350 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4550 5350 4450
$Comp
L power:Earth #PWR?
U 1 1 60E5339A
P 5350 4550
F 0 "#PWR?" H 5350 4300 50  0001 C CNN
F 1 "Earth" H 5350 4400 50  0001 C CNN
F 2 "" H 5350 4550 50  0001 C CNN
F 3 "~" H 5350 4550 50  0001 C CNN
	1    5350 4550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DPST_x2 SW_Bath
U 1 1 60E53394
P 5150 4450
F 0 "SW_Bath" H 5150 4685 50  0000 C CNN
F 1 " " H 5150 4594 50  0000 C CNN
F 2 "" H 5150 4450 50  0001 C CNN
F 3 "~" H 5150 4450 50  0001 C CNN
	1    5150 4450
	1    0    0    -1  
$EndComp
$Comp
L nrf_custom:nRF905 n?
U 1 1 60ECBF8C
P 1650 5900
F 0 "n?" H 1650 6065 50  0000 C CNN
F 1 "nRF905" H 1650 5974 50  0000 C CNN
F 2 "" H 1650 6000 50  0001 C CNN
F 3 "" H 1650 6000 50  0001 C CNN
	1    1650 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_POT P_Temp
U 1 1 60ECFAA7
P 2900 3950
F 0 "P_Temp" H 2831 3996 50  0000 R CNN
F 1 "10k" H 2831 3905 50  0000 R CNN
F 2 "" H 2900 3950 50  0001 C CNN
F 3 "~" H 2900 3950 50  0001 C CNN
	1    2900 3950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2.?
U 1 1 60ECFAAD
P 2900 3650
F 0 "R2.?" H 2970 3696 50  0000 L CNN
F 1 "10k" H 2970 3605 50  0000 L CNN
F 2 "" V 2830 3650 50  0001 C CNN
F 3 "~" H 2900 3650 50  0001 C CNN
	1    2900 3650
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60ECFAB3
P 2900 4100
F 0 "#PWR?" H 2900 3850 50  0001 C CNN
F 1 "Earth" H 2900 3950 50  0001 C CNN
F 2 "" H 2900 4100 50  0001 C CNN
F 3 "~" H 2900 4100 50  0001 C CNN
	1    2900 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60ECFAB9
P 2900 3500
F 0 "#PWR?" H 2900 3350 50  0001 C CNN
F 1 "+3.3V" H 2915 3673 50  0000 C CNN
F 2 "" H 2900 3500 50  0001 C CNN
F 3 "" H 2900 3500 50  0001 C CNN
	1    2900 3500
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DPST_x2 SW_CM_sensor
U 1 1 60ED3EF1
P 2400 2050
F 0 "SW_CM_sensor" H 2400 2285 50  0000 C CNN
F 1 " " H 2400 2194 50  0000 C CNN
F 2 "" H 2400 2050 50  0001 C CNN
F 3 "~" H 2400 2050 50  0001 C CNN
	1    2400 2050
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60ED3EF7
P 2600 2150
F 0 "#PWR?" H 2600 1900 50  0001 C CNN
F 1 "Earth" H 2600 2000 50  0001 C CNN
F 2 "" H 2600 2150 50  0001 C CNN
F 3 "~" H 2600 2150 50  0001 C CNN
	1    2600 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 2150 2600 2050
$Comp
L Device:Buzzer BZ?
U 1 1 60ED4811
P 2550 1450
F 0 "BZ?" H 2702 1479 50  0000 L CNN
F 1 "Buzzer" H 2702 1388 50  0000 L CNN
F 2 "" V 2525 1550 50  0001 C CNN
F 3 "~" V 2525 1550 50  0001 C CNN
	1    2550 1450
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60ED6072
P 2450 1600
F 0 "#PWR?" H 2450 1350 50  0001 C CNN
F 1 "Earth" H 2450 1450 50  0001 C CNN
F 2 "" H 2450 1600 50  0001 C CNN
F 3 "~" H 2450 1600 50  0001 C CNN
	1    2450 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 1550 2450 1600
$Comp
L Motor:Motor_DC M?
U 1 1 60ED779D
P 4450 1500
F 0 "M?" H 4608 1496 50  0000 L CNN
F 1 "Motor_DC" H 4608 1405 50  0000 L CNN
F 2 "" H 4450 1410 50  0001 C CNN
F 3 "~" H 4450 1410 50  0001 C CNN
	1    4450 1500
	1    0    0    -1  
$EndComp
$Comp
L Transistor_FET:BSN20 Q?
U 1 1 60ED996D
P 4350 2100
F 0 "Q?" H 4555 2146 50  0000 L CNN
F 1 "BSN20" H 4555 2055 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 4550 2025 50  0001 L CIN
F 3 "http://www.diodes.com/assets/Datasheets/ds31898.pdf" H 4350 2100 50  0001 L CNN
	1    4350 2100
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D_Sch
U 1 1 60EDE39C
P 4000 1550
F 0 "D_Sch" V 3954 1630 50  0000 L CNN
F 1 " " V 4045 1630 50  0000 L CNN
F 2 "" H 4000 1550 50  0001 C CNN
F 3 "~" H 4000 1550 50  0001 C CNN
	1    4000 1550
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 1400 4000 1300
Wire Wire Line
	4000 1300 4450 1300
Wire Wire Line
	4000 1700 4000 1800
Wire Wire Line
	4000 1800 4450 1800
Wire Wire Line
	4450 1800 4450 1900
Connection ~ 4450 1800
$Comp
L power:Earth #PWR?
U 1 1 60EE1730
P 4450 2300
F 0 "#PWR?" H 4450 2050 50  0001 C CNN
F 1 "Earth" H 4450 2150 50  0001 C CNN
F 2 "" H 4450 2300 50  0001 C CNN
F 3 "~" H 4450 2300 50  0001 C CNN
	1    4450 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60EE1F42
P 4450 1300
F 0 "#PWR?" H 4450 1150 50  0001 C CNN
F 1 "+3.3V" H 4465 1473 50  0000 C CNN
F 2 "" H 4450 1300 50  0001 C CNN
F 3 "" H 4450 1300 50  0001 C CNN
	1    4450 1300
	1    0    0    -1  
$EndComp
$Comp
L Sensor_Pressure:BMP280 U?
U 1 1 60EE8640
P 1600 3900
F 0 "U?" H 1830 3996 50  0000 L CNN
F 1 "BMP280" H 1830 3905 50  0000 L CNN
F 2 "Package_LGA:Bosch_LGA-8_2x2.5mm_P0.65mm_ClockwisePinNumbering" H 1600 3200 50  0001 C CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMP280-DS001.pdf" H 1600 3900 50  0001 C CNN
	1    1600 3900
	1    0    0    -1  
$EndComp
$Comp
L power:Earth #PWR?
U 1 1 60EEF0EE
P 1700 4200
F 0 "#PWR?" H 1700 3950 50  0001 C CNN
F 1 "Earth" H 1700 4050 50  0001 C CNN
F 2 "" H 1700 4200 50  0001 C CNN
F 3 "~" H 1700 4200 50  0001 C CNN
	1    1700 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 4200 1700 4200
Connection ~ 1700 4200
$Comp
L power:+3.3V #PWR?
U 1 1 60EF08CF
P 1700 3500
F 0 "#PWR?" H 1700 3350 50  0001 C CNN
F 1 "+3.3V" H 1715 3673 50  0000 C CNN
F 2 "" H 1700 3500 50  0001 C CNN
F 3 "" H 1700 3500 50  0001 C CNN
	1    1700 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 3500 1700 3500
Connection ~ 1700 3500
Wire Notes Line
	3400 7800 3400 500 
Wire Notes Line
	500  2850 6350 2850
Wire Notes Line
	6350 500  6350 6500
Wire Notes Line
	500  5200 6350 5200
Text Notes 500  600  0    79   ~ 0
Kitchen
Text Notes 3400 600  0    79   ~ 0
Living room
Text Notes 500  3000 0    79   ~ 0
Hall\n
Text Notes 3450 3000 0    79   ~ 0
Bathroom\n
Text Notes 6400 650  0    79   ~ 0
MCU devices\n
$Comp
L power:Earth #PWR?
U 1 1 60F034F7
P 1350 6550
F 0 "#PWR?" H 1350 6300 50  0001 C CNN
F 1 "Earth" H 1350 6400 50  0001 C CNN
F 2 "" H 1350 6550 50  0001 C CNN
F 3 "~" H 1350 6550 50  0001 C CNN
	1    1350 6550
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 60F03A6F
P 1350 5950
F 0 "#PWR?" H 1350 5800 50  0001 C CNN
F 1 "+3.3V" H 1365 6123 50  0000 C CNN
F 2 "" H 1350 5950 50  0001 C CNN
F 3 "" H 1350 5950 50  0001 C CNN
	1    1350 5950
	1    0    0    -1  
$EndComp
$EndSCHEMATC
