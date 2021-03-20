EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:suf_device
LIBS:suf_mosfet_driver
LIBS:pcd-cache
EELAYER 25 0
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
L DARL_PNP_BCE Q?
U 1 1 58507768
P 1400 1800
F 0 "Q?" H 1725 1875 50  0000 R CNN
F 1 "TIP127" H 1900 1775 50  0000 R CNN
F 2 "TO_SOT_Packages_THT:TO-220_Vertical" H 850 2300 50  0001 C CNN
F 3 "" H 1250 1800 50  0000 C CNN
	1    1400 1800
	0    -1   1    0   
$EndComp
$Comp
L ZENERsmall D?
U 1 1 58507844
P 900 1200
F 0 "D?" H 900 1300 50  0000 C CNN
F 1 "BZX55C10" H 900 1100 50  0000 C CNN
F 2 "Diodes_ThroughHole:D_DO-35_SOD27_P7.62mm_Horizontal" H 900 1200 50  0001 C CNN
F 3 "" H 900 1200 50  0000 C CNN
	1    900  1200
	0    1    1    0   
$EndComp
$Comp
L R_Small R?
U 1 1 58507899
P 900 1725
F 0 "R?" H 930 1745 50  0000 L CNN
F 1 "3,9K" H 930 1685 50  0000 L CNN
F 2 "Resistors_ThroughHole:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 900 1725 50  0001 C CNN
F 3 "" H 900 1725 50  0000 C CNN
	1    900  1725
	1    0    0    -1  
$EndComp
$Comp
L CP_Small C?
U 1 1 585078EE
P 1400 1200
F 0 "C?" H 1410 1270 50  0000 L CNN
F 1 "220uF/25V" H 1410 1120 50  0000 L CNN
F 2 "" H 1400 1200 50  0000 C CNN
F 3 "" H 1400 1200 50  0000 C CNN
	1    1400 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 1300 1400 1475
Wire Wire Line
	900  1825 900  1975
Wire Wire Line
	900  1900 1100 1900
Wire Wire Line
	900  1300 900  1625
Wire Wire Line
	900  1400 1400 1400
Connection ~ 1400 1400
Connection ~ 900  1400
Wire Wire Line
	900  900  900  1100
Wire Wire Line
	900  975  2000 975 
Wire Wire Line
	1400 975  1400 1100
$Comp
L +48V #PWR?
U 1 1 5850C570
P 900 900
F 0 "#PWR?" H 900 750 50  0001 C CNN
F 1 "+48V" H 900 1040 50  0000 C CNN
F 2 "" H 900 900 50  0000 C CNN
F 3 "" H 900 900 50  0000 C CNN
	1    900  900 
	1    0    0    -1  
$EndComp
Connection ~ 900  975 
$Comp
L GND #PWR?
U 1 1 5850C6DD
P 900 1975
F 0 "#PWR?" H 900 1725 50  0001 C CNN
F 1 "GND" H 900 1825 50  0000 C CNN
F 2 "" H 900 1975 50  0000 C CNN
F 3 "" H 900 1975 50  0000 C CNN
	1    900  1975
	1    0    0    -1  
$EndComp
Connection ~ 900  1900
$Comp
L +36V #PWR?
U 1 1 5850C70B
P 2225 1850
F 0 "#PWR?" H 2225 1700 50  0001 C CNN
F 1 "+36V" H 2225 1990 50  0000 C CNN
F 2 "" H 2225 1850 50  0000 C CNN
F 3 "" H 2225 1850 50  0000 C CNN
	1    2225 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 1900 2225 1900
Wire Wire Line
	2225 1900 2225 1850
$Comp
L CP_Small C?
U 1 1 5850CE69
P 2000 1200
F 0 "C?" H 2010 1270 50  0000 L CNN
F 1 "220uF/25V" H 2010 1120 50  0000 L CNN
F 2 "" H 2000 1200 50  0000 C CNN
F 3 "" H 2000 1200 50  0000 C CNN
	1    2000 1200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 975  2000 1100
Connection ~ 1400 975 
Wire Wire Line
	2000 1300 2000 1900
Connection ~ 2000 1900
$Comp
L TC4452 U?
U 1 1 5850D1EC
P 5075 1600
F 0 "U?" H 5275 1300 60  0000 C CNN
F 1 "TC4452" H 5375 1900 60  0000 C CNN
F 2 "" H 5075 1700 60  0000 C CNN
F 3 "" H 5075 1700 60  0000 C CNN
	1    5075 1600
	1    0    0    -1  
$EndComp
$Comp
L +48V #PWR?
U 1 1 5850D269
P 5075 1050
F 0 "#PWR?" H 5075 900 50  0001 C CNN
F 1 "+48V" H 5075 1190 50  0000 C CNN
F 2 "" H 5075 1050 50  0000 C CNN
F 3 "" H 5075 1050 50  0000 C CNN
	1    5075 1050
	1    0    0    -1  
$EndComp
$Comp
L +36V #PWR?
U 1 1 5850D289
P 5075 2150
F 0 "#PWR?" H 5075 2000 50  0001 C CNN
F 1 "+36V" H 5075 2290 50  0000 C CNN
F 2 "" H 5075 2150 50  0000 C CNN
F 3 "" H 5075 2150 50  0000 C CNN
	1    5075 2150
	-1   0    0    1   
$EndComp
Wire Wire Line
	5025 2050 5025 2100
Wire Wire Line
	5025 2100 5125 2100
Wire Wire Line
	5125 2100 5125 2050
Wire Wire Line
	5075 2100 5075 2150
Connection ~ 5075 2100
Wire Wire Line
	5025 1150 5025 1100
Wire Wire Line
	5025 1100 5125 1100
Wire Wire Line
	5125 1100 5125 1150
Wire Wire Line
	5075 1050 5075 1100
Connection ~ 5075 1100
$Comp
L Q_PMOS_GDS Q?
U 1 1 5850D326
P 6075 1600
F 0 "Q?" H 6375 1650 50  0000 R CNN
F 1 "IRF5210PbF" H 6725 1550 50  0000 R CNN
F 2 "TO_SOT_Packages_THT:TO-220_Vertical" H 6275 1700 50  0001 C CNN
F 3 "" H 6075 1600 50  0000 C CNN
	1    6075 1600
	1    0    0    1   
$EndComp
$Comp
L D_Schottky_Small D?
U 1 1 5850D38B
P 6175 2125
F 0 "D?" H 6125 2205 50  0000 L CNN
F 1 "MBR10100G" H 5895 2045 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-2_Vertical" V 6175 2125 50  0001 C CNN
F 3 "" V 6175 2125 50  0000 C CNN
	1    6175 2125
	0    1    1    0   
$EndComp
$Comp
L Q_NMOS_SGD Q?
U 1 1 5850D3F6
P 4450 2225
F 0 "Q?" H 4750 2275 50  0000 R CNN
F 1 "VN2222LG" H 5000 2075 50  0000 R CNN
F 2 "TO_SOT_Packages_THT:TO-92_Molded_Narrow" H 4650 2325 50  0001 C CNN
F 3 "" H 4450 2225 50  0000 C CNN
	1    4450 2225
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5850D461
P 4550 1400
F 0 "R?" H 4580 1420 50  0000 L CNN
F 1 "12K" H 4580 1360 50  0000 L CNN
F 2 "Resistors_ThroughHole:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 4550 1400 50  0001 C CNN
F 3 "" H 4550 1400 50  0000 C CNN
	1    4550 1400
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5850D4E8
P 4550 1800
F 0 "R?" H 4580 1820 50  0000 L CNN
F 1 "33K" H 4580 1760 50  0000 L CNN
F 2 "Resistors_ThroughHole:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 4550 1800 50  0001 C CNN
F 3 "" H 4550 1800 50  0000 C CNN
	1    4550 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5850D59C
P 4550 2525
F 0 "#PWR?" H 4550 2275 50  0001 C CNN
F 1 "GND" H 4550 2375 50  0000 C CNN
F 2 "" H 4550 2525 50  0000 C CNN
F 3 "" H 4550 2525 50  0000 C CNN
	1    4550 2525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5850D5C6
P 6175 2300
F 0 "#PWR?" H 6175 2050 50  0001 C CNN
F 1 "GND" H 6175 2150 50  0000 C CNN
F 2 "" H 6175 2300 50  0000 C CNN
F 3 "" H 6175 2300 50  0000 C CNN
	1    6175 2300
	1    0    0    -1  
$EndComp
$Comp
L +48V #PWR?
U 1 1 5850D5F0
P 6175 1275
F 0 "#PWR?" H 6175 1125 50  0001 C CNN
F 1 "+48V" H 6175 1415 50  0000 C CNN
F 2 "" H 6175 1275 50  0000 C CNN
F 3 "" H 6175 1275 50  0000 C CNN
	1    6175 1275
	1    0    0    -1  
$EndComp
$Comp
L +48V #PWR?
U 1 1 5850DA0E
P 4550 1200
F 0 "#PWR?" H 4550 1050 50  0001 C CNN
F 1 "+48V" H 4550 1340 50  0000 C CNN
F 2 "" H 4550 1200 50  0000 C CNN
F 3 "" H 4550 1200 50  0000 C CNN
	1    4550 1200
	1    0    0    -1  
$EndComp
$Comp
L R_Small R?
U 1 1 5850DA43
P 4250 2400
F 0 "R?" H 4280 2420 50  0000 L CNN
F 1 "100K" H 4280 2360 50  0000 L CNN
F 2 "Resistors_ThroughHole:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" H 4250 2400 50  0001 C CNN
F 3 "" H 4250 2400 50  0000 C CNN
	1    4250 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 1200 4550 1300
Wire Wire Line
	4550 1500 4550 1700
Wire Wire Line
	4550 1900 4550 2025
Wire Wire Line
	4550 2425 4550 2525
Wire Wire Line
	4250 2500 4250 2525
Wire Wire Line
	4250 2525 4550 2525
Wire Wire Line
	4250 2225 4250 2300
Wire Wire Line
	4350 1600 4675 1600
Connection ~ 4550 1600
Wire Wire Line
	5525 1550 5575 1550
Wire Wire Line
	5575 1550 5575 1650
Wire Wire Line
	5575 1650 5525 1650
Wire Wire Line
	5875 1600 5575 1600
Connection ~ 5575 1600
Wire Wire Line
	6175 1275 6175 1400
Wire Wire Line
	6175 1800 6175 2025
Wire Wire Line
	6175 2225 6175 2300
$Comp
L D_Small D?
U 1 1 59A392D0
P 4250 1600
F 0 "D?" H 4200 1680 50  0000 L CNN
F 1 "1N4148" H 4100 1520 50  0000 L CNN
F 2 "Diodes_ThroughHole:D_DO-35_SOD27_P7.62mm_Horizontal" V 4250 1600 50  0001 C CNN
F 3 "" V 4250 1600 50  0001 C CNN
	1    4250 1600
	-1   0    0    1   
$EndComp
$Comp
L +36V #PWR?
U 1 1 59A393AF
P 4050 1675
F 0 "#PWR?" H 4050 1525 50  0001 C CNN
F 1 "+36V" H 4050 1815 50  0000 C CNN
F 2 "" H 4050 1675 50  0000 C CNN
F 3 "" H 4050 1675 50  0000 C CNN
	1    4050 1675
	-1   0    0    1   
$EndComp
Wire Wire Line
	4150 1600 4050 1600
Wire Wire Line
	4050 1600 4050 1675
$EndSCHEMATC