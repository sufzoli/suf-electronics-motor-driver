EESchema Schematic File Version 2
LIBS:suf
LIBS:conn
LIBS:device
LIBS:power
LIBS:transistors
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
LIBS:suf_regulator
LIBS:PSU-cache
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
L CP C1
U 1 1 5602186D
P 2550 1950
F 0 "C1" H 2550 1800 40  0000 C CNN
F 1 "100uF/63V" H 2550 2100 40  0000 C CNN
F 2 "Capacitors_ThroughHole:C_Radial_D8_L11.5_P3.5" H 2550 1950 60  0001 C CNN
F 3 "" H 2550 1950 60  0000 C CNN
	1    2550 1950
	0    1    1    0   
$EndComp
$Comp
L CONN_2 J1
U 1 1 5602191E
P 1975 1775
F 0 "J1" V 1925 1775 40  0000 C CNN
F 1 "IN" V 2025 1775 40  0000 C CNN
F 2 "suf_connector_ncw:CONN_NCW254-02R" H 1975 1775 60  0001 C CNN
F 3 "" H 1975 1775 60  0000 C CNN
	1    1975 1775
	-1   0    0    1   
$EndComp
$Comp
L CONN_2 J2
U 1 1 56021A0F
P 6050 1775
F 0 "J2" V 6000 1775 40  0000 C CNN
F 1 "OUT" V 6100 1775 40  0000 C CNN
F 2 "suf_connector_ncw:CONN_NCW254-02R" H 6050 1775 60  0001 C CNN
F 3 "" H 6050 1775 60  0000 C CNN
	1    6050 1775
	1    0    0    1   
$EndComp
$Comp
L CP C3
U 1 1 56021A70
P 5475 1925
F 0 "C3" H 5475 1775 40  0000 C CNN
F 1 "10uF/6,3V" H 5475 2075 40  0000 C CNN
F 2 "Capacitors_ThroughHole:C_Radial_D6.3_L11.2_P2.5" H 5475 1925 60  0001 C CNN
F 3 "" H 5475 1925 60  0000 C CNN
	1    5475 1925
	0    1    1    0   
$EndComp
$Comp
L LM2575 U1
U 1 1 56021C61
P 3275 1725
F 0 "U1" H 3525 1525 60  0000 C CNN
F 1 "LM2575" H 3175 1925 60  0000 C CNN
F 2 "Power_Packages_ThroughHole:Pentawatt_Neutral_Staggered_Verical_TO220-5-T05D" H 3275 1725 60  0001 C CNN
F 3 "" H 3275 1725 60  0000 C CNN
	1    3275 1725
	1    0    0    -1  
$EndComp
$Comp
L LD1117V33 U2
U 1 1 56021FF6
P 5025 1725
F 0 "U2" H 5025 1975 40  0000 C CNN
F 1 "LD1117V33" H 5025 1925 40  0000 C CNN
F 2 "Transistors_TO-220:TO-220_Neutral123_Vertical_LargePads" H 5025 1825 40  0001 C CNN
F 3 "" H 5025 1725 60  0000 C CNN
	1    5025 1725
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D1
U 1 1 56022153
P 3825 1925
F 0 "D1" H 3825 2025 40  0000 C CNN
F 1 "1N5819" H 3825 1825 40  0000 C CNN
F 2 "Diodes_ThroughHole:Diode_DO-35_SOD27_Horizontal_RM10" H 3825 1925 60  0001 C CNN
F 3 "" H 3825 1925 60  0000 C CNN
	1    3825 1925
	0    -1   -1   0   
$EndComp
$Comp
L INDUCTOR L1
U 1 1 560221A6
P 4075 1675
F 0 "L1" H 4075 1625 60  0000 C CNN
F 1 "330uH" H 4075 1775 60  0000 C CNN
F 2 "Inductors:INDUCTOR_V" H 4075 1675 60  0001 C CNN
F 3 "" H 4075 1675 60  0000 C CNN
	1    4075 1675
	1    0    0    -1  
$EndComp
$Comp
L CP C2
U 1 1 560221F1
P 4325 1925
F 0 "C2" H 4325 1775 40  0000 C CNN
F 1 "330uF/6.3V" H 4325 2075 40  0000 C CNN
F 2 "suf_capacitor:C4P_5X5" H 4325 1925 60  0001 C CNN
F 3 "" H 4325 1925 60  0000 C CNN
	1    4325 1925
	0    1    1    0   
$EndComp
$Comp
L R R1
U 1 1 560223C7
P 4550 1925
F 0 "R1" H 4550 1850 40  0000 C CNN
F 1 "10K" H 4550 2000 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" H 4550 1925 60  0001 C CNN
F 3 "" H 4550 1925 60  0000 C CNN
	1    4550 1925
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 5602243A
P 4550 2375
F 0 "R2" H 4550 2300 40  0000 C CNN
F 1 "3,3K" H 4550 2450 40  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" H 4550 2375 60  0001 C CNN
F 3 "" H 4550 2375 60  0000 C CNN
	1    4550 2375
	0    1    1    0   
$EndComp
$Comp
L GND #PWR01
U 1 1 5602250C
P 4550 2625
F 0 "#PWR01" H 4550 2375 50  0001 C CNN
F 1 "GND" H 4550 2475 50  0000 C CNN
F 2 "" H 4550 2625 60  0000 C CNN
F 3 "" H 4550 2625 60  0000 C CNN
	1    4550 2625
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 56022536
P 3825 2175
F 0 "#PWR02" H 3825 1925 50  0001 C CNN
F 1 "GND" H 3825 2025 50  0000 C CNN
F 2 "" H 3825 2175 60  0000 C CNN
F 3 "" H 3825 2175 60  0000 C CNN
	1    3825 2175
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 56022560
P 3275 2175
F 0 "#PWR03" H 3275 1925 50  0001 C CNN
F 1 "GND" H 3275 2025 50  0000 C CNN
F 2 "" H 3275 2175 60  0000 C CNN
F 3 "" H 3275 2175 60  0000 C CNN
	1    3275 2175
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 5602258A
P 2550 2175
F 0 "#PWR04" H 2550 1925 50  0001 C CNN
F 1 "GND" H 2550 2025 50  0000 C CNN
F 2 "" H 2550 2175 60  0000 C CNN
F 3 "" H 2550 2175 60  0000 C CNN
	1    2550 2175
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 560225B4
P 2325 2175
F 0 "#PWR05" H 2325 1925 50  0001 C CNN
F 1 "GND" H 2325 2025 50  0000 C CNN
F 2 "" H 2325 2175 60  0000 C CNN
F 3 "" H 2325 2175 60  0000 C CNN
	1    2325 2175
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 560225DE
P 4325 2175
F 0 "#PWR06" H 4325 1925 50  0001 C CNN
F 1 "GND" H 4325 2025 50  0000 C CNN
F 2 "" H 4325 2175 60  0000 C CNN
F 3 "" H 4325 2175 60  0000 C CNN
	1    4325 2175
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 56022608
P 5025 2175
F 0 "#PWR07" H 5025 1925 50  0001 C CNN
F 1 "GND" H 5025 2025 50  0000 C CNN
F 2 "" H 5025 2175 60  0000 C CNN
F 3 "" H 5025 2175 60  0000 C CNN
	1    5025 2175
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 56022632
P 5475 2175
F 0 "#PWR08" H 5475 1925 50  0001 C CNN
F 1 "GND" H 5475 2025 50  0000 C CNN
F 2 "" H 5475 2175 60  0000 C CNN
F 3 "" H 5475 2175 60  0000 C CNN
	1    5475 2175
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5602265C
P 5700 2175
F 0 "#PWR09" H 5700 1925 50  0001 C CNN
F 1 "GND" H 5700 2025 50  0000 C CNN
F 2 "" H 5700 2175 60  0000 C CNN
F 3 "" H 5700 2175 60  0000 C CNN
	1    5700 2175
	1    0    0    -1  
$EndComp
Wire Wire Line
	2325 1675 2775 1675
Wire Wire Line
	3775 1675 3875 1675
Wire Wire Line
	4275 1675 4625 1675
Wire Wire Line
	5425 1675 5700 1675
Wire Wire Line
	5700 1875 5700 2175
Wire Wire Line
	2325 1875 2325 2175
Wire Wire Line
	5475 1825 5475 1675
Connection ~ 5475 1675
Wire Wire Line
	5475 2025 5475 2175
Wire Wire Line
	5025 1975 5025 2175
Wire Wire Line
	4325 1675 4325 1825
Connection ~ 4325 1675
Wire Wire Line
	4325 2025 4325 2175
Wire Wire Line
	3375 2075 3375 2150
Wire Wire Line
	3375 2150 4550 2150
Wire Wire Line
	4550 2125 4550 2175
Connection ~ 4550 2150
Wire Wire Line
	3825 2125 3825 2175
Wire Wire Line
	3825 1675 3825 1725
Connection ~ 3825 1675
Wire Wire Line
	3275 2075 3275 2175
Wire Wire Line
	3175 2075 3175 2125
Wire Wire Line
	3175 2125 3275 2125
Connection ~ 3275 2125
Wire Wire Line
	2550 2050 2550 2175
Wire Wire Line
	2550 1850 2550 1675
Connection ~ 2550 1675
Wire Wire Line
	4550 1725 4550 1675
Connection ~ 4550 1675
Wire Wire Line
	4550 2575 4550 2625
$EndSCHEMATC
