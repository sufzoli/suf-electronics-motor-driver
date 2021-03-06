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
LIBS:special
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
LIBS:motor-driver-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "21 feb 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Wire Wire Line
	9125 1550 10225 1550
$Comp
L LM2594 U?
U 1 1 54950B3E
P 9200 1100
F 0 "U?" H 9425 900 60  0000 C CNN
F 1 "LM2594HVM-5.0" H 9200 1350 60  0000 C CNN
F 2 "DIP-8" H 9200 1100 60  0001 C CNN
F 3 "" H 9200 1100 60  0000 C CNN
	1    9200 1100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 1100 8575 1100
Wire Wire Line
	9275 1450 9275 1650
$Comp
L INDUCTOR L?
U 1 1 54950B40
P 10000 1100
F 0 "L?" H 10000 1050 60  0000 C CNN
F 1 "100uH" H 10000 1200 60  0000 C CNN
F 2 "R5" H 10000 1100 60  0001 C CNN
F 3 "~" H 10000 1100 60  0000 C CNN
	1    10000 1100
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D?
U 1 1 54950B41
P 9750 1350
F 0 "D?" H 9750 1450 40  0000 C CNN
F 1 "1N5817" H 9750 1250 40  0000 C CNN
F 2 "D4" H 9750 1350 60  0001 C CNN
F 3 "~" H 9750 1350 60  0000 C CNN
	1    9750 1350
	0    -1   -1   0   
$EndComp
$Comp
L CP C?
U 1 1 54950B42
P 10225 1325
F 0 "C?" H 10225 1175 40  0000 C CNN
F 1 "220uF/10V" H 10225 1475 40  0000 C CNN
F 2 "C1P_D2.7" H 10225 1325 60  0001 C CNN
F 3 "~" H 10225 1325 60  0000 C CNN
	1    10225 1325
	0    1    1    0   
$EndComp
Wire Wire Line
	10225 1550 10225 1425
Connection ~ 9275 1550
Wire Wire Line
	9700 1100 9800 1100
Wire Wire Line
	9750 1100 9750 1150
Connection ~ 9750 1100
Wire Wire Line
	9700 950  10225 950 
Wire Wire Line
	10225 850  10225 1225
Wire Wire Line
	10200 1100 10225 1100
Connection ~ 10225 1100
Connection ~ 10225 950 
Connection ~ 9750 1550
$Comp
L GND #PWR?
U 1 1 54950C85
P 9275 1650
F 0 "#PWR?" H 9275 1650 30  0001 C CNN
F 1 "GND" H 9275 1580 30  0001 C CNN
F 2 "" H 9275 1650 60  0000 C CNN
F 3 "" H 9275 1650 60  0000 C CNN
	1    9275 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	9125 1450 9125 1550
$Comp
L VDD #PWR?
U 1 1 54950D58
P 8575 1000
F 0 "#PWR?" H 8575 1100 30  0001 C CNN
F 1 "VDD" H 8575 1110 30  0000 C CNN
F 2 "" H 8575 1000 60  0000 C CNN
F 3 "" H 8575 1000 60  0000 C CNN
	1    8575 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8575 1100 8575 1000
$Comp
L VCC #PWR?
U 1 1 54950E07
P 10225 850
F 0 "#PWR?" H 10225 950 30  0001 C CNN
F 1 "VCC" H 10225 950 30  0000 C CNN
F 2 "" H 10225 850 60  0000 C CNN
F 3 "" H 10225 850 60  0000 C CNN
	1    10225 850 
	1    0    0    -1  
$EndComp
$EndSCHEMATC
