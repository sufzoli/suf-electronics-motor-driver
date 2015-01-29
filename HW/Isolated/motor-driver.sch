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
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "26 jan 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L FUSE F?
U 1 1 54950B27
P 1400 2975
F 0 "F?" H 1500 3025 40  0000 C CNN
F 1 "FUSE" H 1300 2925 40  0000 C CNN
F 2 "AUTOFUSE" H 1400 2975 60  0001 C CNN
F 3 "~" H 1400 2975 60  0000 C CNN
	1    1400 2975
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 54950B2A
P 1400 6825
F 0 "R?" H 1400 6750 40  0000 C CNN
F 1 "R005/1W" H 1400 6900 40  0000 C CNN
F 2 "R4_5_6" H 1400 6825 60  0001 C CNN
F 3 "~" H 1400 6825 60  0000 C CNN
	1    1400 6825
	0    1    1    0   
$EndComp
$Comp
L CONN_2 J?
U 1 1 54950B37
P 900 1050
F 0 "J?" V 850 1050 40  0000 C CNN
F 1 "PWR" V 950 1050 40  0000 C CNN
F 2 "" H 900 1050 60  0000 C CNN
F 3 "" H 900 1050 60  0000 C CNN
	1    900  1050
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54950B38
P 1250 1250
F 0 "#PWR?" H 1250 1250 30  0001 C CNN
F 1 "GND" H 1250 1180 30  0001 C CNN
F 2 "" H 1250 1250 60  0000 C CNN
F 3 "" H 1250 1250 60  0000 C CNN
	1    1250 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 1150 1250 1250
$Comp
L VDD #PWR?
U 1 1 54950B39
P 1250 850
F 0 "#PWR?" H 1250 950 30  0001 C CNN
F 1 "VDD" H 1250 960 30  0000 C CNN
F 2 "" H 1250 850 60  0000 C CNN
F 3 "" H 1250 850 60  0000 C CNN
	1    1250 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 850  1250 950 
Wire Wire Line
	2075 1400 3175 1400
$Comp
L LM2594 U?
U 1 1 54950B3E
P 2150 950
F 0 "U?" H 2375 750 60  0000 C CNN
F 1 "LM2594HVM-5.0" H 2150 1200 60  0000 C CNN
F 2 "DIP-8" H 2150 950 60  0001 C CNN
F 3 "" H 2150 950 60  0000 C CNN
	1    2150 950 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 950  1525 950 
Wire Wire Line
	2225 1300 2225 1500
$Comp
L INDUCTOR L?
U 1 1 54950B40
P 2950 950
F 0 "L?" H 2950 900 60  0000 C CNN
F 1 "100uH" H 2950 1050 60  0000 C CNN
F 2 "R5" H 2950 950 60  0001 C CNN
F 3 "~" H 2950 950 60  0000 C CNN
	1    2950 950 
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D?
U 1 1 54950B41
P 2700 1200
F 0 "D?" H 2700 1300 40  0000 C CNN
F 1 "1N5817" H 2700 1100 40  0000 C CNN
F 2 "D4" H 2700 1200 60  0001 C CNN
F 3 "~" H 2700 1200 60  0000 C CNN
	1    2700 1200
	0    -1   -1   0   
$EndComp
$Comp
L CP C?
U 1 1 54950B42
P 3175 1175
F 0 "C?" H 3175 1025 40  0000 C CNN
F 1 "220uF/10V" H 3175 1325 40  0000 C CNN
F 2 "C1P_D2.7" H 3175 1175 60  0001 C CNN
F 3 "~" H 3175 1175 60  0000 C CNN
	1    3175 1175
	0    1    1    0   
$EndComp
Wire Wire Line
	3175 1400 3175 1275
Connection ~ 2225 1400
Wire Wire Line
	2650 950  2750 950 
Wire Wire Line
	2700 950  2700 1000
Connection ~ 2700 950 
Wire Wire Line
	2650 800  3175 800 
Wire Wire Line
	3175 700  3175 1075
Wire Wire Line
	3150 950  3175 950 
Connection ~ 3175 950 
Connection ~ 3175 800 
Connection ~ 2700 1400
$Comp
L GND #PWR?
U 1 1 54950C85
P 2225 1500
F 0 "#PWR?" H 2225 1500 30  0001 C CNN
F 1 "GND" H 2225 1430 30  0001 C CNN
F 2 "" H 2225 1500 60  0000 C CNN
F 3 "" H 2225 1500 60  0000 C CNN
	1    2225 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2075 1300 2075 1400
$Comp
L VDD #PWR?
U 1 1 54950D58
P 1525 850
F 0 "#PWR?" H 1525 950 30  0001 C CNN
F 1 "VDD" H 1525 960 30  0000 C CNN
F 2 "" H 1525 850 60  0000 C CNN
F 3 "" H 1525 850 60  0000 C CNN
	1    1525 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1525 950  1525 850 
$Comp
L VCC #PWR?
U 1 1 54950E07
P 3175 700
F 0 "#PWR?" H 3175 800 30  0001 C CNN
F 1 "VCC" H 3175 800 30  0000 C CNN
F 2 "" H 3175 700 60  0000 C CNN
F 3 "" H 3175 700 60  0000 C CNN
	1    3175 700 
	1    0    0    -1  
$EndComp
$Comp
L DIODESCH D?
U 1 1 5495134B
P 1400 5800
F 0 "D?" H 1400 5900 40  0000 C CNN
F 1 "1N5817" H 1400 5700 40  0000 C CNN
F 2 "D4" H 1400 5800 60  0001 C CNN
F 3 "~" H 1400 5800 60  0000 C CNN
	1    1400 5800
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 549513A5
P 1400 7100
F 0 "#PWR?" H 1400 7100 30  0001 C CNN
F 1 "GND" H 1400 7030 30  0001 C CNN
F 2 "" H 1400 7100 60  0000 C CNN
F 3 "" H 1400 7100 60  0000 C CNN
	1    1400 7100
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR?
U 1 1 549513B4
P 1400 2650
F 0 "#PWR?" H 1400 2750 30  0001 C CNN
F 1 "VDD" H 1400 2760 30  0000 C CNN
F 2 "" H 1400 2650 60  0000 C CNN
F 3 "" H 1400 2650 60  0000 C CNN
	1    1400 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 4750 1400 5175
$Comp
L C C?
U 1 1 549514ED
P 1625 5275
F 0 "C?" H 1625 5125 40  0000 C CNN
F 1 "1uF/63V" H 1625 5425 40  0000 C CNN
F 2 "~" H 1625 5275 60  0000 C CNN
F 3 "~" H 1625 5275 60  0000 C CNN
	1    1625 5275
	0    1    1    0   
$EndComp
$Comp
L CONN_2 P?
U 1 1 549514FC
P 1050 5275
F 0 "P?" V 1000 5275 40  0000 C CNN
F 1 "CONN_2" V 1100 5275 40  0000 C CNN
F 2 "" H 1050 5275 60  0000 C CNN
F 3 "" H 1050 5275 60  0000 C CNN
	1    1050 5275
	-1   0    0    -1  
$EndComp
Wire Wire Line
	1400 5375 1400 5600
Wire Wire Line
	1400 6000 1400 6625
Wire Wire Line
	1400 7025 1400 7100
$Comp
L INDUCTOR L?
U 1 1 54967ABF
P 1400 4550
F 0 "L?" H 1400 4500 60  0000 C CNN
F 1 "22uH/10A" H 1400 4650 60  0000 C CNN
F 2 "~" H 1400 4550 60  0000 C CNN
F 3 "~" H 1400 4550 60  0000 C CNN
	1    1400 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 5100 1925 5100
Wire Wire Line
	1625 5100 1625 5175
Connection ~ 1400 5100
$Comp
L GND #PWR?
U 1 1 54967B94
P 1625 5425
F 0 "#PWR?" H 1625 5425 30  0001 C CNN
F 1 "GND" H 1625 5355 30  0001 C CNN
F 2 "" H 1625 5425 60  0000 C CNN
F 3 "" H 1625 5425 60  0000 C CNN
	1    1625 5425
	1    0    0    -1  
$EndComp
Wire Wire Line
	1625 5375 1625 5425
Wire Wire Line
	1400 4075 1400 4350
Wire Wire Line
	1400 2650 1400 2725
$Comp
L MOSFET_P_GDS Q?
U 1 1 54B59908
P 1475 3800
F 0 "Q?" H 1250 3875 70  0000 C CNN
F 1 "IRF5210" H 1300 3575 60  0000 C CNN
F 2 "~" H 1475 3800 60  0000 C CNN
F 3 "~" H 1475 3800 60  0000 C CNN
	1    1475 3800
	-1   0    0    1   
$EndComp
$Comp
L C C?
U 1 1 54B5A225
P 2125 5600
F 0 "C?" H 2125 5450 40  0000 C CNN
F 1 "1,5uF/100V" H 2125 5750 40  0000 C CNN
F 2 "~" H 2125 5600 60  0000 C CNN
F 3 "~" H 2125 5600 60  0000 C CNN
	1    2125 5600
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 54B5A234
P 2350 5150
F 0 "R?" H 2350 5075 40  0000 C CNN
F 1 "220K" H 2350 5225 40  0000 C CNN
F 2 "R4" H 2350 5150 60  0001 C CNN
F 3 "~" H 2350 5150 60  0000 C CNN
	1    2350 5150
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 54B5A243
P 2350 5900
F 0 "R?" H 2350 5825 40  0000 C CNN
F 1 "220K" H 2350 5975 40  0000 C CNN
F 2 "R4" H 2350 5900 60  0001 C CNN
F 3 "~" H 2350 5900 60  0000 C CNN
	1    2350 5900
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 54B5A252
P 2500 5150
F 0 "R?" H 2500 5075 40  0000 C CNN
F 1 "10K" H 2500 5225 40  0000 C CNN
F 2 "R4" H 2500 5150 60  0001 C CNN
F 3 "~" H 2500 5150 60  0000 C CNN
	1    2500 5150
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 54B5A2A7
P 2500 5900
F 0 "R?" H 2500 5825 40  0000 C CNN
F 1 "10K" H 2500 5975 40  0000 C CNN
F 2 "R4" H 2500 5900 60  0001 C CNN
F 3 "~" H 2500 5900 60  0000 C CNN
	1    2500 5900
	0    1    1    0   
$EndComp
$Comp
L R R?
U 1 1 54B5A2B6
P 3400 5150
F 0 "R?" H 3400 5075 40  0000 C CNN
F 1 "R" H 3400 5225 40  0000 C CNN
F 2 "~" H 3400 5150 60  0000 C CNN
F 3 "~" H 3400 5150 60  0000 C CNN
	1    3400 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 5600 2025 5600
Wire Wire Line
	2225 5600 2600 5600
Wire Wire Line
	2350 5350 2350 5700
Wire Wire Line
	2500 5350 2500 5700
Wire Wire Line
	2600 5400 2500 5400
Connection ~ 2500 5400
Connection ~ 2350 5600
Wire Wire Line
	2350 4950 2500 4950
Wire Wire Line
	2350 6100 2500 6100
Wire Wire Line
	1400 3225 1400 3525
$Comp
L R R?
U 1 1 54B5A69B
P 1950 4050
F 0 "R?" H 1950 3975 40  0000 C CNN
F 1 "750/2W" H 1950 4125 40  0000 C CNN
F 2 "R6" H 1950 4050 60  0001 C CNN
F 3 "~" H 1950 4050 60  0000 C CNN
	1    1950 4050
	0    -1   -1   0   
$EndComp
$Comp
L ZENER D?
U 1 1 54B5A6F5
P 1950 3475
F 0 "D?" H 1950 3575 50  0000 C CNN
F 1 "ZENER" H 1950 3375 40  0000 C CNN
F 2 "" H 1950 3475 60  0000 C CNN
F 3 "" H 1950 3475 60  0000 C CNN
	1    1950 3475
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1750 3725 2600 3725
Wire Wire Line
	1950 3675 1950 3850
Connection ~ 1950 3725
$Comp
L R R?
U 1 1 54B5A882
P 2500 4525
F 0 "R?" H 2500 4450 40  0000 C CNN
F 1 "1K8" H 2500 4600 40  0000 C CNN
F 2 "R4" H 2500 4525 60  0001 C CNN
F 3 "~" H 2500 4525 60  0000 C CNN
	1    2500 4525
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 4250 1950 4325
Wire Wire Line
	2300 4525 2250 4525
$Comp
L GND #PWR?
U 1 1 54B5A9AB
P 1950 4800
F 0 "#PWR?" H 1950 4800 30  0001 C CNN
F 1 "GND" H 1950 4730 30  0001 C CNN
F 2 "" H 1950 4800 60  0000 C CNN
F 3 "" H 1950 4800 60  0000 C CNN
	1    1950 4800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54B5A9BA
P 2425 6200
F 0 "#PWR?" H 2425 6200 30  0001 C CNN
F 1 "GND" H 2425 6130 30  0001 C CNN
F 2 "" H 2425 6200 60  0000 C CNN
F 3 "" H 2425 6200 60  0000 C CNN
	1    2425 6200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54B5A9C9
P 1925 5425
F 0 "#PWR?" H 1925 5425 30  0001 C CNN
F 1 "GND" H 1925 5355 30  0001 C CNN
F 2 "" H 1925 5425 60  0000 C CNN
F 3 "" H 1925 5425 60  0000 C CNN
	1    1925 5425
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54B5A9D8
P 3000 5950
F 0 "#PWR?" H 3000 5950 30  0001 C CNN
F 1 "GND" H 3000 5880 30  0001 C CNN
F 2 "" H 3000 5950 60  0000 C CNN
F 3 "" H 3000 5950 60  0000 C CNN
	1    3000 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 4725 1950 4800
Wire Wire Line
	2425 6100 2425 6200
Connection ~ 2425 6100
$Comp
L VCC #PWR?
U 1 1 54B9E27C
P 2425 4875
F 0 "#PWR?" H 2425 4975 30  0001 C CNN
F 1 "VCC" H 2425 4975 30  0000 C CNN
F 2 "" H 2425 4875 60  0000 C CNN
F 3 "" H 2425 4875 60  0000 C CNN
	1    2425 4875
	1    0    0    -1  
$EndComp
Wire Wire Line
	2425 4875 2425 4950
Connection ~ 2425 4950
Wire Wire Line
	3000 5900 3000 5950
Wire Wire Line
	3600 5150 3600 5500
Wire Wire Line
	3200 5150 2600 5150
Wire Wire Line
	2600 5150 2600 5400
$Comp
L CP C?
U 1 1 54C3562D
P 1925 5275
F 0 "C?" H 1925 5125 40  0000 C CNN
F 1 "1000uF/63V" H 1925 5425 40  0000 C CNN
F 2 "~" H 1925 5275 60  0000 C CNN
F 3 "~" H 1925 5275 60  0000 C CNN
	1    1925 5275
	0    1    1    0   
$EndComp
Wire Wire Line
	1925 5100 1925 5175
Connection ~ 1625 5100
Wire Wire Line
	1925 5375 1925 5425
Wire Wire Line
	1400 3275 2100 3275
Connection ~ 1400 3275
$Comp
L NPN_CBE Q?
U 1 1 54C35829
P 2050 4525
F 0 "Q?" H 2000 4650 50  0000 C CNN
F 1 "BC546B" H 1875 4375 50  0000 C CNN
F 2 "TO-92" H 2050 4525 60  0001 C CNN
F 3 "~" H 2050 4525 60  0000 C CNN
	1    2050 4525
	-1   0    0    -1  
$EndComp
$Comp
L NPN_CBE Q?
U 1 1 54C35838
P 2700 3500
F 0 "Q?" H 2650 3625 50  0000 C CNN
F 1 "BC546B" H 2525 3350 50  0000 C CNN
F 2 "TO-92" H 2700 3500 60  0001 C CNN
F 3 "~" H 2700 3500 60  0000 C CNN
	1    2700 3500
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 54C35874
P 2300 3275
F 0 "R?" H 2300 3200 40  0000 C CNN
F 1 "1" H 2300 3350 40  0000 C CNN
F 2 "R4" H 2300 3275 60  0001 C CNN
F 3 "~" H 2300 3275 60  0000 C CNN
	1    2300 3275
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 54C35883
P 3425 3750
F 0 "R?" H 3425 3675 40  0000 C CNN
F 1 "750" H 3425 3825 40  0000 C CNN
F 2 "R6" H 3425 3750 60  0001 C CNN
F 3 "~" H 3425 3750 60  0000 C CNN
	1    3425 3750
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 54C358C7
P 3425 3250
F 0 "R?" H 3425 3175 40  0000 C CNN
F 1 "100" H 3425 3325 40  0000 C CNN
F 2 "R4" H 3425 3250 60  0001 C CNN
F 3 "~" H 3425 3250 60  0000 C CNN
	1    3425 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	2600 3725 2600 3700
Connection ~ 1950 3275
Wire Wire Line
	2500 3275 2600 3275
Wire Wire Line
	2600 3275 2600 3300
$Comp
L VDD #PWR?
U 1 1 54C359A3
P 3425 2975
F 0 "#PWR?" H 3425 3075 30  0001 C CNN
F 1 "VDD" H 3425 3085 30  0000 C CNN
F 2 "" H 3425 2975 60  0000 C CNN
F 3 "" H 3425 2975 60  0000 C CNN
	1    3425 2975
	1    0    0    -1  
$EndComp
Wire Wire Line
	3425 2975 3425 3050
$Comp
L DIODESCH D?
U 1 1 54C35AF0
P 3175 3500
F 0 "D?" H 3175 3600 40  0000 C CNN
F 1 "DIODESCH" H 3175 3400 40  0000 C CNN
F 2 "" H 3175 3500 60  0000 C CNN
F 3 "" H 3175 3500 60  0000 C CNN
	1    3175 3500
	-1   0    0    1   
$EndComp
Wire Wire Line
	2900 3500 2975 3500
Wire Wire Line
	3425 3450 3425 3550
Wire Wire Line
	3375 3500 3425 3500
Connection ~ 3425 3500
Wire Wire Line
	1950 4275 3425 4275
Wire Wire Line
	3425 4275 3425 3950
Connection ~ 1950 4275
$Comp
L MCP6002 U?
U 1 1 54C67B76
P 3100 5500
F 0 "U?" H 3250 5800 70  0000 C CNN
F 1 "MCP6002" H 3250 5700 70  0000 C CNN
F 2 "~" H 3100 5500 60  0000 C CNN
F 3 "~" H 3100 5500 60  0000 C CNN
	1    3100 5500
	1    0    0    -1  
$EndComp
$Comp
L MCP6002 U?
U 2 1 54C67B85
P 3100 6700
F 0 "U?" H 3250 7000 70  0000 C CNN
F 1 "MCP6002" H 3250 6900 70  0000 C CNN
F 2 "~" H 3100 6700 60  0000 C CNN
F 3 "~" H 3100 6700 60  0000 C CNN
	2    3100 6700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 6600 1400 6600
Connection ~ 1400 6600
$Comp
L R 390K
U 1 1 54C67C2A
P 3700 7050
F 0 "390K" H 3700 6975 40  0000 C CNN
F 1 "R" H 3700 7125 40  0000 C CNN
F 2 "~" H 3700 7050 60  0000 C CNN
F 3 "~" H 3700 7050 60  0000 C CNN
	1    3700 7050
	1    0    0    -1  
$EndComp
$Comp
L R 10K
U 1 1 54C67C39
P 2400 7050
F 0 "10K" H 2400 6975 40  0000 C CNN
F 1 "R" H 2400 7125 40  0000 C CNN
F 2 "~" H 2400 7050 60  0000 C CNN
F 3 "~" H 2400 7050 60  0000 C CNN
	1    2400 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 6800 2600 7050
$Comp
L GND #PWR?
U 1 1 54C67D32
P 2200 7200
F 0 "#PWR?" H 2200 7200 30  0001 C CNN
F 1 "GND" H 2200 7130 30  0001 C CNN
F 2 "" H 2200 7200 60  0000 C CNN
F 3 "" H 2200 7200 60  0000 C CNN
	1    2200 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 7050 2200 7200
$Comp
L R 100K
U 1 1 54C67DB6
P 3250 7050
F 0 "100K" H 3250 6975 40  0000 C CNN
F 1 "R" H 3250 7125 40  0000 C CNN
F 2 "~" H 3250 7050 60  0000 C CNN
F 3 "~" H 3250 7050 60  0000 C CNN
	1    3250 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 6700 3900 6700
Wire Wire Line
	3900 6700 3900 7050
Wire Wire Line
	3500 7050 3450 7050
Wire Wire Line
	2600 7050 3050 7050
$EndSCHEMATC
