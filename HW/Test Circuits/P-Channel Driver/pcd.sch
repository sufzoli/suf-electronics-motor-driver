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
LIBS:pcd-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "13 feb 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L FUSE F1
U 1 1 54950B27
P 4350 2975
F 0 "F1" H 4450 3025 40  0000 C CNN
F 1 "FUSE" H 4250 2925 40  0000 C CNN
F 2 "AUTOFUSE" H 4350 2975 60  0001 C CNN
F 3 "~" H 4350 2975 60  0000 C CNN
	1    4350 2975
	0    -1   1    0   
$EndComp
$Comp
L VDD #PWR01
U 1 1 549513B4
P 4350 2650
F 0 "#PWR01" H 4350 2750 30  0001 C CNN
F 1 "VDD" H 4350 2760 30  0000 C CNN
F 2 "" H 4350 2650 60  0000 C CNN
F 3 "" H 4350 2650 60  0000 C CNN
	1    4350 2650
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4350 2650 4350 2725
$Comp
L MOSFET_P_GDS Q3
U 1 1 54B59908
P 4275 3800
F 0 "Q3" H 4050 3875 70  0000 C CNN
F 1 "IRF9530" H 4100 3575 60  0000 C CNN
F 2 "TO-220_V" H 4275 3800 60  0001 C CNN
F 3 "~" H 4275 3800 60  0000 C CNN
	1    4275 3800
	1    0    0    1   
$EndComp
Wire Wire Line
	4350 3225 4350 3525
$Comp
L R R5
U 1 1 54B5A69B
P 3800 4050
F 0 "R5" H 3800 3975 40  0000 C CNN
F 1 "750/2W" H 3800 4125 40  0000 C CNN
F 2 "R6" H 3800 4050 60  0001 C CNN
F 3 "~" H 3800 4050 60  0000 C CNN
	1    3800 4050
	0    1    -1   0   
$EndComp
$Comp
L ZENER D2
U 1 1 54B5A6F5
P 3800 3475
F 0 "D2" H 3800 3575 50  0000 C CNN
F 1 "ZENER" H 3800 3375 40  0000 C CNN
F 2 "D2" H 3800 3475 60  0001 C CNN
F 3 "" H 3800 3475 60  0000 C CNN
	1    3800 3475
	0    1    -1   0   
$EndComp
Wire Wire Line
	4000 3725 3150 3725
Wire Wire Line
	3800 3675 3800 3850
Connection ~ 3800 3725
$Comp
L R R3
U 1 1 54B5A882
P 3250 4525
F 0 "R3" H 3250 4450 40  0000 C CNN
F 1 "1K8" H 3250 4600 40  0000 C CNN
F 2 "R4" H 3250 4525 60  0001 C CNN
F 3 "~" H 3250 4525 60  0000 C CNN
	1    3250 4525
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3800 4250 3800 4325
Wire Wire Line
	3450 4525 3500 4525
$Comp
L GND #PWR02
U 1 1 54B5A9AB
P 3800 4800
F 0 "#PWR02" H 3800 4800 30  0001 C CNN
F 1 "GND" H 3800 4730 30  0001 C CNN
F 2 "" H 3800 4800 60  0000 C CNN
F 3 "" H 3800 4800 60  0000 C CNN
	1    3800 4800
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3800 4725 3800 4800
Wire Wire Line
	4350 3275 3650 3275
Connection ~ 4350 3275
$Comp
L NPN_CBE Q2
U 1 1 54C35829
P 3700 4525
F 0 "Q2" H 3650 4650 50  0000 C CNN
F 1 "BC546B" H 3525 4375 50  0000 C CNN
F 2 "TO-92_V" H 3700 4525 60  0001 C CNN
F 3 "~" H 3700 4525 60  0000 C CNN
	1    3700 4525
	1    0    0    -1  
$EndComp
$Comp
L NPN_CBE Q1
U 1 1 54C35838
P 3050 3500
F 0 "Q1" H 3000 3625 50  0000 C CNN
F 1 "BC546B" H 2875 3350 50  0000 C CNN
F 2 "TO-92_V" H 3050 3500 60  0001 C CNN
F 3 "~" H 3050 3500 60  0000 C CNN
	1    3050 3500
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 54C35874
P 3450 3275
F 0 "R4" H 3450 3200 40  0000 C CNN
F 1 "1" H 3450 3350 40  0000 C CNN
F 2 "R4" H 3450 3275 60  0001 C CNN
F 3 "~" H 3450 3275 60  0000 C CNN
	1    3450 3275
	-1   0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 54C35883
P 2325 3750
F 0 "R2" H 2325 3675 40  0000 C CNN
F 1 "750/2W" H 2325 3825 40  0000 C CNN
F 2 "R6" H 2325 3750 60  0001 C CNN
F 3 "~" H 2325 3750 60  0000 C CNN
	1    2325 3750
	0    1    -1   0   
$EndComp
$Comp
L R R1
U 1 1 54C358C7
P 2325 3250
F 0 "R1" H 2325 3175 40  0000 C CNN
F 1 "100" H 2325 3325 40  0000 C CNN
F 2 "R4" H 2325 3250 60  0001 C CNN
F 3 "~" H 2325 3250 60  0000 C CNN
	1    2325 3250
	0    -1   1    0   
$EndComp
Wire Wire Line
	3150 3725 3150 3700
Connection ~ 3800 3275
Wire Wire Line
	3250 3275 3150 3275
Wire Wire Line
	3150 3275 3150 3300
Wire Wire Line
	2325 2975 2325 3050
$Comp
L DIODESCH D1
U 1 1 54C35AF0
P 2575 3500
F 0 "D1" H 2575 3600 40  0000 C CNN
F 1 "DIODESCH" H 2575 3400 40  0000 C CNN
F 2 "D2" H 2575 3500 60  0001 C CNN
F 3 "" H 2575 3500 60  0000 C CNN
	1    2575 3500
	1    0    0    1   
$EndComp
Wire Wire Line
	2850 3500 2775 3500
Wire Wire Line
	2325 3450 2325 3550
Wire Wire Line
	2375 3500 2325 3500
Connection ~ 2325 3500
Wire Wire Line
	3800 4275 2325 4275
Wire Wire Line
	2325 4275 2325 3950
Connection ~ 3800 4275
$Comp
L DIODE-HEXFRED D3
U 1 1 54DCAB09
P 4350 4500
F 0 "D3" H 4350 4600 40  0000 C CNN
F 1 "MBR10100" H 4350 4400 40  0000 C CNN
F 2 "TO220-2" H 4350 4500 60  0001 C CNN
F 3 "~" H 4350 4500 60  0000 C CNN
	1    4350 4500
	0    1    -1   0   
$EndComp
$Comp
L MD-TEST-HDR J3
U 1 1 54DCAB2B
P 5100 4200
F 0 "J3" H 5100 3900 60  0000 C CNN
F 1 "OUT" V 5100 4200 60  0000 C CNN
F 2 "MD-TEST-HDR-M" H 5100 4200 60  0001 C CNN
F 3 "~" H 5100 4200 60  0000 C CNN
	1    5100 4200
	-1   0    0    1   
$EndComp
$Comp
L CONN_2 J1
U 1 1 54DCAB3A
P 1650 3450
F 0 "J1" V 1600 3450 40  0000 C CNN
F 1 "PWR" V 1700 3450 40  0000 C CNN
F 2 "MOLEX_36538_2" H 1650 3450 60  0001 C CNN
F 3 "" H 1650 3450 60  0000 C CNN
	1    1650 3450
	-1   0    0    1   
$EndComp
$Comp
L CONN_2 J2
U 1 1 54DCAB49
P 1650 4625
F 0 "J2" V 1600 4625 40  0000 C CNN
F 1 "PWM_IN" V 1700 4625 40  0000 C CNN
F 2 "CONN_NCW254-02S" H 1650 4625 60  0001 C CNN
F 3 "" H 1650 4625 60  0000 C CNN
	1    1650 4625
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2000 4525 3050 4525
Wire Wire Line
	4350 4075 4350 4300
$Comp
L GND #PWR03
U 1 1 54DCABA5
P 4350 4800
F 0 "#PWR03" H 4350 4800 30  0001 C CNN
F 1 "GND" H 4350 4730 30  0001 C CNN
F 2 "" H 4350 4800 60  0000 C CNN
F 3 "" H 4350 4800 60  0000 C CNN
	1    4350 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4700 4350 4800
$Comp
L GND #PWR04
U 1 1 54DCABCD
P 2000 4850
F 0 "#PWR04" H 2000 4850 30  0001 C CNN
F 1 "GND" H 2000 4780 30  0001 C CNN
F 2 "" H 2000 4850 60  0000 C CNN
F 3 "" H 2000 4850 60  0000 C CNN
	1    2000 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 4725 2000 4850
$Comp
L GND #PWR05
U 1 1 54DCABF6
P 2000 3650
F 0 "#PWR05" H 2000 3650 30  0001 C CNN
F 1 "GND" H 2000 3580 30  0001 C CNN
F 2 "" H 2000 3650 60  0000 C CNN
F 3 "" H 2000 3650 60  0000 C CNN
	1    2000 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3550 2000 3650
$Comp
L VDD #PWR06
U 1 1 54DCAC20
P 2000 3275
F 0 "#PWR06" H 2000 3375 30  0001 C CNN
F 1 "VDD" H 2000 3385 30  0000 C CNN
F 2 "" H 2000 3275 60  0000 C CNN
F 3 "" H 2000 3275 60  0000 C CNN
	1    2000 3275
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 3275 2000 3350
$Comp
L GND #PWR07
U 1 1 54DCAC4B
P 5100 4800
F 0 "#PWR07" H 5100 4800 30  0001 C CNN
F 1 "GND" H 5100 4730 30  0001 C CNN
F 2 "" H 5100 4800 60  0000 C CNN
F 3 "" H 5100 4800 60  0000 C CNN
	1    5100 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 4300 4800 4700
Wire Wire Line
	4800 4700 5400 4700
Wire Wire Line
	5400 4700 5400 4300
Connection ~ 4800 4400
Connection ~ 5400 4400
Wire Wire Line
	4350 4075 4800 4075
Wire Wire Line
	4800 3825 4800 4100
Connection ~ 4800 4075
Wire Wire Line
	4800 3825 5400 3825
Wire Wire Line
	5400 3825 5400 4100
Connection ~ 5400 4000
Wire Wire Line
	5100 4700 5100 4800
Connection ~ 5100 4700
Connection ~ 4800 4000
Wire Wire Line
	2325 2975 3800 2975
Wire Wire Line
	3800 2975 3800 3275
$EndSCHEMATC
