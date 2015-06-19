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
LIBS:mm-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "23 apr 2015"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L R R1
U 1 1 54950B2A
P 6550 4100
F 0 "R1" H 6550 4025 40  0000 C CNN
F 1 "R005/1W" H 6550 4175 40  0000 C CNN
F 2 "SM2512" H 6550 4100 60  0001 C CNN
F 3 "~" H 6550 4100 60  0000 C CNN
	1    6550 4100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR01
U 1 1 549513A5
P 6550 4375
F 0 "#PWR01" H 6550 4375 30  0001 C CNN
F 1 "GND" H 6550 4305 30  0001 C CNN
F 2 "" H 6550 4375 60  0000 C CNN
F 3 "" H 6550 4375 60  0000 C CNN
	1    6550 4375
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 J2
U 1 1 549514FC
P 6450 1775
F 0 "J2" V 6400 1775 40  0000 C CNN
F 1 "MOTOR" V 6500 1775 40  0000 C CNN
F 2 "MOLEX_36538_2" H 6450 1775 60  0001 C CNN
F 3 "" H 6450 1775 60  0000 C CNN
	1    6450 1775
	0    1    -1   0   
$EndComp
$Comp
L C C1
U 1 1 54B5A225
P 6775 3200
F 0 "C1" H 6775 3050 40  0000 C CNN
F 1 "1uF/63V" H 6775 3350 40  0000 C CNN
F 2 "C2_3x1,75" H 6775 3200 10  0001 C CNN
F 3 "~" H 6775 3200 60  0000 C CNN
	1    6775 3200
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 54B5A234
P 6950 2900
F 0 "R4" H 7000 2825 40  0000 C CNN
F 1 "220K" H 6825 2825 40  0000 C CNN
F 2 "R4" H 6950 2900 60  0001 C CNN
F 3 "~" H 6950 2900 60  0000 C CNN
	1    6950 2900
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 54B5A243
P 6950 3500
F 0 "R5" H 7050 3425 40  0000 C CNN
F 1 "220K" H 6850 3425 40  0000 C CNN
F 2 "R4" H 6950 3500 60  0001 C CNN
F 3 "~" H 6950 3500 60  0000 C CNN
	1    6950 3500
	0    1    1    0   
$EndComp
$Comp
L R R17
U 1 1 54B5A252
P 9000 2425
F 0 "R17" H 9100 2500 40  0000 C CNN
F 1 "10K" H 8875 2500 40  0000 C CNN
F 2 "R4" H 9000 2425 60  0001 C CNN
F 3 "~" H 9000 2425 60  0000 C CNN
	1    9000 2425
	0    1    1    0   
$EndComp
$Comp
L R R18
U 1 1 54B5A2A7
P 9000 3175
F 0 "R18" H 9125 3250 40  0000 C CNN
F 1 "10K" H 8875 3250 40  0000 C CNN
F 2 "R4" H 9000 3175 60  0001 C CNN
F 3 "~" H 9000 3175 60  0000 C CNN
	1    9000 3175
	0    1    1    0   
$EndComp
$Comp
L R R21
U 1 1 54B5A2B6
P 9900 2425
F 0 "R21" H 9900 2350 40  0000 C CNN
F 1 "680K" H 9900 2500 40  0000 C CNN
F 2 "R4" H 9900 2425 60  0001 C CNN
F 3 "~" H 9900 2425 60  0000 C CNN
	1    9900 2425
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 54B5A9BA
P 9000 3425
F 0 "#PWR02" H 9000 3425 30  0001 C CNN
F 1 "GND" H 9000 3355 30  0001 C CNN
F 2 "" H 9000 3425 60  0000 C CNN
F 3 "" H 9000 3425 60  0000 C CNN
	1    9000 3425
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 54B5A9D8
P 9500 3225
F 0 "#PWR03" H 9500 3225 30  0001 C CNN
F 1 "GND" H 9500 3155 30  0001 C CNN
F 2 "" H 9500 3225 60  0000 C CNN
F 3 "" H 9500 3225 60  0000 C CNN
	1    9500 3225
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR04
U 1 1 54B9E27C
P 9000 2175
F 0 "#PWR04" H 9000 2275 30  0001 C CNN
F 1 "VCC" H 9000 2275 30  0000 C CNN
F 2 "" H 9000 2175 60  0000 C CNN
F 3 "" H 9000 2175 60  0000 C CNN
	1    9000 2175
	1    0    0    -1  
$EndComp
$Comp
L MCP6002 U2
U 2 1 54C67B76
P 9600 2775
F 0 "U2" H 9750 2625 70  0000 C CNN
F 1 "MCP6002" H 9750 2975 70  0000 C CNN
F 2 "DIP-8" H 9600 2775 60  0001 C CNN
F 3 "~" H 9600 2775 60  0000 C CNN
	2    9600 2775
	1    0    0    -1  
$EndComp
$Comp
L MCP6002 U2
U 1 1 54C67B85
P 9600 3975
F 0 "U2" H 9750 4275 70  0000 C CNN
F 1 "MCP6002" H 9750 4175 70  0000 C CNN
F 2 "DIP-8" H 9600 3975 60  0001 C CNN
F 3 "~" H 9600 3975 60  0000 C CNN
	1    9600 3975
	1    0    0    -1  
$EndComp
$Comp
L R R22
U 1 1 54C67C2A
P 10200 4325
F 0 "R22" H 10200 4250 40  0000 C CNN
F 1 "390K" H 10200 4400 40  0000 C CNN
F 2 "R4" H 10200 4325 60  0001 C CNN
F 3 "~" H 10200 4325 60  0000 C CNN
	1    10200 4325
	1    0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 54C67C39
P 8900 4325
F 0 "R16" H 8900 4250 40  0000 C CNN
F 1 "10K" H 8900 4400 40  0000 C CNN
F 2 "R4" H 8900 4325 60  0001 C CNN
F 3 "~" H 8900 4325 60  0000 C CNN
	1    8900 4325
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 54C67D32
P 8700 4475
F 0 "#PWR05" H 8700 4475 30  0001 C CNN
F 1 "GND" H 8700 4405 30  0001 C CNN
F 2 "" H 8700 4475 60  0000 C CNN
F 3 "" H 8700 4475 60  0000 C CNN
	1    8700 4475
	1    0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 54C67DB6
P 9750 4325
F 0 "R19" H 9750 4250 40  0000 C CNN
F 1 "100K" H 9750 4400 40  0000 C CNN
F 2 "R4" H 9750 4325 60  0001 C CNN
F 3 "~" H 9750 4325 60  0000 C CNN
	1    9750 4325
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 54CAAB1A
P 9500 4425
F 0 "#PWR06" H 9500 4425 30  0001 C CNN
F 1 "GND" H 9500 4355 30  0001 C CNN
F 2 "" H 9500 4425 60  0000 C CNN
F 3 "" H 9500 4425 60  0000 C CNN
	1    9500 4425
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR07
U 1 1 54CAAB77
P 9500 2275
F 0 "#PWR07" H 9500 2375 30  0001 C CNN
F 1 "VCC" H 9500 2375 30  0000 C CNN
F 2 "" H 9500 2275 60  0000 C CNN
F 3 "" H 9500 2275 60  0000 C CNN
	1    9500 2275
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR08
U 1 1 54CAAB86
P 9500 3475
F 0 "#PWR08" H 9500 3575 30  0001 C CNN
F 1 "VCC" H 9500 3575 30  0000 C CNN
F 2 "" H 9500 3475 60  0000 C CNN
F 3 "" H 9500 3475 60  0000 C CNN
	1    9500 3475
	1    0    0    -1  
$EndComp
$Comp
L MD-TEST-HDR J1
U 1 1 54D98F8B
P 5900 2500
F 0 "J1" H 5900 2200 60  0000 C CNN
F 1 "IN" V 5900 2500 60  0000 C CNN
F 2 "MD-TEST-HDR-F" H 5900 2500 60  0001 C CNN
F 3 "~" H 5900 2500 60  0000 C CNN
	1    5900 2500
	-1   0    0    1   
$EndComp
$Comp
L CONN_2 J3
U 1 1 54D98F9A
P 11000 2475
F 0 "J3" V 10950 2475 40  0000 C CNN
F 1 "PWR" V 11050 2475 40  0000 C CNN
F 2 "CONN_NCW254-02S" H 11000 2475 60  0001 C CNN
F 3 "" H 11000 2475 60  0000 C CNN
	1    11000 2475
	1    0    0    -1  
$EndComp
$Comp
L CONN_5 J4
U 1 1 54D98FA9
P 11050 3275
F 0 "J4" V 11000 3275 50  0000 C CNN
F 1 "OUT" V 11100 3275 50  0000 C CNN
F 2 "CONN_NCW254-05S" H 11050 3275 60  0001 C CNN
F 3 "" H 11050 3275 60  0000 C CNN
	1    11050 3275
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 54DA61AC
P 5900 3050
F 0 "#PWR09" H 5900 3050 30  0001 C CNN
F 1 "GND" H 5900 2980 30  0001 C CNN
F 2 "" H 5900 3050 60  0000 C CNN
F 3 "" H 5900 3050 60  0000 C CNN
	1    5900 3050
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 54DA636C
P 7150 3200
F 0 "R6" H 7150 3125 40  0000 C CNN
F 1 "1K" H 7150 3275 40  0000 C CNN
F 2 "R4" H 7150 3200 60  0001 C CNN
F 3 "~" H 7150 3200 60  0000 C CNN
	1    7150 3200
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 54DA6404
P 7600 2800
F 0 "C2" H 7725 2750 40  0000 C CNN
F 1 "10nF" H 7700 2850 40  0000 C CNN
F 2 "C1" H 7600 2800 60  0001 C CNN
F 3 "~" H 7600 2800 60  0000 C CNN
	1    7600 2800
	0    1    1    0   
$EndComp
$Comp
L C C3
U 1 1 54DA6413
P 7600 3600
F 0 "C3" H 7525 3550 40  0000 C CNN
F 1 "10nF" H 7500 3650 40  0000 C CNN
F 2 "C1" H 7600 3600 60  0001 C CNN
F 3 "~" H 7600 3600 60  0000 C CNN
	1    7600 3600
	0    1    1    0   
$EndComp
$Comp
L GND #PWR010
U 1 1 54DADD93
P 9925 3475
F 0 "#PWR010" H 9925 3475 30  0001 C CNN
F 1 "GND" H 9925 3405 30  0001 C CNN
F 2 "" H 9925 3475 60  0000 C CNN
F 3 "" H 9925 3475 60  0000 C CNN
	1    9925 3475
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR011
U 1 1 54DADDA3
P 9925 3125
F 0 "#PWR011" H 9925 3225 30  0001 C CNN
F 1 "VCC" H 9925 3225 30  0000 C CNN
F 2 "" H 9925 3125 60  0000 C CNN
F 3 "" H 9925 3125 60  0000 C CNN
	1    9925 3125
	1    0    0    -1  
$EndComp
$Comp
L C C12
U 1 1 54DADDB2
P 9925 3300
F 0 "C12" H 9925 3150 40  0000 C CNN
F 1 "100nF" H 10000 3425 40  0000 C CNN
F 2 "C1" H 9925 3300 60  0001 C CNN
F 3 "~" H 9925 3300 60  0000 C CNN
	1    9925 3300
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR012
U 1 1 54DAE1A9
P 10600 2300
F 0 "#PWR012" H 10600 2400 30  0001 C CNN
F 1 "VCC" H 10600 2400 30  0000 C CNN
F 2 "" H 10600 2300 60  0000 C CNN
F 3 "" H 10600 2300 60  0000 C CNN
	1    10600 2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 54DAE1B8
P 10600 2700
F 0 "#PWR013" H 10600 2700 30  0001 C CNN
F 1 "GND" H 10600 2630 30  0001 C CNN
F 2 "" H 10600 2700 60  0000 C CNN
F 3 "" H 10600 2700 60  0000 C CNN
	1    10600 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 54DBCBE2
P 10650 3600
F 0 "#PWR014" H 10650 3600 30  0001 C CNN
F 1 "GND" H 10650 3530 30  0001 C CNN
F 2 "" H 10650 3600 60  0000 C CNN
F 3 "" H 10650 3600 60  0000 C CNN
	1    10650 3600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR015
U 1 1 54DBCBF1
P 10500 3075
F 0 "#PWR015" H 10500 3175 30  0001 C CNN
F 1 "VCC" H 10500 3175 30  0000 C CNN
F 2 "" H 10500 3075 60  0000 C CNN
F 3 "" H 10500 3075 60  0000 C CNN
	1    10500 3075
	1    0    0    -1  
$EndComp
$Comp
L DIODE-HEXFRED D3
U 1 1 54DBCDAB
P 6550 3575
F 0 "D3" H 6550 3675 40  0000 C CNN
F 1 "MBR10100" H 6550 3475 40  0000 C CNN
F 2 "TO220-2" H 6550 3575 60  0001 C CNN
F 3 "~" H 6550 3575 60  0000 C CNN
	1    6550 3575
	0    1    1    0   
$EndComp
$Comp
L DIODE-HEXFRED D2
U 1 1 5536BB6F
P 6350 2825
F 0 "D2" H 6350 2925 40  0000 C CNN
F 1 "MBR10100" H 6350 2725 40  0000 C CNN
F 2 "TO220-2" H 6350 2825 60  0001 C CNN
F 3 "~" H 6350 2825 60  0000 C CNN
	1    6350 2825
	0    -1   -1   0   
$EndComp
$Comp
L DIODE-HEXFRED D1
U 1 1 5536BB7A
P 6350 2375
F 0 "D1" H 6350 2475 40  0000 C CNN
F 1 "MBR10100" H 6350 2275 40  0000 C CNN
F 2 "TO220-2" H 6350 2375 60  0001 C CNN
F 3 "~" H 6350 2375 60  0000 C CNN
	1    6350 2375
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D4
U 1 1 5536BD8C
P 7350 2900
F 0 "D4" H 7350 3000 40  0000 C CNN
F 1 "1N4148" H 7350 2800 40  0000 C CNN
F 2 "D2" H 7350 2900 60  0001 C CNN
F 3 "" H 7350 2900 60  0000 C CNN
	1    7350 2900
	0    -1   -1   0   
$EndComp
$Comp
L DIODE D5
U 1 1 5536BD9B
P 7350 3500
F 0 "D5" H 7350 3600 40  0000 C CNN
F 1 "1N4148" H 7350 3400 40  0000 C CNN
F 2 "D2" H 7350 3500 60  0001 C CNN
F 3 "" H 7350 3500 60  0000 C CNN
	1    7350 3500
	0    -1   -1   0   
$EndComp
$Comp
L R R9
U 1 1 5536BDBE
P 7750 3200
F 0 "R9" H 7750 3125 40  0000 C CNN
F 1 "4K7" H 7750 3275 40  0000 C CNN
F 2 "R4" H 7750 3200 60  0001 C CNN
F 3 "~" H 7750 3200 60  0000 C CNN
	1    7750 3200
	0    1    1    0   
$EndComp
$Comp
L C C4
U 1 1 5536C11D
P 7875 2800
F 0 "C4" H 8000 2750 40  0000 C CNN
F 1 "2.2nF" H 8000 2850 40  0000 C CNN
F 2 "C1" H 7875 2800 60  0001 C CNN
F 3 "~" H 7875 2800 60  0000 C CNN
	1    7875 2800
	0    1    1    0   
$EndComp
$Comp
L C C6
U 1 1 5536C123
P 8150 2800
F 0 "C6" H 8275 2750 40  0000 C CNN
F 1 "470pF" H 8275 2850 40  0000 C CNN
F 2 "C1" H 8150 2800 60  0001 C CNN
F 3 "~" H 8150 2800 60  0000 C CNN
	1    8150 2800
	0    1    1    0   
$EndComp
$Comp
L C C10
U 1 1 5536C14D
P 8700 2800
F 0 "C10" H 8825 2750 40  0000 C CNN
F 1 "22pF" H 8825 2850 40  0000 C CNN
F 2 "C1" H 8700 2800 60  0001 C CNN
F 3 "~" H 8700 2800 60  0000 C CNN
	1    8700 2800
	0    1    1    0   
$EndComp
$Comp
L R R10
U 1 1 5536C155
P 8025 3200
F 0 "R10" H 8025 3125 40  0000 C CNN
F 1 "22K" H 8025 3275 40  0000 C CNN
F 2 "R4" H 8025 3200 60  0001 C CNN
F 3 "~" H 8025 3200 60  0000 C CNN
	1    8025 3200
	0    1    1    0   
$EndComp
$Comp
L R R11
U 1 1 5536C164
P 8300 3200
F 0 "R11" H 8300 3125 40  0000 C CNN
F 1 "100K" H 8300 3275 40  0000 C CNN
F 2 "R4" H 8300 3200 60  0001 C CNN
F 3 "~" H 8300 3200 60  0000 C CNN
	1    8300 3200
	0    1    1    0   
$EndComp
$Comp
L R R12
U 1 1 5536C17B
P 8575 3200
F 0 "R12" H 8575 3125 40  0000 C CNN
F 1 "470K" H 8575 3275 40  0000 C CNN
F 2 "R4" H 8575 3200 60  0001 C CNN
F 3 "~" H 8575 3200 60  0000 C CNN
	1    8575 3200
	0    1    1    0   
$EndComp
$Comp
L C C5
U 1 1 5536C618
P 7875 3600
F 0 "C5" H 7800 3550 40  0000 C CNN
F 1 "2.2nF" H 7750 3650 40  0000 C CNN
F 2 "C1" H 7875 3600 60  0001 C CNN
F 3 "~" H 7875 3600 60  0000 C CNN
	1    7875 3600
	0    1    1    0   
$EndComp
$Comp
L C C7
U 1 1 5536CA57
P 8150 3600
F 0 "C7" H 8075 3550 40  0000 C CNN
F 1 "470pF" H 8025 3650 40  0000 C CNN
F 2 "C1" H 8150 3600 60  0001 C CNN
F 3 "~" H 8150 3600 60  0000 C CNN
	1    8150 3600
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 5536C129
P 8425 2800
F 0 "C8" H 8550 2750 40  0000 C CNN
F 1 "100pF" H 8550 2850 40  0000 C CNN
F 2 "C1" H 8425 2800 60  0001 C CNN
F 3 "~" H 8425 2800 60  0000 C CNN
	1    8425 2800
	0    1    1    0   
$EndComp
$Comp
L C C9
U 1 1 5536CA5D
P 8425 3600
F 0 "C9" H 8350 3550 40  0000 C CNN
F 1 "100pF" H 8300 3650 40  0000 C CNN
F 2 "C1" H 8425 3600 60  0001 C CNN
F 3 "~" H 8425 3600 60  0000 C CNN
	1    8425 3600
	0    1    1    0   
$EndComp
$Comp
L C C11
U 1 1 5536CA63
P 8700 3600
F 0 "C11" H 8625 3550 40  0000 C CNN
F 1 "22pF" H 8575 3650 40  0000 C CNN
F 2 "C1" H 8700 3600 60  0001 C CNN
F 3 "~" H 8700 3600 60  0000 C CNN
	1    8700 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 2125 6550 3375
Wire Wire Line
	6550 3775 6550 3900
Wire Wire Line
	6550 4300 6550 4375
Wire Wire Line
	9000 2625 9000 2975
Wire Wire Line
	9100 2675 9000 2675
Connection ~ 9000 2675
Wire Wire Line
	9500 3175 9500 3225
Wire Wire Line
	10100 2425 10100 3275
Wire Wire Line
	9700 2425 9100 2425
Wire Wire Line
	9100 2425 9100 2675
Connection ~ 6550 3875
Wire Wire Line
	9100 4075 9100 4325
Wire Wire Line
	8700 4325 8700 4475
Wire Wire Line
	10100 3975 10400 3975
Wire Wire Line
	10400 3175 10400 4325
Wire Wire Line
	10000 4325 9950 4325
Wire Wire Line
	9100 4325 9550 4325
Wire Wire Line
	9500 4375 9500 4425
Wire Wire Line
	9500 3475 9500 3575
Wire Wire Line
	9500 2275 9500 2375
Wire Wire Line
	5600 2600 5600 2900
Wire Wire Line
	5600 2900 6200 2900
Wire Wire Line
	6200 2900 6200 2600
Connection ~ 5600 2700
Connection ~ 6200 2700
Wire Wire Line
	5900 2900 5900 3050
Connection ~ 5900 2900
Wire Wire Line
	5600 2150 5600 2400
Wire Wire Line
	5600 2150 6575 2150
Wire Wire Line
	6200 2150 6200 2400
Connection ~ 5600 2300
Connection ~ 6200 2300
Connection ~ 6200 2150
Wire Wire Line
	6950 3200 6875 3200
Wire Wire Line
	9925 3200 9925 3125
Wire Wire Line
	9925 3400 9925 3475
Wire Wire Line
	10650 2375 10600 2375
Wire Wire Line
	10600 2375 10600 2300
Wire Wire Line
	10650 2575 10600 2575
Wire Wire Line
	10600 2575 10600 2700
Connection ~ 10400 3975
Wire Wire Line
	10500 3375 10650 3375
Wire Wire Line
	10650 3475 10650 3600
Wire Wire Line
	10400 3175 10650 3175
Wire Wire Line
	10100 3275 10650 3275
Connection ~ 10100 2775
Wire Wire Line
	10500 3375 10500 3075
Wire Wire Line
	6550 3875 9100 3875
Wire Wire Line
	6675 3200 6550 3200
Connection ~ 6550 2875
Wire Wire Line
	6350 3025 6550 3025
Connection ~ 6550 3025
Wire Wire Line
	6350 2575 6350 2625
Wire Wire Line
	6350 2125 6350 2175
Connection ~ 6350 2150
Connection ~ 6550 3200
Wire Wire Line
	9100 2875 8900 2875
Wire Wire Line
	7650 1650 7650 1725
Wire Wire Line
	7750 1650 7750 1725
Connection ~ 7750 1650
Wire Wire Line
	7850 1650 7850 1725
Connection ~ 7850 1650
Wire Wire Line
	7950 1650 7950 1725
Connection ~ 7950 1650
Wire Wire Line
	8050 1650 8050 1725
Connection ~ 8050 1650
Wire Wire Line
	7350 3100 7350 3300
Connection ~ 7350 3200
Wire Wire Line
	7600 2900 7600 3500
Wire Wire Line
	7350 3200 7600 3200
Connection ~ 7600 3200
Connection ~ 7475 3200
Wire Wire Line
	7750 3400 7875 3400
Wire Wire Line
	7875 2900 7875 3500
Connection ~ 7875 3400
Wire Wire Line
	7650 2525 7725 2525
Wire Wire Line
	7725 2525 7725 2900
Wire Wire Line
	7725 2900 7600 2900
Wire Wire Line
	7750 3000 7750 3000
Wire Wire Line
	7750 3000 7750 2525
Wire Wire Line
	7850 2525 7850 2650
Wire Wire Line
	7850 2650 8000 2650
Wire Wire Line
	8000 2650 8000 2900
Wire Wire Line
	8000 2900 7875 2900
Wire Wire Line
	7950 2525 7950 2625
Wire Wire Line
	7950 2625 8025 2625
Wire Wire Line
	8025 2625 8025 3000
Wire Wire Line
	6950 2700 8700 2700
Connection ~ 7600 2700
Connection ~ 7875 2700
Connection ~ 8150 2700
Connection ~ 8425 2700
Wire Wire Line
	8150 2900 8150 3500
Wire Wire Line
	8425 2900 8425 3500
Wire Wire Line
	8700 2900 8700 3500
Wire Wire Line
	6950 3700 8700 3700
Connection ~ 8425 3700
Connection ~ 8150 3700
Connection ~ 7875 3700
Connection ~ 7600 3700
Connection ~ 7350 3700
Connection ~ 7350 2700
Wire Wire Line
	6950 3100 6950 3300
Connection ~ 6950 3200
Wire Wire Line
	8150 2900 8275 2900
Wire Wire Line
	8275 2900 8275 2675
Wire Wire Line
	8275 2675 8050 2675
Wire Wire Line
	8050 2675 8050 2525
Wire Wire Line
	8150 2525 8150 2650
Wire Wire Line
	8150 2650 8300 2650
Wire Wire Line
	8300 2650 8300 3000
Wire Wire Line
	8250 2525 8250 2625
Wire Wire Line
	8250 2625 8550 2625
Wire Wire Line
	8550 2625 8550 2900
Wire Wire Line
	8550 2900 8425 2900
Wire Wire Line
	8350 2525 8350 2600
Wire Wire Line
	8350 2600 8575 2600
Wire Wire Line
	8575 2600 8575 3000
Wire Wire Line
	8450 2525 8450 2575
Wire Wire Line
	8450 2575 8825 2575
Wire Wire Line
	8825 2575 8825 2900
Wire Wire Line
	8825 2900 8700 2900
Wire Wire Line
	8025 3400 8150 3400
Connection ~ 8150 3400
Wire Wire Line
	8300 3400 8425 3400
Connection ~ 8425 3400
Wire Wire Line
	8575 3400 8700 3400
Connection ~ 8700 3400
$Comp
L CONN_9X2 J5
U 1 1 5536D55A
P 8100 2125
F 0 "J5" H 8100 2575 60  0000 C CNN
F 1 "CONN_9X2" V 8100 2125 50  0000 C CNN
F 2 "HDR-9x2" H 8100 2125 60  0001 C CNN
F 3 "" H 8100 2125 60  0000 C CNN
	1    8100 2125
	0    1    -1   0   
$EndComp
$Comp
L VCC #PWR016
U 1 1 5536D650
P 7350 2650
F 0 "#PWR016" H 7350 2750 30  0001 C CNN
F 1 "VCC" H 7350 2750 30  0000 C CNN
F 2 "" H 7350 2650 60  0000 C CNN
F 3 "" H 7350 2650 60  0000 C CNN
	1    7350 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5536D656
P 7350 3775
F 0 "#PWR017" H 7350 3775 30  0001 C CNN
F 1 "GND" H 7350 3705 30  0001 C CNN
F 2 "" H 7350 3775 60  0000 C CNN
F 3 "" H 7350 3775 60  0000 C CNN
	1    7350 3775
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 2650 7350 2700
Wire Wire Line
	7350 3700 7350 3775
Wire Wire Line
	9000 2175 9000 2225
Wire Wire Line
	9000 3375 9000 3425
Wire Wire Line
	8150 1725 8150 1650
Connection ~ 8150 1650
Wire Wire Line
	8250 1650 8250 1725
Connection ~ 8250 1650
Wire Wire Line
	8350 1725 8350 1650
Connection ~ 8350 1650
Wire Wire Line
	8450 1650 8450 1725
Connection ~ 8450 1650
$Comp
L MCP6002 U1
U 2 1 55371753
P 9600 1500
F 0 "U1" H 9750 1800 70  0000 C CNN
F 1 "MCP6002" H 9750 1700 70  0000 C CNN
F 2 "DIP-8" H 9600 1500 60  0001 C CNN
F 3 "~" H 9600 1500 60  0000 C CNN
	2    9600 1500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 5537176E
P 9500 1950
F 0 "#PWR018" H 9500 1950 30  0001 C CNN
F 1 "GND" H 9500 1880 30  0001 C CNN
F 2 "" H 9500 1950 60  0000 C CNN
F 3 "" H 9500 1950 60  0000 C CNN
	1    9500 1950
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR019
U 1 1 5537177D
P 9500 1050
F 0 "#PWR019" H 9500 1150 30  0001 C CNN
F 1 "VCC" H 9500 1150 30  0000 C CNN
F 2 "" H 9500 1050 60  0000 C CNN
F 3 "" H 9500 1050 60  0000 C CNN
	1    9500 1050
	1    0    0    -1  
$EndComp
Wire Wire Line
	9500 1050 9500 1100
Wire Wire Line
	9500 1900 9500 1950
$Comp
L R R2
U 1 1 553718D6
P 6775 2150
F 0 "R2" H 6700 2225 40  0000 C CNN
F 1 "21K" H 6825 2225 40  0000 C CNN
F 2 "R4" H 6775 2150 60  0001 C CNN
F 3 "~" H 6775 2150 60  0000 C CNN
	1    6775 2150
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 553718E5
P 7275 2150
F 0 "R7" H 7200 2225 40  0000 C CNN
F 1 "1K1" H 7325 2225 40  0000 C CNN
F 2 "R4" H 7275 2150 60  0001 C CNN
F 3 "~" H 7275 2150 60  0000 C CNN
	1    7275 2150
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 553718F4
P 6775 2300
F 0 "R3" H 6700 2225 40  0000 C CNN
F 1 "21K" H 6825 2225 40  0000 C CNN
F 2 "R4" H 6775 2300 60  0001 C CNN
F 3 "~" H 6775 2300 60  0000 C CNN
	1    6775 2300
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5537193F
P 7275 2300
F 0 "R8" H 7200 2225 40  0000 C CNN
F 1 "21K" H 7325 2225 40  0000 C CNN
F 2 "R4" H 7275 2300 60  0001 C CNN
F 3 "~" H 7275 2300 60  0000 C CNN
	1    7275 2300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 5537196C
P 7475 2375
F 0 "#PWR020" H 7475 2375 30  0001 C CNN
F 1 "GND" H 7475 2305 30  0001 C CNN
F 2 "" H 7475 2375 60  0000 C CNN
F 3 "" H 7475 2375 60  0000 C CNN
	1    7475 2375
	1    0    0    -1  
$EndComp
Wire Wire Line
	7475 2150 7475 2375
Connection ~ 7475 2300
Wire Wire Line
	6575 2300 6550 2300
Connection ~ 6550 2300
Wire Wire Line
	6975 2150 7075 2150
Wire Wire Line
	6975 2300 7075 2300
$Comp
L R R20
U 1 1 55371D45
P 9900 1750
F 0 "R20" H 9900 1675 40  0000 C CNN
F 1 "1M" H 9900 1825 40  0000 C CNN
F 2 "R4" H 9900 1750 60  0001 C CNN
F 3 "~" H 9900 1750 60  0000 C CNN
	1    9900 1750
	1    0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 55371D54
P 8850 1850
F 0 "R15" H 8850 1775 40  0000 C CNN
F 1 "1M" H 8850 1925 40  0000 C CNN
F 2 "R4" H 8850 1850 60  0001 C CNN
F 3 "~" H 8850 1850 60  0000 C CNN
	1    8850 1850
	0    1    1    0   
$EndComp
Wire Wire Line
	7650 1650 8550 1650
Wire Wire Line
	8550 1650 8550 2525
Wire Wire Line
	8550 2525 8900 2525
Wire Wire Line
	8900 2525 8900 2875
Wire Wire Line
	10100 1500 10100 1750
Wire Wire Line
	9100 1750 9700 1750
Wire Wire Line
	9100 1500 9100 1750
Wire Wire Line
	8800 1400 9100 1400
Wire Wire Line
	8850 1400 8850 1650
$Comp
L GND #PWR021
U 1 1 55371FDF
P 8850 2100
F 0 "#PWR021" H 8850 2100 30  0001 C CNN
F 1 "GND" H 8850 2030 30  0001 C CNN
F 2 "" H 8850 2100 60  0000 C CNN
F 3 "" H 8850 2100 60  0000 C CNN
	1    8850 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 2050 8850 2100
$Comp
L R R13
U 1 1 5537208C
P 8600 1400
F 0 "R13" H 8525 1475 40  0000 C CNN
F 1 "1M" H 8675 1475 40  0000 C CNN
F 2 "R4" H 8600 1400 60  0001 C CNN
F 3 "~" H 8600 1400 60  0000 C CNN
	1    8600 1400
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 5537209B
P 8600 1500
F 0 "R14" H 8525 1425 40  0000 C CNN
F 1 "1M" H 8700 1425 40  0000 C CNN
F 2 "R4" H 8600 1500 60  0001 C CNN
F 3 "~" H 8600 1500 60  0000 C CNN
	1    8600 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1500 7050 1500
Wire Wire Line
	7050 1500 7050 2300
Connection ~ 7050 2300
Wire Wire Line
	7000 2150 7000 1400
Wire Wire Line
	7000 1400 8400 1400
Connection ~ 7000 2150
Connection ~ 8850 1400
Wire Wire Line
	8800 1500 9100 1500
Connection ~ 9100 1600
Wire Wire Line
	10100 1750 10350 1750
Wire Wire Line
	10350 1750 10350 2925
Wire Wire Line
	10350 2925 10650 2925
Wire Wire Line
	10650 2925 10650 3075
$Comp
L GND #PWR022
U 1 1 55372510
P 10575 1600
F 0 "#PWR022" H 10575 1600 30  0001 C CNN
F 1 "GND" H 10575 1530 30  0001 C CNN
F 2 "" H 10575 1600 60  0000 C CNN
F 3 "" H 10575 1600 60  0000 C CNN
	1    10575 1600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR023
U 1 1 55372516
P 10575 1250
F 0 "#PWR023" H 10575 1350 30  0001 C CNN
F 1 "VCC" H 10575 1350 30  0000 C CNN
F 2 "" H 10575 1250 60  0000 C CNN
F 3 "" H 10575 1250 60  0000 C CNN
	1    10575 1250
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 5537251C
P 10575 1425
F 0 "C13" H 10575 1275 40  0000 C CNN
F 1 "100nF" H 10650 1550 40  0000 C CNN
F 2 "C1" H 10575 1425 60  0001 C CNN
F 3 "~" H 10575 1425 60  0000 C CNN
	1    10575 1425
	0    1    1    0   
$EndComp
Wire Wire Line
	10575 1325 10575 1250
Wire Wire Line
	10575 1525 10575 1600
$EndSCHEMATC