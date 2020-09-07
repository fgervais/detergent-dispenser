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
L TINYPICO:TINYPICO XA1
U 1 1 5F5506BC
P 3450 1950
F 0 "XA1" H 3750 2175 50  0000 C CNN
F 1 "TINYPICO" H 3750 2084 50  0000 C CNN
F 2 "tinypico-kicad-footprint:TINYPICO" H 3700 1950 50  0001 C CNN
F 3 "" H 3700 1950 50  0001 C CNN
	1    3450 1950
	1    0    0    -1  
$EndComp
$Comp
L RJHSE-5080:RJHSE-5080 J1
U 1 1 5F551818
P 7200 1900
F 0 "J1" H 7430 1896 50  0000 L CNN
F 1 "RJ45 (Bartendro)" H 7430 1805 50  0000 L CNN
F 2 "AMPHENOL_RJHSE-5080" H 7200 1900 50  0001 L BNN
F 3 "13.21 mm" H 7200 1900 50  0001 L BNN
F 4 "Amphenol" H 7200 1900 50  0001 L BNN "Field4"
F 5 "Manufacturer Recommendations" H 7200 1900 50  0001 L BNN "Field5"
F 6 "G" H 7200 1900 50  0001 L BNN "Field6"
	1    7200 1900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J3
U 1 1 5F5582B1
P 1850 2300
F 0 "J3" H 1930 2292 50  0000 L CNN
F 1 "20v Input" H 1930 2201 50  0000 L CNN
F 2 "1546215-2:TE_1546215-2" H 1850 2300 50  0001 C CNN
F 3 "~" H 1850 2300 50  0001 C CNN
	1    1850 2300
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J2
U 1 1 5F559196
P 1850 1600
F 0 "J2" H 1930 1592 50  0000 L CNN
F 1 "User Button" H 1930 1501 50  0000 L CNN
F 2 "1546215-2:TE_1546215-2" H 1850 1600 50  0001 C CNN
F 3 "~" H 1850 1600 50  0001 C CNN
	1    1850 1600
	-1   0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7805 U1
U 1 1 5F55C658
P 2700 2300
F 0 "U1" H 2700 2542 50  0000 C CNN
F 1 "MIC2940A-5.0WU" H 2700 2451 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-263-3_TabPin2" H 2725 2150 50  0001 L CIN
F 3 "" H 2700 2250 50  0001 C CNN
	1    2700 2300
	1    0    0    -1  
$EndComp
$Comp
L PS1240P02BT:PS1240P02BT LS1
U 1 1 5F55F1A3
P 5150 3200
F 0 "LS1" H 5150 3467 50  0000 C CNN
F 1 "PS1240P02BT" H 5150 3376 50  0000 C CNN
F 2 "XDCR_PS1240P02BT" H 5150 3200 50  0001 L BNN
F 3 "Unavailable" H 5150 3200 50  0001 L BNN
F 4 "PS1240P02CT3" H 5150 3200 50  0001 L BNN "Field4"
F 5 "7.0mm" H 5150 3200 50  0001 L BNN "Field5"
F 6 "None" H 5150 3200 50  0001 L BNN "Field6"
F 7 "Buzzers Transducer, Externally Driven Piezo 3V 4kHz 60dB @ 3V, 10cm Through Hole PC Pins" H 5150 3200 50  0001 L BNN "Field7"
F 8 "SIP-5 TDK" H 5150 3200 50  0001 L BNN "Field8"
F 9 "TDK" H 5150 3200 50  0001 L BNN "Field9"
	1    5150 3200
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5F561014
P 4950 2200
F 0 "SW1" H 4950 2485 50  0000 C CNN
F 1 "Reset" H 4950 2394 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 4950 2400 50  0001 C CNN
F 3 "~" H 4950 2400 50  0001 C CNN
	1    4950 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5F56556C
P 5250 2300
F 0 "#PWR0101" H 5250 2050 50  0001 C CNN
F 1 "GND" H 5255 2127 50  0000 C CNN
F 2 "" H 5250 2300 50  0001 C CNN
F 3 "" H 5250 2300 50  0001 C CNN
	1    5250 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 2200 5250 2200
Wire Wire Line
	5250 2200 5250 2300
$Comp
L power:GND #PWR0102
U 1 1 5F56AF54
P 3150 2800
F 0 "#PWR0102" H 3150 2550 50  0001 C CNN
F 1 "GND" H 3155 2627 50  0000 C CNN
F 2 "" H 3150 2800 50  0001 C CNN
F 3 "" H 3150 2800 50  0001 C CNN
	1    3150 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2800 3150 2750
Wire Wire Line
	3150 2750 3450 2750
Wire Wire Line
	5250 1750 5250 2200
Connection ~ 5250 2200
Wire Wire Line
	4650 2200 4750 2200
Wire Wire Line
	4800 2350 4800 3200
Wire Wire Line
	4800 3200 4950 3200
$Comp
L power:GND #PWR0103
U 1 1 5F577C76
P 5500 3600
F 0 "#PWR0103" H 5500 3350 50  0001 C CNN
F 1 "GND" H 5505 3427 50  0000 C CNN
F 2 "" H 5500 3600 50  0001 C CNN
F 3 "" H 5500 3600 50  0001 C CNN
	1    5500 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 3200 5500 3200
$Comp
L Device:R R1
U 1 1 5F5786A5
P 4800 3350
F 0 "R1" H 4870 3396 50  0000 L CNN
F 1 "10k" H 4870 3305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 4730 3350 50  0001 C CNN
F 3 "~" H 4800 3350 50  0001 C CNN
	1    4800 3350
	1    0    0    -1  
$EndComp
Connection ~ 4800 3200
Wire Wire Line
	4800 3500 4800 3550
Wire Wire Line
	4800 3550 5500 3550
Wire Wire Line
	5500 3550 5500 3600
Wire Wire Line
	5500 3200 5500 3550
Connection ~ 5500 3550
Wire Wire Line
	2700 2600 2700 2750
Wire Wire Line
	2700 2750 3150 2750
Connection ~ 3150 2750
Wire Wire Line
	3000 2300 3250 2300
Wire Wire Line
	3250 2300 3250 2650
Wire Wire Line
	3250 2650 3450 2650
Wire Wire Line
	2050 2300 2400 2300
Wire Wire Line
	2050 2400 2250 2400
Wire Wire Line
	2250 2400 2250 2750
Wire Wire Line
	2250 2750 2700 2750
Connection ~ 2700 2750
$Comp
L power:GND #PWR0104
U 1 1 5F58ABDC
P 2150 1800
F 0 "#PWR0104" H 2150 1550 50  0001 C CNN
F 1 "GND" H 2155 1627 50  0000 C CNN
F 2 "" H 2150 1800 50  0001 C CNN
F 3 "" H 2150 1800 50  0001 C CNN
	1    2150 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2050 1700 2150 1700
Wire Wire Line
	2150 1700 2150 1800
Wire Wire Line
	2050 1600 3200 1600
Wire Wire Line
	3200 1600 3200 1950
Wire Wire Line
	3200 1950 3450 1950
Text Label 3100 2300 0    50   ~ 0
5V
Text Label 2100 2300 0    50   ~ 0
20V
Wire Wire Line
	6550 1600 6800 1600
Wire Wire Line
	6550 1700 6800 1700
Text Label 6550 1600 0    50   ~ 0
20V
Text Label 6550 1700 0    50   ~ 0
5V
$Comp
L power:GND #PWR0105
U 1 1 5F59362C
P 6650 2400
F 0 "#PWR0105" H 6650 2150 50  0001 C CNN
F 1 "GND" H 6655 2227 50  0000 C CNN
F 2 "" H 6650 2400 50  0001 C CNN
F 3 "" H 6650 2400 50  0001 C CNN
	1    6650 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 2400 6650 2300
Wire Wire Line
	6650 2300 6800 2300
Wire Wire Line
	4050 2250 4300 2250
Wire Wire Line
	4050 2150 4300 2150
Text Label 4300 2150 2    50   ~ 0
TX
Text Label 4300 2250 2    50   ~ 0
RX
Wire Wire Line
	4050 1950 4200 1950
Wire Wire Line
	4200 1950 4200 1750
Wire Wire Line
	4200 1750 5250 1750
Wire Wire Line
	4650 2050 4650 2200
Wire Wire Line
	4050 2050 4650 2050
Wire Wire Line
	6800 1900 6550 1900
Wire Wire Line
	6800 2000 6550 2000
Text Label 6550 1900 0    50   ~ 0
TX
Text Label 6550 2000 0    50   ~ 0
RX
Wire Wire Line
	6800 2100 6550 2100
Text Label 6550 2100 0    50   ~ 0
RESET
Wire Wire Line
	4050 2350 4800 2350
Wire Wire Line
	4050 2550 4300 2550
Text Label 4300 2550 2    50   ~ 0
RESET
Wire Wire Line
	4050 2650 4300 2650
Text Label 4300 2650 2    50   ~ 0
SYNC
Wire Wire Line
	6800 2200 6550 2200
Text Label 6550 2200 0    50   ~ 0
SYNC
$Comp
L Mechanical:MountingHole_Pad H1
U 1 1 5F5D1CE1
P 2150 3300
F 0 "H1" V 2387 3303 50  0000 C CNN
F 1 "MountingHole_Pad" V 2296 3303 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad" H 2150 3300 50  0001 C CNN
F 3 "~" H 2150 3300 50  0001 C CNN
	1    2150 3300
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5F5D912A
P 2150 3500
F 0 "H2" V 2387 3503 50  0000 C CNN
F 1 "MountingHole_Pad" V 2296 3503 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad" H 2150 3500 50  0001 C CNN
F 3 "~" H 2150 3500 50  0001 C CNN
	1    2150 3500
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5F5D94AF
P 2150 3700
F 0 "H3" V 2387 3703 50  0000 C CNN
F 1 "MountingHole_Pad" V 2296 3703 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad" H 2150 3700 50  0001 C CNN
F 3 "~" H 2150 3700 50  0001 C CNN
	1    2150 3700
	0    -1   -1   0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5F5D96EF
P 2150 3900
F 0 "H4" V 2387 3903 50  0000 C CNN
F 1 "MountingHole_Pad" V 2296 3903 50  0000 C CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_Pad" H 2150 3900 50  0001 C CNN
F 3 "~" H 2150 3900 50  0001 C CNN
	1    2150 3900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2250 3300 2450 3300
Wire Wire Line
	2450 3300 2450 3500
$Comp
L power:GND #PWR0106
U 1 1 5F5DAB63
P 2450 4100
F 0 "#PWR0106" H 2450 3850 50  0001 C CNN
F 1 "GND" H 2455 3927 50  0000 C CNN
F 2 "" H 2450 4100 50  0001 C CNN
F 3 "" H 2450 4100 50  0001 C CNN
	1    2450 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 3500 2450 3500
Connection ~ 2450 3500
Wire Wire Line
	2450 3500 2450 3700
Wire Wire Line
	2250 3700 2450 3700
Connection ~ 2450 3700
Wire Wire Line
	2450 3700 2450 3900
Wire Wire Line
	2250 3900 2450 3900
Connection ~ 2450 3900
Wire Wire Line
	2450 3900 2450 4100
$EndSCHEMATC
