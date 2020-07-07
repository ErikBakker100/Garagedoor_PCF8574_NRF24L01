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
L Device:C C1
U 1 1 5BE87DFA
P 8950 3500
F 0 "C1" H 9065 3546 50  0000 L CNN
F 1 "10pF" H 9065 3455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8988 3350 50  0001 C CNN
F 3 "~" H 8950 3500 50  0001 C CNN
	1    8950 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5BE87E90
P 9350 3500
F 0 "C2" H 9465 3546 50  0000 L CNN
F 1 "10pF" H 9465 3455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9388 3350 50  0001 C CNN
F 3 "~" H 9350 3500 50  0001 C CNN
	1    9350 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5BE87F8E
P 9150 3350
F 0 "R1" V 9050 3350 50  0000 C CNN
F 1 "1M" V 9150 3350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 9080 3350 50  0001 C CNN
F 3 "~" H 9150 3350 50  0001 C CNN
	1    9150 3350
	0    1    1    0   
$EndComp
Text Label 7600 2500 0    50   ~ 0
CE
Wire Wire Line
	7750 2000 7500 2000
Text Label 7550 2000 0    50   ~ 0
MOSI
Wire Wire Line
	7750 2100 7500 2100
Text Label 7550 2100 0    50   ~ 0
MISO
Wire Wire Line
	7750 2200 7500 2200
Text Label 7550 2200 0    50   ~ 0
SCK
Text Label 7550 2600 0    50   ~ 0
IRQ
$Comp
L Device:R R2
U 1 1 5BE88DFB
P 7600 2950
F 0 "R2" V 7500 2950 50  0000 C CNN
F 1 "22k" V 7600 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7530 2950 50  0001 C CNN
F 3 "~" H 7600 2950 50  0001 C CNN
	1    7600 2950
	-1   0    0    1   
$EndComp
$Comp
L Device:C C7
U 1 1 5BE89318
P 7750 3500
F 0 "C7" H 7865 3546 50  0000 L CNN
F 1 "33nF" H 7865 3455 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7788 3350 50  0001 C CNN
F 3 "~" H 7750 3500 50  0001 C CNN
	1    7750 3500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5BE89710
P 8900 1700
F 0 "C9" H 9015 1746 50  0000 L CNN
F 1 "10nF" H 9015 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8938 1550 50  0001 C CNN
F 3 "~" H 8900 1700 50  0001 C CNN
	1    8900 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5BE89873
P 9300 1700
F 0 "C8" H 9415 1746 50  0000 L CNN
F 1 "1nF" H 9415 1655 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9338 1550 50  0001 C CNN
F 3 "~" H 9300 1700 50  0001 C CNN
	1    9300 1700
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5BE8B589
P 9000 2300
F 0 "L1" H 8900 2300 50  0000 L CNN
F 1 "8.2nH" H 9050 2300 50  0000 L CNN
F 2 "Inductor_SMD:L_0402_1005Metric" H 9000 2300 50  0001 C CNN
F 3 "~" H 9000 2300 50  0001 C CNN
	1    9000 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8950 2400 8950 2450
$Comp
L Device:L L3
U 1 1 5BE8D9CC
P 9200 2450
F 0 "L3" V 9250 2450 50  0000 C CNN
F 1 "3.9nH" V 9150 2450 50  0000 C CNN
F 2 "Inductor_SMD:L_0402_1005Metric" H 9200 2450 50  0001 C CNN
F 3 "~" H 9200 2450 50  0001 C CNN
	1    9200 2450
	0    -1   -1   0   
$EndComp
$Comp
L Device:L L2
U 1 1 5BE8F30B
P 9200 2150
F 0 "L2" V 9250 2150 50  0000 C CNN
F 1 "2.7nH" V 9150 2150 50  0000 C CNN
F 2 "Inductor_SMD:L_0402_1005Metric" H 9200 2150 50  0001 C CNN
F 3 "~" H 9200 2150 50  0001 C CNN
	1    9200 2150
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C3
U 1 1 5BE901E8
P 9500 2150
F 0 "C3" H 9615 2196 50  0000 L CNN
F 1 "2.2nF" H 9615 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 9538 2000 50  0001 C CNN
F 3 "~" H 9500 2150 50  0001 C CNN
	1    9500 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5BE902AC
P 9950 2150
F 0 "C4" H 10065 2196 50  0000 L CNN
F 1 "4.7pF" H 10065 2105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 9988 2000 50  0001 C CNN
F 3 "~" H 9950 2150 50  0001 C CNN
	1    9950 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5BE9147D
P 9700 2350
F 0 "#PWR012" H 9700 2100 50  0001 C CNN
F 1 "GND" H 9705 2177 50  0000 C CNN
F 2 "" H 9700 2350 50  0001 C CNN
F 3 "" H 9700 2350 50  0001 C CNN
	1    9700 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 2300 9700 2300
$Comp
L power:GND #PWR011
U 1 1 5BE91D11
P 9300 1850
F 0 "#PWR011" H 9300 1600 50  0001 C CNN
F 1 "GND" H 9450 1800 50  0000 C CNN
F 2 "" H 9300 1850 50  0001 C CNN
F 3 "" H 9300 1850 50  0001 C CNN
	1    9300 1850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5BE9267F
P 8700 3700
F 0 "#PWR09" H 8700 3450 50  0001 C CNN
F 1 "GND" H 8705 3527 50  0000 C CNN
F 2 "" H 8700 3700 50  0001 C CNN
F 3 "" H 8700 3700 50  0001 C CNN
	1    8700 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 1700 8450 1550
$Comp
L Device:C C5
U 1 1 5BE97978
P 9500 2600
F 0 "C5" V 9350 2600 50  0000 C CNN
F 1 "1.5pF" V 9650 2600 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 9538 2450 50  0001 C CNN
F 3 "~" H 9500 2600 50  0001 C CNN
	1    9500 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	9350 2450 9350 2600
$Comp
L Device:C C6
U 1 1 5BE986DB
P 9650 2750
F 0 "C6" H 9600 2650 50  0000 R CNN
F 1 "1.5pF" H 9650 2850 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 9688 2600 50  0001 C CNN
F 3 "~" H 9650 2750 50  0001 C CNN
	1    9650 2750
	-1   0    0    1   
$EndComp
Wire Wire Line
	9300 3350 9350 3350
Wire Wire Line
	9000 3350 8950 3350
Wire Wire Line
	7750 3350 7750 3000
Wire Wire Line
	7600 3100 7600 3650
Wire Wire Line
	8250 3300 8250 3650
$Comp
L Connector2:Conn_01x01_Shielded J5
U 1 1 5BEAC4A5
P 10000 2600
F 0 "J5" H 10088 2565 50  0000 L CNN
F 1 "50 Ohm Antenne" H 10088 2474 50  0000 L CNN
F 2 "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical" H 10000 2600 50  0001 C CNN
F 3 "~" H 10000 2600 50  0001 C CNN
	1    10000 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 2800 10000 3650
Wire Wire Line
	10000 3650 9650 3650
$Comp
L power:+3.3V #PWR010
U 1 1 5BEAF702
P 8750 1500
F 0 "#PWR010" H 8750 1350 50  0001 C CNN
F 1 "+3.3V" H 8765 1673 50  0000 C CNN
F 2 "" H 8750 1500 50  0001 C CNN
F 3 "" H 8750 1500 50  0001 C CNN
	1    8750 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 3300 8450 3650
Connection ~ 8450 3650
Wire Wire Line
	9350 2150 9350 2000
Connection ~ 9650 3650
Wire Wire Line
	8450 3650 8550 3650
$Comp
L RF:NRF24L01 U4
U 1 1 5BE87CAB
P 8350 2500
F 0 "U4" H 8350 2450 50  0000 C CNN
F 1 "NRF24L01" H 8350 2350 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.5x2.5mm" H 8550 3300 50  0001 L CIN
F 3 "http://www.nordicsemi.com/eng/content/download/2730/34105/file/nRF24L01_Product_Specification_v2_0.pdf" H 8350 3386 50  0001 C CNN
	1    8350 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 3300 8550 3650
Connection ~ 8550 3650
Wire Wire Line
	8900 1850 9300 1850
Wire Wire Line
	8450 1550 8750 1550
Connection ~ 8450 1550
Wire Wire Line
	8750 1500 8750 1550
Wire Wire Line
	8250 3650 8350 3650
Wire Wire Line
	8350 3300 8350 3650
Connection ~ 8350 3650
Wire Wire Line
	8350 3650 8450 3650
Wire Wire Line
	8550 3650 8700 3650
Wire Wire Line
	8700 3700 8700 3650
Wire Wire Line
	8250 1700 8250 1550
Wire Wire Line
	8250 1550 8350 1550
Wire Wire Line
	8350 1700 8350 1550
Connection ~ 8350 1550
Wire Wire Line
	8350 1550 8450 1550
Wire Wire Line
	9700 2350 9700 2300
Connection ~ 9700 2300
Wire Wire Line
	9700 2300 9500 2300
$Comp
L Device:Q_NMOS_GSD Q1
U 1 1 5AB59630
P 7350 5100
F 0 "Q1" H 7550 5175 50  0000 L CNN
F 1 "AO3402" H 7550 5100 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23" H 7550 5025 50  0001 L CIN
F 3 "" H 7350 5100 50  0001 L CNN
	1    7350 5100
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 5BEB5DE5
P 7850 4050
F 0 "J4" V 7850 3850 50  0000 C CNN
F 1 "Deur" V 7950 4000 50  0000 C CNN
F 2 "Connector2:JST_NV_B02P-NV_1x02_P5.00mm_Vertical" H 7850 4050 50  0001 C CNN
F 3 "" H 7850 4050 50  0001 C CNN
	1    7850 4050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4150 5650 4150 5700
Wire Wire Line
	4450 4000 4450 4050
Wire Wire Line
	2350 5000 2200 5000
Wire Wire Line
	2350 4900 2200 4900
Wire Wire Line
	2350 4800 2200 4800
Text Label 2200 5000 0    50   ~ 0
MOSI
Text Label 2200 4900 0    50   ~ 0
MISO
Text Label 2200 4800 0    50   ~ 0
SCK
NoConn ~ 2350 4500
NoConn ~ 3500 5600
NoConn ~ 3400 5600
NoConn ~ 3100 5600
NoConn ~ 3000 5600
Connection ~ 4150 5650
$Comp
L Device:R R5
U 1 1 5AB57601
P 4450 4200
F 0 "R5" V 4530 4200 50  0000 C CNN
F 1 "10k" V 4450 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4380 4200 50  0001 C CNN
F 3 "" H 4450 4200 50  0001 C CNN
	1    4450 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 5650 4250 5650
Wire Wire Line
	4150 5100 4150 5650
$Comp
L power:GND #PWR04
U 1 1 5AB54FC9
P 4150 5700
F 0 "#PWR04" H 4150 5450 50  0001 C CNN
F 1 "GND" H 4150 5550 50  0000 C CNN
F 2 "" H 4150 5700 50  0001 C CNN
F 3 "" H 4150 5700 50  0001 C CNN
	1    4150 5700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR01
U 1 1 5BEB5DDD
P 1900 4000
F 0 "#PWR01" H 1900 3850 50  0001 C CNN
F 1 "+3.3V" H 1900 4140 50  0000 C CNN
F 2 "" H 1900 4000 50  0001 C CNN
F 3 "" H 1900 4000 50  0001 C CNN
	1    1900 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4600 2200 4600
$Comp
L Device:R R3
U 1 1 5BEB5DD9
P 2200 4200
F 0 "R3" V 2280 4200 50  0000 C CNN
F 1 "10k" V 2200 4200 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 2130 4200 50  0001 C CNN
F 3 "" H 2200 4200 50  0001 C CNN
	1    2200 4200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5BF0624A
P 4200 4000
F 0 "#PWR05" H 4200 3850 50  0001 C CNN
F 1 "+3.3V" H 4200 4140 50  0000 C CNN
F 2 "" H 4200 4000 50  0001 C CNN
F 3 "" H 4200 4000 50  0001 C CNN
	1    4200 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 4000 4200 4050
Wire Wire Line
	2350 5100 1900 5100
Wire Wire Line
	1900 5100 1900 4000
Connection ~ 1900 4000
$Comp
L Device:R R4
U 1 1 5BFFF019
P 4250 5400
F 0 "R4" V 4330 5400 50  0000 C CNN
F 1 "10k" V 4250 5400 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4180 5400 50  0001 C CNN
F 3 "" H 4250 5400 50  0001 C CNN
	1    4250 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 5550 4250 5650
Connection ~ 4250 5650
Wire Wire Line
	4250 5250 4250 5200
Text Label 2200 4600 0    50   ~ 0
RTS
Text Label 4650 4600 0    50   ~ 0
DTR
Wire Wire Line
	4450 4000 4200 4000
Connection ~ 4200 4000
Wire Wire Line
	4450 4350 4450 4600
$Comp
L Connector_Generic:Conn_01x05 J3
U 1 1 5C08C17F
P 4950 4600
F 0 "J3" H 4900 5000 50  0000 L CNN
F 1 "Serial" H 4850 4900 50  0000 L CNN
F 2 "Connector2:Molex_PicoBlade_53047-0510_1x05_P1.25mm_Vertical" H 4950 4600 50  0001 C CNN
F 3 "~" H 4950 4600 50  0001 C CNN
	1    4950 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 4800 4450 4800
Wire Wire Line
	4150 4400 4750 4400
Wire Wire Line
	4150 4500 4750 4500
Wire Wire Line
	4450 4600 4750 4600
Connection ~ 4450 4600
Wire Wire Line
	4450 4600 4450 4800
Wire Wire Line
	4650 4700 4750 4700
Text Label 4650 4700 0    50   ~ 0
RTS
Wire Wire Line
	7450 5300 7450 5650
Wire Wire Line
	7050 5100 7150 5100
$Comp
L power:+5V #PWR07
U 1 1 5C1C1DBF
P 7450 3950
F 0 "#PWR07" H 7450 3800 50  0001 C CNN
F 1 "+5V" H 7465 4123 50  0000 C CNN
F 2 "" H 7450 3950 50  0001 C CNN
F 3 "" H 7450 3950 50  0001 C CNN
	1    7450 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7850 4250 7850 4300
Wire Wire Line
	7850 4900 8050 4900
Wire Wire Line
	8050 4250 7950 4250
$Comp
L Connector:Barrel_Jack_Switch J2
U 1 1 5BEB5DDF
P 2000 2400
F 0 "J2" H 2000 2610 50  0000 C CNN
F 1 "Barrel_Jack" H 2000 2225 50  0000 C CNN
F 2 "Connector2:BarrelJack_Horizontal" H 2050 2360 50  0001 C CNN
F 3 "" H 2050 2360 50  0001 C CNN
	1    2000 2400
	1    0    0    -1  
$EndComp
$Comp
L Device:Fuse F1
U 1 1 5BEB5DE0
P 2450 2300
F 0 "F1" V 2530 2300 50  0000 C CNN
F 1 "3Amp" V 2375 2300 50  0000 C CNN
F 2 "FuseClip:fuse-clip-5x20mm" V 2380 2300 50  0001 C CNN
F 3 "" H 2450 2300 50  0001 C CNN
	1    2450 2300
	0    1    1    0   
$EndComp
$Comp
L Device:CP C11
U 1 1 5AB55640
P 3000 2600
F 0 "C11" H 3025 2700 50  0000 L CNN
F 1 "22uF / 25V" H 3025 2500 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3038 2450 50  0001 C CNN
F 3 "" H 3000 2600 50  0001 C CNN
	1    3000 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2750 3000 2850
$Comp
L power:PWR_FLAG #FLG02
U 1 1 5AB6E459
P 3000 2100
F 0 "#FLG02" H 3000 2175 50  0001 C CNN
F 1 "PWR_FLAG" H 3000 2250 50  0000 C CNN
F 2 "" H 3000 2100 50  0001 C CNN
F 3 "" H 3000 2100 50  0001 C CNN
	1    3000 2100
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad MK1
U 1 1 5AC1D8C3
P 4300 2750
F 0 "MK1" H 4400 2801 50  0000 L CNN
F 1 "Mounting_Hole_PAD" H 4400 2710 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 4300 2750 50  0001 C CNN
F 3 "" H 4300 2750 50  0001 C CNN
	1    4300 2750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad MK2
U 1 1 5AC1DA5F
P 4650 2750
F 0 "MK2" H 4750 2801 50  0000 L CNN
F 1 "Mounting_Hole_PAD" H 4750 2710 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 4650 2750 50  0001 C CNN
F 3 "" H 4650 2750 50  0001 C CNN
	1    4650 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 2300 3000 2450
$Comp
L Regulator_Linear:LM1117-3.3 U2
U 1 1 5AFF3235
P 3550 2300
F 0 "U2" H 3550 2542 50  0000 C CNN
F 1 "LM1117-3.3_SOT223" H 3550 2451 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 3550 2500 50  0001 C CNN
F 3 "http://www.onsemi.com/pub_link/Collateral/NCP1117-D.PDF" H 3650 2050 50  0001 C CNN
	1    3550 2300
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C12
U 1 1 5AFF3359
P 4000 2600
F 0 "C12" H 4050 2800 50  0000 L CNN
F 1 "22uF / 25V" H 4050 2700 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 4038 2450 50  0001 C CNN
F 3 "" H 4000 2600 50  0001 C CNN
	1    4000 2600
	1    0    0    -1  
$EndComp
Connection ~ 3000 2850
Wire Wire Line
	4000 2850 4000 2750
Wire Wire Line
	3850 2300 4000 2300
Wire Wire Line
	4000 2300 4000 2450
Wire Wire Line
	3250 2300 3000 2300
Wire Wire Line
	2900 2300 3000 2300
Connection ~ 3000 2300
Wire Wire Line
	3000 2100 3000 2300
$Comp
L Device:LED D2
U 1 1 5B00AAE2
P 6850 4250
F 0 "D2" H 6850 4350 50  0000 C CNN
F 1 "LED" H 6850 4150 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 6850 4250 50  0001 C CNN
F 3 "" H 6850 4250 50  0001 C CNN
	1    6850 4250
	0    1    -1   0   
$EndComp
$Comp
L Device:R R6
U 1 1 5B00CB9D
P 6850 4600
F 0 "R6" H 6930 4600 50  0000 C CNN
F 1 "330" V 6850 4600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6780 4600 50  0001 C CNN
F 3 "" H 6850 4600 50  0001 C CNN
	1    6850 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6850 4450 6850 4400
$Comp
L power:+5V #PWR02
U 1 1 5B01AC94
P 2650 2100
F 0 "#PWR02" H 2650 1950 50  0001 C CNN
F 1 "+5V" H 2665 2273 50  0000 C CNN
F 2 "" H 2650 2100 50  0001 C CNN
F 3 "" H 2650 2100 50  0001 C CNN
	1    2650 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2100 3000 2100
Connection ~ 3000 2100
Wire Wire Line
	3000 2850 3550 2850
Wire Wire Line
	3550 2600 3550 2850
Connection ~ 3550 2850
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5B01F25F
P 2700 2850
F 0 "#FLG01" H 2700 2925 50  0001 C CNN
F 1 "PWR_FLAG" H 2700 3000 50  0000 C CNN
F 2 "" H 2700 2850 50  0001 C CNN
F 3 "" H 2700 2850 50  0001 C CNN
	1    2700 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	2700 2850 3000 2850
$Comp
L Mechanical:MountingHole_Pad MK3
U 1 1 5B0490F6
P 5000 2750
F 0 "MK3" H 5100 2801 50  0000 L CNN
F 1 "Mounting_Hole_PAD" H 5100 2710 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 5000 2750 50  0001 C CNN
F 3 "" H 5000 2750 50  0001 C CNN
	1    5000 2750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad MK4
U 1 1 5B0491E3
P 5350 2750
F 0 "MK4" H 5450 2801 50  0000 L CNN
F 1 "Mounting_Hole_PAD" H 5450 2710 50  0001 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965_Pad" H 5350 2750 50  0001 C CNN
F 3 "" H 5350 2750 50  0001 C CNN
	1    5350 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 5BEC2DD0
P 1700 5550
F 0 "J1" H 1700 5750 50  0000 C CNN
F 1 "DHT22" H 1700 5350 50  0000 C CNN
F 2 "Connector2:JST_XH_B3B-XH-AM_1x03_P2.50mm_Vertical" H 1700 5550 50  0001 C CNN
F 3 "" H 1700 5550 50  0001 C CNN
	1    1700 5550
	-1   0    0    1   
$EndComp
Wire Wire Line
	1900 5100 1900 5450
Connection ~ 1900 5100
Text Label 7550 2300 0    50   ~ 0
CSN
Wire Wire Line
	4250 5000 4150 5000
$Comp
L Relais:SONGLE_SRD_05VDC-SL-A K1
U 1 1 5BEEDA21
P 7650 4600
F 0 "K1" H 7980 4646 50  0000 L CNN
F 1 "SONGLE_SRD-05VDC-SL-A" H 7980 4555 50  0000 L CNN
F 2 "Relais2:SRD_05VDC-SL-A" H 8500 4550 50  0001 C CNN
F 3 "" H 7650 4600 50  0001 C CNN
	1    7650 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 4250 8050 4900
$Comp
L Device:Crystal_GND24 X1
U 1 1 5BF43799
P 9150 3000
F 0 "X1" H 9341 3046 50  0000 L CNN
F 1 "16MHz" H 9341 2955 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 9150 3000 50  0001 C CNN
F 3 "~" H 9150 3000 50  0001 C CNN
	1    9150 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9650 2900 9650 3650
Wire Wire Line
	8950 3000 9000 3000
Wire Wire Line
	9300 3000 9350 3000
Wire Wire Line
	8950 2800 8950 2750
Wire Wire Line
	9150 2800 9500 2800
Wire Wire Line
	9500 2800 9500 3200
Wire Wire Line
	9500 3650 9650 3650
Wire Wire Line
	9150 3200 9500 3200
Connection ~ 9500 3200
Wire Wire Line
	9500 3200 9500 3650
$Comp
L power:+5V #PWR08
U 1 1 5BEEFF82
P 8450 5100
F 0 "#PWR08" H 8450 4950 50  0001 C CNN
F 1 "+5V" H 8465 5273 50  0000 C CNN
F 2 "" H 8450 5100 50  0001 C CNN
F 3 "" H 8450 5100 50  0001 C CNN
	1    8450 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 5100 8500 5100
$Comp
L Device:R R8
U 1 1 5BF0BB28
P 8100 5150
F 0 "R8" V 8180 5150 50  0000 C CNN
F 1 "2K2" V 8100 5150 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8030 5150 50  0001 C CNN
F 3 "" H 8100 5150 50  0001 C CNN
	1    8100 5150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5BF0BBE8
P 8100 5450
F 0 "R9" V 8180 5450 50  0000 C CNN
F 1 "4K7" V 8100 5450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 8030 5450 50  0001 C CNN
F 3 "" H 8100 5450 50  0001 C CNN
	1    8100 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 5000 8200 5000
Wire Wire Line
	8200 5000 8200 5300
Wire Wire Line
	8200 5300 8500 5300
Text Label 7850 5300 0    50   ~ 0
ECHO
Wire Wire Line
	7450 5650 8100 5650
Wire Wire Line
	8100 5650 8100 5600
Wire Wire Line
	8100 5650 8500 5650
Wire Wire Line
	8500 5650 8500 5400
Connection ~ 8100 5650
$Comp
L Sensor2:HC-SR04-HCSR04 U5
U 1 1 5BFA751A
P 8700 5250
F 0 "U5" H 8372 5303 60  0000 R CNN
F 1 "HC-SR04" H 8372 5197 60  0000 R CNN
F 2 "Sensor2:HC-SR04" H 8372 5144 60  0001 R CNN
F 3 "" H 8700 5250 60  0000 C CNN
	1    8700 5250
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5BF26CEA
P 3700 2950
F 0 "#PWR03" H 3700 2700 50  0001 C CNN
F 1 "GND" H 3700 2800 50  0000 C CNN
F 2 "" H 3700 2950 50  0001 C CNN
F 3 "" H 3700 2950 50  0001 C CNN
	1    3700 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 2950 3700 2850
$Comp
L Device:CP C14
U 1 1 5BFE435F
P 7150 2800
F 0 "C14" H 7175 2900 50  0000 L CNN
F 1 "10uF" H 7175 2700 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 7188 2650 50  0001 C CNN
F 3 "" H 7150 2800 50  0001 C CNN
	1    7150 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 3650 7150 3650
Wire Wire Line
	7150 1550 8250 1550
Connection ~ 8250 1550
$Comp
L Device:R R7
U 1 1 5C1F3392
P 6900 5100
F 0 "R7" V 6980 5100 50  0000 C CNN
F 1 "10k" V 6900 5100 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6830 5100 50  0001 C CNN
F 3 "" H 6900 5100 50  0001 C CNN
	1    6900 5100
	0    1    1    0   
$EndComp
Wire Wire Line
	4250 5650 4750 5650
Wire Wire Line
	4750 4800 4750 5650
NoConn ~ 3200 5600
NoConn ~ 3300 5600
Wire Wire Line
	1900 5650 2350 5650
Wire Wire Line
	1900 5550 1950 5550
Wire Wire Line
	1950 5550 1950 4700
Wire Wire Line
	1950 4700 2350 4700
Wire Wire Line
	2350 4400 2350 4000
Connection ~ 2350 4000
Wire Wire Line
	2200 4050 2200 4000
Connection ~ 2200 4000
Wire Wire Line
	2200 4000 2350 4000
Wire Wire Line
	2200 4350 2200 4600
Wire Wire Line
	1900 4000 2200 4000
$Comp
L Device:C C10
U 1 1 5E3672C6
P 2350 5350
F 0 "C10" H 2465 5396 50  0000 L CNN
F 1 "100nF" H 2465 5305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2388 5200 50  0001 C CNN
F 3 "~" H 2350 5350 50  0001 C CNN
	1    2350 5350
	1    0    0    -1  
$EndComp
$Comp
L ESP8266:ESP-12E U1
U 1 1 5BEB5DE8
P 3250 4700
F 0 "U1" H 3250 4600 50  0000 C CNN
F 1 "ESP-12E" H 3250 4800 50  0000 C CNN
F 2 "ESP8266:ESP-12E_SMD" H 3250 4700 50  0001 C CNN
F 3 "" H 3250 4700 50  0001 C CNN
	1    3250 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 5500 2350 5650
Connection ~ 2350 5650
Wire Wire Line
	2350 5650 4150 5650
$Comp
L Interface_Expansion:PCF8574 U3
U 1 1 5E37D383
P 5850 4800
F 0 "U3" H 5850 5000 50  0000 C CNN
F 1 "PCF8574" H 5850 4900 50  0000 C CNN
F 2 "ICs:SOIC127P1032X265-16N" H 5850 4800 50  0001 C CNN
F 3 "http://www.nxp.com/documents/data_sheet/PCF8574_PCF8574A.pdf" H 5850 4800 50  0001 C CNN
	1    5850 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5350 4700 5350 4800
Connection ~ 5350 4800
Wire Wire Line
	5350 4800 5350 4900
Wire Wire Line
	4150 4600 4400 4600
Wire Wire Line
	4400 4600 4400 4900
Wire Wire Line
	4400 4900 5050 4900
Wire Wire Line
	5050 4900 5050 4400
Wire Wire Line
	5350 4500 5100 4500
Wire Wire Line
	5100 4500 5100 4950
Wire Wire Line
	5100 4950 4350 4950
Wire Wire Line
	4350 4950 4350 4700
Wire Wire Line
	4350 4700 4150 4700
NoConn ~ 4150 4900
$Comp
L Device:C C13
U 1 1 5E40F11C
P 6600 5400
F 0 "C13" H 6715 5446 50  0000 L CNN
F 1 "100nF" H 6715 5355 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6638 5250 50  0001 C CNN
F 3 "~" H 6600 5400 50  0001 C CNN
	1    6600 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 5650 5300 5650
Connection ~ 4750 5650
Wire Wire Line
	5850 5500 5850 5650
Wire Wire Line
	5350 4800 5300 4800
Wire Wire Line
	5300 4800 5300 5650
Wire Wire Line
	6600 5550 6600 5650
Wire Wire Line
	6600 5250 6600 4000
Connection ~ 4450 4000
Text Label 4650 4400 0    50   ~ 0
TX
Text Label 4650 4500 0    50   ~ 0
RX
Wire Wire Line
	5050 4400 5350 4400
Connection ~ 5300 5650
Wire Wire Line
	5850 4100 5850 4000
Connection ~ 5850 4000
Wire Wire Line
	5850 4000 6600 4000
Wire Wire Line
	6850 4750 6850 4900
Wire Wire Line
	6850 4900 7050 4900
Wire Wire Line
	7450 5650 6600 5650
Wire Wire Line
	5300 5650 5850 5650
Connection ~ 7450 5650
Connection ~ 5850 5650
Connection ~ 6600 5650
Wire Wire Line
	6600 5650 5850 5650
Wire Wire Line
	6850 4100 6850 4000
Wire Wire Line
	6850 4000 7050 4000
Wire Wire Line
	7450 4000 7450 4300
Wire Wire Line
	7450 3950 7450 4000
Connection ~ 7450 4000
Wire Wire Line
	6350 4900 6750 4900
Wire Wire Line
	6750 4900 6750 5100
Wire Wire Line
	7800 5300 7800 5800
Wire Wire Line
	7800 5800 6400 5800
Wire Wire Line
	6400 5800 6400 4800
Wire Wire Line
	6400 4800 6350 4800
Wire Wire Line
	6350 4700 6450 4700
Wire Wire Line
	6450 4700 6450 5750
Wire Wire Line
	6450 5750 8450 5750
Wire Wire Line
	8450 5750 8450 5200
Wire Wire Line
	8450 5200 8500 5200
Wire Wire Line
	6350 4600 6500 4600
Wire Wire Line
	6350 4500 6450 4500
Wire Wire Line
	6350 4400 6400 4400
Wire Wire Line
	6400 4400 6400 2500
Wire Wire Line
	6400 2500 7750 2500
Wire Wire Line
	6450 2300 6450 4500
Wire Wire Line
	6450 2300 7750 2300
Wire Wire Line
	6500 4600 6500 2600
Wire Wire Line
	6500 2600 7750 2600
Wire Wire Line
	4000 2300 4250 2300
Wire Wire Line
	4250 2300 4250 2100
Connection ~ 4000 2300
$Comp
L Diode:1.5KExxA D3
U 1 1 5E337F68
P 7050 4550
F 0 "D3" H 7150 4650 50  0000 L CNN
F 1 "SMAJ50" H 6650 4650 50  0000 L CNN
F 2 "Diode_SMD:D_SMA" H 7050 4350 50  0001 C CNN
F 3 "https://www.vishay.com/docs/88301/15ke.pdf" H 7000 4550 50  0001 C CNN
	1    7050 4550
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 4900 7050 4700
Connection ~ 7050 4900
Wire Wire Line
	7050 4900 7450 4900
Wire Wire Line
	7050 4400 7050 4000
Connection ~ 7050 4000
Wire Wire Line
	7050 4000 7450 4000
Wire Wire Line
	7800 5300 8100 5300
Connection ~ 8100 5300
$Comp
L Device:D D1
U 1 1 5E3A5BBA
P 2750 2300
F 0 "D1" H 2800 2400 50  0000 C CNN
F 1 "1N5822" H 2750 2200 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 2750 2300 50  0001 C CNN
F 3 "~" H 2750 2300 50  0001 C CNN
	1    2750 2300
	-1   0    0    1   
$EndComp
Wire Wire Line
	2700 2850 2300 2850
Wire Wire Line
	2300 2850 2300 2500
Connection ~ 2700 2850
Connection ~ 2300 2500
Wire Wire Line
	2300 2500 2300 2400
Connection ~ 4300 2850
Wire Wire Line
	4300 2850 4650 2850
Connection ~ 4650 2850
Wire Wire Line
	4650 2850 5000 2850
Connection ~ 5000 2850
Wire Wire Line
	5000 2850 5350 2850
Wire Wire Line
	4000 2850 4300 2850
Wire Wire Line
	3550 2850 3700 2850
Wire Wire Line
	3700 2850 4000 2850
Connection ~ 3700 2850
Connection ~ 4000 2850
Wire Wire Line
	8950 2450 9000 2450
Connection ~ 9000 2450
Wire Wire Line
	9000 2450 9050 2450
Wire Wire Line
	8950 2200 8950 2150
Wire Wire Line
	8950 2150 9000 2150
Connection ~ 9000 2150
Wire Wire Line
	9000 2150 9050 2150
Wire Wire Line
	8950 2000 9350 2000
Wire Wire Line
	9950 2000 9500 2000
Connection ~ 9350 2000
Connection ~ 9500 2000
Wire Wire Line
	9500 2000 9350 2000
Wire Wire Line
	9300 1550 8900 1550
Connection ~ 8750 1550
Connection ~ 8900 1550
Wire Wire Line
	8900 1550 8750 1550
Wire Wire Line
	8250 3650 7750 3650
Connection ~ 8250 3650
Connection ~ 7600 3650
Connection ~ 7750 3650
Wire Wire Line
	7750 3650 7600 3650
Wire Wire Line
	8700 3650 8950 3650
Connection ~ 8700 3650
Connection ~ 9500 3650
Connection ~ 8950 3650
Wire Wire Line
	8950 3650 9350 3650
Connection ~ 9350 3650
Wire Wire Line
	9350 3650 9500 3650
Wire Wire Line
	7750 2800 7600 2800
Wire Wire Line
	7150 1550 7150 2650
Wire Wire Line
	7150 2950 7150 3650
Wire Wire Line
	8950 3000 8950 3350
Connection ~ 8950 3000
Connection ~ 8950 3350
Wire Wire Line
	9350 3350 9350 3000
Connection ~ 9350 3350
Connection ~ 9350 3000
Wire Wire Line
	9350 2750 8950 2750
Wire Wire Line
	9350 2750 9350 3000
Wire Wire Line
	9800 2600 9650 2600
Connection ~ 9650 2600
NoConn ~ 6350 5000
NoConn ~ 6350 5100
$Comp
L power:+3.3V #PWR06
U 1 1 5E4F5C8F
P 4250 2100
F 0 "#PWR06" H 4250 1950 50  0001 C CNN
F 1 "+3.3V" H 4250 2240 50  0000 C CNN
F 2 "" H 4250 2100 50  0001 C CNN
F 3 "" H 4250 2100 50  0001 C CNN
	1    4250 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2350 4000 4200 4000
Wire Wire Line
	2350 5200 2350 5100
Connection ~ 2350 5100
Wire Wire Line
	4250 5200 5350 5200
Wire Wire Line
	4450 4000 5850 4000
Connection ~ 4250 5200
Wire Wire Line
	4250 5200 4250 5000
$EndSCHEMATC
