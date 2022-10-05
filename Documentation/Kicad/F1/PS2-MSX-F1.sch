EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 2
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
U 1 1 5A733A7F
P 8150 2350
F 0 "C1" H 8175 2450 50  0000 L CNN
F 1 "100nF" H 7900 2200 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 8188 2200 50  0001 C CNN
F 3 "~" H 8150 2350 50  0000 C CNN
	1    8150 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5A733FEE
P 7300 2300
F 0 "C5" H 7325 2400 50  0000 L CNN
F 1 "100nF" H 7325 2200 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 7338 2150 50  0001 C CNN
F 3 "~" H 7300 2300 50  0000 C CNN
	1    7300 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 4650 6450 4650
Text Label 7500 2950 0    50   ~ 0
X5'
Text Label 5250 2750 0    50   ~ 0
X0'
Text Label 5250 2850 0    50   ~ 0
X1'
Text Label 5250 2950 0    50   ~ 0
X2'
Text Label 5250 3050 0    50   ~ 0
X3'
Text Label 5200 4350 0    50   ~ 0
X7'
$Comp
L Device:R R10
U 1 1 60D17220
P 8300 3150
F 0 "R10" V 8300 3050 50  0000 L CNN
F 1 "33K" V 8200 3050 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8230 3150 50  0001 C CNN
F 3 "~" H 8300 3150 50  0001 C CNN
	1    8300 3150
	0    1    -1   0   
$EndComp
Wire Wire Line
	8800 2850 8800 2950
Wire Wire Line
	8800 3350 8800 3450
Connection ~ 6550 4650
Wire Wire Line
	6650 4650 6550 4650
Wire Wire Line
	8150 2550 8150 2500
Text Notes 3445 7025 0    50   ~ 0
KANA LED is connected to YM2149 IOB7, pin 6 of DIP package.\nActive LOW.\nIf not used, let pin floating, as it is already connected to an internal pull up.
Text Notes 3445 6775 0    50   ~ 0
CAPS LED is connected to PPI 8255 PC6, pin 11 of DIP package.\nActive LOW.
Text Notes 8250 7650 0    50   ~ 0
November 1st, 2021
Text Notes 8000 7500 0    50   ~ 0
PS/2 Adapter for MSX keyboard Project using STM32 (Blue Pill)
Wire Wire Line
	5550 2150 5550 2200
Wire Wire Line
	5550 2500 5550 2550
Text Notes 9100 2850 0    50   ~ 0
This module have the function to cut\npower of the keyboard connector\nif it is not detect at power up reset.\n\nIt's for protection.\nIf you don't need protection, let all\ncomponents open but short Q2 1-3.\nput 
$Comp
L Device:R R18
U 1 1 60C010C5
P 3450 3300
F 0 "R18" V 3450 3200 50  0000 L CNN
F 1 "22R" V 3550 3200 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 3300 50  0001 C CNN
F 3 "~" H 3450 3300 50  0001 C CNN
	1    3450 3300
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 60C0FA28
P 3450 4050
F 0 "R12" V 3450 3950 50  0000 L CNN
F 1 "10K" V 3350 3950 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 4050 50  0001 C CNN
F 3 "~" H 3450 4050 50  0001 C CNN
	1    3450 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	2300 2100 2700 2100
Wire Wire Line
	2700 2100 2700 2150
NoConn ~ 6450 2450
NoConn ~ 6650 2450
NoConn ~ 7450 2750
NoConn ~ 7450 3250
NoConn ~ 7450 3350
NoConn ~ 7450 3450
NoConn ~ 7450 3550
NoConn ~ 7450 3850
NoConn ~ 7450 4050
NoConn ~ 7450 4150
NoConn ~ 7450 4250
Text Label 2400 1800 0    50   ~ 0
+5V_PS2_Keyboard
Text Label 2500 1900 0    50   ~ 0
PS2ClkPin
Wire Wire Line
	4000 2800 4000 3300
$Comp
L Device:R R13
U 1 1 60BD8FFB
P 4000 2650
F 0 "R13" H 4070 2696 50  0000 L CNN
F 1 "10K" H 4070 2605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3930 2650 50  0001 C CNN
F 3 "~" H 4000 2650 50  0001 C CNN
	1    4000 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R16
U 1 1 60BD7CF8
P 4300 2650
F 0 "R16" H 4370 2696 50  0000 L CNN
F 1 "10K" H 4370 2605 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 4230 2650 50  0001 C CNN
F 3 "~" H 4300 2650 50  0001 C CNN
	1    4300 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 4500 4000 4550
$Comp
L Device:C C3
U 1 1 60BE9F6A
P 4000 4400
F 0 "C3" H 4000 4500 50  0000 L CNN
F 1 "220pF" H 4000 4300 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 4000 4400 50  0001 C CNN
F 3 "~" H 4000 4400 50  0001 C CNN
	1    4000 4400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 60BD8DCF
P 4300 4400
F 0 "C2" H 4300 4500 50  0000 L CNN
F 1 "220pF" H 4300 4300 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 4300 4400 50  0001 C CNN
F 3 "~" H 4300 4400 50  0001 C CNN
	1    4300 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8450 3150 8500 3150
Text Label 2500 2000 0    50   ~ 0
PS2DataPin
Wire Wire Line
	9950 3925 9950 3950
Text Label 9350 3725 0    50   ~ 0
3.3V_Serial_RX
Text Label 9350 3825 0    50   ~ 0
3.3V_Serial_TX
Text Label 2400 5650 0    50   ~ 0
X7
Text Label 2400 5550 0    50   ~ 0
X6
Text Label 2400 5450 0    50   ~ 0
X5
Text Label 2400 5350 0    50   ~ 0
X4
Text Label 2400 5250 0    50   ~ 0
X3
Text Label 2400 5150 0    50   ~ 0
X2
Text Label 2400 5050 0    50   ~ 0
X1
Text Label 2400 4950 0    50   ~ 0
X0
$Comp
L power:+5VD #PWR0111
U 1 1 60DC6D85
P 2555 6450
F 0 "#PWR0111" H 2555 6300 50  0001 C CNN
F 1 "+5VD" V 2570 6623 50  0000 C CNN
F 2 "" H 2555 6450 50  0001 C CNN
F 3 "" H 2555 6450 50  0001 C CNN
	1    2555 6450
	0    1    1    0   
$EndComp
Wire Wire Line
	6550 2450 6550 2150
Wire Wire Line
	8400 2200 8400 2150
Wire Wire Line
	6550 4800 6550 4650
$Comp
L Connector_Generic:Conn_01x04 J3
U 1 1 60C1756C
P 2100 1900
F 0 "J3" H 2018 2217 50  0000 C CNN
F 1 "Conn_01x04" H 2018 2126 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 2100 1900 50  0001 C CNN
F 3 "~" H 2100 1900 50  0001 C CNN
	1    2100 1900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2300 1900 3050 1900
Wire Wire Line
	2300 2000 2950 2000
Text Notes 7350 6950 0    71   ~ 0
Designed by Evandro Souza\nFree to use as is.\nFor more details, see the technical docs.
Wire Wire Line
	7450 2850 7750 2850
Wire Wire Line
	7450 2950 7650 2950
Text Label 2400 6250 0    50   ~ 0
KanaLed
Text Label 2400 6150 0    50   ~ 0
CapsLed
Text Label 2400 5750 0    50   ~ 0
Y0
Text Label 2400 5850 0    50   ~ 0
Y1
Text Label 2400 5950 0    50   ~ 0
Y2
Text Label 2400 6050 0    50   ~ 0
Y3
Wire Wire Line
	2090 5950 5545 5950
Wire Wire Line
	5450 5850 5450 3250
Wire Wire Line
	2090 5850 5450 5850
Wire Wire Line
	5450 3250 5850 3250
Wire Wire Line
	5350 5750 5350 3150
Wire Wire Line
	2090 5750 5350 5750
Wire Wire Line
	5350 3150 5850 3150
$Comp
L Device:R R11
U 1 1 60C10DA4
P 3450 3850
F 0 "R11" V 3450 3750 50  0000 L CNN
F 1 "10K" V 3550 3750 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 3850 50  0001 C CNN
F 3 "~" H 3450 3850 50  0001 C CNN
	1    3450 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	5850 2950 4850 2950
Text Label 5650 3550 0    50   ~ 0
Y3
Text Label 5650 3450 0    50   ~ 0
Y2
Text Label 5650 3250 0    50   ~ 0
Y1
Text Label 5650 3150 0    50   ~ 0
Y0
Wire Wire Line
	5850 4350 5150 4350
Wire Wire Line
	7450 3750 8125 3750
$Comp
L Device:R R9
U 1 1 60C98265
P 8275 3750
F 0 "R9" V 8275 3700 50  0000 L CNN
F 1 "22R" V 8375 3650 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8205 3750 50  0001 C CNN
F 3 "~" H 8275 3750 50  0001 C CNN
	1    8275 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 4950 1400 5050
Wire Wire Line
	2800 6150 2090 6150
Wire Wire Line
	2090 6050 5635 6050
Wire Wire Line
	5635 3550 5850 3550
Wire Wire Line
	2090 6250 2900 6250
Wire Wire Line
	1590 4950 1400 4950
Wire Wire Line
	1590 5050 1400 5050
Connection ~ 1400 5050
Wire Wire Line
	1400 5050 1400 5150
Wire Wire Line
	1590 5150 1400 5150
Connection ~ 1400 5150
Wire Wire Line
	1400 5150 1400 5250
Wire Wire Line
	1590 5250 1400 5250
Connection ~ 1400 5250
Wire Wire Line
	1400 5250 1400 5350
Wire Wire Line
	1590 5350 1400 5350
Connection ~ 1400 5350
Wire Wire Line
	1400 5350 1400 5450
Wire Wire Line
	1590 5450 1400 5450
Connection ~ 1400 5450
Wire Wire Line
	1400 5450 1400 5550
Wire Wire Line
	1590 5550 1400 5550
Connection ~ 1400 5550
Wire Wire Line
	1590 5650 1400 5650
Wire Wire Line
	1400 5550 1400 5650
Connection ~ 1400 5650
Wire Wire Line
	1400 5650 1400 5750
Wire Wire Line
	1590 5750 1400 5750
Connection ~ 1400 5750
Wire Wire Line
	1400 5750 1400 5850
Wire Wire Line
	1590 5850 1400 5850
Connection ~ 1400 5850
Wire Wire Line
	1400 5850 1400 5950
Wire Wire Line
	1590 5950 1400 5950
Connection ~ 1400 5950
Wire Wire Line
	1400 5950 1400 6050
Wire Wire Line
	1590 6050 1400 6050
Connection ~ 1400 6050
Wire Wire Line
	1400 6050 1400 6150
Wire Wire Line
	1590 6150 1400 6150
Connection ~ 1400 6150
Wire Wire Line
	1400 6150 1400 6250
Wire Wire Line
	1590 6250 1400 6250
Connection ~ 1400 6250
Wire Wire Line
	1400 6250 1400 6350
Wire Wire Line
	1590 6350 1400 6350
Connection ~ 1400 6350
Wire Wire Line
	1400 6350 1400 6450
Wire Wire Line
	1400 6450 1590 6450
Connection ~ 1400 6450
Wire Wire Line
	1400 6450 1400 6550
Text Label 5200 4250 0    50   ~ 0
X6'
Wire Wire Line
	8150 2150 8150 2200
Wire Wire Line
	8400 2500 8800 2500
$Comp
L Device:R JP1
U 1 1 60E1A1F2
P 4450 5650
F 0 "JP1" V 4450 5600 50  0000 L CNN
F 1 ".01" V 4350 5600 50  0001 L CNN
F 2 "Kicad:SolderWire-0.25sqmm_1x01_D0.65mm_OD1.7mm_Relief.kicad_SM2000" V 4380 5650 50  0001 C CNN
F 3 "~" H 4450 5650 50  0001 C CNN
	1    4450 5650
	0    -1   -1   0   
$EndComp
Text Label 5000 3650 0    50   ~ 0
PS2Clk
Text Label 5000 4150 0    50   ~ 0
PS2Data
Wire Wire Line
	4000 2150 4000 2500
Text Label 5000 4050 0    50   ~ 0
KanaL
Text Label 5000 3850 0    50   ~ 0
CapsL
Text Notes 3445 7225 0    50   ~ 0
JP1 is not a resistor, but wire jumper, to make the pcb a single copper layer one.
Connection ~ 8400 2150
Wire Wire Line
	8400 2150 8600 2150
Wire Wire Line
	9000 2150 9000 1800
Wire Wire Line
	8800 2450 8800 2500
Connection ~ 8800 2500
Wire Wire Line
	8800 2500 8800 2550
Wire Wire Line
	5150 5650 4600 5650
Wire Wire Line
	8400 2150 8150 2150
Wire Wire Line
	2950 3300 3300 3300
Wire Wire Line
	3050 3000 3300 3000
$Comp
L Device:R R17
U 1 1 60C11556
P 3450 3000
F 0 "R17" V 3450 2900 50  0000 L CNN
F 1 "22R" V 3550 2900 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 3000 50  0001 C CNN
F 3 "~" H 3450 3000 50  0001 C CNN
	1    3450 3000
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 4500 4300 4550
NoConn ~ -2150 2900
Wire Wire Line
	4000 4150 5850 4150
Wire Wire Line
	5850 3850 3600 3850
Connection ~ 8150 2150
Connection ~ 6550 2150
Wire Wire Line
	5550 2150 6550 2150
NoConn ~ 5850 3750
NoConn ~ 5850 3950
Wire Wire Line
	4950 3050 5850 3050
Wire Wire Line
	5850 4050 3600 4050
Wire Wire Line
	4750 2850 5850 2850
Wire Wire Line
	4650 2750 4650 4950
Wire Wire Line
	2090 4950 3300 4950
Wire Wire Line
	4650 2750 5850 2750
Text Label 7500 2850 0    50   ~ 0
X4'
Wire Wire Line
	4750 5050 4750 2850
Wire Wire Line
	7650 5450 7650 2950
Wire Wire Line
	2900 6250 2900 4050
Wire Wire Line
	2900 4050 3300 4050
Wire Wire Line
	2800 6150 2800 3850
Wire Wire Line
	2800 3850 3300 3850
Wire Wire Line
	2090 6450 2245 6450
$Comp
L power:+5VD #PWR0101
U 1 1 610F50E6
P 6550 2150
F 0 "#PWR0101" H 6550 2000 50  0001 C CNN
F 1 "+5VD" H 6565 2323 50  0000 C CNN
F 2 "" H 6550 2150 50  0001 C CNN
F 3 "" H 6550 2150 50  0001 C CNN
	1    6550 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 5550 5050 4250
Wire Wire Line
	5050 4250 5850 4250
Wire Wire Line
	5150 4350 5150 5650
Wire Wire Line
	4300 2150 4300 2500
Wire Wire Line
	4300 2800 4300 3000
Wire Wire Line
	3600 3300 4000 3300
Connection ~ 4000 3300
Wire Wire Line
	4000 3300 4000 4150
Wire Wire Line
	3600 3000 4300 3000
Connection ~ 4300 3000
Wire Wire Line
	4300 3000 4300 3650
$Comp
L Device:R R1
U 1 1 60E4FC55
P 3450 4950
F 0 "R1" V 3450 4900 50  0000 L CNN
F 1 "22R" V 3550 4850 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 4950 50  0001 C CNN
F 3 "~" H 3450 4950 50  0001 C CNN
	1    3450 4950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3600 4950 4650 4950
Wire Wire Line
	3600 5050 4750 5050
$Comp
L Device:R R2
U 1 1 60E5E39D
P 3450 5050
F 0 "R2" V 3450 5000 50  0000 L CNN
F 1 "22R" V 3350 4950 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 5050 50  0001 C CNN
F 3 "~" H 3450 5050 50  0001 C CNN
	1    3450 5050
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 60E6AB1A
P 3450 5150
F 0 "R3" V 3450 5100 50  0000 L CNN
F 1 "22R" V 3350 5050 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 5150 50  0001 C CNN
F 3 "~" H 3450 5150 50  0001 C CNN
	1    3450 5150
	0    -1   1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 60E6B09F
P 3450 5250
F 0 "R4" V 3450 5200 50  0000 L CNN
F 1 "22R" V 3350 5150 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 5250 50  0001 C CNN
F 3 "~" H 3450 5250 50  0001 C CNN
	1    3450 5250
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R5
U 1 1 60E6B6A2
P 3450 5350
F 0 "R5" V 3450 5300 50  0000 L CNN
F 1 "22R" V 3350 5250 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 5350 50  0001 C CNN
F 3 "~" H 3450 5350 50  0001 C CNN
	1    3450 5350
	0    -1   1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 60E6BE6E
P 3450 5450
F 0 "R6" V 3450 5400 50  0000 L CNN
F 1 "22R" V 3350 5350 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 5450 50  0001 C CNN
F 3 "~" H 3450 5450 50  0001 C CNN
	1    3450 5450
	0    -1   1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 60E6C456
P 3450 5550
F 0 "R7" V 3450 5500 50  0000 L CNN
F 1 "22R" V 3350 5450 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 5550 50  0001 C CNN
F 3 "~" H 3450 5550 50  0001 C CNN
	1    3450 5550
	0    -1   1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 60E6C9B8
P 3450 5650
F 0 "R8" V 3450 5600 50  0000 L CNN
F 1 "22R" V 3350 5550 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 5650 50  0001 C CNN
F 3 "~" H 3450 5650 50  0001 C CNN
	1    3450 5650
	0    -1   1    0   
$EndComp
Wire Wire Line
	2090 5050 3300 5050
Wire Wire Line
	2090 5150 3300 5150
Wire Wire Line
	2090 5250 3300 5250
Wire Wire Line
	2090 5350 3300 5350
Wire Wire Line
	2090 5450 3300 5450
Wire Wire Line
	2090 5550 3300 5550
Wire Wire Line
	2090 5650 3300 5650
Wire Wire Line
	3600 5450 7650 5450
Wire Wire Line
	4300 5650 3600 5650
Wire Wire Line
	5550 2150 4300 2150
Connection ~ 5550 2150
Connection ~ 4300 2150
Wire Wire Line
	4300 2150 4000 2150
Wire Wire Line
	4850 2950 4850 5150
Wire Wire Line
	4850 5150 3600 5150
Wire Wire Line
	2950 2000 2950 3300
Wire Wire Line
	3050 1900 3050 3000
$Sheet
S 8625 4650 1950 1400
U 60F35E5D
F0 "MiniDIN 6Pin Cable" 50
F1 "Cable.sch" 50
$EndSheet
Wire Notes Line
	10600 1700 10600 3600
Wire Notes Line
	10600 3550 7900 3550
Wire Notes Line
	7900 3550 7900 1700
Wire Notes Line
	7900 1700 10600 1700
Wire Notes Line
	3300 2900 3600 2900
Wire Notes Line
	3300 5700 3600 5700
Wire Notes Line
	3600 5700 3600 2900
Wire Notes Line
	3300 5700 3300 2900
Text Notes 3000 4750 0    50   ~ 0
Protection only. If you\ndon't want protection,\nlet these resistors short.
Wire Notes Line
	8125 3675 8475 3675
Wire Notes Line
	8475 3675 8475 3925
Wire Notes Line
	8475 3925 8125 3925
Wire Notes Line
	8125 3925 8125 3675
Wire Notes Line
	3825 2475 4550 2475
Wire Notes Line
	4550 2475 4550 2825
Wire Notes Line
	4550 2825 3825 2825
Wire Notes Line
	3825 2825 3825 2475
Text Notes 7925 4175 0    50   ~ 0
Protection only. If you\ndon't want protection,\nlet this resistor short.
Text Notes 3550 2450 0    50   ~ 0
Pull up of PS/2 keyboard lines.\nThis is the one that might left open.
Wire Wire Line
	4300 3650 5850 3650
Wire Wire Line
	4950 3050 4950 5250
Wire Wire Line
	3600 5250 4950 5250
Wire Wire Line
	3600 5350 7750 5350
Wire Wire Line
	7750 2850 7750 5350
$Comp
L power:+3.3V #PWR0102
U 1 1 6145DE8F
P 6750 2400
F 0 "#PWR0102" H 6750 2250 50  0001 C CNN
F 1 "+3.3V" H 6765 2573 50  0000 C CNN
F 2 "" H 6750 2400 50  0001 C CNN
F 3 "" H 6750 2400 50  0001 C CNN
	1    6750 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6750 2400 6750 2450
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 61464F8D
P 10150 3825
F 0 "J2" H 10150 4025 50  0000 C CNN
F 1 "Conn_01x04" H 10450 3775 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 10150 3825 50  0001 C CNN
F 3 "~" H 10150 3825 50  0001 C CNN
	1    10150 3825
	1    0    0    1   
$EndComp
Wire Wire Line
	9950 3825 8725 3825
Wire Wire Line
	9950 3725 8800 3725
Wire Wire Line
	8800 3725 8800 3650
Wire Wire Line
	8800 3650 7450 3650
$Comp
L power:+3.3V #PWR0105
U 1 1 6149C09E
P 9875 3625
F 0 "#PWR0105" H 9875 3475 50  0001 C CNN
F 1 "+3.3V" V 9890 3753 50  0000 L CNN
F 2 "" H 9875 3625 50  0001 C CNN
F 3 "" H 9875 3625 50  0001 C CNN
	1    9875 3625
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9875 3625 9950 3625
Wire Wire Line
	8425 3750 8725 3750
Wire Wire Line
	8725 3750 8725 3825
Wire Wire Line
	3600 5550 5050 5550
Wire Wire Line
	6550 2150 7300 2150
$Comp
L PS2-MSX-F1-rescue:BluePill-BlueBlackPill U1
U 1 1 61386AF8
P 6650 3550
F 0 "U1" H 6650 2219 50  0000 C CNN
F 1 "STM32F103C6T6 BluePill" H 6125 2350 50  0000 C CNN
F 2 "BlueBlackPill:BluePill_1" V 6575 4500 50  0001 C CNN
F 3 "" V 6575 4500 50  0001 C CNN
	1    6650 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0103
U 1 1 61389AE5
P 6550 4800
F 0 "#PWR0103" H 6550 4550 50  0001 C CNN
F 1 "GNDD" H 6554 4645 50  0000 C CNN
F 2 "" H 6550 4800 50  0001 C CNN
F 3 "" H 6550 4800 50  0001 C CNN
	1    6550 4800
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:2N3905 Q2
U 1 1 6138E201
P 8800 2250
F 0 "Q2" V 9128 2250 50  0000 C CNN
F 1 "2N2907" V 9037 2250 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 9000 2175 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3905.pdf" H 8800 2250 50  0001 L CNN
	1    8800 2250
	0    1    -1   0   
$EndComp
$Comp
L Device:R R14
U 1 1 6139D81B
P 8800 2700
F 0 "R14" H 8870 2746 50  0000 L CNN
F 1 "33K" H 8870 2655 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8730 2700 50  0001 C CNN
F 3 "~" H 8800 2700 50  0001 C CNN
	1    8800 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R15
U 1 1 613A47B7
P 8400 2350
F 0 "R15" H 8470 2396 50  0000 L CNN
F 1 "150K" H 8470 2305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8330 2350 50  0001 C CNN
F 3 "~" H 8400 2350 50  0001 C CNN
	1    8400 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0104
U 1 1 613A5644
P 8150 2550
F 0 "#PWR0104" H 8150 2300 50  0001 C CNN
F 1 "GNDD" H 8154 2395 50  0001 C CNN
F 2 "" H 8150 2550 50  0001 C CNN
F 3 "" H 8150 2550 50  0001 C CNN
	1    8150 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0106
U 1 1 613AC922
P 8800 3450
F 0 "#PWR0106" H 8800 3200 50  0001 C CNN
F 1 "GNDD" H 8804 3295 50  0000 C CNN
F 2 "" H 8800 3450 50  0001 C CNN
F 3 "" H 8800 3450 50  0001 C CNN
	1    8800 3450
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0107
U 1 1 613AD05E
P 9950 3950
F 0 "#PWR0107" H 9950 3700 50  0001 C CNN
F 1 "GNDD" H 9954 3795 50  0001 C CNN
F 2 "" H 9950 3950 50  0001 C CNN
F 3 "" H 9950 3950 50  0001 C CNN
	1    9950 3950
	1    0    0    -1  
$EndComp
$Comp
L Transistor_BJT:2SC1815 Q1
U 1 1 613ADFEB
P 8700 3150
F 0 "Q1" H 8890 3196 50  0000 L CNN
F 1 "2SC1815" H 8890 3105 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 8900 3075 50  0001 L CIN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Toshiba%20PDFs/2SC1815.pdf" H 8700 3150 50  0001 L CNN
	1    8700 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0108
U 1 1 613B0393
P 7300 2500
F 0 "#PWR0108" H 7300 2250 50  0001 C CNN
F 1 "GNDD" H 7304 2345 50  0001 C CNN
F 2 "" H 7300 2500 50  0001 C CNN
F 3 "" H 7300 2500 50  0001 C CNN
	1    7300 2500
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0109
U 1 1 613B235D
P 5550 2550
F 0 "#PWR0109" H 5550 2300 50  0001 C CNN
F 1 "GNDD" H 5554 2395 50  0001 C CNN
F 2 "" H 5550 2550 50  0001 C CNN
F 3 "" H 5550 2550 50  0001 C CNN
	1    5550 2550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0112
U 1 1 613B59C8
P 4000 4550
F 0 "#PWR0112" H 4000 4300 50  0001 C CNN
F 1 "GNDD" H 4004 4395 50  0001 C CNN
F 2 "" H 4000 4550 50  0001 C CNN
F 3 "" H 4000 4550 50  0001 C CNN
	1    4000 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0113
U 1 1 613B6F78
P 4300 4550
F 0 "#PWR0113" H 4300 4300 50  0001 C CNN
F 1 "GNDD" H 4304 4395 50  0001 C CNN
F 2 "" H 4300 4550 50  0001 C CNN
F 3 "" H 4300 4550 50  0001 C CNN
	1    4300 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GNDD #PWR0114
U 1 1 613B77D3
P 2700 2150
F 0 "#PWR0114" H 2700 1900 50  0001 C CNN
F 1 "GNDD" H 2704 1995 50  0001 C CNN
F 2 "" H 2700 2150 50  0001 C CNN
F 3 "" H 2700 2150 50  0001 C CNN
	1    2700 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 4250 4000 4150
Connection ~ 4000 4150
Connection ~ 7300 2150
Wire Wire Line
	7300 2150 8150 2150
Wire Wire Line
	7300 2450 7300 2500
Wire Wire Line
	2300 1800 9000 1800
$Comp
L Device:CP C4
U 1 1 614738A3
P 5550 2350
F 0 "C4" H 5668 2396 50  0000 L CNN
F 1 "47uF x 10V" H 5668 2305 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 5588 2200 50  0001 C CNN
F 3 "~" H 5550 2350 50  0001 C CNN
	1    5550 2350
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x17_Odd_Even J1
U 1 1 617F6327
P 1790 5750
F 0 "J1" H 1840 6767 50  0000 C CNN
F 1 "Conn_02x17_Odd_Even" H 1840 6676 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x17_P2.54mm_Vertical" H 1790 5750 50  0001 C CNN
F 3 "~" H 1790 5750 50  0001 C CNN
	1    1790 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1590 6550 1400 6550
Connection ~ 1400 6550
Wire Wire Line
	1400 6550 1400 6600
$Comp
L power:GNDD #PWR0110
U 1 1 613B4A69
P 1400 6600
F 0 "#PWR0110" H 1400 6350 50  0001 C CNN
F 1 "GNDD" H 1404 6445 50  0001 C CNN
F 2 "" H 1400 6600 50  0001 C CNN
F 3 "" H 1400 6600 50  0001 C CNN
	1    1400 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2090 6550 2245 6550
Wire Wire Line
	2245 6550 2245 6450
Connection ~ 2245 6450
Wire Wire Line
	4300 4250 4300 3650
Connection ~ 4300 3650
Wire Wire Line
	2245 6450 2300 6450
Wire Wire Line
	2560 6450 2555 6450
$Comp
L Device:Polyfuse_Small F1
U 1 1 61220104
P 2400 6450
F 0 "F1" V 2330 6450 50  0000 C CNN
F 1 "Poly Fuse 300mA SMD 0603 or 1206" V 2250 6850 50  0000 C CNN
F 2 "BlueBlackPill:Fuse_1206_0603_3216Metric_Pad1.42x1.75mm_HandSolder" H 1610 6110 50  0001 L CNN
F 3 "" H 2400 6450 50  0001 C CNN
	1    2400 6450
	0    1    1    0   
$EndComp
$Comp
L power:GNDD #PWR0115
U 1 1 6187BF44
P 2150 6605
F 0 "#PWR0115" H 2150 6355 50  0001 C CNN
F 1 "GNDD" H 2154 6450 50  0001 C CNN
F 2 "" H 2150 6605 50  0001 C CNN
F 3 "" H 2150 6605 50  0001 C CNN
	1    2150 6605
	1    0    0    -1  
$EndComp
Connection ~ 2555 6450
Wire Wire Line
	2555 6450 2500 6450
Wire Wire Line
	2150 6605 2150 6350
Wire Wire Line
	2150 6350 2090 6350
NoConn ~ 5850 3350
Wire Wire Line
	5635 3550 5635 6050
Wire Wire Line
	5850 3450 5545 3450
Wire Wire Line
	5545 3450 5545 5950
NoConn ~ 7450 3155
Wire Wire Line
	7450 3050 8075 3050
Wire Wire Line
	8150 3150 8075 3150
Wire Wire Line
	8075 3150 8075 3050
$EndSCHEMATC
