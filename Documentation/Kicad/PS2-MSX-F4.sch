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
L PS2-MSX-F4-rescue:C-Device-PS2-MSX-F4-rescue C5
U 1 1 5A733FEE
P 6600 2300
F 0 "C5" H 6625 2400 50  0000 L CNN
F 1 "100nF" H 6625 2200 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 6638 2150 50  0001 C CNN
F 3 "~" H 6600 2300 50  0000 C CNN
	1    6600 2300
	1    0    0    -1  
$EndComp
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #PWR00
U 1 1 5A73727A
P 5200 2550
F 0 "#PWR00" H 5200 2300 50  0001 C CNN
F 1 "GNDD-power" H 5200 2400 50  0001 C CNN
F 2 "" H 5200 2550 50  0000 C CNN
F 3 "" H 5200 2550 50  0000 C CNN
	1    5200 2550
	1    0    0    -1  
$EndComp
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #DGND0101
U 1 1 60C49885
P 6150 4700
F 0 "#DGND0101" H 6150 4450 50  0001 C CNN
F 1 "GNDD-power" H 6150 4550 50  0000 C CNN
F 2 "" H 6150 4700 50  0000 C CNN
F 3 "" H 6150 4700 50  0000 C CNN
	1    6150 4700
	1    0    0    -1  
$EndComp
Text Label 5175 2750 0    50   ~ 0
X0'
Text Label 5175 2850 0    50   ~ 0
X2'
Text Label 5175 2950 0    50   ~ 0
X4'
Text Label 7250 2750 0    50   ~ 0
X1'
Text Label 7250 3050 0    50   ~ 0
X7'
Wire Wire Line
	6250 4650 6150 4650
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #PWR0104
U 1 1 60D3AEBE
P 6600 2500
F 0 "#PWR0104" H 6600 2250 50  0001 C CNN
F 1 "GNDD-power" H 6700 2350 50  0001 C CNN
F 2 "" H 6600 2500 50  0000 C CNN
F 3 "" H 6600 2500 50  0000 C CNN
	1    6600 2500
	1    0    0    -1  
$EndComp
Text Notes 2375 7175 0    50   ~ 0
KANA LED is connected to YM2149 IOB7, pin 6 of DIP package. (Active LOW.)\nIf not used, let pin floating, as it is already connected to an internal pull up.
Text Notes 2375 7000 0    50   ~ 0
CAPS LED is connected to PPI 8255 PC6, pin 11 of DIP package. (Active LOW)
Text Notes 8250 7650 0    50   ~ 0
October, 31th 2021
Text Notes 7400 7500 0    50   ~ 0
PS/2 Adapter for MSX keyboard Project using WeAct Studio STM32F401CCU6 V2.0+ (Black Pill)
Wire Wire Line
	5200 2150 5200 2200
Wire Wire Line
	5200 2500 5200 2550
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R18
U 1 1 60C010C5
P 3450 3050
F 0 "R18" V 3450 2950 50  0000 L CNN
F 1 "22R" V 3550 2950 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 3050 50  0001 C CNN
F 3 "~" H 3450 3050 50  0001 C CNN
	1    3450 3050
	0    1    1    0   
$EndComp
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R12
U 1 1 60C0FA28
P 3450 4050
F 0 "R12" V 3450 3950 50  0000 L CNN
F 1 "10K" V 3350 3950 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 4050 50  0001 C CNN
F 3 "~" H 3450 4050 50  0001 C CNN
	1    3450 4050
	0    1    1    0   
$EndComp
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #PWR0107
U 1 1 60C6F39E
P 2700 2150
F 0 "#PWR0107" H 2700 1900 50  0001 C CNN
F 1 "GNDD-power" H 2700 2000 50  0001 C CNN
F 2 "" H 2700 2150 50  0000 C CNN
F 3 "" H 2700 2150 50  0000 C CNN
	1    2700 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 2100 2700 2100
Wire Wire Line
	2700 2100 2700 2150
NoConn ~ 6050 2450
NoConn ~ 6250 2450
NoConn ~ 7050 3850
NoConn ~ 7050 4050
NoConn ~ 7050 4150
NoConn ~ 7050 4250
Text Label 2400 1800 0    50   ~ 0
+5V_PS2_Keyboard
Text Label 2500 1900 0    50   ~ 0
PS2ClkPin
Wire Wire Line
	4000 2800 4000 3050
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R13
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
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R16
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
L PS2-MSX-F4-rescue:C_Small-Device-PS2-MSX-F4-rescue C3
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
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #DGND0103
U 1 1 60BDA83D
P 4000 4550
F 0 "#DGND0103" H 4000 4300 50  0001 C CNN
F 1 "GNDD-power" H 4000 4400 50  0001 C CNN
F 2 "" H 4000 4550 50  0000 C CNN
F 3 "" H 4000 4550 50  0000 C CNN
	1    4000 4550
	1    0    0    -1  
$EndComp
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #DGND0102
U 1 1 60BD95E2
P 4300 4550
F 0 "#DGND0102" H 4300 4300 50  0001 C CNN
F 1 "GNDD-power" H 4300 4400 50  0001 C CNN
F 2 "" H 4300 4550 50  0000 C CNN
F 3 "" H 4300 4550 50  0000 C CNN
	1    4300 4550
	1    0    0    -1  
$EndComp
$Comp
L PS2-MSX-F4-rescue:C_Small-Device-PS2-MSX-F4-rescue C2
U 1 1 60BD8DCF
P 4300 4400
F 0 "C2" H 4300 4500 50  0000 L CNN
F 1 "220pF" H 4300 4300 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 4300 4400 50  0001 C CNN
F 3 "~" H 4300 4400 50  0001 C CNN
	1    4300 4400
	1    0    0    -1  
$EndComp
Text Label 2500 2000 0    50   ~ 0
PS2DataPin
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
L PS2-MSX-F4-rescue:+5VD-power-PS2-MSX-F4-rescue #PWR0111
U 1 1 60DC6D85
P 2650 6450
F 0 "#PWR0111" H 2650 6300 50  0001 C CNN
F 1 "+5VD-power" V 2650 6800 50  0000 C CNN
F 2 "" H 2650 6450 50  0001 C CNN
F 3 "" H 2650 6450 50  0001 C CNN
	1    2650 6450
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 2450 6150 2150
Wire Wire Line
	6600 2500 6600 2450
Wire Wire Line
	6150 4700 6150 4650
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #DGND0104
U 1 1 60E3070B
P 1400 6600
F 0 "#DGND0104" H 1400 6350 50  0001 C CNN
F 1 "GNDD-power" H 1400 6450 50  0001 C CNN
F 2 "" H 1400 6600 50  0000 C CNN
F 3 "" H 1400 6600 50  0000 C CNN
	1    1400 6600
	1    0    0    -1  
$EndComp
$Comp
L PS2-MSX-F4-rescue:Conn_01x04-Connector_Generic-PS2-MSX-F4-rescue J3
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
	2300 1900 3200 1900
Wire Wire Line
	2300 2000 3100 2000
Text Notes 7350 6950 0    71   ~ 0
Designed by Evandro Souza\nFree to use as is.\nFor more details, see the technical docs.
Wire Wire Line
	7050 2750 7650 2750
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
Wire Wire Line
	2100 5950 7350 5950
Wire Wire Line
	7275 5850 7275 3150
Wire Wire Line
	2100 5850 7275 5850
Wire Wire Line
	7050 3250 7350 3250
Wire Wire Line
	5325 5750 5325 3150
Wire Wire Line
	2100 5750 5325 5750
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R11
U 1 1 60C10DA4
P 3450 3850
F 0 "R11" V 3450 3750 50  0000 L CNN
F 1 "10K" V 3550 3750 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 3850 50  0001 C CNN
F 3 "~" H 3450 3850 50  0001 C CNN
	1    3450 3850
	0    1    1    0   
$EndComp
Text Label 5325 3150 0    50   ~ 0
Y0
Text Label 7100 3150 0    50   ~ 0
Y1
Text Label 7100 3250 0    50   ~ 0
Y2
Text Label 7100 3350 0    50   ~ 0
Y3
Wire Wire Line
	7950 3050 7050 3050
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R9
U 1 1 60C98265
P 3450 3250
F 0 "R9" V 3450 3200 50  0000 L CNN
F 1 "22R" V 3350 3175 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 3250 50  0001 C CNN
F 3 "~" H 3450 3250 50  0001 C CNN
	1    3450 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	1400 4950 1400 5050
Wire Wire Line
	2800 6150 2100 6150
Wire Wire Line
	2100 6250 2900 6250
Wire Wire Line
	1600 4950 1400 4950
Wire Wire Line
	1600 5050 1400 5050
Connection ~ 1400 5050
Wire Wire Line
	1400 5050 1400 5150
Wire Wire Line
	1600 5150 1400 5150
Connection ~ 1400 5150
Wire Wire Line
	1400 5150 1400 5250
Wire Wire Line
	1600 5250 1400 5250
Connection ~ 1400 5250
Wire Wire Line
	1400 5250 1400 5350
Wire Wire Line
	1600 5350 1400 5350
Connection ~ 1400 5350
Wire Wire Line
	1400 5350 1400 5450
Wire Wire Line
	1600 5450 1400 5450
Connection ~ 1400 5450
Wire Wire Line
	1400 5450 1400 5550
Wire Wire Line
	1600 5550 1400 5550
Connection ~ 1400 5550
Wire Wire Line
	1600 5650 1400 5650
Wire Wire Line
	1400 5550 1400 5650
Connection ~ 1400 5650
Wire Wire Line
	1400 5650 1400 5750
Wire Wire Line
	1600 5750 1400 5750
Connection ~ 1400 5750
Wire Wire Line
	1400 5750 1400 5850
Wire Wire Line
	1600 5850 1400 5850
Connection ~ 1400 5850
Wire Wire Line
	1400 5850 1400 5950
Wire Wire Line
	1600 5950 1400 5950
Connection ~ 1400 5950
Wire Wire Line
	1400 5950 1400 6050
Wire Wire Line
	1600 6050 1400 6050
Connection ~ 1400 6050
Wire Wire Line
	1400 6050 1400 6150
Wire Wire Line
	1600 6150 1400 6150
Connection ~ 1400 6150
Wire Wire Line
	1400 6150 1400 6250
Wire Wire Line
	1600 6250 1400 6250
Connection ~ 1400 6250
Wire Wire Line
	1400 6250 1400 6350
Wire Wire Line
	1600 6350 1400 6350
Connection ~ 1400 6350
Wire Wire Line
	1400 6350 1400 6450
Wire Wire Line
	1400 6450 1600 6450
Connection ~ 1400 6450
Wire Wire Line
	1400 6450 1400 6550
Text Label 7250 2950 0    50   ~ 0
X5'
Text Label 4950 3650 0    50   ~ 0
PS2Clk
Text Label 4950 3950 0    50   ~ 0
PS2Data
Wire Wire Line
	4000 2150 4000 2500
Text Label 4950 4050 0    50   ~ 0
KanaL
Text Label 4950 3850 0    50   ~ 0
CapsL
Wire Wire Line
	3100 3050 3300 3050
Wire Wire Line
	3200 2950 3300 2950
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R17
U 1 1 60C11556
P 3450 2950
F 0 "R17" V 3450 2850 50  0000 L CNN
F 1 "22R" V 3550 2850 50  0001 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 3380 2950 50  0001 C CNN
F 3 "~" H 3450 2950 50  0001 C CNN
	1    3450 2950
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 4500 4300 4550
NoConn ~ -2150 2900
Connection ~ 6150 2150
Connection ~ 4300 3650
Wire Wire Line
	4300 3650 4300 4300
Wire Wire Line
	4925 3050 5450 3050
Wire Wire Line
	4660 2850 5450 2850
Wire Wire Line
	4570 2750 4570 4955
Wire Wire Line
	2100 4950 3300 4950
Wire Wire Line
	4570 2750 5450 2750
Text Label 5175 3050 0    50   ~ 0
X6'
Wire Wire Line
	4660 5150 4660 2850
Wire Wire Line
	2800 3850 3300 3850
$Comp
L PS2-MSX-F4-rescue:+5VD-power-PS2-MSX-F4-rescue #PWR0101
U 1 1 610F50E6
P 6150 2075
F 0 "#PWR0101" H 6150 1925 50  0001 C CNN
F 1 "+5VD-power" H 5875 2150 50  0000 C CNN
F 2 "" H 6150 2075 50  0001 C CNN
F 3 "" H 6150 2075 50  0001 C CNN
	1    6150 2075
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2950 7850 2950
Wire Wire Line
	4300 2150 4300 2500
Wire Wire Line
	4300 2800 4300 2950
Wire Wire Line
	3600 3050 4000 3050
Connection ~ 4000 3050
Wire Wire Line
	3600 2950 4300 2950
Connection ~ 4300 2950
Wire Wire Line
	4300 2950 4300 3650
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R1
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
	3600 4950 4570 4955
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R2
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
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R3
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
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R4
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
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R5
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
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R6
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
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R7
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
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R8
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
	2100 5050 3300 5050
Wire Wire Line
	2100 5150 3300 5150
Wire Wire Line
	2100 5250 3300 5250
Wire Wire Line
	2100 5350 3300 5350
Wire Wire Line
	2100 5450 3300 5450
Wire Wire Line
	2100 5550 3300 5550
Wire Wire Line
	2100 5650 3300 5650
Wire Wire Line
	5200 2150 4300 2150
Connection ~ 5200 2150
Connection ~ 4300 2150
Wire Wire Line
	4300 2150 4000 2150
Wire Wire Line
	3100 2000 3100 3050
Wire Wire Line
	3200 1900 3200 2950
$Sheet
S 8625 4650 1950 1400
U 60F35E5D
F0 "MiniDIN 6Pin Cable" 50
F1 "Cable.sch" 50
$EndSheet
Wire Notes Line
	10725 1700 10725 3800
Text Notes 3000 4750 0    50   ~ 0
Protection only. If you\ndon't want protection,\nlet these resistors short.
Wire Notes Line
	3825 2475 4550 2475
Wire Notes Line
	4550 2475 4550 2825
Wire Notes Line
	4550 2825 3825 2825
Wire Notes Line
	3825 2825 3825 2475
Text Notes 3550 2450 0    50   ~ 0
Pull up of PS/2 keyboard lines.\nThis is the one that might left open.
Wire Wire Line
	7050 3150 7275 3150
Wire Wire Line
	7650 2750 7650 5050
$Comp
L PS2-MSX-F4-rescue:BlackPill-minif4_board-1-YAAJ_BluePill-PS2-MSX-F4-rescue U1
U 1 1 60FE37A6
P 6250 3550
F 0 "U1" H 5650 2600 50  0000 C CNN
F 1 "WeAct Studio STM32F401CEU6 V3.0+ (Black Pill)" H 6325 2150 50  0000 C CNN
F 2 "BlueBlackPill:BlackPill-minif4_board-1" V 6175 4500 50  0001 C CNN
F 3 "" V 6175 4500 50  0001 C CNN
	1    6250 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3050 4000 3950
Connection ~ 4000 3950
Wire Wire Line
	4000 3950 4000 4300
Text Label 2400 6050 0    50   ~ 0
Y3
Text Label 5165 3750 0    50   ~ 0
X3'
Wire Wire Line
	2100 6050 7425 6050
Wire Wire Line
	4830 2950 5450 2950
Wire Wire Line
	3600 3850 5450 3850
Wire Wire Line
	4300 3650 5450 3650
Wire Wire Line
	4000 3950 5450 3950
Wire Wire Line
	3600 4050 5450 4050
Wire Wire Line
	7350 3250 7350 5950
Wire Wire Line
	5450 3150 5325 3150
Wire Wire Line
	5200 2150 6150 2150
NoConn ~ 5950 2450
NoConn ~ 7050 3950
NoConn ~ 5450 3450
NoConn ~ 5450 3550
NoConn ~ 7050 3750
Wire Wire Line
	7850 2950 7850 5450
Wire Wire Line
	7950 5650 7950 3050
NoConn ~ 5450 4150
NoConn ~ 5450 4350
NoConn ~ 5450 4250
Wire Wire Line
	4830 2950 4830 5355
Wire Wire Line
	4925 5550 4925 3050
Wire Wire Line
	3600 5650 7950 5650
Text Notes 2850 3775 0    50   ~ 0
Protection only. If you\ndon't want protection,\nlet this resistor short.
Wire Wire Line
	7425 3350 7425 6050
Wire Notes Line
	8025 1700 10725 1700
Wire Notes Line
	8025 3800 8025 1700
Wire Notes Line
	10725 3800 8025 3800
Connection ~ 8275 2150
Wire Wire Line
	8525 2150 8275 2150
Wire Wire Line
	8925 2500 8925 2750
Connection ~ 8925 2500
Wire Wire Line
	8925 2450 8925 2500
Wire Wire Line
	8525 2150 8725 2150
Connection ~ 8525 2150
Wire Wire Line
	8525 2500 8925 2500
Wire Wire Line
	8275 2150 8275 2200
Wire Wire Line
	7050 3350 7425 3350
Wire Wire Line
	8525 2200 8525 2150
Wire Wire Line
	8575 3450 8625 3450
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R15
U 1 1 60C44C05
P 8525 2350
F 0 "R15" H 8575 2400 50  0000 L CNN
F 1 "330K" H 8595 2305 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8455 2350 50  0001 C CNN
F 3 "~" H 8525 2350 50  0001 C CNN
	1    8525 2350
	1    0    0    -1  
$EndComp
Text Notes 9225 2925 0    50   ~ 0
This module have the function to cut\npower of the keyboard connector\nif it is not detect at power up reset.\n\nIt's for protection.\nIf you don't need protection, let all\ncomponents open but short Q2 1-3.\nput 
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #PWR0106
U 1 1 60D1B504
P 8925 3675
F 0 "#PWR0106" H 8925 3425 50  0001 C CNN
F 1 "GNDD-power" H 8925 3525 50  0001 C CNN
F 2 "" H 8925 3675 50  0000 C CNN
F 3 "" H 8925 3675 50  0000 C CNN
	1    8925 3675
	1    0    0    -1  
$EndComp
Wire Wire Line
	8275 2550 8275 2500
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #PWR0103
U 1 1 60D3942A
P 8275 2550
F 0 "#PWR0103" H 8275 2300 50  0001 C CNN
F 1 "GNDD-power" H 8275 2400 50  0001 C CNN
F 2 "" H 8275 2550 50  0000 C CNN
F 3 "" H 8275 2550 50  0000 C CNN
	1    8275 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8925 3650 8925 3675
Wire Wire Line
	8925 3050 8925 3250
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R14
U 1 1 60D17D37
P 8925 2900
F 0 "R14" H 8995 2946 50  0000 L CNN
F 1 "33K" H 8995 2855 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8855 2900 50  0001 C CNN
F 3 "~" H 8925 2900 50  0001 C CNN
	1    8925 2900
	1    0    0    -1  
$EndComp
$Comp
L PS2-MSX-F4-rescue:R-Device-PS2-MSX-F4-rescue R10
U 1 1 60D17220
P 8425 3450
F 0 "R10" V 8425 3350 50  0000 L CNN
F 1 "33K" V 8325 3350 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0204_L3.6mm_D1.6mm_P7.62mm_Horizontal" V 8355 3450 50  0001 C CNN
F 3 "~" H 8425 3450 50  0001 C CNN
	1    8425 3450
	0    1    -1   0   
$EndComp
$Comp
L PS2-MSX-F4-rescue:C-Device-PS2-MSX-F4-rescue C1
U 1 1 5A733A7F
P 8275 2350
F 0 "C1" H 8300 2450 50  0000 L CNN
F 1 "100nF" H 8025 2200 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 8313 2200 50  0001 C CNN
F 3 "~" H 8275 2350 50  0000 C CNN
	1    8275 2350
	1    0    0    -1  
$EndComp
NoConn ~ 7050 3650
NoConn ~ 7050 3550
Wire Wire Line
	2300 3150 2425 3150
Text Label 2500 3250 0    50   ~ 0
3.3V_Serial_TX
Text Label 2500 3350 0    50   ~ 0
3.3V_Serial_RX
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #PWR0109
U 1 1 60C7B787
P 2425 3150
F 0 "#PWR0109" H 2425 2900 50  0001 C CNN
F 1 "GNDD-power" H 2425 3000 50  0001 C CNN
F 2 "" H 2425 3150 50  0000 C CNN
F 3 "" H 2425 3150 50  0000 C CNN
	1    2425 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3350 5450 3350
Wire Wire Line
	2300 3250 3300 3250
Wire Wire Line
	3600 3250 5450 3250
Wire Wire Line
	3600 5150 4660 5150
Wire Wire Line
	3600 5050 7650 5050
Wire Wire Line
	3600 5350 4830 5355
Wire Wire Line
	3600 5250 4745 5250
Wire Wire Line
	3600 5550 4925 5550
Wire Wire Line
	3600 5450 7850 5450
Wire Notes Line
	3300 2875 3600 2875
Wire Wire Line
	2800 6150 2800 3850
Wire Wire Line
	2900 4050 3300 4050
Wire Wire Line
	2900 6250 2900 4050
Wire Notes Line
	3300 2875 3300 5700
Wire Notes Line
	3300 5700 3600 5700
Wire Notes Line
	3600 2875 3600 5700
Wire Wire Line
	7050 3450 8275 3450
$Comp
L PS2-MSX-F4-rescue:2SC1815-Transistor_BJT-PS2-MSX-F4-rescue Q1
U 1 1 611133C5
P 8825 3450
F 0 "Q1" H 9015 3496 50  0000 L CNN
F 1 "2SC1815" H 9015 3405 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 9025 3375 50  0001 L CIN
F 3 "https://media.digikey.com/pdf/Data%20Sheets/Toshiba%20PDFs/2SC1815.pdf" H 8825 3450 50  0001 L CNN
	1    8825 3450
	1    0    0    -1  
$EndComp
$Comp
L PS2-MSX-F4-rescue:Polyfuse_Small-Device-PS2-MSX-F4-rescue F1
U 1 1 61180481
P 2475 6450
F 0 "F1" V 2555 6500 50  0000 R CNN
F 1 "350mA Polyfuse SMD 0603 or 1206" V 2375 7655 50  0000 R CNN
F 2 "BlueBlackPill:Fuse_1206_0603_3216Metric_Pad1.42x1.75mm_HandSolder" H 2525 6250 50  0001 L CNN
F 3 "~" H 2475 6450 50  0001 C CNN
	1    2475 6450
	0    1    1    0   
$EndComp
Wire Wire Line
	6150 2150 6600 2150
$Comp
L PS2-MSX-F4-rescue:2N3906-Transistor_BJT-PS2-MSX-F4-rescue Q2
U 1 1 610ED9BF
P 8925 2250
F 0 "Q2" V 9253 2250 50  0000 C CNN
F 1 "2N2907" V 9162 2250 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 9125 2175 50  0001 L CIN
F 3 "https://www.fairchildsemi.com/datasheets/2N/2N3906.pdf" H 8925 2250 50  0001 L CNN
	1    8925 2250
	0    1    -1   0   
$EndComp
Wire Wire Line
	9125 1800 9125 2150
Connection ~ 6600 2150
Wire Wire Line
	6600 2150 8275 2150
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 612F6220
P 2100 3250
F 0 "J2" H 2110 3445 50  0000 C CNN
F 1 "Conn_01x04" H 2018 3476 50  0001 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 2100 3250 50  0001 C CNN
F 3 "~" H 2100 3250 50  0001 C CNN
	1    2100 3250
	-1   0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 613037C7
P 2435 3450
F 0 "#PWR0102" H 2435 3300 50  0001 C CNN
F 1 "+3.3V" V 2450 3578 50  0000 L CNN
F 2 "" H 2435 3450 50  0001 C CNN
F 3 "" H 2435 3450 50  0001 C CNN
	1    2435 3450
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0105
U 1 1 61305367
P 6350 2400
F 0 "#PWR0105" H 6350 2250 50  0001 C CNN
F 1 "+3.3V" H 6365 2573 50  0000 C CNN
F 2 "" H 6350 2400 50  0001 C CNN
F 3 "" H 6350 2400 50  0001 C CNN
	1    6350 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6350 2450 6350 2400
Wire Wire Line
	2300 3450 2435 3450
$Comp
L Device:CP C4
U 1 1 614261B2
P 5200 2350
F 0 "C4" H 5318 2396 50  0000 L CNN
F 1 "47uF x 10V" H 5318 2305 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 5238 2200 50  0001 C CNN
F 3 "~" H 5200 2350 50  0001 C CNN
	1    5200 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 2075 6150 2150
Wire Wire Line
	2300 1800 9125 1800
Wire Wire Line
	2650 6450 2575 6450
Wire Wire Line
	4745 3750 4745 5250
Wire Wire Line
	5450 3750 4745 3750
NoConn ~ 7050 2850
$Comp
L Connector_Generic:Conn_02x17_Odd_Even J1
U 1 1 6182BBA5
P 1800 5750
F 0 "J1" H 1850 6767 50  0000 C CNN
F 1 "Conn_02x17_Odd_Even" H 1850 6676 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x17_P2.54mm_Vertical" H 1800 5750 50  0001 C CNN
F 3 "~" H 1800 5750 50  0001 C CNN
	1    1800 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 6550 1400 6550
Connection ~ 1400 6550
Wire Wire Line
	1400 6550 1400 6600
Wire Wire Line
	2100 6550 2250 6550
Wire Wire Line
	2250 6450 2375 6450
Wire Wire Line
	2100 6350 2160 6350
Wire Wire Line
	2250 6550 2250 6450
Wire Wire Line
	2100 6450 2250 6450
Connection ~ 2250 6450
$Comp
L PS2-MSX-F4-rescue:GNDD-power-PS2-MSX-F4-rescue #DGND0105
U 1 1 618E7E2F
P 2160 6600
F 0 "#DGND0105" H 2160 6350 50  0001 C CNN
F 1 "GNDD-power" H 2160 6450 50  0001 C CNN
F 2 "" H 2160 6600 50  0000 C CNN
F 3 "" H 2160 6600 50  0000 C CNN
	1    2160 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2160 6350 2160 6600
$EndSCHEMATC
