EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
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
L PS2-MSX-F1-rescue:Mini-DIN-6-Connector-PS2-MSX-rescue-PS2-MSX-rescue J4
U 1 1 6105226E
P 5625 3000
AR Path="/6105226E" Ref="J4"  Part="1" 
AR Path="/60F35E5D/6105226E" Ref="J4"  Part="1" 
F 0 "J4" H 5625 3367 50  0000 C CNN
F 1 "Mini-DIN-6" H 5625 3276 50  0000 C CNN
F 2 "" H 5625 3000 50  0001 C CNN
F 3 "http://service.powerdynamics.com/ec/Catalog17/Section%2011.pdf" H 5625 3000 50  0001 C CNN
	1    5625 3000
	1    0    0    -1  
$EndComp
NoConn ~ 5325 2900
NoConn ~ 5325 3100
$Comp
L PS2-MSX-F1-rescue:Conn_01x04-Connector_Generic-PS2-MSX-rescue-PS2-MSX-rescue J5
U 1 1 6105227C
P 4925 3250
AR Path="/6105227C" Ref="J5"  Part="1" 
AR Path="/60F35E5D/6105227C" Ref="J5"  Part="1" 
F 0 "J5" H 4843 3567 50  0000 C CNN
F 1 "Conn_01x04" H 4843 3476 50  0000 C CNN
F 2 "" H 4925 3250 50  0001 C CNN
F 3 "~" H 4925 3250 50  0001 C CNN
	1    4925 3250
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5325 3000 5125 3000
Wire Wire Line
	5125 3000 5125 3150
Wire Wire Line
	5925 3350 5925 3100
Wire Wire Line
	5125 3350 5925 3350
Wire Wire Line
	5125 3250 5975 3250
Wire Wire Line
	5975 3250 5975 2900
Wire Wire Line
	5975 2900 5925 2900
Wire Wire Line
	5925 3000 6125 3000
Wire Wire Line
	6125 3000 6125 3450
Wire Wire Line
	6125 3450 5125 3450
$EndSCHEMATC
