EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
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
LIBS:polystack
LIBS:stm32
LIBS:invensense
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 3
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
L +5VA #PWR032
U 1 1 563342DB
P 7750 2800
F 0 "#PWR032" H 7750 2650 50  0001 C CNN
F 1 "+5VA" H 7750 2940 50  0000 C CNN
F 2 "" H 7750 2800 60  0000 C CNN
F 3 "" H 7750 2800 60  0000 C CNN
	1    7750 2800
	1    0    0    -1
$EndComp
Text Notes 8050 2600 2    60   ~ 0
USB Power
$Comp
L +5V #PWR033
U 1 1 56334305
P 3350 2100
F 0 "#PWR033" H 3350 1950 50  0001 C CNN
F 1 "+5V" H 3350 2240 50  0000 C CNN
F 2 "" H 3350 2100 60  0000 C CNN
F 3 "" H 3350 2100 60  0000 C CNN
	1    3350 2100
	1    0    0    -1
$EndComp
$Comp
L VDD #PWR034
U 1 1 5633432D
P 7300 4050
F 0 "#PWR034" H 7300 3900 50  0001 C CNN
F 1 "VDD" H 7300 4200 50  0000 C CNN
F 2 "" H 7300 4050 60  0000 C CNN
F 3 "" H 7300 4050 60  0000 C CNN
	1    7300 4050
	1    0    0    -1
$EndComp
$Comp
L GNDPWR #PWR035
U 1 1 56334341
P 7750 5100
F 0 "#PWR035" H 7750 4900 50  0001 C CNN
F 1 "GNDPWR" H 7750 4970 50  0000 C CNN
F 2 "" H 7750 5050 60  0000 C CNN
F 3 "" H 7750 5050 60  0000 C CNN
	1    7750 5100
	1    0    0    -1
$EndComp
Wire Wire Line
	6900 2600 6900 3300
Wire Wire Line
	5950 3500 5950 4400
Wire Wire Line
	7000 4300 8500 4300
Wire Wire Line
	7300 4050 7300 4400
Wire Wire Line
	7750 4700 7750 5100
$Comp
L CP1_Small C12
U 1 1 5633D516
P 7300 4500
F 0 "C12" H 7310 4570 50  0000 L CNN
F 1 "1uF+" H 7310 4420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 7300 4500 60  0001 C CNN
F 3 "" H 7300 4500 60  0000 C CNN
	1    7300 4500
	1    0    0    -1
$EndComp
$Comp
L CP1_Small C11
U 1 1 5633D535
P 5950 4500
F 0 "C11" H 5960 4570 50  0000 L CNN
F 1 "10uF+" H 5960 4420 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 5950 4500 60  0001 C CNN
F 3 "" H 5950 4500 60  0000 C CNN
	1    5950 4500
	1    0    0    -1
$EndComp
Connection ~ 7300 4300
Wire Wire Line
	7300 4700 7300 4600
Connection ~ 7300 4700
Connection ~ 5950 4300
Wire Wire Line
	5950 4600 5950 4700
Connection ~ 6550 4700
Connection ~ 7750 3300
Wire Wire Line
	3350 2100 3350 2750
Wire Wire Line
	7750 2800 7750 3300
Connection ~ 3350 2250
Connection ~ 5950 4700
Wire Wire Line
	7750 3300 7500 3300
$Comp
L PWR_FLAG #FLG036
U 1 1 5634570D
P 3700 2150
F 0 "#FLG036" H 3700 2245 50  0001 C CNN
F 1 "PWR_FLAG" H 3700 2330 50  0000 C CNN
F 2 "" H 3700 2150 60  0000 C CNN
F 3 "" H 3700 2150 60  0000 C CNN
	1    3700 2150
	1    0    0    -1
$EndComp
Wire Wire Line
	3700 2150 3700 2250
Connection ~ 3700 2250
$Comp
L CONN_02X10 P5
U 1 1 5634F19E
P 4100 3200
F 0 "P5" H 4100 3750 50  0000 C CNN
F 1 "CONN_02X10" V 4100 3200 50  0000 C CNN
F 2 "hirose-df40:DF40-20pin-Header" H 4100 2000 60  0001 C CNN
F 3 "" H 4100 2000 60  0000 C CNN
	1    4100 3200
	1    0    0    -1
$EndComp
$Comp
L +5V #PWR037
U 1 1 5634F1D6
P 6900 2600
F 0 "#PWR037" H 6900 2450 50  0001 C CNN
F 1 "+5V" H 6900 2740 50  0000 C CNN
F 2 "" H 6900 2600 60  0000 C CNN
F 3 "" H 6900 2600 60  0000 C CNN
	1    6900 2600
	1    0    0    -1
$EndComp
Wire Wire Line
	4350 4700 7850 4700
Wire Wire Line
	4350 2750 4350 4700
Connection ~ 4350 3650
Connection ~ 4350 3550
Connection ~ 4350 3450
Connection ~ 4350 3350
Connection ~ 4350 3250
Connection ~ 4350 2850
Connection ~ 4350 2950
Connection ~ 4350 3050
Connection ~ 4350 3150
Wire Wire Line
	3550 2750 3550 3550
Wire Wire Line
	3550 2850 3850 2850
Wire Wire Line
	3550 2950 3850 2950
Connection ~ 3550 2850
Wire Wire Line
	3550 3050 3850 3050
Connection ~ 3550 2950
Wire Wire Line
	3550 3150 3850 3150
Connection ~ 3550 3050
Wire Wire Line
	3550 3250 3850 3250
Connection ~ 3550 3150
Wire Wire Line
	3550 3350 3850 3350
Connection ~ 3550 3250
Wire Wire Line
	3550 3450 3850 3450
Connection ~ 3550 3350
Wire Wire Line
	3550 3550 3850 3550
Connection ~ 3550 3450
Wire Wire Line
	3700 2250 3350 2250
$Comp
L +BATT #PWR038
U 1 1 5634F8B8
P 3300 3550
F 0 "#PWR038" H 3300 3400 50  0001 C CNN
F 1 "+BATT" H 3300 3690 50  0000 C CNN
F 2 "" H 3300 3550 60  0000 C CNN
F 3 "" H 3300 3550 60  0000 C CNN
	1    3300 3550
	1    0    0    -1
$EndComp
$Comp
L PWR_FLAG #FLG039
U 1 1 5637C30C
P 2900 3550
F 0 "#FLG039" H 2900 3645 50  0001 C CNN
F 1 "PWR_FLAG" H 2900 3730 50  0000 C CNN
F 2 "" H 2900 3550 60  0000 C CNN
F 3 "" H 2900 3550 60  0000 C CNN
	1    2900 3550
	1    0    0    -1
$EndComp
Wire Wire Line
	4550 4050 4350 4050
Connection ~ 4350 4050
Wire Wire Line
	2900 3650 3850 3650
Wire Wire Line
	3300 3650 3300 3550
Wire Wire Line
	2900 3650 2900 3550
Connection ~ 3300 3650
$Comp
L TLV70233DBV U3
U 1 1 563C7B33
P 6550 4350
F 0 "U3" H 6300 4550 40  0000 C CNN
F 1 "TLV70233DBV" H 6650 4550 40  0000 C CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23-5" H 6550 4450 35  0000 C CIN
F 3 "" H 6550 4350 60  0000 C CNN
	1    6550 4350
	1    0    0    -1
$EndComp
Wire Wire Line
	6100 4450 6100 4300
Wire Wire Line
	6100 4300 5950 4300
Wire Wire Line
	6550 4700 6550 4650
Wire Wire Line
	7200 3500 5950 3500
$Comp
L PWR_FLAG #FLG040
U 1 1 5637C41B
P 4550 4050
F 0 "#FLG040" H 4550 4145 50  0001 C CNN
F 1 "PWR_FLAG" H 4550 4230 50  0000 C CNN
F 2 "" H 4550 4050 60  0000 C CNN
F 3 "" H 4550 4050 60  0000 C CNN
	1    4550 4050
	1    0    0    -1
$EndComp
$Comp
L CONN_01X01 P1
U 1 1 5654FA4D
P 8150 4100
F 0 "P1" H 8150 4200 50  0000 C CNN
F 1 "3V3" V 8250 4100 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8150 4100 60  0001 C CNN
F 3 "" H 8150 4100 60  0000 C CNN
	1    8150 4100
	0    -1   -1   0
$EndComp
$Comp
L D_Schottky_x2_KCom_AAK D4
U 1 1 5654FAE8
P 7200 3300
F 0 "D4" H 7250 3200 50  0000 C CNN
F 1 "D_Schottky_x2_KCom_AAK" H 7200 3400 50  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 7200 3300 60  0001 C CNN
F 3 "" H 7200 3300 60  0000 C CNN
	1    7200 3300
	1    0    0    -1
$EndComp
$Comp
L R_Small R12
U 1 1 565552F2
P 7950 4700
F 0 "R12" H 7980 4720 50  0000 L CNN
F 1 "1k" H 7980 4660 50  0000 L CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" H 7950 4700 60  0001 C CNN
F 3 "" H 7950 4700 60  0000 C CNN
	1    7950 4700
	0    1    1    0
$EndComp
$Comp
L LED D5
U 1 1 5655531F
P 8300 4700
F 0 "D5" H 8300 4800 50  0000 C CNN
F 1 "Red" H 8300 4600 50  0000 C CNN
F 2 "LEDs:LED-0603" H 8300 4700 60  0001 C CNN
F 3 "" H 8300 4700 60  0000 C CNN
	1    8300 4700
	1    0    0    -1
$EndComp
Connection ~ 7750 4700
Wire Wire Line
	8050 4700 8100 4700
Wire Wire Line
	8500 4300 8500 4700
Connection ~ 8150 4300
$Comp
L CONN_01X01 P9
U 1 1 56555904
P 5950 3300
F 0 "P9" H 5950 3400 50  0000 C CNN
F 1 "5V" V 6050 3300 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 5950 3300 60  0001 C CNN
F 3 "" H 5950 3300 60  0000 C CNN
	1    5950 3300
	0    -1   -1   0
$EndComp
Wire Wire Line
	3350 2750 3550 2750
Text HLabel 3850 2750 1    60   Input ~ 0
CURRENT
$Comp
L CONN_01X01 P8
U 1 1 56DE4DD7
P 5100 4400
F 0 "P8" H 5100 4500 50  0000 C CNN
F 1 "CONN_01X01" V 5200 4400 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Square-SMD-Pad_Small" H 5100 4400 50  0001 C CNN
F 3 "" H 5100 4400 50  0000 C CNN
	1    5100 4400
	0    -1   -1   0
$EndComp
Wire Wire Line
	5100 4600 5100 4700
Connection ~ 5100 4700
Text Notes 7000 7050 0    60   ~ 0
Copyright 2016 Chickadee Tech LLC, <add attributions here>\n\nThis work is licensed under the Creative Commons Attribution 4.0 International License.\nTo view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/\nor send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
$EndSCHEMATC
