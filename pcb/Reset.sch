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
LIBS:ckd_sandwich
LIBS:stm32
LIBS:stm-ldo
LIBS:ti-regu
LIBS:ti-power-mux
LIBS:invensense
LIBS:FlightController-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
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
L SW_PUSH SW1
U 1 1 5615B5FF
P 5100 3900
F 0 "SW1" H 5250 4010 50  0000 C CNN
F 1 "SW_PUSH" H 5100 3820 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVQP7A" H 5100 3900 60  0001 C CNN
F 3 "" H 5100 3900 60  0000 C CNN
	1    5100 3900
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR041
U 1 1 5615B692
P 4800 4550
F 0 "#PWR041" H 4800 4350 50  0001 C CNN
F 1 "GNDPWR" H 4800 4420 50  0000 C CNN
F 2 "" H 4800 4500 60  0000 C CNN
F 3 "" H 4800 4500 60  0000 C CNN
	1    4800 4550
	1    0    0    -1  
$EndComp
Text HLabel 5400 4550 3    60   Input ~ 0
RESET
$Comp
L C C15
U 1 1 56161A55
P 6950 3800
F 0 "C15" H 6975 3900 50  0000 L CNN
F 1 "2.2uF" H 6975 3700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 6988 3650 30  0001 C CNN
F 3 "" H 6950 3800 60  0000 C CNN
	1    6950 3800
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 56161A89
P 6000 3250
F 0 "R9" V 6080 3250 50  0000 C CNN
F 1 "470k" V 6000 3250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5930 3250 30  0001 C CNN
F 3 "" H 6000 3250 30  0000 C CNN
	1    6000 3250
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 56161B4F
P 6450 3550
F 0 "R10" V 6530 3550 50  0000 C CNN
F 1 "100k" V 6450 3550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 6380 3550 30  0001 C CNN
F 3 "" H 6450 3550 30  0000 C CNN
	1    6450 3550
	0    1    1    0   
$EndComp
Text HLabel 8700 3850 2    60   Input ~ 0
BOOT0
$Comp
L GNDPWR #PWR042
U 1 1 56161C45
P 6950 4400
F 0 "#PWR042" H 6950 4200 50  0001 C CNN
F 1 "GNDPWR" H 6950 4270 50  0000 C CNN
F 2 "" H 6950 4350 60  0000 C CNN
F 3 "" H 6950 4350 60  0000 C CNN
	1    6950 4400
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR043
U 1 1 56161C6E
P 6000 4400
F 0 "#PWR043" H 6000 4200 50  0001 C CNN
F 1 "GNDPWR" H 6000 4270 50  0000 C CNN
F 2 "" H 6000 4350 60  0000 C CNN
F 3 "" H 6000 4350 60  0000 C CNN
	1    6000 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3900 4800 4550
Wire Wire Line
	5400 3650 5400 4550
Wire Wire Line
	6600 3550 7300 3550
Wire Wire Line
	6950 3650 6950 3550
Connection ~ 6950 3550
Wire Wire Line
	6000 3400 6000 3700
Wire Wire Line
	6300 3550 6000 3550
Connection ~ 6000 3550
Wire Wire Line
	5400 3900 5700 3900
Wire Wire Line
	7600 2950 7600 3350
Wire Wire Line
	8550 3200 8550 3850
Wire Wire Line
	6950 3950 6950 4400
Wire Wire Line
	6000 4100 6000 4400
Text Notes 5500 4900 0    60   ~ 0
Internally Pulled High
Wire Wire Line
	8550 3850 8700 3850
Text Notes 8700 4000 0    60   ~ 0
Pulled Low
$Comp
L VDD #PWR044
U 1 1 563D51E3
P 7600 2650
F 0 "#PWR044" H 7600 2500 50  0001 C CNN
F 1 "VDD" H 7600 2800 50  0000 C CNN
F 2 "" H 7600 2650 60  0000 C CNN
F 3 "" H 7600 2650 60  0000 C CNN
	1    7600 2650
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR045
U 1 1 563D5246
P 6000 2950
F 0 "#PWR045" H 6000 2800 50  0001 C CNN
F 1 "VDD" H 6000 3100 50  0000 C CNN
F 2 "" H 6000 2950 60  0000 C CNN
F 3 "" H 6000 2950 60  0000 C CNN
	1    6000 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3100 6000 2950
$Comp
L Q_NMOS_GSD Q3
U 1 1 5654B19F
P 5900 3900
F 0 "Q3" H 6200 3950 50  0000 R CNN
F 1 "Q_NMOS_GSD" H 6550 3850 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 6100 4000 29  0001 C CNN
F 3 "" H 5900 3900 60  0000 C CNN
	1    5900 3900
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_GSD Q4
U 1 1 5654B200
P 7500 3550
F 0 "Q4" H 7800 3600 50  0000 R CNN
F 1 "Q_NMOS_GSD" H 8150 3500 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 7700 3650 29  0001 C CNN
F 3 "" H 7500 3550 60  0000 C CNN
	1    7500 3550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 P6
U 1 1 56554BA3
P 5400 3450
F 0 "P6" H 5400 3550 50  0000 C CNN
F 1 "RESET" V 5500 3450 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 5400 3450 60  0001 C CNN
F 3 "" H 5400 3450 60  0000 C CNN
	1    5400 3450
	0    -1   -1   0   
$EndComp
Connection ~ 5400 3900
$Comp
L CONN_01X01 P7
U 1 1 56554C77
P 9000 3500
F 0 "P7" H 9000 3600 50  0000 C CNN
F 1 "BOOT0" H 9200 3500 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 9000 3500 60  0001 C CNN
F 3 "" H 9000 3500 60  0000 C CNN
	1    9000 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8800 3500 8550 3500
Connection ~ 8550 3500
$Comp
L Q_PMOS_GSD Q1
U 1 1 56C79F95
P 8450 3000
F 0 "Q1" H 8750 3050 50  0000 R CNN
F 1 "Q_PMOS_GSD" H 9100 2950 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23" H 8650 3100 50  0001 C CNN
F 3 "" H 8450 3000 50  0000 C CNN
	1    8450 3000
	1    0    0    1   
$EndComp
$Comp
L VDD #PWR046
U 1 1 56C79FCA
P 8550 2800
F 0 "#PWR046" H 8550 2650 50  0001 C CNN
F 1 "VDD" H 8550 2950 50  0000 C CNN
F 2 "" H 8550 2800 60  0000 C CNN
F 3 "" H 8550 2800 60  0000 C CNN
	1    8550 2800
	1    0    0    -1  
$EndComp
$Comp
L GNDPWR #PWR047
U 1 1 56C7A03D
P 7600 3750
F 0 "#PWR047" H 7600 3550 50  0001 C CNN
F 1 "GNDPWR" H 7600 3620 50  0000 C CNN
F 2 "" H 7600 3700 60  0000 C CNN
F 3 "" H 7600 3700 60  0000 C CNN
	1    7600 3750
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 56C7A063
P 7600 2800
F 0 "R7" V 7680 2800 50  0000 C CNN
F 1 "10k" V 7600 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 7530 2800 30  0001 C CNN
F 3 "" H 7600 2800 30  0000 C CNN
	1    7600 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3000 7600 3000
Connection ~ 7600 3000
Text Notes 7050 7050 0    60   ~ 0
Copyright 2016 Chickadee Tech LLC, <add attributions here>\n\nThis work is licensed under the Creative Commons Attribution 4.0 International License.\nTo view a copy of this license, visit http://creativecommons.org/licenses/by/4.0/\nor send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
$EndSCHEMATC
