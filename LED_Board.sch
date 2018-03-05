EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
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
LIBS:TJS
LIBS:LED_Board-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "LED Board - Hot IRmadillo Lid"
Date "2018-03-05"
Rev ""
Comp "KEIT Ltd. "
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L LED D1
U 1 1 5A8C452A
P 5650 2500
F 0 "D1" H 5650 2600 50  0000 C CNN
F 1 "LED" H 5650 2400 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 5650 2500 50  0001 C CNN
F 3 "" H 5650 2500 50  0001 C CNN
	1    5650 2500
	0    -1   -1   0   
$EndComp
$Comp
L LED D2
U 1 1 5A8C4691
P 6150 2500
F 0 "D2" H 6150 2600 50  0000 C CNN
F 1 "LED" H 6150 2400 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 6150 2500 50  0001 C CNN
F 3 "" H 6150 2500 50  0001 C CNN
	1    6150 2500
	0    -1   -1   0   
$EndComp
$Comp
L LED D3
U 1 1 5A8C46CE
P 6750 2500
F 0 "D3" H 6750 2600 50  0000 C CNN
F 1 "LED" H 6750 2400 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 6750 2500 50  0001 C CNN
F 3 "" H 6750 2500 50  0001 C CNN
	1    6750 2500
	0    -1   -1   0   
$EndComp
$Comp
L LED D4
U 1 1 5A8C4703
P 7350 2500
F 0 "D4" H 7350 2600 50  0000 C CNN
F 1 "LED" H 7350 2400 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 7350 2500 50  0001 C CNN
F 3 "" H 7350 2500 50  0001 C CNN
	1    7350 2500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5650 2750 5650 2650
Wire Wire Line
	4850 2950 6950 2950
$Comp
L LED D5
U 1 1 5A8C47F2
P 7750 2500
F 0 "D5" H 7750 2600 50  0000 C CNN
F 1 "LED" H 7750 2400 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 7750 2500 50  0001 C CNN
F 3 "" H 7750 2500 50  0001 C CNN
	1    7750 2500
	0    -1   -1   0   
$EndComp
$Comp
L LED D6
U 1 1 5A8C481A
P 8300 2500
F 0 "D6" H 8300 2600 50  0000 C CNN
F 1 "LED" H 8300 2400 50  0000 C CNN
F 2 "LEDs:LED_1206_HandSoldering" H 8300 2500 50  0001 C CNN
F 3 "" H 8300 2500 50  0001 C CNN
	1    8300 2500
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 5A8C496F
P 5650 2050
F 0 "R3" V 5730 2050 50  0000 C CNN
F 1 "R" V 5650 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5580 2050 50  0001 C CNN
F 3 "" H 5650 2050 50  0001 C CNN
	1    5650 2050
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5A8C49FC
P 6150 2050
F 0 "R4" V 6230 2050 50  0000 C CNN
F 1 "R" V 6150 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6080 2050 50  0001 C CNN
F 3 "" H 6150 2050 50  0001 C CNN
	1    6150 2050
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 5A8C4A35
P 6750 2050
F 0 "R5" V 6830 2050 50  0000 C CNN
F 1 "R" V 6750 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6680 2050 50  0001 C CNN
F 3 "" H 6750 2050 50  0001 C CNN
	1    6750 2050
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 5A8C4A77
P 7350 2050
F 0 "R6" V 7430 2050 50  0000 C CNN
F 1 "R" V 7350 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7280 2050 50  0001 C CNN
F 3 "" H 7350 2050 50  0001 C CNN
	1    7350 2050
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5A8C4AC1
P 7750 2050
F 0 "R7" V 7830 2050 50  0000 C CNN
F 1 "R" V 7750 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7680 2050 50  0001 C CNN
F 3 "" H 7750 2050 50  0001 C CNN
	1    7750 2050
	1    0    0    -1  
$EndComp
$Comp
L R R9
U 1 1 5A8C4AFD
P 8300 2050
F 0 "R9" V 8380 2050 50  0000 C CNN
F 1 "R" V 8300 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 8230 2050 50  0001 C CNN
F 3 "" H 8300 2050 50  0001 C CNN
	1    8300 2050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 5A8C4C16
P 3300 3700
F 0 "#PWR01" H 3300 3450 50  0001 C CNN
F 1 "GND" H 3300 3550 50  0000 C CNN
F 2 "" H 3300 3700 50  0001 C CNN
F 3 "" H 3300 3700 50  0001 C CNN
	1    3300 3700
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR02
U 1 1 5A8C4CAC
P 2400 2150
F 0 "#PWR02" H 2400 2000 50  0001 C CNN
F 1 "VDD" H 2400 2300 50  0000 C CNN
F 2 "" H 2400 2150 50  0001 C CNN
F 3 "" H 2400 2150 50  0001 C CNN
	1    2400 2150
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5A8C4CE5
P 2800 2750
F 0 "R2" V 2880 2750 50  0000 C CNN
F 1 "10k" V 2800 2750 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2730 2750 50  0001 C CNN
F 3 "" H 2800 2750 50  0001 C CNN
	1    2800 2750
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 5A8C4D6B
P 2550 2750
F 0 "R1" V 2630 2750 50  0000 C CNN
F 1 "10k" V 2550 2750 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2480 2750 50  0001 C CNN
F 3 "" H 2550 2750 50  0001 C CNN
	1    2550 2750
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x06 J1
U 1 1 5A8C4E77
P 1900 6200
F 0 "J1" H 1900 6500 50  0000 C CNN
F 1 "Conn_01x06" H 1900 5800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x06_Pitch2.54mm" H 1900 6200 50  0001 C CNN
F 3 "" H 1900 6200 50  0001 C CNN
	1    1900 6200
	-1   0    0    1   
$EndComp
Wire Wire Line
	2400 2350 3500 2350
Wire Wire Line
	2400 2350 2400 2150
Wire Wire Line
	2550 2600 2550 2350
Wire Wire Line
	2550 2350 2600 2350
Connection ~ 2600 2350
Wire Wire Line
	2800 2600 2800 2350
Connection ~ 2800 2350
Wire Wire Line
	3500 3050 2800 3050
Wire Wire Line
	2800 3050 2800 2900
Wire Wire Line
	3500 3150 2550 3150
Wire Wire Line
	2550 3150 2550 2900
Text Label 2100 6400 0    60   ~ 0
GND
Text Label 2100 6300 0    60   ~ 0
~RESET
Text Label 2100 6200 0    60   ~ 0
SDA
Text Label 2100 6100 0    60   ~ 0
SCL
$Comp
L VDD #PWR03
U 1 1 5A8C59DD
P 2750 5850
F 0 "#PWR03" H 2750 5700 50  0001 C CNN
F 1 "VDD" H 2750 6000 50  0000 C CNN
F 2 "" H 2750 5850 50  0001 C CNN
F 3 "" H 2750 5850 50  0001 C CNN
	1    2750 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2750 5850 2750 5900
Wire Wire Line
	2750 5900 2100 5900
$Comp
L VDD #PWR04
U 1 1 5A8C5B62
P 6550 1350
F 0 "#PWR04" H 6550 1200 50  0001 C CNN
F 1 "VDD" H 6550 1500 50  0000 C CNN
F 2 "" H 6550 1350 50  0001 C CNN
F 3 "" H 6550 1350 50  0001 C CNN
	1    6550 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 1700 6550 1350
Wire Wire Line
	5650 1700 5650 1900
Wire Wire Line
	8300 1700 8300 1900
Connection ~ 6550 1700
Wire Wire Line
	7750 1700 7750 1900
Wire Wire Line
	7350 1700 7350 1900
Wire Wire Line
	5650 2200 5650 2350
Wire Wire Line
	6150 2350 6150 2200
Wire Wire Line
	6750 2200 6750 2350
Wire Wire Line
	7350 2350 7350 2200
Wire Wire Line
	7750 2200 7750 2350
Wire Wire Line
	8300 2350 8300 2200
Wire Wire Line
	3500 3450 3300 3450
Wire Wire Line
	3300 3450 3300 3700
$Comp
L SW_Push SW1
U 1 1 5A8C64D1
P 7200 4950
F 0 "SW1" H 7250 5050 50  0000 L CNN
F 1 "SW_Push" H 7200 4890 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_TH_Tactile_Omron_B3F-10xx" H 7200 5150 50  0001 C CNN
F 3 "" H 7200 5150 50  0001 C CNN
	1    7200 4950
	0    1    1    0   
$EndComp
$Comp
L VDD #PWR05
U 1 1 5A8C6705
P 7200 4150
F 0 "#PWR05" H 7200 4000 50  0001 C CNN
F 1 "VDD" H 7200 4300 50  0000 C CNN
F 2 "" H 7200 4150 50  0001 C CNN
F 3 "" H 7200 4150 50  0001 C CNN
	1    7200 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 5A8C6746
P 7200 5500
F 0 "#PWR06" H 7200 5250 50  0001 C CNN
F 1 "GND" H 7200 5350 50  0000 C CNN
F 2 "" H 7200 5500 50  0001 C CNN
F 3 "" H 7200 5500 50  0001 C CNN
	1    7200 5500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5A8C6805
P 3900 6200
F 0 "#PWR07" H 3900 5950 50  0001 C CNN
F 1 "GND" H 3900 6050 50  0000 C CNN
F 2 "" H 3900 6200 50  0001 C CNN
F 3 "" H 3900 6200 50  0001 C CNN
	1    3900 6200
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x02 J2
U 1 1 5A8C6138
P 3650 6150
F 0 "J2" H 3650 6250 50  0000 C CNN
F 1 "Conn_01x02" H 3650 5950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x02_Pitch2.54mm" H 3650 6150 50  0001 C CNN
F 3 "" H 3650 6150 50  0001 C CNN
	1    3650 6150
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 6150 3900 6150
Wire Wire Line
	3900 6150 3900 6200
Text Label 4850 3350 0    60   ~ 0
BEEP
Text Label 3850 6050 0    60   ~ 0
BEEP
$Comp
L R R8
U 1 1 5A8C6D1B
P 7200 4500
F 0 "R8" V 7280 4500 50  0000 C CNN
F 1 "10k" V 7200 4500 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7130 4500 50  0001 C CNN
F 3 "" H 7200 4500 50  0001 C CNN
	1    7200 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 4750 7200 4650
Wire Wire Line
	7200 4350 7200 4150
Wire Wire Line
	7200 5150 7200 5500
Wire Wire Line
	6550 4750 7200 4750
Text Label 6550 4750 0    60   ~ 0
SWITCH
Text Label 4850 3450 0    60   ~ 0
SWITCH
$Comp
L GND #PWR08
U 1 1 5A8C80B3
P 5200 2550
F 0 "#PWR08" H 5200 2300 50  0001 C CNN
F 1 "GND" H 5200 2400 50  0000 C CNN
F 2 "" H 5200 2550 50  0001 C CNN
F 3 "" H 5200 2550 50  0001 C CNN
	1    5200 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2550 5200 2550
Connection ~ 4850 2450
Text Label 3500 2700 0    60   ~ 0
~RESET
Text Label 2900 3050 0    60   ~ 0
SDA
Text Label 2750 3150 0    60   ~ 0
SCL
$Comp
L GND #PWR09
U 1 1 5A8D6850
P 2200 6500
F 0 "#PWR09" H 2200 6250 50  0001 C CNN
F 1 "GND" H 2200 6350 50  0000 C CNN
F 2 "" H 2200 6500 50  0001 C CNN
F 3 "" H 2200 6500 50  0001 C CNN
	1    2200 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 6400 2200 6400
Wire Wire Line
	2200 6400 2200 6500
$Comp
L R R10
U 1 1 5A8D796E
P 5850 2050
F 0 "R10" V 5930 2050 50  0000 C CNN
F 1 "100k" V 5850 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 5780 2050 50  0001 C CNN
F 3 "" H 5850 2050 50  0001 C CNN
	1    5850 2050
	1    0    0    -1  
$EndComp
$Comp
L R R11
U 1 1 5A8D79FA
P 6350 2050
F 0 "R11" V 6430 2050 50  0000 C CNN
F 1 "100k" V 6350 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6280 2050 50  0001 C CNN
F 3 "" H 6350 2050 50  0001 C CNN
	1    6350 2050
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 5A8D7A7C
P 6950 2050
F 0 "R12" V 7030 2050 50  0000 C CNN
F 1 "100k" V 6950 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 6880 2050 50  0001 C CNN
F 3 "" H 6950 2050 50  0001 C CNN
	1    6950 2050
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 5A8D7AEE
P 7500 2050
F 0 "R13" V 7580 2050 50  0000 C CNN
F 1 "100k" V 7500 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7430 2050 50  0001 C CNN
F 3 "" H 7500 2050 50  0001 C CNN
	1    7500 2050
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 5A8D7B64
P 7900 2050
F 0 "R14" V 7980 2050 50  0000 C CNN
F 1 "100k" V 7900 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 7830 2050 50  0001 C CNN
F 3 "" H 7900 2050 50  0001 C CNN
	1    7900 2050
	1    0    0    -1  
$EndComp
$Comp
L R R15
U 1 1 5A8D7C0A
P 8500 2050
F 0 "R15" V 8580 2050 50  0000 C CNN
F 1 "100k" V 8500 2050 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 8430 2050 50  0001 C CNN
F 3 "" H 8500 2050 50  0001 C CNN
	1    8500 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 1700 8500 1700
Wire Wire Line
	5850 1900 5850 1700
Connection ~ 5850 1700
Wire Wire Line
	5850 2750 5850 2200
Wire Wire Line
	6350 2850 6350 2200
Wire Wire Line
	6350 1900 6350 1700
Connection ~ 6350 1700
Wire Wire Line
	6150 1900 6150 1700
Wire Wire Line
	6150 1700 6200 1700
Connection ~ 6200 1700
Wire Wire Line
	6750 2950 6750 2650
Wire Wire Line
	6750 1700 6750 1900
Wire Wire Line
	6950 1700 6950 1900
Connection ~ 6750 1700
Connection ~ 6950 1700
Wire Wire Line
	7500 1700 7500 1900
Connection ~ 7350 1700
Connection ~ 7500 1700
Wire Wire Line
	7900 1700 7900 1900
Connection ~ 7750 1700
Connection ~ 7900 1700
Wire Wire Line
	8500 1700 8500 1900
Connection ~ 8300 1700
Wire Wire Line
	6950 2950 6950 2200
Connection ~ 6750 2950
Wire Wire Line
	7350 2650 7350 3050
Wire Wire Line
	4850 3050 7500 3050
Wire Wire Line
	7500 3050 7500 2200
Connection ~ 7350 3050
Wire Wire Line
	7750 2650 7750 3150
Wire Wire Line
	4850 3150 7900 3150
Wire Wire Line
	7900 3150 7900 2200
Connection ~ 7750 3150
Wire Wire Line
	8300 2650 8300 3250
Wire Wire Line
	4850 3250 8500 3250
Wire Wire Line
	8500 3250 8500 2200
Connection ~ 8300 3250
$Comp
L PCA9531PW_LED_Driver_i2c U1
U 1 1 5A8C44CB
P 4200 2250
F 0 "U1" H 4200 2350 60  0000 C CNN
F 1 "PCA9531PW_LED_Driver_i2c" H 4200 850 60  0000 C CNN
F 2 "Housings_SSOP:TSSOP-16_4.4x5mm_Pitch0.65mm" H 4200 2350 60  0001 C CNN
F 3 "" H 4200 2350 60  0001 C CNN
	1    4200 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2450 4850 2550
Text Label 4850 2350 0    60   ~ 0
A0
Text Label 2100 6000 0    60   ~ 0
A0
Wire Wire Line
	4850 2750 5850 2750
Connection ~ 5650 2750
Wire Wire Line
	6150 2650 6150 2850
Wire Wire Line
	4850 2850 6350 2850
Connection ~ 6150 2850
$Comp
L R R16
U 1 1 5A8D9047
P 3250 2550
F 0 "R16" V 3330 2550 50  0000 C CNN
F 1 "10k" V 3250 2550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3180 2550 50  0001 C CNN
F 3 "" H 3250 2550 50  0001 C CNN
	1    3250 2550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 2400 3250 2350
Connection ~ 3250 2350
Wire Wire Line
	3500 2700 3250 2700
$EndSCHEMATC
