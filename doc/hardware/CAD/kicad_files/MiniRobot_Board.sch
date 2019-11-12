EESchema Schematic File Version 4
LIBS:MiniRobot_Board-cache
EELAYER 26 0
EELAYER END
$Descr User 11693 9843
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
L Connector:Conn_01x03_Female J5
U 1 1 5D1521A5
P 8850 1450
F 0 "J5" H 8878 1476 50  0000 L CNN
F 1 "QTR-1A_Conn" H 8878 1385 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 8850 1450 50  0001 C CNN
F 3 "~" H 8850 1450 50  0001 C CNN
	1    8850 1450
	1    0    0    1   
$EndComp
$Comp
L Device:R_Small R_qdiv1_1
U 1 1 5D152275
P 8200 1150
F 0 "R_qdiv1_1" H 7800 1200 50  0000 L CNN
F 1 "27K" H 7800 1100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8200 1150 50  0001 C CNN
F 3 "~" H 8200 1150 50  0001 C CNN
	1    8200 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R_qdiv2_1
U 1 1 5D1522C1
P 8200 1450
F 0 "R_qdiv2_1" H 7800 1500 50  0000 L CNN
F 1 "51K" H 7800 1400 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8200 1450 50  0001 C CNN
F 3 "~" H 8200 1450 50  0001 C CNN
	1    8200 1450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR024
U 1 1 5D152476
P 8400 1400
F 0 "#PWR024" H 8400 1250 50  0001 C CNN
F 1 "+5V" H 8415 1573 50  0000 C CNN
F 2 "" H 8400 1400 50  0001 C CNN
F 3 "" H 8400 1400 50  0001 C CNN
	1    8400 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR021
U 1 1 5D1524CE
P 8300 1750
F 0 "#PWR021" H 8300 1500 50  0001 C CNN
F 1 "GND" H 8450 1650 50  0000 C CNN
F 2 "" H 8300 1750 50  0001 C CNN
F 3 "" H 8300 1750 50  0001 C CNN
	1    8300 1750
	1    0    0    -1  
$EndComp
Text Label 7500 1300 0    50   ~ 0
qout_1
$Comp
L Connector:Conn_01x04_Female J7
U 1 1 5D15A905
P 8850 3350
F 0 "J7" H 8877 3326 50  0000 L CNN
F 1 "VL53L0X_Conn" H 8877 3235 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8850 3350 50  0001 C CNN
F 3 "~" H 8850 3350 50  0001 C CNN
	1    8850 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 3250 8100 3350
Wire Wire Line
	8650 3450 8400 3450
Wire Wire Line
	8650 3550 8400 3550
$Comp
L power:GND #PWR020
U 1 1 5D1635E9
P 8100 3350
F 0 "#PWR020" H 8100 3100 50  0001 C CNN
F 1 "GND" H 8105 3177 50  0000 C CNN
F 2 "" H 8100 3350 50  0001 C CNN
F 3 "" H 8100 3350 50  0001 C CNN
	1    8100 3350
	1    0    0    -1  
$EndComp
Text Label 8400 3450 0    50   ~ 0
SDA
Text Label 8400 3550 0    50   ~ 0
SCL
$Comp
L Connector:Conn_01x06_Female J11
U 1 1 5D178341
P 10400 5650
F 0 "J11" H 10428 5626 50  0000 L CNN
F 1 "DC_Mot_Conn" H 10428 5535 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 10400 5650 50  0001 C CNN
F 3 "~" H 10400 5650 50  0001 C CNN
	1    10400 5650
	1    0    0    1   
$EndComp
Wire Wire Line
	10200 5850 10050 5850
Wire Wire Line
	10050 5850 10050 5900
Wire Wire Line
	10300 7650 10150 7650
Wire Wire Line
	10150 7650 10150 7700
$Comp
L power:GND #PWR037
U 1 1 5D17C098
P 10050 5900
F 0 "#PWR037" H 10050 5650 50  0001 C CNN
F 1 "GND" H 10200 5850 50  0000 C CNN
F 2 "" H 10050 5900 50  0001 C CNN
F 3 "" H 10050 5900 50  0001 C CNN
	1    10050 5900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR038
U 1 1 5D17C0D2
P 10150 7700
F 0 "#PWR038" H 10150 7450 50  0001 C CNN
F 1 "GND" H 10300 7650 50  0000 C CNN
F 2 "" H 10150 7700 50  0001 C CNN
F 3 "" H 10150 7700 50  0001 C CNN
	1    10150 7700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 7550 9900 7550
Wire Wire Line
	9950 7450 10300 7450
Wire Wire Line
	10200 5750 9800 5750
Wire Wire Line
	10200 5650 9850 5650
Text Label 9800 5750 0    50   ~ 0
B1
Text Label 9850 5650 0    50   ~ 0
A1
Text Label 9900 7550 0    50   ~ 0
B2
Text Label 9950 7450 0    50   ~ 0
A2
Wire Wire Line
	10200 5550 9500 5550
Wire Wire Line
	9500 5550 9500 5450
Wire Wire Line
	5050 4350 5050 4150
Connection ~ 5050 4350
$Comp
L Device:R_Small R_SDA1
U 1 1 5D191A85
P 5050 4050
F 0 "R_SDA1" V 5250 4100 50  0000 L CNN
F 1 "2.7K" V 5150 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5050 4050 50  0001 C CNN
F 3 "~" H 5050 4050 50  0001 C CNN
	1    5050 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R_SCL1
U 1 1 5D191C28
P 5300 4050
F 0 "R_SCL1" V 5500 4100 50  0000 L CNN
F 1 "2.7K" V 5400 4150 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5300 4050 50  0001 C CNN
F 3 "~" H 5300 4050 50  0001 C CNN
	1    5300 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 4350 5350 4350
Wire Wire Line
	5300 4150 5300 4250
Connection ~ 5300 4250
Wire Wire Line
	5300 4250 5350 4250
Wire Wire Line
	5300 3950 5300 3850
Text Label 5350 4350 0    50   ~ 0
SDA
Text Label 5350 4250 0    50   ~ 0
SCL
Wire Wire Line
	5600 1600 5600 1500
$Comp
L power:PWR_FLAG #FLG03
U 1 1 5D1A7C53
P 5600 1700
F 0 "#FLG03" H 5600 1775 50  0001 C CNN
F 1 "PWR_FLAG" H 5600 1850 50  0001 C CNN
F 2 "" H 5600 1700 50  0001 C CNN
F 3 "~" H 5600 1700 50  0001 C CNN
	1    5600 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	5600 1600 5600 1700
Connection ~ 5600 1600
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5D1AC556
P 3800 1700
F 0 "#FLG01" H 3800 1775 50  0001 C CNN
F 1 "PWR_FLAG" H 3800 1873 50  0001 C CNN
F 2 "" H 3800 1700 50  0001 C CNN
F 3 "~" H 3800 1700 50  0001 C CNN
	1    3800 1700
	-1   0    0    1   
$EndComp
Wire Wire Line
	3800 1600 3800 1700
$Comp
L power:GND #PWR08
U 1 1 5D1B0D07
P 4600 2350
F 0 "#PWR08" H 4600 2100 50  0001 C CNN
F 1 "GND" H 4605 2177 50  0000 C CNN
F 2 "" H 4600 2350 50  0001 C CNN
F 3 "" H 4600 2350 50  0001 C CNN
	1    4600 2350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR017
U 1 1 5D1CD51C
P 5600 1500
F 0 "#PWR017" H 5600 1350 50  0001 C CNN
F 1 "+5V" H 5615 1673 50  0000 C CNN
F 2 "" H 5600 1500 50  0001 C CNN
F 3 "" H 5600 1500 50  0001 C CNN
	1    5600 1500
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR07
U 1 1 5D1CD5F2
P 3800 1500
F 0 "#PWR07" H 3800 1350 50  0001 C CNN
F 1 "+12V" H 3815 1673 50  0000 C CNN
F 2 "" H 3800 1500 50  0001 C CNN
F 3 "" H 3800 1500 50  0001 C CNN
	1    3800 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1500 3800 1600
Connection ~ 3800 1600
$Comp
L power:+5V #PWR09
U 1 1 5D1D71F6
P 4750 3500
F 0 "#PWR09" H 4750 3350 50  0001 C CNN
F 1 "+5V" H 4700 3650 50  0000 C CNN
F 2 "" H 4750 3500 50  0001 C CNN
F 3 "" H 4750 3500 50  0001 C CNN
	1    4750 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5D1E5F92
P 4750 3700
F 0 "#PWR010" H 4750 3450 50  0001 C CNN
F 1 "GND" H 4750 3800 50  0001 C CNN
F 2 "" H 4750 3700 50  0001 C CNN
F 3 "" H 4750 3700 50  0001 C CNN
	1    4750 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 4250 5300 4250
Wire Wire Line
	4300 4350 5050 4350
Wire Wire Line
	4750 3550 4750 3500
Wire Wire Line
	4300 3550 4750 3550
Wire Wire Line
	4750 3650 4750 3700
Wire Wire Line
	3250 3550 2450 3550
Wire Wire Line
	2450 3550 2450 3600
$Comp
L power:GND #PWR02
U 1 1 5D210595
P 2450 3600
F 0 "#PWR02" H 2450 3350 50  0001 C CNN
F 1 "GND" H 2455 3427 50  0000 C CNN
F 2 "" H 2450 3600 50  0001 C CNN
F 3 "" H 2450 3600 50  0001 C CNN
	1    2450 3600
	1    0    0    -1  
$EndComp
NoConn ~ 3600 5200
NoConn ~ 3900 5200
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 5D18353A
P 1500 1600
F 0 "J1" H 1606 1778 50  0000 C CNN
F 1 "Batt_Conn" H 1606 1687 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 1500 1600 50  0001 C CNN
F 3 "~" H 1500 1600 50  0001 C CNN
	1    1500 1600
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J3
U 1 1 5D16BC76
P 3800 6050
F 0 "J3" H 3850 6367 50  0000 C CNN
F 1 "ESP_expansion" H 3850 6276 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 3800 6050 50  0001 C CNN
F 3 "~" H 3800 6050 50  0001 C CNN
	1    3800 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5D17224D
P 3100 6350
F 0 "#PWR04" H 3100 6100 50  0001 C CNN
F 1 "GND" H 3105 6177 50  0000 C CNN
F 2 "" H 3100 6350 50  0001 C CNN
F 3 "" H 3100 6350 50  0001 C CNN
	1    3100 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 6250 4350 6250
Wire Wire Line
	3250 4150 2700 4150
Wire Wire Line
	3250 4250 2700 4250
Wire Wire Line
	3250 4550 2700 4550
Wire Wire Line
	3250 4650 2700 4650
Wire Wire Line
	3250 4750 2850 4750
Wire Wire Line
	3250 4850 2850 4850
Wire Wire Line
	3250 4350 2850 4350
Wire Wire Line
	3250 4450 2850 4450
Wire Wire Line
	2750 3650 3250 3650
Wire Wire Line
	2750 3750 3250 3750
Text Label 2750 3650 0    50   ~ 0
RX_teensy
Text Label 2750 3750 0    50   ~ 0
TX_teensy
Text Label 2850 4350 0    50   ~ 0
A1
Text Label 2850 4450 0    50   ~ 0
B1
Text Label 2850 4750 0    50   ~ 0
A2
Text Label 2850 4850 0    50   ~ 0
B2
Wire Wire Line
	4300 4650 4500 4650
Text Label 4500 4750 0    50   ~ 0
qout_2
Text Label 4500 4650 0    50   ~ 0
qout_1
NoConn ~ 4000 5200
Text Label 4350 6250 0    50   ~ 0
RX_teensy
Text Label 3250 5950 0    50   ~ 0
TX_teensy
Wire Wire Line
	3250 5950 3600 5950
$Comp
L Device:R_Small R_PU_CE2
U 1 1 5D1D00B7
P 4300 6100
F 0 "R_PU_CE2" V 4250 6100 50  0001 C CNN
F 1 "27K" V 4300 6100 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4300 6100 50  0001 C CNN
F 3 "~" H 4300 6100 50  0001 C CNN
	1    4300 6100
	0    1    1    0   
$EndComp
Wire Wire Line
	4300 4750 4500 4750
$Comp
L Connector:Conn_01x04_Male J9
U 1 1 5D22E4EA
P 10550 1500
F 0 "J9" H 10522 1380 50  0000 R CNN
F 1 "Servo_Conn" H 10522 1471 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10550 1500 50  0001 C CNN
F 3 "~" H 10550 1500 50  0001 C CNN
	1    10550 1500
	-1   0    0    1   
$EndComp
Wire Wire Line
	10350 1600 10100 1600
Wire Wire Line
	10100 1600 10100 1650
$Comp
L power:GND #PWR035
U 1 1 5D2445B2
P 10100 1650
F 0 "#PWR035" H 10100 1400 50  0001 C CNN
F 1 "GND" H 10105 1477 50  0000 C CNN
F 2 "" H 10100 1650 50  0001 C CNN
F 3 "" H 10100 1650 50  0001 C CNN
	1    10100 1650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR032
U 1 1 5D2446AF
P 9850 1450
F 0 "#PWR032" H 9850 1300 50  0001 C CNN
F 1 "+5V" H 9865 1623 50  0000 C CNN
F 2 "" H 9850 1450 50  0001 C CNN
F 3 "" H 9850 1450 50  0001 C CNN
	1    9850 1450
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 1300 10150 1300
Text Label 10150 1300 0    50   ~ 0
fb_s1
Wire Wire Line
	9850 1500 9850 1450
Wire Wire Line
	9850 1500 10350 1500
Wire Wire Line
	10350 1400 10150 1400
Text Label 10150 1400 0    50   ~ 0
sig_s1
$Comp
L Connector:Conn_01x04_Male J10
U 1 1 5D2724D4
P 10550 2400
F 0 "J10" H 10522 2280 50  0000 R CNN
F 1 "Servo_Conn" H 10522 2371 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10550 2400 50  0001 C CNN
F 3 "~" H 10550 2400 50  0001 C CNN
	1    10550 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	10350 2500 10100 2500
Wire Wire Line
	10100 2500 10100 2550
$Comp
L power:GND #PWR036
U 1 1 5D2724DD
P 10100 2550
F 0 "#PWR036" H 10100 2300 50  0001 C CNN
F 1 "GND" H 10105 2377 50  0000 C CNN
F 2 "" H 10100 2550 50  0001 C CNN
F 3 "" H 10100 2550 50  0001 C CNN
	1    10100 2550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR033
U 1 1 5D2724E3
P 9850 2350
F 0 "#PWR033" H 9850 2200 50  0001 C CNN
F 1 "+5V" H 9865 2523 50  0000 C CNN
F 2 "" H 9850 2350 50  0001 C CNN
F 3 "" H 9850 2350 50  0001 C CNN
	1    9850 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10350 2200 10150 2200
Text Label 10150 2200 0    50   ~ 0
fb_s2
Wire Wire Line
	9850 2400 9850 2350
Wire Wire Line
	9850 2400 10350 2400
Wire Wire Line
	10350 2300 10150 2300
Text Label 10150 2300 0    50   ~ 0
sig_s2
$Comp
L Connector:Conn_01x04_Male J8
U 1 1 5D279FB4
P 10500 3350
F 0 "J8" H 10472 3230 50  0000 R CNN
F 1 "Servo_Conn" H 10472 3321 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 10500 3350 50  0001 C CNN
F 3 "~" H 10500 3350 50  0001 C CNN
	1    10500 3350
	-1   0    0    1   
$EndComp
Wire Wire Line
	10300 3450 10050 3450
Wire Wire Line
	10050 3450 10050 3500
$Comp
L power:GND #PWR034
U 1 1 5D279FBD
P 10050 3500
F 0 "#PWR034" H 10050 3250 50  0001 C CNN
F 1 "GND" H 10055 3327 50  0000 C CNN
F 2 "" H 10050 3500 50  0001 C CNN
F 3 "" H 10050 3500 50  0001 C CNN
	1    10050 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR031
U 1 1 5D279FC3
P 9800 3300
F 0 "#PWR031" H 9800 3150 50  0001 C CNN
F 1 "+5V" H 9815 3473 50  0000 C CNN
F 2 "" H 9800 3300 50  0001 C CNN
F 3 "" H 9800 3300 50  0001 C CNN
	1    9800 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	10300 3150 10100 3150
Text Label 10100 3150 0    50   ~ 0
fb_s3
Wire Wire Line
	9800 3350 9800 3300
Wire Wire Line
	9800 3350 10300 3350
Wire Wire Line
	10300 3250 10100 3250
Text Label 10100 3250 0    50   ~ 0
sig_s3
Wire Wire Line
	4300 3850 4350 3850
Wire Wire Line
	4300 3950 4350 3950
Wire Wire Line
	4350 4050 4300 4050
Text Label 4350 3850 0    50   ~ 0
sig_s1
Text Label 4350 3950 0    50   ~ 0
sig_s2
Text Label 4350 4050 0    50   ~ 0
sig_s3
Wire Wire Line
	4300 4150 4400 4150
Wire Wire Line
	4300 4450 4400 4450
Wire Wire Line
	4300 4550 4400 4550
Text Label 4400 4150 0    50   ~ 0
fb_s1
Text Label 4400 4450 0    50   ~ 0
fb_s2
Text Label 4400 4550 0    50   ~ 0
fb_s3
Wire Wire Line
	5150 1600 5150 1700
Connection ~ 5150 1600
Wire Wire Line
	5150 1600 5600 1600
$Comp
L Device:R_Small R_pwr1
U 1 1 5D3332A2
P 5150 1800
F 0 "R_pwr1" H 5209 1846 50  0000 L CNN
F 1 "5.1K" H 5209 1755 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5150 1800 50  0001 C CNN
F 3 "~" H 5150 1800 50  0001 C CNN
	1    5150 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:LED_Small D_pwr1
U 1 1 5D3453B2
P 5150 2050
F 0 "D_pwr1" V 5150 1982 50  0000 R CNN
F 1 "LED_Small" V 5105 1982 50  0001 R CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 5150 2050 50  0001 C CNN
F 3 "~" V 5150 2050 50  0001 C CNN
	1    5150 2050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5150 1900 5150 1950
Wire Wire Line
	4600 1900 4600 2200
Wire Wire Line
	5150 2150 5150 2200
Wire Wire Line
	5150 2200 4600 2200
Connection ~ 4600 2200
Wire Wire Line
	4600 2200 4600 2350
NoConn ~ 4300 4850
$Comp
L Device:R_Small R_PU_CE1
U 1 1 5D180207
P 3300 6050
F 0 "R_PU_CE1" V 3250 6050 50  0001 C CNN
F 1 "27K" V 3300 6050 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3300 6050 50  0001 C CNN
F 3 "~" H 3300 6050 50  0001 C CNN
	1    3300 6050
	0    1    1    0   
$EndComp
Wire Wire Line
	3200 6050 3100 6050
Wire Wire Line
	3100 6050 3100 6000
$Comp
L power:+3V3 #PWR06
U 1 1 5D193EE0
P 3100 6000
F 0 "#PWR06" H 3100 5850 50  0001 C CNN
F 1 "+3V3" H 2950 6100 50  0000 C CNN
F 2 "" H 3100 6000 50  0001 C CNN
F 3 "" H 3100 6000 50  0001 C CNN
	1    3100 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GSD Q2
U 1 1 5D1A97AF
P 3000 2100
F 0 "Q2" V 3250 2100 50  0000 C CNN
F 1 "SQ2318AES - Vishay" V 3341 2100 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 3200 2200 50  0001 C CNN
F 3 "~" H 3000 2100 50  0001 C CNN
	1    3000 2100
	0    -1   1    0   
$EndComp
Wire Wire Line
	2500 1600 3000 1600
$Comp
L Device:R_Small R_prot1
U 1 1 5D208516
P 3000 1750
F 0 "R_prot1" H 2700 1800 50  0000 L CNN
F 1 "15K" H 2800 1700 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3000 1750 50  0001 C CNN
F 3 "~" H 3000 1750 50  0001 C CNN
	1    3000 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1900 3000 1850
Wire Wire Line
	3000 1650 3000 1600
Wire Wire Line
	3000 1600 3800 1600
Connection ~ 3000 1600
Wire Wire Line
	3300 2200 4600 2200
Wire Wire Line
	3200 2200 3300 2200
Connection ~ 3300 2200
Wire Wire Line
	3300 2100 3300 2200
$Comp
L Device:D_Zener_Small D_prot1
U 1 1 5D208789
P 3300 2000
F 0 "D_prot1" V 3254 2068 50  0000 L CNN
F 1 "BZX84 - 12V" V 3345 2068 50  0000 L CNN
F 2 "Diode_SMD:D_SOT-23_ANK" V 3300 2000 50  0001 C CNN
F 3 "~" V 3300 2000 50  0001 C CNN
	1    3300 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 1850 3300 1900
Connection ~ 3000 1850
Wire Wire Line
	3250 3850 2150 3850
$Comp
L Device:R_Small R1
U 1 1 5D299F93
P 2050 3850
F 0 "R1" V 1854 3850 50  0000 C CNN
F 1 "1K" V 1945 3850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2050 3850 50  0001 C CNN
F 3 "~" H 2050 3850 50  0001 C CNN
	1    2050 3850
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 3850 1850 3850
$Comp
L Device:LED D1
U 1 1 5D2A5705
P 1700 3850
F 0 "D1" H 1691 4066 50  0000 C CNN
F 1 "LED" H 1691 3975 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 1700 3850 50  0001 C CNN
F 3 "~" H 1700 3850 50  0001 C CNN
	1    1700 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1550 3850 1300 3850
Wire Wire Line
	1300 3850 1300 3900
$Comp
L power:GND #PWR01
U 1 1 5D2B15BE
P 1300 3900
F 0 "#PWR01" H 1300 3650 50  0001 C CNN
F 1 "GND" H 1305 3727 50  0000 C CNN
F 2 "" H 1300 3900 50  0001 C CNN
F 3 "" H 1300 3900 50  0001 C CNN
	1    1300 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 1000 8200 1050
Wire Wire Line
	8200 1250 8200 1300
Wire Wire Line
	8400 1450 8400 1400
Wire Wire Line
	8400 1450 8650 1450
Wire Wire Line
	8600 1350 8650 1350
Wire Wire Line
	8200 1300 7500 1300
Connection ~ 8200 1300
Wire Wire Line
	8200 1300 8200 1350
Wire Wire Line
	8650 1550 8400 1550
Wire Wire Line
	8200 1000 8600 1000
Wire Wire Line
	8600 1000 8600 1350
Wire Wire Line
	8200 1750 8300 1750
Wire Wire Line
	8400 1550 8400 1750
Wire Wire Line
	8200 1550 8200 1750
Connection ~ 8300 1750
Wire Wire Line
	8300 1750 8400 1750
$Comp
L Connector:Conn_01x03_Female J6
U 1 1 5D24D2D3
P 8850 2450
F 0 "J6" H 8878 2476 50  0000 L CNN
F 1 "QTR-1A_Conn" H 8878 2385 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 8850 2450 50  0001 C CNN
F 3 "~" H 8850 2450 50  0001 C CNN
	1    8850 2450
	1    0    0    1   
$EndComp
$Comp
L Device:R_Small R_qdiv1_2
U 1 1 5D24D2DA
P 8200 2150
F 0 "R_qdiv1_2" H 7800 2200 50  0000 L CNN
F 1 "27K" H 7800 2100 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8200 2150 50  0001 C CNN
F 3 "~" H 8200 2150 50  0001 C CNN
	1    8200 2150
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R_qdiv2_2
U 1 1 5D24D2E1
P 8200 2450
F 0 "R_qdiv2_2" H 7800 2500 50  0000 L CNN
F 1 "51K" H 7800 2400 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8200 2450 50  0001 C CNN
F 3 "~" H 8200 2450 50  0001 C CNN
	1    8200 2450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR025
U 1 1 5D24D2E8
P 8400 2400
F 0 "#PWR025" H 8400 2250 50  0001 C CNN
F 1 "+5V" H 8415 2573 50  0000 C CNN
F 2 "" H 8400 2400 50  0001 C CNN
F 3 "" H 8400 2400 50  0001 C CNN
	1    8400 2400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5D24D2EE
P 8300 2750
F 0 "#PWR022" H 8300 2500 50  0001 C CNN
F 1 "GND" H 8450 2650 50  0000 C CNN
F 2 "" H 8300 2750 50  0001 C CNN
F 3 "" H 8300 2750 50  0001 C CNN
	1    8300 2750
	1    0    0    -1  
$EndComp
Text Label 7500 2300 0    50   ~ 0
qout_2
Wire Wire Line
	8200 2000 8200 2050
Wire Wire Line
	8200 2250 8200 2300
Wire Wire Line
	8400 2450 8400 2400
Wire Wire Line
	8400 2450 8650 2450
Wire Wire Line
	8600 2350 8650 2350
Wire Wire Line
	8200 2300 7500 2300
Connection ~ 8200 2300
Wire Wire Line
	8200 2300 8200 2350
Wire Wire Line
	8650 2550 8400 2550
Wire Wire Line
	8200 2000 8600 2000
Wire Wire Line
	8600 2000 8600 2350
Wire Wire Line
	8200 2750 8300 2750
Wire Wire Line
	8400 2550 8400 2750
Wire Wire Line
	8200 2550 8200 2750
Connection ~ 8300 2750
Wire Wire Line
	8300 2750 8400 2750
Wire Wire Line
	1700 1500 2100 1500
NoConn ~ 3700 5200
NoConn ~ 3800 5200
$Comp
L Device:D_Small D2
U 1 1 5D3AF6B7
P 2800 7450
F 0 "D2" V 2800 7550 50  0000 L CNN
F 1 "1N4001" V 2845 7518 50  0001 L CNN
F 2 "Diode_SMD:D_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 2800 7450 50  0001 C CNN
F 3 "~" V 2800 7450 50  0001 C CNN
	1    2800 7450
	0    1    1    0   
$EndComp
$Comp
L Device:Q_NMOS_GSD Q1
U 1 1 5D3B1A9C
P 2750 8000
F 0 "Q1" V 3000 8000 50  0000 C CNN
F 1 "SQ2318AES - Vishay" V 3091 8000 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 2950 8100 50  0001 C CNN
F 3 "~" H 2750 8000 50  0001 C CNN
	1    2750 8000
	-1   0    0    -1  
$EndComp
Wire Wire Line
	2950 8000 3050 8000
Connection ~ 3050 8000
Wire Wire Line
	3050 8000 3200 8000
$Comp
L Device:R_Small R2
U 1 1 5D3D4201
P 3050 8250
F 0 "R2" H 3109 8296 50  0000 L CNN
F 1 "51K" H 3109 8205 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3050 8250 50  0001 C CNN
F 3 "~" H 3050 8250 50  0001 C CNN
	1    3050 8250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 8500 2850 8500
Wire Wire Line
	2650 8500 2650 8200
Wire Wire Line
	2850 8500 2850 8600
Connection ~ 2850 8500
Wire Wire Line
	2850 8500 2650 8500
$Comp
L power:GND #PWR05
U 1 1 5D3EAFAB
P 2850 8600
F 0 "#PWR05" H 2850 8350 50  0001 C CNN
F 1 "GND" H 2855 8427 50  0000 C CNN
F 2 "" H 2850 8600 50  0001 C CNN
F 3 "" H 2850 8600 50  0001 C CNN
	1    2850 8600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 8000 3050 8150
Wire Wire Line
	3050 8350 3050 8500
Text Label 2500 4050 0    50   ~ 0
sig_mos1
Wire Wire Line
	2650 7800 2650 7650
Wire Wire Line
	2650 7500 2300 7500
$Comp
L Connector:Conn_01x02_Female J2
U 1 1 5D41A75E
P 2100 7500
F 0 "J2" H 1994 7175 50  0000 C CNN
F 1 "MOS_conn" H 1994 7266 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B02B-EH-A_1x02_P2.50mm_Vertical" H 2100 7500 50  0001 C CNN
F 3 "~" H 2100 7500 50  0001 C CNN
	1    2100 7500
	-1   0    0    1   
$EndComp
Wire Wire Line
	2300 7400 2650 7400
Wire Wire Line
	2650 7400 2650 7250
$Comp
L power:+12V #PWR03
U 1 1 5D4275D8
P 2650 7100
F 0 "#PWR03" H 2650 6950 50  0001 C CNN
F 1 "+12V" H 2665 7273 50  0000 C CNN
F 2 "" H 2650 7100 50  0001 C CNN
F 3 "" H 2650 7100 50  0001 C CNN
	1    2650 7100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 7350 2800 7250
Wire Wire Line
	2800 7250 2650 7250
Connection ~ 2650 7250
Wire Wire Line
	2650 7250 2650 7150
Wire Wire Line
	2800 7550 2800 7650
Wire Wire Line
	2800 7650 2650 7650
Connection ~ 2650 7650
Wire Wire Line
	2650 7650 2650 7500
Text Label 3200 8000 0    50   ~ 0
sig_mos1
$Comp
L Device:C_Small C_pmp_1
U 1 1 5D1B54F8
P 3100 7200
F 0 "C_pmp_1" H 3200 7200 50  0000 L CNN
F 1 "4.7uF" H 3192 7155 50  0001 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3100 7200 50  0001 C CNN
F 3 "~" H 3100 7200 50  0001 C CNN
	1    3100 7200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 7150 2900 7150
Wire Wire Line
	2900 7150 2900 7100
Wire Wire Line
	2900 7100 3100 7100
Connection ~ 2650 7150
Wire Wire Line
	2650 7150 2650 7100
Wire Wire Line
	3100 7300 3100 7350
Wire Wire Line
	3100 7350 3300 7350
Wire Wire Line
	3300 7350 3300 7400
$Comp
L power:GND #PWR0101
U 1 1 5D1F9D8E
P 3300 7400
F 0 "#PWR0101" H 3300 7150 50  0001 C CNN
F 1 "GND" H 3305 7227 50  0000 C CNN
F 2 "" H 3300 7400 50  0001 C CNN
F 3 "" H 3300 7400 50  0001 C CNN
	1    3300 7400
	1    0    0    -1  
$EndComp
NoConn ~ 2100 1700
Wire Wire Line
	1700 1600 1900 1600
Wire Wire Line
	1900 1600 1900 2200
Wire Wire Line
	1900 2200 2800 2200
Wire Wire Line
	8100 3250 8650 3250
Wire Wire Line
	8650 3350 8250 3350
Wire Wire Line
	8250 3350 8250 3200
$Comp
L Connector:Conn_01x06_Female J12
U 1 1 5D1784AB
P 10500 7450
F 0 "J12" H 10528 7426 50  0000 L CNN
F 1 "DC_Mot_Conn" H 10528 7335 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 10500 7450 50  0001 C CNN
F 3 "~" H 10500 7450 50  0001 C CNN
	1    10500 7450
	1    0    0    1   
$EndComp
Wire Wire Line
	4100 5950 4500 5950
Wire Wire Line
	4500 5950 4500 5850
Wire Wire Line
	4100 6050 4200 6050
Wire Wire Line
	4200 6050 4200 6100
Wire Wire Line
	4200 6150 4100 6150
Connection ~ 4200 6100
Wire Wire Line
	4200 6100 4200 6150
Wire Wire Line
	4400 6100 4500 6100
Wire Wire Line
	4500 6100 4500 5950
Connection ~ 4500 5950
Wire Wire Line
	3600 6250 3100 6250
Wire Wire Line
	3100 6250 3100 6350
Wire Wire Line
	3400 6050 3600 6050
NoConn ~ 3600 6150
$Comp
L power:+3.3V #PWR0103
U 1 1 5D3A66A0
P 8250 3200
F 0 "#PWR0103" H 8250 3050 50  0001 C CNN
F 1 "+3.3V" H 8265 3373 50  0000 C CNN
F 2 "" H 8250 3200 50  0001 C CNN
F 3 "" H 8250 3200 50  0001 C CNN
	1    8250 3200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0104
U 1 1 5D3A6980
P 9500 5450
F 0 "#PWR0104" H 9500 5300 50  0001 C CNN
F 1 "+3.3V" H 9515 5623 50  0000 C CNN
F 2 "" H 9500 5450 50  0001 C CNN
F 3 "" H 9500 5450 50  0001 C CNN
	1    9500 5450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0106
U 1 1 5D3A6E2F
P 4500 5850
F 0 "#PWR0106" H 4500 5700 50  0001 C CNN
F 1 "+3.3V" H 4515 6023 50  0000 C CNN
F 2 "" H 4500 5850 50  0001 C CNN
F 3 "" H 4500 5850 50  0001 C CNN
	1    4500 5850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5D3A6FD2
P 5050 3850
F 0 "#PWR0108" H 5050 3700 50  0001 C CNN
F 1 "+3.3V" H 5065 4023 50  0000 C CNN
F 2 "" H 5050 3850 50  0001 C CNN
F 3 "" H 5050 3850 50  0001 C CNN
	1    5050 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 5D3A7069
P 5300 3850
F 0 "#PWR0109" H 5300 3700 50  0001 C CNN
F 1 "+3.3V" H 5315 4023 50  0000 C CNN
F 2 "" H 5300 3850 50  0001 C CNN
F 3 "" H 5300 3850 50  0001 C CNN
	1    5300 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 1850 3300 1850
Wire Wire Line
	2500 4050 3250 4050
Wire Wire Line
	4300 3650 4750 3650
Wire Wire Line
	4300 3750 4650 3750
Wire Wire Line
	4650 3750 4650 3900
Wire Wire Line
	5050 3850 5050 3950
Wire Wire Line
	4650 3900 4900 3900
Wire Wire Line
	4900 3900 4900 3600
$Comp
L power:+3.3V #PWR0102
U 1 1 5D41E178
P 4900 3600
F 0 "#PWR0102" H 4900 3450 50  0001 C CNN
F 1 "+3.3V" H 4915 3773 50  0000 C CNN
F 2 "" H 4900 3600 50  0001 C CNN
F 3 "" H 4900 3600 50  0001 C CNN
	1    4900 3600
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Switching:R-785.0-1.0 U2
U 1 1 5D8351E4
P 4600 1600
F 0 "U2" H 4600 1842 50  0000 C CNN
F 1 "R-785.0-1.0" H 4600 1751 50  0000 C CNN
F 2 "Converter_DCDC:Converter_DCDC_RECOM_R-78E-0.5_THT" H 4650 1350 50  0001 L CIN
F 3 "https://www.recom-power.com/pdf/Innoline/R-78xx-1.0.pdf" H 4600 1600 50  0001 C CNN
	1    4600 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 1600 4300 1600
Wire Wire Line
	4900 1600 5150 1600
$Comp
L Connector:Conn_01x04_Female J4
U 1 1 5D8585BF
P 8850 4000
F 0 "J4" H 8877 3976 50  0000 L CNN
F 1 "I2C_Generic" H 8877 3885 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 8850 4000 50  0001 C CNN
F 3 "~" H 8850 4000 50  0001 C CNN
	1    8850 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8100 3900 8100 4000
Wire Wire Line
	8650 4100 8400 4100
Wire Wire Line
	8650 4200 8400 4200
$Comp
L power:GND #PWR0107
U 1 1 5D8585C9
P 8100 4000
F 0 "#PWR0107" H 8100 3750 50  0001 C CNN
F 1 "GND" H 8105 3827 50  0000 C CNN
F 2 "" H 8100 4000 50  0001 C CNN
F 3 "" H 8100 4000 50  0001 C CNN
	1    8100 4000
	1    0    0    -1  
$EndComp
Text Label 8400 4100 0    50   ~ 0
SDA
Text Label 8400 4200 0    50   ~ 0
SCL
Wire Wire Line
	8100 3900 8650 3900
Wire Wire Line
	8650 4000 8250 4000
Wire Wire Line
	8250 4000 8250 3850
$Comp
L power:+3.3V #PWR0110
U 1 1 5D8585D4
P 8250 3850
F 0 "#PWR0110" H 8250 3700 50  0001 C CNN
F 1 "+3.3V" H 8265 4023 50  0000 C CNN
F 2 "" H 8250 3850 50  0001 C CNN
F 3 "" H 8250 3850 50  0001 C CNN
	1    8250 3850
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:A4950E U3
U 1 1 5D882F7A
P 8500 5550
F 0 "U3" H 8700 5900 50  0000 C CNN
F 1 "A4950E" H 8250 5900 50  0000 C CNN
F 2 "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.35x2.35mm" H 8500 5000 50  0001 C CNN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A4950-Datasheet.ashx" H 8200 5900 50  0001 C CNN
	1    8500 5550
	1    0    0    -1  
$EndComp
$Comp
L Driver_Motor:A4950E U4
U 1 1 5D88308F
P 8600 7350
F 0 "U4" H 8850 7700 50  0000 C CNN
F 1 "A4950E" H 8350 7700 50  0000 C CNN
F 2 "Package_SO:SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.35x2.35mm" H 8600 6800 50  0001 C CNN
F 3 "http://www.allegromicro.com/~/media/Files/Datasheets/A4950-Datasheet.ashx" H 8300 7700 50  0001 C CNN
	1    8600 7350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10200 5350 9850 5350
Wire Wire Line
	9850 5350 9850 5100
Wire Wire Line
	9850 5100 9100 5100
Wire Wire Line
	9100 5100 9100 5350
Wire Wire Line
	9100 5350 8900 5350
Wire Wire Line
	8900 5450 9200 5450
Wire Wire Line
	9200 5450 9200 5200
Wire Wire Line
	9200 5200 9750 5200
Wire Wire Line
	9750 5200 9750 5450
Wire Wire Line
	9750 5450 10200 5450
Wire Wire Line
	10300 7350 9600 7350
Wire Wire Line
	9600 7350 9600 7250
$Comp
L power:+3.3V #PWR0105
U 1 1 5D8ABAF9
P 9600 7250
F 0 "#PWR0105" H 9600 7100 50  0001 C CNN
F 1 "+3.3V" H 9615 7423 50  0000 C CNN
F 2 "" H 9600 7250 50  0001 C CNN
F 3 "" H 9600 7250 50  0001 C CNN
	1    9600 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 6900 9200 6900
Wire Wire Line
	9200 6900 9200 7150
Wire Wire Line
	9200 7150 9000 7150
Wire Wire Line
	9000 7250 9300 7250
Wire Wire Line
	9300 7250 9300 7000
Wire Wire Line
	9300 7000 9850 7000
Wire Wire Line
	8200 7150 7850 7150
Wire Wire Line
	8200 7250 7850 7250
Wire Wire Line
	8100 5350 7750 5350
Wire Wire Line
	8100 5450 7750 5450
Text Label 7850 7150 0    50   ~ 0
IN1_1
Text Label 7850 7250 0    50   ~ 0
IN2_1
Text Label 7750 5350 0    50   ~ 0
IN1_2
Text Label 7750 5450 0    50   ~ 0
IN2_2
Text Label 2700 4150 0    50   ~ 0
IN1_1
Text Label 2700 4250 0    50   ~ 0
IN2_1
Text Label 2700 4650 0    50   ~ 0
IN2_2
Text Label 2700 4550 0    50   ~ 0
IN1_2
Wire Wire Line
	3250 3950 2500 3950
Text Label 2500 3950 0    50   ~ 0
buzzer_out
Wire Wire Line
	9000 7350 9100 7350
Wire Wire Line
	9100 7350 9100 7400
Wire Wire Line
	8900 5550 9000 5550
Wire Wire Line
	9000 5550 9000 5600
$Comp
L Device:R_Small R_lim1
U 1 1 5D923BC4
P 9000 5700
F 0 "R_lim1" H 9059 5746 50  0000 L CNN
F 1 "0.4R" H 9059 5655 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric_Pad1.52x3.35mm_HandSolder" H 9000 5700 50  0001 C CNN
F 3 "~" H 9000 5700 50  0001 C CNN
	1    9000 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R_lim2
U 1 1 5D923D80
P 9100 7500
F 0 "R_lim2" H 9159 7546 50  0000 L CNN
F 1 "0.4R" H 9159 7455 50  0000 L CNN
F 2 "Resistor_SMD:R_2512_6332Metric_Pad1.52x3.35mm_HandSolder" H 9100 7500 50  0001 C CNN
F 3 "~" H 9100 7500 50  0001 C CNN
	1    9100 7500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 5800 9000 6000
Wire Wire Line
	9000 6000 8800 6000
Wire Wire Line
	8500 6000 8500 5950
Wire Wire Line
	8600 5950 8600 6000
Connection ~ 8600 6000
Wire Wire Line
	8600 6000 8500 6000
Wire Wire Line
	8600 7750 8600 7800
Wire Wire Line
	8600 7800 8700 7800
Wire Wire Line
	9100 7800 9100 7600
Wire Wire Line
	8700 7750 8700 7800
Connection ~ 8700 7800
Wire Wire Line
	8800 6000 8800 6050
Connection ~ 8800 6000
Wire Wire Line
	8800 6000 8600 6000
Wire Wire Line
	8700 7800 8900 7800
Wire Wire Line
	8900 7800 8900 7850
Connection ~ 8900 7800
Wire Wire Line
	8900 7800 9100 7800
$Comp
L power:GND #PWR0111
U 1 1 5D981386
P 8900 7850
F 0 "#PWR0111" H 8900 7600 50  0001 C CNN
F 1 "GND" H 9050 7800 50  0000 C CNN
F 2 "" H 8900 7850 50  0001 C CNN
F 3 "" H 8900 7850 50  0001 C CNN
	1    8900 7850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5D9813D9
P 8800 6050
F 0 "#PWR0112" H 8800 5800 50  0001 C CNN
F 1 "GND" H 8950 6000 50  0000 C CNN
F 2 "" H 8800 6050 50  0001 C CNN
F 3 "" H 8800 6050 50  0001 C CNN
	1    8800 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:Buzzer BZ1
U 1 1 5D981F07
P 5100 7400
F 0 "BZ1" H 5253 7429 50  0000 L CNN
F 1 "Buzzer" H 5253 7338 50  0000 L CNN
F 2 "Buzzer_Beeper:MagneticBuzzer_ProSignal_ABT-410-RC" V 5075 7500 50  0001 C CNN
F 3 "~" V 5075 7500 50  0001 C CNN
	1    5100 7400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8200 7450 7550 7450
Wire Wire Line
	7550 7450 7550 7250
Wire Wire Line
	8100 5650 7450 5650
Wire Wire Line
	7450 5650 7450 5450
$Comp
L power:+5V #PWR0113
U 1 1 5DA0107D
P 7450 5450
F 0 "#PWR0113" H 7450 5300 50  0001 C CNN
F 1 "+5V" H 7465 5623 50  0000 C CNN
F 2 "" H 7450 5450 50  0001 C CNN
F 3 "" H 7450 5450 50  0001 C CNN
	1    7450 5450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0114
U 1 1 5DA011FA
P 7550 7250
F 0 "#PWR0114" H 7550 7100 50  0001 C CNN
F 1 "+5V" H 7565 7423 50  0000 C CNN
F 2 "" H 7550 7250 50  0001 C CNN
F 3 "" H 7550 7250 50  0001 C CNN
	1    7550 7250
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C_Mot1
U 1 1 5D8858C8
P 7900 4850
F 0 "C_Mot1" V 7750 4800 50  0000 L CNN
F 1 "10uF" V 7800 4800 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 7900 4850 50  0001 C CNN
F 3 "~" H 7900 4850 50  0001 C CNN
	1    7900 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 4950 7900 5000
Wire Wire Line
	7900 4750 7900 4700
$Comp
L power:+12V #PWR0115
U 1 1 5D8B1C5D
P 8500 4650
F 0 "#PWR0115" H 8500 4500 50  0001 C CNN
F 1 "+12V" H 8515 4823 50  0000 C CNN
F 2 "" H 8500 4650 50  0001 C CNN
F 3 "" H 8500 4650 50  0001 C CNN
	1    8500 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5D8C119F
P 7900 5000
F 0 "#PWR0116" H 7900 4750 50  0001 C CNN
F 1 "GND" H 8050 4950 50  0000 C CNN
F 2 "" H 7900 5000 50  0001 C CNN
F 3 "" H 7900 5000 50  0001 C CNN
	1    7900 5000
	1    0    0    -1  
$EndComp
Text Notes 9150 6200 0    50   ~ 0
2512, 1W, SMD sense resistor, 1% or greater\n
$Comp
L Device:Q_NPN_BEC Q3
U 1 1 5D95975B
P 4900 8000
F 0 "Q3" H 5091 8046 50  0000 L CNN
F 1 "PMBT2222" H 5091 7955 50  0000 L CNN
F 2 "Package_TO_SOT_SMD:SOT-23_Handsoldering" H 5100 8100 50  0001 C CNN
F 3 "~" H 4900 8000 50  0001 C CNN
	1    4900 8000
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 8200 5000 8300
$Comp
L power:GND #PWR0119
U 1 1 5D959966
P 5000 8300
F 0 "#PWR0119" H 5000 8050 50  0001 C CNN
F 1 "GND" H 5005 8127 50  0000 C CNN
F 2 "" H 5000 8300 50  0001 C CNN
F 3 "" H 5000 8300 50  0001 C CNN
	1    5000 8300
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R_Bz1
U 1 1 5D96A7AE
P 4450 8000
F 0 "R_Bz1" V 4350 7900 50  0000 L CNN
F 1 "4.3k" V 4250 7950 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4450 8000 50  0001 C CNN
F 3 "~" H 4450 8000 50  0001 C CNN
	1    4450 8000
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4550 8000 4700 8000
Text Label 3950 8000 0    50   ~ 0
buzzer_out
Wire Wire Line
	3950 8000 4350 8000
Wire Wire Line
	5000 7500 5000 7600
Wire Wire Line
	5000 7050 5000 7250
$Comp
L power:+5V #PWR0120
U 1 1 5D9D3D5B
P 5000 7050
F 0 "#PWR0120" H 5000 6900 50  0001 C CNN
F 1 "+5V" H 5015 7223 50  0000 C CNN
F 2 "" H 5000 7050 50  0001 C CNN
F 3 "" H 5000 7050 50  0001 C CNN
	1    5000 7050
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Small D3
U 1 1 5D9D439D
P 4700 7450
F 0 "D3" V 4700 7550 50  0000 L CNN
F 1 "1N4001" V 4745 7518 50  0001 L CNN
F 2 "Diode_SMD:D_1206_3216Metric_Pad1.42x1.75mm_HandSolder" V 4700 7450 50  0001 C CNN
F 3 "~" V 4700 7450 50  0001 C CNN
	1    4700 7450
	0    1    1    0   
$EndComp
Wire Wire Line
	5000 7600 4700 7600
Wire Wire Line
	4700 7600 4700 7550
Connection ~ 5000 7600
Wire Wire Line
	5000 7600 5000 7800
Wire Wire Line
	4700 7350 4700 7250
Wire Wire Line
	4700 7250 5000 7250
Connection ~ 5000 7250
Wire Wire Line
	5000 7250 5000 7300
Wire Wire Line
	8500 4650 8500 4700
Connection ~ 8500 4700
Wire Wire Line
	8500 4700 8500 5150
$Comp
L power:+12V #PWR0117
U 1 1 5D8D12E0
P 8600 6450
F 0 "#PWR0117" H 8600 6300 50  0001 C CNN
F 1 "+12V" H 8615 6623 50  0000 C CNN
F 2 "" H 8600 6450 50  0001 C CNN
F 3 "" H 8600 6450 50  0001 C CNN
	1    8600 6450
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 6450 8600 6500
$Comp
L Device:C_Small C_Mot2
U 1 1 5DA4F47B
P 8000 6650
F 0 "C_Mot2" V 7850 6600 50  0000 L CNN
F 1 "10uF" V 7900 6600 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 8000 6650 50  0001 C CNN
F 3 "~" H 8000 6650 50  0001 C CNN
	1    8000 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 6750 8000 6800
Wire Wire Line
	8000 6550 8000 6500
$Comp
L power:GND #PWR0118
U 1 1 5DA4F484
P 8000 6800
F 0 "#PWR0118" H 8000 6550 50  0001 C CNN
F 1 "GND" H 8150 6750 50  0000 C CNN
F 2 "" H 8000 6800 50  0001 C CNN
F 3 "" H 8000 6800 50  0001 C CNN
	1    8000 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 6500 8250 6500
Wire Wire Line
	8250 6550 8250 6500
Connection ~ 8000 6750
Wire Wire Line
	8000 6750 8250 6750
$Comp
L Device:C_Small C_Mot22
U 1 1 5DA660EF
P 8250 6650
F 0 "C_Mot22" V 8100 6600 50  0000 L CNN
F 1 "10uF" V 8150 6600 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 8250 6650 50  0001 C CNN
F 3 "~" H 8250 6650 50  0001 C CNN
	1    8250 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	8150 4700 8500 4700
Wire Wire Line
	7900 4700 8150 4700
Connection ~ 8150 4700
Wire Wire Line
	8150 4750 8150 4700
Connection ~ 7900 4950
Wire Wire Line
	7900 4950 8150 4950
$Comp
L Device:C_Small C_Mot11
U 1 1 5DA65F27
P 8150 4850
F 0 "C_Mot11" V 8000 4800 50  0000 L CNN
F 1 "10uF" V 8050 4800 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 8150 4850 50  0001 C CNN
F 3 "~" H 8150 4850 50  0001 C CNN
	1    8150 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 6500 8600 6500
Connection ~ 8250 6500
Connection ~ 8600 6500
Wire Wire Line
	8600 6500 8600 6950
Wire Wire Line
	9950 7250 10300 7250
Wire Wire Line
	9950 6900 9950 7250
Wire Wire Line
	10300 7150 9850 7150
Wire Wire Line
	9850 7150 9850 7000
$Comp
L Switch:SW_SPDT SW1
U 1 1 5D272F2F
P 2300 1600
F 0 "SW1" H 2300 1885 50  0000 C CNN
F 1 "CS12ANW03 - NKK" H 2300 1794 50  0000 C CNN
F 2 "MiniRobot_Lib:CS12ANW03 - NKK" H 2300 1600 50  0001 C CNN
F 3 "" H 2300 1600 50  0001 C CNN
	1    2300 1600
	-1   0    0    1   
$EndComp
$Comp
L MiniKiwiLib:Teensy3.2 U1
U 1 1 5DCB9D08
P 3700 3350
F 0 "U1" H 3775 3460 70  0000 C CNN
F 1 "Teensy3.2" H 3775 3339 70  0000 C CNN
F 2 "" H 3650 3350 50  0001 C CNN
F 3 "" H 3650 3350 50  0001 C CNN
	1    3700 3350
	1    0    0    -1  
$EndComp
Text Notes 7350 9200 0    79   ~ 0
Mini Kiwi, main board schematic, revision 2\n\n
Text Notes 8100 9200 0    51   ~ 0
Last revised November 12
$EndSCHEMATC
