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
L TTGO:TTGO_LoRa_SSD1306 U1
U 1 1 622D2B8D
P 4700 3650
F 0 "U1" H 4700 4765 50  0000 C CNN
F 1 "TTGO_LoRa_SSD1306" H 4700 4674 50  0000 C CNN
F 2 "" H 4650 4650 50  0001 C CNN
F 3 "" H 4650 4650 50  0001 C CNN
	1    4700 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 622D4BAE
P 3350 3100
F 0 "#PWR01" H 3350 2850 50  0001 C CNN
F 1 "GND" H 3355 2927 50  0000 C CNN
F 2 "" H 3350 3100 50  0001 C CNN
F 3 "" H 3350 3100 50  0001 C CNN
	1    3350 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 3000 3350 3100
$Comp
L power:+3.3V #PWR03
U 1 1 622D6649
P 3850 3300
F 0 "#PWR03" H 3850 3150 50  0001 C CNN
F 1 "+3.3V" H 3865 3473 50  0000 C CNN
F 2 "" H 3850 3300 50  0001 C CNN
F 3 "" H 3850 3300 50  0001 C CNN
	1    3850 3300
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 3300 3850 3200
$Comp
L power:GND #PWR06
U 1 1 622D7723
P 6050 3150
F 0 "#PWR06" H 6050 2900 50  0001 C CNN
F 1 "GND" H 6055 2977 50  0000 C CNN
F 2 "" H 6050 3150 50  0001 C CNN
F 3 "" H 6050 3150 50  0001 C CNN
	1    6050 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR04
U 1 1 622D868A
P 5650 3200
F 0 "#PWR04" H 5650 3050 50  0001 C CNN
F 1 "+3.3V" H 5665 3373 50  0000 C CNN
F 2 "" H 5650 3200 50  0001 C CNN
F 3 "" H 5650 3200 50  0001 C CNN
	1    5650 3200
	-1   0    0    1   
$EndComp
Wire Wire Line
	6050 3000 5950 3000
Wire Wire Line
	6050 3000 6050 3150
Wire Wire Line
	5650 3200 5500 3200
Wire Wire Line
	5250 4100 6150 4100
Wire Wire Line
	6150 4250 6850 4250
$Comp
L power:GND #PWR0102
U 1 1 622E9864
P 6700 4550
F 0 "#PWR0102" H 6700 4300 50  0001 C CNN
F 1 "GND" H 6705 4377 50  0000 C CNN
F 2 "" H 6700 4550 50  0001 C CNN
F 3 "" H 6700 4550 50  0001 C CNN
	1    6700 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4150 6850 4150
Wire Wire Line
	6850 4450 6700 4450
Wire Wire Line
	6700 4450 6700 4550
NoConn ~ 6850 4350
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 622EAA20
P 5950 3000
F 0 "#FLG0101" H 5950 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 5950 3173 50  0000 C CNN
F 2 "" H 5950 3000 50  0001 C CNN
F 3 "~" H 5950 3000 50  0001 C CNN
	1    5950 3000
	1    0    0    -1  
$EndComp
Connection ~ 5950 3000
Wire Wire Line
	5950 3000 5250 3000
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 622EB803
P 5500 3200
F 0 "#FLG0103" H 5500 3275 50  0001 C CNN
F 1 "PWR_FLAG" H 5500 3373 50  0000 C CNN
F 2 "" H 5500 3200 50  0001 C CNN
F 3 "~" H 5500 3200 50  0001 C CNN
	1    5500 3200
	1    0    0    -1  
$EndComp
Connection ~ 5500 3200
Wire Wire Line
	5500 3200 5250 3200
NoConn ~ 4150 3400
NoConn ~ 4150 3500
NoConn ~ 4150 3600
NoConn ~ 4150 3700
NoConn ~ 4150 3800
NoConn ~ 4150 3900
NoConn ~ 4150 4000
NoConn ~ 4150 4200
NoConn ~ 4150 4400
NoConn ~ 4150 4600
NoConn ~ 4150 4700
NoConn ~ 5250 4700
NoConn ~ 5250 4600
NoConn ~ 5250 4500
NoConn ~ 5250 4400
NoConn ~ 5250 4300
NoConn ~ 5250 4200
NoConn ~ 5250 3900
NoConn ~ 5250 3600
NoConn ~ 5250 3500
NoConn ~ 5250 3400
NoConn ~ 5250 3300
NoConn ~ 5250 3800
NoConn ~ 4150 3300
Wire Wire Line
	3350 3000 4150 3000
Wire Wire Line
	3850 3200 4150 3200
$Comp
L DHT22:DHT22 U3
U 1 1 622DD3C9
P 7000 4300
F 0 "U3" V 6954 5128 50  0000 L CNN
F 1 "DHT22" V 7045 5128 50  0000 L CNN
F 2 "" H 7000 4700 50  0001 C CNN
F 3 "" H 7000 4700 50  0001 C CNN
	1    7000 4300
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR02
U 1 1 622E4AA3
P 6700 3950
F 0 "#PWR02" H 6700 3800 50  0001 C CNN
F 1 "+3.3V" H 6715 4123 50  0000 C CNN
F 2 "" H 6700 3950 50  0001 C CNN
F 3 "" H 6700 3950 50  0001 C CNN
	1    6700 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 4150 6700 3950
NoConn ~ 5250 3100
NoConn ~ 4150 3100
$Comp
L power:+3.3V #PWR0101
U 1 1 622E62B4
P 3400 5050
F 0 "#PWR0101" H 3400 4900 50  0001 C CNN
F 1 "+3.3V" H 3415 5223 50  0000 C CNN
F 2 "" H 3400 5050 50  0001 C CNN
F 3 "" H 3400 5050 50  0001 C CNN
	1    3400 5050
	-1   0    0    1   
$EndComp
Wire Wire Line
	3050 4900 3400 4900
Wire Wire Line
	3400 4900 3400 5050
$Comp
L power:GND #PWR0103
U 1 1 622E6AAB
P 3650 4900
F 0 "#PWR0103" H 3650 4650 50  0001 C CNN
F 1 "GND" H 3655 4727 50  0000 C CNN
F 2 "" H 3650 4900 50  0001 C CNN
F 3 "" H 3650 4900 50  0001 C CNN
	1    3650 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 4800 3450 4800
Wire Wire Line
	3650 4800 3650 4900
NoConn ~ 4150 4100
Wire Wire Line
	3050 4600 3800 4600
Wire Wire Line
	3800 4600 3800 4500
Wire Wire Line
	3800 4500 4150 4500
Wire Wire Line
	3050 4700 3650 4700
Wire Wire Line
	3650 4700 3650 4300
Wire Wire Line
	3650 4300 4150 4300
Wire Wire Line
	3050 4500 3450 4500
Wire Wire Line
	3450 4500 3450 4800
Connection ~ 3450 4800
Wire Wire Line
	3450 4800 3650 4800
NoConn ~ 3050 4400
NoConn ~ 3050 4300
NoConn ~ 3050 4200
NoConn ~ 5250 3700
$Comp
L CCS811:CCS811 U2
U 1 1 622EA7DC
P 2650 4700
F 0 "U2" H 2733 5425 50  0000 C CNN
F 1 "CCS811" H 2733 5334 50  0000 C CNN
F 2 "" H 2500 4650 50  0001 C CNN
F 3 "" H 2500 4650 50  0001 C CNN
	1    2650 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 4100 6150 4250
NoConn ~ 5250 4000
$EndSCHEMATC
