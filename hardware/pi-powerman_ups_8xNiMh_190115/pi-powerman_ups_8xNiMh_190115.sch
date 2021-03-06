EESchema Schematic File Version 4
LIBS:pi-powerman_ups_8xNiMh_190115-cache
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Pi-Powerman UPS 8*NiMh 1900mAh"
Date "2019-04-04"
Rev ""
Comp "obbo.pl"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:Battery BT1
U 1 1 5CA10D0D
P 7550 5000
F 0 "BT1" H 7658 5046 50  0000 L CNN
F 1 "Battery 8*NiMh" H 7658 4955 50  0000 L CNN
F 2 "" V 7550 5060 50  0001 C CNN
F 3 "~" V 7550 5060 50  0001 C CNN
	1    7550 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:Thermistor_NTC TH1
U 1 1 5CA10DEA
P 7300 5000
F 0 "TH1" H 7398 5046 50  0000 L CNN
F 1 "NTC 10k B=3435" H 7398 4955 50  0000 L CNN
F 2 "" H 7300 5050 50  0001 C CNN
F 3 "~" H 7300 5050 50  0001 C CNN
	1    7300 5000
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5C9ADE36
P 7050 3350
F 0 "R1" H 7120 3396 50  0000 L CNN
F 1 "100m" H 7120 3305 50  0000 L CNN
F 2 "" V 6980 3350 50  0001 C CNN
F 3 "~" H 7050 3350 50  0001 C CNN
	1    7050 3350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5C9ADEEE
P 5850 5000
F 0 "C5" H 5965 5046 50  0000 L CNN
F 1 "100n" H 5965 4955 50  0000 L CNN
F 2 "" H 5888 4850 50  0001 C CNN
F 3 "~" H 5850 5000 50  0001 C CNN
	1    5850 5000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5C9ADF5E
P 6400 4150
F 0 "C4" H 6515 4196 50  0000 L CNN
F 1 "100n" H 6515 4105 50  0000 L CNN
F 2 "" H 6438 4000 50  0001 C CNN
F 3 "~" H 6400 4150 50  0001 C CNN
	1    6400 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5C9ADF9A
P 8000 2250
F 0 "C1" H 8115 2296 50  0000 L CNN
F 1 "4u7" H 8115 2205 50  0000 L CNN
F 2 "" H 8038 2100 50  0001 C CNN
F 3 "~" H 8000 2250 50  0001 C CNN
	1    8000 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5C9ADFDE
P 7050 2900
F 0 "L1" V 7240 2900 50  0000 C CNN
F 1 "15u" V 7149 2900 50  0000 C CNN
F 2 "" H 7050 2900 50  0001 C CNN
F 3 "~" H 7050 2900 50  0001 C CNN
	1    7050 2900
	1    0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5C9AE629
P 4300 3950
F 0 "C2" H 4415 3996 50  0000 L CNN
F 1 "100n" H 4415 3905 50  0000 L CNN
F 2 "" H 4338 3800 50  0001 C CNN
F 3 "~" H 4300 3950 50  0001 C CNN
	1    4300 3950
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Row_Letter_First J1
U 1 1 5C9AF635
P 3250 2500
F 0 "J1" H 3300 2717 50  0000 C CNN
F 1 "UPS_in" H 3300 2626 50  0000 C CNN
F 2 "" H 3250 2500 50  0001 C CNN
F 3 "~" H 3250 2500 50  0001 C CNN
	1    3250 2500
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Row_Letter_First J2
U 1 1 5C9AF737
P 9400 2550
F 0 "J2" H 9450 2767 50  0000 C CNN
F 1 "UPS_out" H 9450 2676 50  0000 C CNN
F 2 "" H 9400 2550 50  0001 C CNN
F 3 "~" H 9400 2550 50  0001 C CNN
	1    9400 2550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR014
U 1 1 5CA01530
P 8550 5000
F 0 "#PWR014" H 8550 4750 50  0001 C CNN
F 1 "GND" H 8555 4827 50  0000 C CNN
F 2 "" H 8550 5000 50  0001 C CNN
F 3 "" H 8550 5000 50  0001 C CNN
	1    8550 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5CA01581
P 6250 3150
F 0 "#PWR06" H 6250 2900 50  0001 C CNN
F 1 "GND" H 6255 2977 50  0000 C CNN
F 2 "" H 6250 3150 50  0001 C CNN
F 3 "" H 6250 3150 50  0001 C CNN
	1    6250 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5CA30AFA
P 3150 3100
F 0 "#PWR04" H 3150 2850 50  0001 C CNN
F 1 "GND" H 3155 2927 50  0000 C CNN
F 2 "" H 3150 3100 50  0001 C CNN
F 3 "" H 3150 3100 50  0001 C CNN
	1    3150 3100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5CA30BED
P 9300 3100
F 0 "#PWR05" H 9300 2850 50  0001 C CNN
F 1 "GND" H 9305 2927 50  0000 C CNN
F 2 "" H 9300 3100 50  0001 C CNN
F 3 "" H 9300 3100 50  0001 C CNN
	1    9300 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 3100 9300 2900
Wire Wire Line
	3250 2000 3250 2300
Wire Wire Line
	3150 2300 3150 2000
Wire Wire Line
	3150 2000 3250 2000
Wire Wire Line
	3150 2800 3150 2900
Wire Wire Line
	3250 2800 3250 2900
Wire Wire Line
	3250 2900 3150 2900
Connection ~ 3150 2900
Wire Wire Line
	3150 2900 3150 3100
Wire Wire Line
	9400 2850 9400 2900
Wire Wire Line
	9400 2900 9300 2900
Connection ~ 9300 2900
Wire Wire Line
	9300 2900 9300 2850
Wire Wire Line
	9400 2350 9400 2000
Wire Wire Line
	9300 2350 9300 2000
Wire Wire Line
	9300 2000 9400 2000
$Comp
L Connector_Generic:Conn_02x02_Row_Letter_First J3
U 1 1 5CBC4BC4
P 2250 5950
F 0 "J3" H 2300 6167 50  0000 C CNN
F 1 "UPS_status" H 2300 6076 50  0000 C CNN
F 2 "" H 2250 5950 50  0001 C CNN
F 3 "~" H 2250 5950 50  0001 C CNN
	1    2250 5950
	0    1    1    0   
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Row_Letter_First J4
U 1 1 5CBC4D48
P 4500 5950
F 0 "J4" H 4550 6167 50  0000 C CNN
F 1 "UPS_bat" H 4550 6076 50  0000 C CNN
F 2 "" H 4500 5950 50  0001 C CNN
F 3 "~" H 4500 5950 50  0001 C CNN
	1    4500 5950
	0    1    1    0   
$EndComp
Text GLabel 1800 5600 0    50   Input ~ 0
UPS_FAULT
Text GLabel 1800 5750 0    50   Input ~ 0
UPS_CHRG
Text GLabel 1800 6250 0    50   Input ~ 0
UPS_TOC
Text GLabel 1800 6400 0    50   Input ~ 0
UPS_READY
Wire Wire Line
	2250 5600 2250 5750
Wire Wire Line
	2250 6400 2250 6250
Text GLabel 4050 5600 0    50   Input ~ 0
VBAT
Text GLabel 4050 6250 0    50   Input ~ 0
BAT_TEST
Text GLabel 4050 6400 0    50   Input ~ 0
CHRG_EN__BAT_TEMP
Wire Wire Line
	4500 6400 4500 6250
Wire Wire Line
	4050 6400 4500 6400
Wire Wire Line
	4050 6250 4400 6250
Wire Wire Line
	4050 5600 4500 5600
Wire Wire Line
	4500 5600 4500 5750
Wire Wire Line
	1800 5600 2250 5600
Wire Wire Line
	1800 5750 2150 5750
Wire Wire Line
	1800 6250 2150 6250
Wire Wire Line
	1800 6400 2250 6400
$Comp
L Diode:B340 D1
U 1 1 5C9AE1EE
P 7550 3150
F 0 "D1" V 7504 3229 50  0000 L CNN
F 1 "B340" V 7595 3229 50  0000 L CNN
F 2 "Diode_SMD:D_SMC" H 7550 2975 50  0001 C CNN
F 3 "http://www.jameco.com/Jameco/Products/ProdDS/1538777.pdf" H 7550 3150 50  0001 C CNN
	1    7550 3150
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR013
U 1 1 5CB290A2
P 7550 5400
F 0 "#PWR013" H 7550 5150 50  0001 C CNN
F 1 "GND" H 7555 5227 50  0000 C CNN
F 2 "" H 7550 5400 50  0001 C CNN
F 3 "" H 7550 5400 50  0001 C CNN
	1    7550 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 5400 7550 5250
Wire Wire Line
	7300 5150 7300 5250
Wire Wire Line
	7300 5250 7550 5250
Connection ~ 7550 5250
Wire Wire Line
	7550 5250 7550 5200
$Comp
L Device:R R8
U 1 1 5CB2F0E4
P 8550 4300
F 0 "R8" H 8620 4346 50  0000 L CNN
F 1 "24/5W" H 8620 4255 50  0000 L CNN
F 2 "" V 8480 4300 50  0001 C CNN
F 3 "~" H 8550 4300 50  0001 C CNN
	1    8550 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 5CB2F134
P 8550 3950
F 0 "R6" H 8620 3996 50  0000 L CNN
F 1 "24/5W" H 8620 3905 50  0000 L CNN
F 2 "" V 8480 3950 50  0001 C CNN
F 3 "~" H 8550 3950 50  0001 C CNN
	1    8550 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 4100 8550 4150
$Comp
L Device:R R10
U 1 1 5CB2FDAE
P 8900 4950
F 0 "R10" H 8970 4996 50  0000 L CNN
F 1 "20k" H 8970 4905 50  0000 L CNN
F 2 "" V 8830 4950 50  0001 C CNN
F 3 "~" H 8900 4950 50  0001 C CNN
	1    8900 4950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR015
U 1 1 5CB2FE08
P 8900 5150
F 0 "#PWR015" H 8900 4900 50  0001 C CNN
F 1 "GND" H 8905 4977 50  0000 C CNN
F 2 "" H 8900 5150 50  0001 C CNN
F 3 "" H 8900 5150 50  0001 C CNN
	1    8900 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 5150 8900 5100
Wire Wire Line
	8900 4700 8900 4800
Text GLabel 9050 4700 2    50   Input ~ 0
BAT_TEST
Wire Wire Line
	9050 4700 8900 4700
Wire Wire Line
	8850 4700 8900 4700
Connection ~ 8900 4700
Wire Wire Line
	8550 4900 8550 5000
Wire Wire Line
	8550 4500 8550 4450
Wire Wire Line
	4450 2300 4450 2400
Wire Wire Line
	4450 2400 4750 2400
Wire Wire Line
	5750 2500 5950 2500
Wire Wire Line
	5950 2500 5950 2350
Wire Wire Line
	5750 2850 5950 2850
Wire Wire Line
	6250 2650 6250 2600
Wire Wire Line
	6250 3150 6250 3100
Connection ~ 6250 3100
Wire Wire Line
	6250 3100 6250 3050
Wire Wire Line
	4650 2000 5800 2000
Wire Wire Line
	6250 2000 6250 2150
Wire Wire Line
	7050 2750 7050 2600
Wire Wire Line
	7050 2600 6250 2600
Connection ~ 6250 2600
Wire Wire Line
	6250 2600 6250 2550
Text GLabel 4300 2600 0    50   Input ~ 0
UPS_FAULT
Text GLabel 4300 2700 0    50   Input ~ 0
UPS_CHRG
Text GLabel 4300 2800 0    50   Input ~ 0
UPS_TOC
Text GLabel 4300 2900 0    50   Input ~ 0
UPS_READY
Wire Wire Line
	4300 2600 4750 2600
Wire Wire Line
	4300 2700 4750 2700
Wire Wire Line
	4300 2800 4750 2800
Wire Wire Line
	4300 2900 4750 2900
Wire Wire Line
	4250 2000 3650 2000
Connection ~ 3250 2000
$Comp
L Device:R R3
U 1 1 5CB4A9F1
P 3650 3500
F 0 "R3" H 3720 3546 50  0000 L CNN
F 1 "20k" H 3720 3455 50  0000 L CNN
F 2 "" V 3580 3500 50  0001 C CNN
F 3 "~" H 3650 3500 50  0001 C CNN
	1    3650 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3350 3650 3100
Connection ~ 3650 2000
Wire Wire Line
	3650 2000 3250 2000
Wire Wire Line
	4750 3100 3650 3100
Connection ~ 3650 3100
Wire Wire Line
	3650 3100 3650 2000
$Comp
L power:GND #PWR07
U 1 1 5CB4DD4F
P 3650 3850
F 0 "#PWR07" H 3650 3600 50  0001 C CNN
F 1 "GND" H 3655 3677 50  0000 C CNN
F 2 "" H 3650 3850 50  0001 C CNN
F 3 "" H 3650 3850 50  0001 C CNN
	1    3650 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 3650 3650 3850
$Comp
L Device:R R2
U 1 1 5CB503FE
P 4050 3400
F 0 "R2" H 4120 3446 50  0000 L CNN
F 1 "100k" H 4120 3355 50  0000 L CNN
F 2 "" V 3980 3400 50  0001 C CNN
F 3 "~" H 4050 3400 50  0001 C CNN
	1    4050 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3200 4050 3250
$Comp
L power:GND #PWR08
U 1 1 5CB51873
P 4050 4250
F 0 "#PWR08" H 4050 4000 50  0001 C CNN
F 1 "GND" H 4055 4077 50  0000 C CNN
F 2 "" H 4050 4250 50  0001 C CNN
F 3 "" H 4050 4250 50  0001 C CNN
	1    4050 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3550 4050 3650
Connection ~ 4050 3650
Wire Wire Line
	4050 3650 4050 3750
Connection ~ 4050 3750
Wire Wire Line
	4050 3750 4050 3950
Wire Wire Line
	4050 3200 4750 3200
Wire Wire Line
	4050 3650 4750 3650
Wire Wire Line
	4050 3750 4750 3750
Wire Wire Line
	4450 3950 4750 3950
Wire Wire Line
	4150 3950 4050 3950
Connection ~ 4050 3950
Wire Wire Line
	4050 3950 4050 4250
$Comp
L Device:R R7
U 1 1 5CB757CD
P 5250 4450
F 0 "R7" H 5320 4496 50  0000 L CNN
F 1 "10k" H 5320 4405 50  0000 L CNN
F 2 "" V 5180 4450 50  0001 C CNN
F 3 "~" H 5250 4450 50  0001 C CNN
	1    5250 4450
	0    1    1    0   
$EndComp
Wire Wire Line
	4750 3850 4650 3850
Wire Wire Line
	4650 3850 4650 4450
Wire Wire Line
	4650 4450 5100 4450
Wire Wire Line
	5400 4450 5850 4450
Wire Wire Line
	5850 4450 5850 3950
Wire Wire Line
	5850 3950 5750 3950
Wire Wire Line
	8000 2100 8000 2000
Wire Wire Line
	8000 2000 7550 2000
Connection ~ 6250 2000
Wire Wire Line
	7550 3000 7550 2000
Connection ~ 7550 2000
Wire Wire Line
	7550 2000 6250 2000
$Comp
L power:GND #PWR03
U 1 1 5CB7EEFB
P 8000 2500
F 0 "#PWR03" H 8000 2250 50  0001 C CNN
F 1 "GND" H 8005 2327 50  0000 C CNN
F 2 "" H 8000 2500 50  0001 C CNN
F 3 "" H 8000 2500 50  0001 C CNN
	1    8000 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8000 2500 8000 2400
$Comp
L Device:R R4
U 1 1 5CB8907F
P 6150 3700
F 0 "R4" H 6220 3746 50  0000 L CNN
F 1 "6k8" H 6220 3655 50  0000 L CNN
F 2 "" V 6080 3700 50  0001 C CNN
F 3 "~" H 6150 3700 50  0001 C CNN
	1    6150 3700
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5CB890F5
P 6750 3700
F 0 "R5" H 6820 3746 50  0000 L CNN
F 1 "47k" H 6820 3655 50  0000 L CNN
F 2 "" V 6680 3700 50  0001 C CNN
F 3 "~" H 6750 3700 50  0001 C CNN
	1    6750 3700
	0    1    1    0   
$EndComp
Wire Wire Line
	7050 3050 7050 3100
Wire Wire Line
	6700 3400 6700 3100
Wire Wire Line
	6700 3100 7050 3100
Connection ~ 7050 3100
Wire Wire Line
	7050 3100 7050 3200
Wire Wire Line
	6300 3700 6400 3700
Wire Wire Line
	6400 4000 6400 3950
Connection ~ 6400 3700
Wire Wire Line
	6400 3700 6600 3700
Wire Wire Line
	7050 3700 7050 3550
Wire Wire Line
	6900 3700 7050 3700
Connection ~ 7050 3550
Wire Wire Line
	7050 3550 7050 3500
$Comp
L Device:C C3
U 1 1 5CBB2756
P 7050 4000
F 0 "C3" H 7165 4046 50  0000 L CNN
F 1 "4u7" H 7165 3955 50  0000 L CNN
F 2 "" H 7088 3850 50  0001 C CNN
F 3 "~" H 7050 4000 50  0001 C CNN
	1    7050 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 3850 7050 3700
Connection ~ 7050 3700
$Comp
L power:GND #PWR010
U 1 1 5CBB501A
P 6400 4400
F 0 "#PWR010" H 6400 4150 50  0001 C CNN
F 1 "GND" H 6405 4227 50  0000 C CNN
F 2 "" H 6400 4400 50  0001 C CNN
F 3 "" H 6400 4400 50  0001 C CNN
	1    6400 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 5CBB505D
P 7050 4250
F 0 "#PWR09" H 7050 4000 50  0001 C CNN
F 1 "GND" H 7055 4077 50  0000 C CNN
F 2 "" H 7050 4250 50  0001 C CNN
F 3 "" H 7050 4250 50  0001 C CNN
	1    7050 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4250 7050 4150
Wire Wire Line
	6400 4400 6400 4300
Wire Wire Line
	5750 3850 6000 3850
Wire Wire Line
	6000 3850 6000 3950
Connection ~ 6400 3950
Wire Wire Line
	6400 3950 6400 3700
Wire Wire Line
	7550 3700 7050 3700
Wire Wire Line
	7550 3300 7550 3700
Wire Wire Line
	5750 3700 6000 3700
Wire Wire Line
	5750 3550 7050 3550
Wire Wire Line
	5750 3400 6700 3400
Wire Wire Line
	5750 3100 6250 3100
Wire Wire Line
	5750 2400 5800 2400
Wire Wire Line
	5800 2400 5800 2000
Connection ~ 5800 2000
Wire Wire Line
	5800 2000 6250 2000
Wire Wire Line
	6000 3950 6400 3950
Wire Wire Line
	7550 3700 7550 4800
Connection ~ 7550 3700
$Comp
L Device:R R9
U 1 1 5CC22F50
P 6250 5000
F 0 "R9" H 6320 5046 50  0000 L CNN
F 1 "33k" H 6320 4955 50  0000 L CNN
F 2 "" V 6180 5000 50  0001 C CNN
F 3 "~" H 6250 5000 50  0001 C CNN
	1    6250 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7300 4850 7300 4750
Wire Wire Line
	7300 4750 6250 4750
Wire Wire Line
	5850 4750 5850 4850
Wire Wire Line
	5850 4750 5850 4450
Connection ~ 5850 4750
Connection ~ 5850 4450
Wire Wire Line
	6250 4850 6250 4750
Connection ~ 6250 4750
Wire Wire Line
	6250 4750 5850 4750
$Comp
L power:GND #PWR012
U 1 1 5CC2C1E0
P 6250 5250
F 0 "#PWR012" H 6250 5000 50  0001 C CNN
F 1 "GND" H 6255 5077 50  0000 C CNN
F 2 "" H 6250 5250 50  0001 C CNN
F 3 "" H 6250 5250 50  0001 C CNN
	1    6250 5250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5CC2C225
P 5850 5250
F 0 "#PWR011" H 5850 5000 50  0001 C CNN
F 1 "GND" H 5855 5077 50  0000 C CNN
F 2 "" H 5850 5250 50  0001 C CNN
F 3 "" H 5850 5250 50  0001 C CNN
	1    5850 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 5250 5850 5150
Wire Wire Line
	6250 5250 6250 5150
Text GLabel 5600 4750 0    50   Input ~ 0
CHRG_EN__BAT_TEMP
Wire Wire Line
	5600 4750 5850 4750
Wire Wire Line
	8000 2000 9300 2000
Connection ~ 8000 2000
Connection ~ 9300 2000
Wire Wire Line
	8550 3800 8550 3700
Wire Wire Line
	8550 3700 7550 3700
Text GLabel 8800 3700 2    50   Input ~ 0
VBAT
Wire Wire Line
	8800 3700 8550 3700
Connection ~ 8550 3700
Wire Notes Line
	7200 4800 7200 5200
Wire Notes Line
	7200 5200 7650 5200
Wire Notes Line
	7650 5200 7650 4800
Wire Notes Line
	7650 4800 7200 4800
$Comp
L obbo_Transistors:TSM2318 Q4
U 1 1 5DDB8B99
P 8550 4700
F 0 "Q4" H 8656 4753 60  0000 L CNN
F 1 "TSM2318" H 8656 4647 60  0000 L CNN
F 2 "obbo_footprints:SOT-23_HandSoldering" H 8650 4950 60  0001 L CIN
F 3 "https://www.taiwansemi.com/products/datasheet/TSM2318_C15.pdf" H 8650 5000 60  0001 L CNN
	1    8550 4700
	-1   0    0    -1  
$EndComp
$Comp
L obbo_Transistors:FDS6975 Q1
U 1 1 5DDD55D3
P 4450 2000
F 0 "Q1" V 4715 2000 60  0000 C CNN
F 1 "FDS6975" V 4609 2000 60  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 4550 2250 60  0001 L CIN
F 3 "https://www.onsemi.com/pub/Collateral/FDS6975-D.PDF" H 4650 2300 60  0001 L CNN
	1    4450 2000
	0    -1   -1   0   
$EndComp
$Comp
L obbo_Transistors:IRF7205 Q2
U 1 1 5DDD6EB0
P 6250 2350
F 0 "Q2" H 6356 2297 60  0000 L CNN
F 1 "IRF7205" H 6356 2403 60  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 6350 2600 60  0001 L CIN
F 3 "http://www.irf.com/part/LEADED-30V-SINGLE-P-CHANNEL-HEXFET-POWER-MOSFET-IN-A-SO-8-PACKAGE/_/A~PB-IRF7205" H 6450 2650 60  0001 L CNN
	1    6250 2350
	1    0    0    1   
$EndComp
$Comp
L obbo_Transistors:IRF7313 Q3
U 1 1 5DDDC7B8
P 6250 2850
F 0 "Q3" H 6356 2903 60  0000 L CNN
F 1 "IRF7313" H 6356 2797 60  0000 L CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 6350 3100 60  0001 L CIN
F 3 "http://www.irf.com/part/LEADED-30V-DUAL-N-CHANNEL-HEXFET-POWER-MOSFET-IN-A-SO-8-PACKAGE/_/A~PB-IRF7313" H 6450 3150 60  0001 L CNN
	1    6250 2850
	1    0    0    -1  
$EndComp
$Comp
L obbo_IC:LTC4011 U1
U 1 1 5DDE41B7
P 5250 3150
F 0 "U1" H 5250 4167 50  0000 C CNN
F 1 "LTC4011" H 5250 4076 50  0000 C CNN
F 2 "obbo_footprints:TSSOP-20_4.4x6.5mm_P0.65mm_EP2.74x3.86mm" H 5250 4150 50  0001 C CIN
F 3 "https://www.analog.com/media/en/technical-documentation/data-sheets/4011fb.pdf" H 5200 3100 50  0001 C CNN
	1    5250 3150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5DDE89F6
P 5250 4150
F 0 "#PWR01" H 5250 3900 50  0001 C CNN
F 1 "GND" H 5255 3977 50  0000 C CNN
F 2 "" H 5250 4150 50  0001 C CNN
F 3 "" H 5250 4150 50  0001 C CNN
	1    5250 4150
	1    0    0    -1  
$EndComp
$EndSCHEMATC
