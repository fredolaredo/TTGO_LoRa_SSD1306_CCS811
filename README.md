# TTGO_LoRa_SSD1306_CCS811

## objectif
Collect Air quality, temp and humidity data


## Send Data
collect CO2 and TVOC data from CCS811

Send data in json format to the broker through LoRa emmitter 

broker forward data to the mosquitto server 

## Display Data
Data transmitted to a mosquitto MQTT Serrver

mosquitto events captured by a Telegraf agent and sent to influxdb database

Data displayed in a chronograf dashboard

## Material 
TTGO board with SSD1306 display and LoRa emmitter

CCS811 on a little board

DHT22 for Temp and humidity data

