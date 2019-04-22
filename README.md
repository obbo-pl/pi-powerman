# Pi-Powerman
  The main goal is to create a universal service operating with Raspberry Pi allowing you to control the power system. Of course, the power supply must be compatible in terms of how to communicate with the service. The current version uses I2C and two I / O ports for this purpose. The solution consists of: a service running on Raspberry and a microcontroller power supply co-operating with it. 

##### Main features:
- Easy adaptation of the power supply system,
- UPS function, automatic switching to backup power supply, battery charging and charging status monitoring,
- Automatic "shutdown" after power loss,
- Up to 8 buttons with LED (1 power button and 7 user defined),
- Ability to define extended scenarios for buttons,
- LEDs automatically indicate the state of buttons and related scenarios,
- Up to 8 connected power circuits (including 1 dedicated for Raspberry Pi),
- Cut off the power after shutting down the system,
- Monitoring of voltage, power consumption, UPS status, temperature and battery voltage,
- Export of monitored values to RRD and .txt files, .csv,
- IoT, sending messages by the MQTT protocol, 
- Communication with Raspberry via I2C and two dedicated I/O ports,
- Dedicated LED for power and Raspberry status indication.

### Requirements:
- Python 3
- Raspbian Jessie or newer

### Installation:
```
sudo apt-get update
sudo apt-get install python3-yaml python3-pip librrd-dev
sudo python3 -m pip install RPi.GPIO smbus rrdtool schedule paho-mqtt

cd /etc/systemd/system/ 
sudo ln -s /usr/local/pi-powerman/pi-powerman.service pi-powerman.service

sudo systemctl daemon-reload
sudo systemctl enable pi-powerman.service
sudo systemctl start pi-powerman.service
```


Katalog Hardware zawiera przykładowe schematy ukłdów zasilania.

Katalog Firmware zawiera orpogramowanie dla przykładowgo hardware.

For more information: [obbo.pl](http://obbo.pl)
