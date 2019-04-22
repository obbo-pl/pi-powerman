# Pi-Powerman
Głównym celem jest stworzenie uniwersalnego serwisu działającego z Raspberry Pi pozwalającego na kontrolowanie układu zasilania. Oczywiście układ zasilania musi być zgodny pod względem sposobu komunikacji z serwisem. Obecna wersja wykorzystuje w tym celu I2C oraz dwa porty I/O. W skład rozwiązania wchodzą zatem: serwis uruchomiony na Raspberry oraz współpracujący z nim układ zasilania z mikrokontrolerem. 

#### Main features:
- Easy adaptation of the power supply system,
- UPS function, automatic switching to backup power supply, charging status monitoring and battery charging,
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

Katalog Hardware zawiera przykładowe schematy ukłdów zasilania.

Katalog Firmware zawiera orpogramowanie dla przykładowgo hardware.

[obbo.pl](http://obbo.pl)
