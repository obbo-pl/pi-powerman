# Pi-Powerman
Głównym celem jest stworzenie uniwersalnego serwisu działającego z Raspberry Pi pozwalającego na kontrolowanie układu zasilania. Oczywiście układ zasilania musi być zgodny pod względem sposobu komunikacji z serwisem. Obecna wersja wykorzystuje w tym celu I2C oraz dwa porty I/O. W skład rozwiązania wchodzą zatem: serwis uruchomiony na Raspberry oraz współpracujący z nim układ zasilania z mikrokontrolerem. 

Podstawowe możliwości:
•Możliwość łatwego dostosowania układu zasilacza,
•Funkcja UPS, automatyczne przełączenie na zasilanie zapasowe, kontrola stanu naładowania i ładowanie akumulatora
•Automatyczny "shutdown" po zaniku zasilania,
•Do 8 przycisków z diodami LED (1 przycisk zasilania i 7 definiowanych przez użytkownika),
•Możliwość definiowania rozbudowanych scenariuszy dla przycisków,
•LED automatycznie sygnalizują stan przycisków i powiązanych scenariuszy,
•Do 8 załączanych obwodów zasilania (w tym 1 dedykowany dla Raspberry Pi),
•Odcięcie zasilania po zamknięciu systemu (shutdown),
•Monitorowanie napięć, pobieranej mocy, stanu UPS, temperatury i napięcia baterii,
•Eksport monitorowanych wartości do RRD i plików .txt, .csv,
•IoT, wysyłanie komunikatów protokołem MQTT 
•Komunikacja z Raspberry przez I2C i dwa dedykowane porty I/O,
•Dedykowana LED dla sygnalizacji statusu układu zasilania i Raspberry,

Katalog Hardware zawiera przykładowe schematy ukłdów zasilania
KataloF Firmware zawiera orpogramowanie dla przykładowgo hardware

[http://obbo.pl](obbo.pl)
