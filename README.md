# MeteoBox
##### Simple Arduino-based weather predictor (pressure change tendention calculator)
------
## Files
`libs/` — Arduino libraries

`pcbs/` — PCBs drawings, also including components list

`print_templates/` — Front panel and insulator print templates

`src/` — Well-commented source code 
## Installition
- Inslall [Arduino IDE](https://www.arduino.cc/en/Main/Software)
- Install [Optiboot bootloader](https://github.com/Optiboot/optiboot#to-install-into-the-arduino-software) into the Arduino software 
- [Burn Optiboot bootloader onto your Arduino Nano board](https://github.com/Optiboot/optiboot#to-burn-optiboot-onto-an-arduino-board)*
- Install the libraries `libs/` into the Arduino software 
- Flash the sketch `src/sketch/sketch.ino` into the Arduino board using Arduino IDE
- Connect the the rest of electronic components from the list `pcbs/Full components list.txt`. You can use PCBs or Bread board for this. I did not make a circuit, but using PCB drawings it's easy to figure out the connection of electronic components
- Power the board on!

*if you don't have an ability to burn Optiboot bootloader, comment out this lines in the source code:
```c
#include <avr/wdt.h> // Avr watchdog timer library 
wdt_disable( );
wdt_enable( WDTO_8S ); // setting up watchdog timer 
wdt_reset( ); // resetting watchdog timer
```

## Links
https://github.com/Optiboot/optiboot — Optiboot bootloader

http://telegra.ph/MeteoBox-v10-04-11 — User guide with photos (in Russian)

https://www.arduino.cc/en/Main/Software — Arduino Software
