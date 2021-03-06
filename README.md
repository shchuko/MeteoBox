# MeteoBox
##### Simple Arduino-based weather predictor (pressure change tendention calculator). 
------
![img_20180410_233451](https://user-images.githubusercontent.com/36963534/38636900-3e3dae22-3dd2-11e8-8c6b-1681ebcb5d30.jpg)
------
## Device functions
- Display current value of atmospheric pressure (in the range from 970 to 1050 hPa)
- Display current atmospheric pressure change tendention (in the range from -225 Pa/h to +225 Pa/h)
- Display of ambient temperature (in the range from +16°C to +28°C)
- Display of archive readings: atmospheric pressure,  atmospheric pressure change tendention (readings that was made 60 minutes ago)

## More information, user guide

[Uploaded to project wiki](https://github.com/shchuko/MeteoBox/wiki)

## Files

`images/` — Device photos

`src/libs/` — Arduino libraries

`src/sketch/` — Well-commented source code 

`pcbs/` — PCBs drawings, also including components list

`print_templates/` — Front panel and insulator print templates

## Installation
- Install [Arduino IDE](https://www.arduino.cc/en/Main/Software)
- Install [Optiboot bootloader](https://github.com/Optiboot/optiboot#to-install-into-the-arduino-software) into the Arduino software 
- [Burn Optiboot bootloader onto your Arduino Nano board](https://github.com/Optiboot/optiboot#to-burn-optiboot-onto-an-arduino-board)*
- Install the libraries `src/libs/` into the Arduino software 
- Flash the sketch `src/sketch/` into the Arduino board using Arduino IDE
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
[Optiboot bootloader](https://github.com/Optiboot/optiboot)

[Arduino Software](https://www.arduino.cc/en/Main/Software)
