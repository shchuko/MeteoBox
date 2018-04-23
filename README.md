
MeteoBox

Simple Arduino-based weather predictor (pressure change tendention calculator).

$

￼

Device functions
• Display current value of atmospheric pressure (in the range from 970 to 1050 hPa)
• Display current atmospheric pressure change tendention (in the range from -225 Pa/h to +225 Pa/h)
• Display of ambient temperature (in the range from +16°C to +28°C)
• Display of archive readings: atmospheric pressure, atmospheric pressure change tendention (readings that was made 60 minutes ago)

Files

 images/ — Device photos

 src/sketch/ — Well-commented source code

 src/libs/ — Arduino libraries

 print_templates/ — Front panel and insulator print templates

 pcbs/ — PCBs drawings, also including components list

Installation
• Install Arduino IDE
• Install Optiboot bootloader into the Arduino software
• Burn Optiboot bootloader onto your Arduino Nano board *
• Install the libraries libs/ into the Arduino software
• Flash the sketch src/sketch/sketch.ino into the Arduino board using Arduino IDE
• Connect the the rest of electronic components from the list pcbs/Full components list.txt . You can use PCBs or Bread board for this. I did not make a circuit, but using PCB drawings it's easy to figure out the connection of electronic components
• Power the board on!

*if you don't have an ability to burn Optiboot bootloader, comment out this lines in the source code:

#include <avr/wdt.h> // Avr watchdog timer library 
wdt_disable( );
wdt_enable( WDTO_8S ); // setting up watchdog timer 
wdt_reset( ); // resetting watchdog timer



Links

https://github.com/Optiboot/optiboot — Optiboot bootloader

http://telegra.ph/MeteoBox-v10-04-11 — User guide with photos (in Russian)

https://www.arduino.cc/en/Main/Software — Arduino Software