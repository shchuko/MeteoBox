/* MeteoBox  
 *  Version: 0.1
 *  Author: Vladislav Yaroshchuk (Shchuko)
 *  Created: 2018
 *  Website: https://github.com/shchuko 
 *  
 *  26/03/2018 Shchuko: First commit with main configurations
 */

 /* PIN CONFIGURATION
  *   D1:  L PWR 
  *   D0:  L +22*C
  *   D2:  L +-25 Pa/h (also: 1010hPa)
  *   D3:  BUTTON1 (INTERRUPT MODE)
  *   D4:  L +50 Pa/h (also: 1020hPa)
  *   D5:  L +100 Pa/h (also: 1030hPa)
  *   D6:  L +150 Pa/h (also: 1040hPa)
  *   D7:  L +200 Pa/h (also: 1050hPa)
  *   D8:  L -50 Pa/h (also: 1000hPa)
  *   D9:  L -100 Pa/h (also: 990hPa)
  *   D10: L -150 Pa/h (also: 980hPa)
  *   D11: L -200 Pa/h (also: 970hPa)
  *   D12: L +16*C
  *   D13: L +18*C
  *   D14: L +20*C
  *   D15: L +24*C
  *   D16: L +26*C
  *   D17: L +28*C
  *   A4: I2C BUS
  *   A5: I2C BUS
  *   A6: N/C
  *   A7: LDR 
  * L - LED
  * N/C - Not connected
  */

#include <Adafruit_BMP085.h>  //Barometr BMPxxx library

//-----Temperature indication-----
#define L_16C   D12 
#define L_18C   D13
#define L_20C   D14
#define L_22C   D0
#define L_24C   D15
#define L_26C   D16
#define L_28C   D17

//-----Pressure indication----
#define L_25P   D2 
#define L_50P   D4
#define L_100P  D5
#define L_150P  D6
#define L_200P  D7
#define L_50PM  D8
#define L_100PM D9
#define L_150PM D10
#define L_200PM D11

//-----Other connections-----
#define LDR     A7  //Light-dependent resistor
#define BT1     D3  //Controll button
#define L_PWR   D1  //Power LED


void setup() {

}

void loop() {
  

}
