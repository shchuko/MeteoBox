/* MeteoBox  
 *  Version: 0.1
 *  Author: Vladislav Yaroshchuk (Shchuko)
 *  Created: 2018
 *  Website: https://github.com/shchuko 
 *  
 *  27/03/2018 Shchuko: Added new class to handy interact with the LEDs. Bug fixes
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

#include <Adafruit_BMP085.h>  // Barometr BMPxxx library

// -----Temperature indication pins-----
#define L_16C   12 
#define L_18C   13
#define L_20C   14
#define L_22C   0
#define L_24C   15
#define L_26C   16
#define L_28C   17

// -----Pressure indication pins----
#define L_25P   2 
#define L_50P   4
#define L_100P  5
#define L_150P  6
#define L_200P  7
#define L_50PM  8
#define L_100PM 9
#define L_150PM 10
#define L_200PM 11

// -----Other-----
#define LDR     A7  // Light-dependent resistor
#define L_PWR   1   // Power LED
#define Pa_to_hPa 0.01 
// button preferences 
#define BT1_INTERRUPT   1       // Controll button interrupt number
#define BT1_TOUCH_MODE  CHANGE  // Controll button interrupt mode
   
// -----Classes------
class LedString { // LED string class 
                  // Contains methods for controlling the 
                  // LED string connected to the digital pins (max is 255)
private:

  uint8_t*  LED_string  = NULL;
  uint8_t*  pins = NULL;
  uint8_t   count = 0;

  uint8_t byte_num( uint8_t pin_num ) { // byte number in allocated memory
    pin_num++;
    return ( pin_num / 8 + ( pin_num / 8.0 > pin_num / 8 ) - 1 );
  }

  uint8_t bit_pos( uint8_t pin_num ) {  // bit position in byte 
    return ( pin_num - pin_num / 8 * 8 ); 
  }
  
public: 

  LedString( uint8_t _count ) {
    count = _count; // saving the count of LEDs
    pins = new uint8_t[ count ];  // allocate memory for pins numbers

    uint8_t byte_count = _count / 8 + ( _count / 8.0 > _count / 8 );  // calculating a sufficient amount of memory (in bytes)
    LED_string = new uint8_t[ byte_count ]; // allocate memory for pins states
    
    for ( uint8_t i = 0; i < byte_count; i++ ) // clearing the memory
      LED_string[ i ] = 0;  
  } 
  
  void init_pin( uint8_t pin_arduino, uint8_t pin_num ) {
    pinMode( pin_arduino, OUTPUT ); // set pin mode as OUTPUT 
    digitalWrite( pin_arduino, 0 ); // set pin state as "0"
   
    pins[ pin_num ] = pin_arduino;
  }
  
  bool get_pin( uint8_t pin_num ) { // returns pin state
    return ( ( LED_string[ byte_num( pin_num ) ] >> bit_pos( pin_num ) ) & ( 1 << 0 ) );
  }
 
  void set_pin( uint8_t pin_num, bool pin_state ) { // set state for pin
    bool old_pin_state = get_pin( pin_num );
    
    if ( pin_state != old_pin_state ) {
      digitalWrite( pins[ pin_num ], pin_state );
      
      if ( pin_state )  // if new state is "1" writing...
        LED_string[ byte_num( pin_num ) ] |= ( 1 << bit_pos( pin_num ) );
      else  // else if new state is "0" writing...
        LED_string[ byte_num( pin_num ) ] &= ~( 1 << bit_pos( pin_num ) );
    }
    
  }

  uint8_t get_byte( uint8_t _byte_num ) { // returns full byte
    return LED_string[ _byte_num ];
  }
  
  void set_byte( uint8_t _byte_num, uint8_t new_byte ) { // set full byte 
    if ( new_byte != LED_string[ _byte_num ] ) {
      LED_string[ _byte_num ] = new_byte;
      for ( uint8_t i = 0; i < 8; i++ )
        digitalWrite( pins[ 8*_byte_num + i ],(( new_byte >> i ) & ( 1 << 0 )) );
    }
  }
  
};

class MedianFilter {  //Median filter class
private:
  int32_t *values = NULL; // pointer at the first saved value
  uint8_t val_count = 0; // count of values
  
  uint8_t new_val_pos = 0;  // position of new value in values[ ] ( used in function new_val )
  
  int32_t result = 0; // result value 

  void calculate( ) { // calculating median value
    // bubble sort algorithm
    for( uint8_t i = 0; i < val_count - 1; i++ )  
      for( uint8_t j = i + 1; j < val_count; j++ )
      { 
        if( values[ i ] > values[ j ] )
        {
          int32_t term_val = values[ i ];
          values[ i ] = values[ j ];
          values[ j ] = term_val;
        }
      }
    result = values[ ( uint8_t )(( val_count - 1 ) / 2 ) ]; // saving median value
  }
  
public:
  MedianFilter( uint8_t _val_count ) {
    val_count = _val_count; // saving the count of values
    values = new uint32_t[ _val_count ]; // allocate memory for saving values    
  }

  void init( int32_t val ) {
    for ( uint8_t i = 0; i < val_count; i++ )
      values[i] = val;
    result = val;
  }
  bool new_val( int32_t val ) { // updating (saving) new value
                                        // returning 1 - updating complete, 0 - updating not complete 
      values[ new_val_pos ] = val; // saving new value
      new_val_pos++; // incriminate value pos counter
    if ( new_val_pos == val_count )  // if all values saved
    {
      new_val_pos = 0; 
      calculate( ); // calculating median value
      return 1; // returning 1
    } 
    // else 
    return 0; // returning 0
  } 

  int32_t get_result( ) {  // returning calculated median value
    return result;
  }

};

// -----Objects------
LedString prs_led(9); // pressure indication string
LedString temp_led(7);  // temperature indication string

Adafruit_BMP085 bmp;  // BMPxxx sensor

MedianFilter avr_pressure( 10 );  // median filter for BMP180 barometer data
MedianFilter avr_luminosity( 10 );  // median filter for light-dependent resistor data

// -----Global variables-----
uint16_t  pressure = 0;       // current pressure in hPa
int16_t   temperature = 0;    // current temperature in *C

uint16_t  luminosity = 1023;  // current illumination value (from ADC: 0..1023)

bool      night_mode = 0;     // night mode flag (0 - LED strings ON, 1 - LED strings OFF)

bool      prs_disp_mode = 0;  // pressure LED string display mode
                              // 0 - current pressure (hPa), 1 - pressure change speed (Pa/h) 
 
// -----Functions-----
void button_touch( ) {
  
}

uint16_t pressure_upd( ) {  // pressure update function
  while ( !avr_pressure.new_val( bmp.readPressure( ) ) ) { }      //updating pressure
  return ( uint16_t )( avr_pressure.get_result( )  * Pa_to_hPa ); //using median filter
}

void luminosity_upd( uint16_t& _luminosity ) {  // luminosity update function
  static uint64_t time_counter = millis( );  // saving current millis( ) value
  
  if ( millis( ) < time_counter ) // if interal arduino millis counter was resetted
    time_counter = millis( ); //  resetting luminosity update time counter
      
  if ( millis( ) - time_counter > 300 ) // if 300ms was left
  { 
    time_counter = millis( ); // updating luminosity update time counter
    if ( avr_luminosity.new_val( analogRead( LDR ) ) ) //updating luminosity
      _luminosity = avr_luminosity.get_result( );      //using median filter
  }
}

void setup( ) {
  bmp.begin( );
  // initialisation temperature indication pins
  temp_led.init_pin( L_16C, 0 );
  temp_led.init_pin( L_18C, 1 );
  temp_led.init_pin( L_20C, 2 );
  temp_led.init_pin( L_22C, 3 );
  temp_led.init_pin( L_24C, 4 );
  temp_led.init_pin( L_26C, 5 );
  temp_led.init_pin( L_28C, 6 );

  // initialisation pressure indication pins
  prs_led.init_pin( L_200PM, 0 );
  prs_led.init_pin( L_150PM, 1 );  
  prs_led.init_pin( L_100PM, 2 );
  prs_led.init_pin( L_50PM, 3 );
  prs_led.init_pin( L_25P, 4 );
  prs_led.init_pin( L_50P, 5 );
  prs_led.init_pin( L_100P, 6 );
  prs_led.init_pin( L_150P, 7 );
  prs_led.init_pin( L_200P, 8 );

  // set pin mode for power LED pin
  pinMode( L_PWR, OUTPUT );
  
  avr_pressure.init( bmp.readPressure( ) );
  avr_luminosity.init( analogRead( A7 ) );
  
  pressure = ( uint16_t )( avr_pressure.get_result( )  * Pa_to_hPa );  // saving current pressure 
  temperature = bmp.readTemperature( ); // saving current temperature
  luminosity = avr_luminosity.get_result( ); //saving current luminosity
  
  //  indication testing:
  for ( uint8_t i = 0; i < 9; i++ ) { // pressure indication string - turning on
     prs_led.set_pin( i, 1 );
     delay( 100 );
  }

  for ( uint8_t i = 0; i < 7; i++ ) { // temperature indication string - turning on
     temp_led.set_pin( i, 1 );
     delay( 100 );
  }

  digitalWrite( L_PWR, 1 ); // turning on PWR LED

  for ( uint8_t i = 0; i < 9; i++ ) { // pressure indication string - turning off
     prs_led.set_pin( i, 0 );
     delay( 100 );
  }

  for ( uint8_t i = 0; i < 7; i++ ) { // temperature indication string - turning off
     temp_led.set_pin( i, 0 );
     delay( 100 );
  }

  digitalWrite( L_PWR, 0 ); // turning off PWR LED
  
  //  attach interrupt for control button
  attachInterrupt( BT1_INTERRUPT, button_touch, BT1_TOUCH_MODE );
}

void loop( ) { 
  luminosity_upd( luminosity ); //updating luminosity
  
  if ( luminosity < 95 )
    digitalWrite( L_PWR, 0 );
  else if ( luminosity > 127 )
    digitalWrite( L_PWR, 1 );
  else {} 

}
