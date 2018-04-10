/* MeteoBox  
 *  Version: 0.9
 *  Author: Vladislav Yaroshchuk (Shchuko)
 *  Created: 2018
 *  Website: https://github.com/shchuko 
 *  
 *  10/04/2018 shchuko: LED indicastion display configured:
 *                     -switching 
 *                     -auto-returning to current data display
 *                     -waking from the night mode if button have touched             
 */

/* PIN CONFIGURATION
 *   D1:  L PWR 
 *   D0:  L +22*C
 *   D2:  L +-25 Pa/h (also: 1010hPa)
 *   D3:  TTP223 BUTTON 
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
 *   A7: Light-dependent resistor (LDR)
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

// -----Pressure forecast-----
#define FORECAST_VAL_NUMBER       7   // Number of saved values
#define DATA_UPDATE_INTERVAL      5   // ... in minutes 
#define FORECAST_UPDATE_INTERVAL  10  // ... in minutes
                                      // Condition: FORECAST_UPDATE_INTERVAL % DATA_UPDATE_INTERVAL = 0 !!!

// -----Other-----
#define LDR     A7  // Light-dependent resistor
#define L_PWR   1   // Power LED
#define Pa_to_hPa 0.01 

// -----Controll button preferences-----
#define BT1_PIN  3 // Interrupt number
#define L_PWR_BTN_BLINK_NUMBER 4 // PWR LED blinks count if button was touched

// -----Classes------
class LedString { // LED string class 
                  // Contains methods for controlling the 
                  // LED string connected to the digital pins (max is 255)
private:

  uint8_t*  LED_string  = NULL;
  uint8_t*  pins = NULL;
  uint8_t   number = 0;

  uint8_t byte_num( uint8_t pin_num ) { // byte number in allocated memory
    pin_num++;
    return ( pin_num / 8 + ( pin_num / 8.0 > pin_num / 8 ) - 1 );
  }

  uint8_t bit_pos( uint8_t pin_num ) {  // bit position in byte 
    return ( pin_num - pin_num / 8 * 8 ); 
  }
  
public: 

  LedString( uint8_t _number ) {
    number = _number; // saving the number of LEDs
    pins = new uint8_t[ number ];  // allocate memory for pins numbers

    uint8_t byte_number = _number / 8 + ( _number / 8.0 > _number / 8 );  // calculating a sufficient number of memory (in bytes)
    LED_string = new uint8_t[ byte_number ]; // allocate memory for pins states
    
    for ( uint8_t i = 0; i < byte_number; i++ ) // clearing the memory
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
        if ( 8 * _byte_num + i < number )
          digitalWrite( pins[ 8 * _byte_num + i ],(( new_byte >> i ) & ( 1 << 0 )) );
        else
          break;
    }
  }

  void on( ) {
    for ( uint8_t i = 0; i < number; i++ ) {
      if ( !(i % 8) ) 
         LED_string[ i / 8 ] = 0xFF;
      digitalWrite( pins[ i ],  1 );
    }
  }

  void off( ) {
    for ( uint8_t i = 0; i < number; i++ ) {
      if ( !(i % 8) ) 
         LED_string[ i / 8 ] = 0;
      digitalWrite( pins[ i ],  0 );
    }
  }
  
};

class MedianFilter {  // Median filter class ( all arguments > 0 )
private:
  uint32_t *values = NULL; // pointer at the first saved value
  uint8_t val_number = 0; // number of values
  
  uint8_t new_val_pos = 0;  // position of new value in values[ ] ( used in function new_val )
  
  uint32_t result = 0; // result value 

  void calculate( ) { // calculating median value
    // bubble sort algorithm
    for( uint8_t i = 0; i < val_number - 1; i++ )  
      for( uint8_t j = i + 1; j < val_number; j++ )
      { 
        if( values[ i ] > values[ j ] )
        {
          uint32_t term_val = values[ i ];
          values[ i ] = values[ j ];
          values[ j ] = term_val;
        }
      }
    result = values[ ( uint8_t )(( val_number - 1 ) / 2 ) ]; // saving median value
  }
  
public:
  MedianFilter( uint8_t _val_number ) {
    val_number = _val_number; // saving the number of values
    values = new uint32_t[ _val_number ]; // allocate memory for saving values    
  }

  void init( uint32_t val ) {
    for ( uint8_t i = 0; i < val_number; i++ )
      values[i] = val;
    result = val;
  }
  
  bool new_val( uint32_t val ) { // updating (saving) new value
                                        // returning 1 - updating complete, 0 - updating not complete 
      values[ new_val_pos ] = val; // saving new value
      new_val_pos++; // incriminate value pos counter
    if ( new_val_pos == val_number )  // if all values saved
    {
      new_val_pos = 0; 
      calculate( ); // calculating median value
      return 1; // returning 1
    } 
    // else 
    return 0; // returning 0
  } 

  uint32_t get_result( ) {  // returning calculated median value
    return result;
  }

};

class PressureForecast {
private:
  uint32_t   *pressure = NULL;        // pointer for dynamic memory allocation
  int16_t    change_hour;             // pressure change forecast (Pa/h)
  uint8_t    measurement_number;       // measurement number used for calculating forecast
  uint16_t   measurement_interval;    // pressure update interval (in minutes)
  uint8_t    meas_updts_counter = 1;  // pressure updates counter
  
  void calculate( ) {   // pressure change calculatioin
                        // using least squares fit
    uint32_t sumx = 0;  
    uint32_t sumy = 0;
    uint32_t sumx2 = 0;
    uint32_t sumxy = 0;
    for ( uint8_t i = 0; i < measurement_number; i++ ) {
      sumx += i;
      sumy += pressure[ i ];
      sumx2 += i * i;
      sumxy += i * pressure[ i ];
    }
    double a;
    a = measurement_number * sumxy;
    a -= sumx * sumy;
    a = ( double )a / ( measurement_number * sumx2 - sumx * sumx );
    change_hour = round( a * 60 / measurement_interval ); // pressure change forecast (Pa/h)
  }
  
public:
  void set( uint32_t _pressure ) {  // set new pressure value
    for ( uint8_t i = 0; i < measurement_number - 1; i++ ) // shifting old saved values
      pressure[ i ] = pressure[ i + 1 ];  
    pressure[ measurement_number - 1 ] = _pressure;  // saving new value

    calculate( ); // re-calculating  forecast
    
    if ( meas_updts_counter < measurement_number )  // if full update did not complete
      meas_updts_counter++; 
  }

  bool is_update( ) {  // returning 1 if full update completed
    return ( meas_updts_counter >= measurement_number );
  }
  
  uint32_t get_pressure( ) { // returning last saved measurement
    return pressure[ measurement_number - 1 ];
  }

  int32_t get_forecast( ) { // returning pressure change forecast (Pa/h)
    return change_hour;
  }
  
  void begin( uint32_t _pressure )  // set first pressure value (in Pa)
  {
    for ( uint8_t i = 0; i < measurement_number; i++ )
      pressure[ i ] = _pressure;
  }
  
  PressureForecast( uint8_t  _measurement_number,         // measurement number used for calculating forecast
                     uint16_t  _measurement_interval ) {  // pressure update interval (in minutes)
    pressure = new uint32_t [ _measurement_number ];
    measurement_number = _measurement_number;
    measurement_interval = _measurement_interval;
  }  
};

class TTP223 {  // TTP223 sensor click handler
                // based on  millis( ) func.
                // Three modes of recognition of button pressing:
                // [short click], [double click], [long press]
private:
  void ( *long_press )( void );    // pointer to a long-press action function 
  void ( *short_click )( void );   // pointer to a short-click action function
  void ( *double_click )( void );  // pointer to a double-click action function
    
  uint8_t pin = 0;  // pin number

  bool touch_flag = 0;  
  bool double_click_flag = 0;
  
  bool last_state = 0;
  
  uint64_t press_time = 0; // last button touching time 

public:
  
  TTP223( uint8_t _pin,                     // interrupt number (0 or 1)
          void ( *_short_click )( void ),   // pointer to the button short-click function
          void ( *_double_click )( void ),  // pointer to the button double-click function
          void ( *_long_press )( void ) ) { // pointer to the button long-press function
    short_click  = _short_click; 
    double_click = _double_click;
    long_press   = _long_press;
   
    pin = _pin;
  }

  void loop_func( ) { // button click handler
    if ( digitalRead( pin ) != last_state ) { // if buton pin state have changed
      last_state = !last_state; // saving new state

      if ( last_state ) { // if button pressed
        press_time = millis( ); // saving button press time
        touch_flag = 1; // press (click) flag
      } 
    }
    
    if ( touch_flag && millis( ) - press_time > 700 ) {  // if 700 ms least after last click
      long_press( );  // long-press function
      touch_flag = 0; 
    }

    if ( double_click_flag && millis() - press_time  > 400 ) {  // if double-click time has expired 
                                                                // (it's not enough time to double-click
                                                                // - more than 400 ms left)
      double_click_flag = 0;                                   
                
      short_click( ); // short-click function
    }
    
    if ( touch_flag ) // if the button was pressed
      if ( double_click_flag ) {  // if double-click is possible
        double_click_flag = 0;  
        touch_flag = 0; 
        double_click( );  // double-click function
        
      } else if ( !last_state ) { // else if button was released
        touch_flag = 0;
          
        if ( millis( ) - press_time <= 400 ) // if it's enough time to double-click 
          double_click_flag = 1;  // setting up double-click flag
        else // else if it's not enough time to double-click
          short_click( ); // short-click function
      }
  }
};

// -----Button functions-----
void button_long_press( );
void button_short_click( );
void button_double_click( );

// -----Objects------
LedString prs_led( 9 ); // pressure indication string - 9 LEDs
LedString temp_led( 7 );  // temperature indication string - 7 LEDs

Adafruit_BMP085 bmp;  // BMPxxx sensor
PressureForecast forecast( FORECAST_VAL_NUMBER, FORECAST_UPDATE_INTERVAL );  //  pressure change forecast

MedianFilter avr_pressure( 10 );  // median filter for BMP180 barometer data - 10 values save
MedianFilter avr_luminosity( 10 );  // median filter for light-dependent resistor data - 10 values save

TTP223 button( BT1_PIN, button_short_click, button_double_click, button_long_press ); // TTP223 button

// -----Global variables-----
uint32_t pressure = 0;       // current pressure in hPa
int16_t  temperature = 0;    // current temperature in *C
uint16_t luminosity = 1023;  // current illumination value (from ADC: 0..1023)

bool     _forecast_actual_flag = 0;      // forecast actual flag
uint8_t  _forecast_range = 4;            // forecast range (for LED string)
uint8_t  _forecast_range_blink_mode = 4; // forecast blink mode 

uint8_t  _forecast_h_ago_range = 4;            // forecast range 1 hour ago (for LED string)
uint8_t  _forecast_h_ago_range_blink_mode = 4; // forecast blink 1 hour ago mode 

uint8_t  _pressure_range = 4;            // pressure range (for LED string)
bool     _pressure_range_blink_flag = 0; // pressure range blink flag

uint8_t  _pressure_h_ago_range = 4;            // pressure range 1 hour ago (for LED string)
bool     _pressure_h_ago_range_blink_flag = 0; // pressure range 1 hour ago blink flag

uint8_t  _temp_range = 3;            // temperature range (for LED string)
bool     _temp_range_blink_flag = 0; // temperature range blink flag

bool night_mode_enable = 1; // night mode flag (0 - enable, 1 - disable)
bool night_mode_state = 0;  // current situation flag (0 - LEDs enable, 1 - LEDs disable)

bool     temprary_touch_awake = 0; // system awaking from the night mode if button touched
uint64_t temprary_touch_awake_counter = 0;  

bool display_mode = 0;  // genegal diplay mode
                        // 0 - current data, 1 - data 1h ago 
bool prs_disp_mode = 0; // pressure display mode
                        // 0 - current pressure (hPa), 1 - pressure forecast display (Pa/h)

uint64_t display_return_counter = 0;  // time counter to return to current 
                                      // data display & current pressure display
uint8_t  display_return_mode = 0; // return mode:
                                  // 0 - disable
                                  // 1 - return from forecast display to the current pressure display
                                  // 2 - return from 1h ago data display to the current data display

uint64_t button_blink_counter_time = 0; // button touch - PWR LED blink time counter
uint8_t  button_blink_counter = L_PWR_BTN_BLINK_NUMBER; // button touch - PWR LED blink(s) counter - max is L_PWR_BTN_BLINK_NUMBER

// -----Functions-----
void serial_test_func( ) {
  Serial.println( );
  Serial.println( "******************************" );
  Serial.println( "Data update..." );
  Serial.println( );
  
  Serial.print( "Pressure: ");
  Serial.println( pressure );
  Serial.print(" Pressure range: ");
  Serial.println( _pressure_range );
  Serial.print(" Pressure range blink flag: ");
  Serial.println( _pressure_range_blink_flag );

  Serial.print( "Temperature: ");
  Serial.println( temperature );
  Serial.print(" Temperature range: ");
  Serial.println( _temp_range );
  Serial.print(" Temperature range blink flag: ");
  Serial.println( _temp_range_blink_flag );

  Serial.print( "Forecast: " );
  Serial.println( forecast.get_forecast( ) );
  Serial.print( " Forecast actual flag: " );
  Serial.println( _forecast_actual_flag );
  Serial.print( " Forecast range: " );
  Serial.println( _forecast_range );
  Serial.print( " Forecast range blink mode: " );
  Serial.println( _forecast_range_blink_mode );
  
}

void serial_flag_test_func( ) {
  Serial.println( );
  Serial.println( "******************************" );
  Serial.println( "Button touched..." );
  Serial.println( );
  
  Serial.print( "Night mode enable flag: " );
  Serial.println( night_mode_enable );

  Serial.print( "Display mode flag: " );
  Serial.println( display_mode );
  
  Serial.print( "Pressure display mode flag: " );
  Serial.println( prs_disp_mode );
}

void button_short_click( ) {
  button_blink_counter = 0; // setting buton blink counter to zero
  digitalWrite( L_PWR, 1 ); // switching on PWR LED
  
  if ( !night_mode_state || temprary_touch_awake )  // if LEDs are enabled or it is temprary awake
  {
    prs_disp_mode = !prs_disp_mode;
    
    if ( display_mode ) { // if it is 1h ago data display 
      
      display_return_mode = 2;  // changing return timer mode 
      display_return_counter = millis( ); // updating time counter
    } 
  
    if ( prs_disp_mode && !display_mode ) {  // if it's switching to the forecast display mode within current data display
      display_return_mode = 1;  // changing return timer mode 
      display_return_counter = millis( ); // updating time counter
    } else if ( display_return_mode && !display_mode ) {  // else if it's current data display within enabled return timer
      display_return_mode = 0;  // disabling timer
    }

    // if it's it is temprary awake, updating awake timer
    if ( temprary_touch_awake ) temprary_touch_awake_counter = millis( );
    
  } else if ( night_mode_state && night_mode_enable ) { // else if luminosity is low, LEDs are disabled
    display_mode = 0; // switching to the  current data fisplay
    prs_disp_mode = 0;  // switching pressure display mode to the pressure mode
    
    temprary_touch_awake = 1; // waking up system for several seconds
    temprary_touch_awake_counter = millis( );
  } 
  
  temp_led.off( );  // clearing temperature LED string
  prs_led.off( );   // clearing pressure LED string
  //serial_flag_test_func( );
}

void button_double_click( ) { 
  button_blink_counter = 0; // setting buton blink counter to zero
  digitalWrite( L_PWR, 1 ); // switching on PWR LED

  if ( !night_mode_state || temprary_touch_awake )  // if LEDs are enabled or it is temprary awake
  {
    prs_disp_mode = 0;  // switching pressure display mode to a pressure mode
    
    if ( display_mode = !display_mode ) { // if it is switching to 1h ago data display 
      display_return_mode = 2;  // changing return timer mode 
      display_return_counter = millis( ); // updating time counter
    } else {
      display_return_mode = 0;
    }

    // if it's it is temprary awake, updating awake timer
    if ( temprary_touch_awake ) temprary_touch_awake_counter = millis( );
    
  } else if ( night_mode_state ) { // else if luminosity is low, LEDs are disabled
    display_mode = 0; // switching to the  current data fisplay
    prs_disp_mode = 0;  // switching pressure display mode to the pressure mode
    
    temprary_touch_awake = 1; // waking up system for several seconds
    temprary_touch_awake_counter = millis( );
  } 
  
  temp_led.off( );  // clearing temperature LED string
  prs_led.off( );   // clearing pressure LED string
  //serial_flag_test_func( );
}

void button_long_press( ) {
  button_blink_counter = 0; // setting buton blink counter to zero
  digitalWrite( L_PWR, 1 ); // switching on PWR LED
  
  if ( !night_mode_state || temprary_touch_awake )  // if LEDs are enabled or it is temprary awake
  {
    if ( !display_mode ) {  // if it's current data display
      if ( night_mode_enable = !night_mode_enable ) // changing night mode
        temprary_touch_awake = 0; // if it's swithching off night mode, disabling temprary awake timer
    } else {  // else if it's not current data display
      display_mode = 0; // switching to a "current data" display
    }

    // if it's it is temprary awake, updating awake timer
    if ( temprary_touch_awake ) temprary_touch_awake_counter = millis( );
    
  } else if ( night_mode_state ) { // else if luminosity is low, LEDs are disabled
    display_mode = 0; // switching to the  current data fisplay
    prs_disp_mode = 0;  // switching pressure display mode to the pressure mode
    
    night_mode_enable = 0; // disabling night mode
    
    temprary_touch_awake = 1; // waking up system for several seconds
    temprary_touch_awake_counter = millis( ); // updating awake timer
  } 
  
  temp_led.off( );  // clearing temperature LED string
  prs_led.off( );   // clearing pressure LED string
  
  //serial_flag_test_func( );
}

uint32_t pressure_upd( ) {  // pressure update function
  while ( !avr_pressure.new_val( bmp.readPressure( ) ) ) { } //updating pressure
  uint32_t res = avr_pressure.get_result( );
  return avr_pressure.get_result( ); //using median filter
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

uint8_t forecast_range( int16_t change_hour, uint8_t& blink_mode ) {  // returning the range of pressure change 
                                                                      // LED string func
  blink_mode = 0; // 0 - LED blink off
                  // 1 - basic one LED blink
                  // 2 - left LED blink
                  // 3 - right LED blink
                  // 4 - right&left LEDs blink
                  // 5 - full LED string blink
  if ( change_hour <= -200 ) {
    if ( change_hour <= -225 )
      blink_mode = 5; // full LED string blink
    return 0;
  }
  
  if ( change_hour <= -150 ) {
    if ( change_hour <= -175 )
      blink_mode = 1; // basic one LED blink
    return 1;
  }
  
  if ( change_hour <= -100 ) {
    if ( change_hour <= -125 )
      blink_mode = 1; // basic one LED blink
    return 2;
  }
  
  if ( change_hour <= -50 ) {
    if ( change_hour <= -75 )
      blink_mode = 1; // basic one LED blink
    return 3;
  }
  
  if ( change_hour > -50 && change_hour < 50 ) {
    if ( change_hour <= -25 )
      blink_mode = 2; // left LED blink
    else if ( change_hour >= 25 )
      blink_mode = 3; // right LED blink
    else
      blink_mode = 4; // right&left LEDs blink
    return 4;
    
  }
  
  if ( change_hour >= 200 ) {
    if ( change_hour >= 225 )
      blink_mode = 5; // full LED string blink
    return 8;
  }

  if ( change_hour >= 150 ) {
    if ( change_hour >= 175 )
      blink_mode = 1; // basic one LED blink
    return 7;
  }
  
  if ( change_hour >= 100 ) {
    if ( change_hour >= 125 )
      blink_mode = 1; // basic one LED blink
    return 6;
  }

  if ( change_hour >= 50 ) {  
    if ( change_hour >= 75 )
      blink_mode = 1; // basic one LED blink
    return 5;
  }  
}

uint8_t temp_range( int16_t temp, bool& blink_flag ) {  // returning the range of temperature
                                                        // LED string func
  blink_flag = 0; // LED blink off
  
  if ( temp <= 16 ) {
    if ( temp < 16 )
      blink_flag = 1; // 16*C LED blink enable
    return 0; // 16*C LED enable
  }
    
  if ( temp <= 18 )
    return 1; // 18*C LED enable
    
  if ( temp <= 20 )
    return 2; // 20*C LED enable
    
  if ( temp > 20 && temp < 24 )
    return 3; // 22*C LED enable
    
  if ( temp >= 28 ) {
    if ( temp > 28 )
      blink_flag = 1; // 28*C LED blink enable
    return 6; // 28*C LED enable
  }
    
  if ( temp >= 26 )
    return 5; // 26*C LED enable
      
  if ( temp >= 24 )
    return 4; // 24*C LED enable
}

uint8_t pressure_range( uint16_t pressure_hPa, bool& blink_flag ) { // returning the range of pressure
                                                                    // LED string func
  blink_flag = 0; // LED blink off
  
  if ( pressure_hPa < 975 ) {
    if ( pressure_hPa < 970 )
      blink_flag = 1; // 970 hPa LED blink enable
    return 0; // 970 hPa LED enable
  }
    
  if ( pressure_hPa < 985 )
    return 1; // 980 hPa LED enable
    
  if ( pressure_hPa < 995 )
    return 2; // 990 hPa LED enable
    
  if ( pressure_hPa < 1005 )
    return 3; // 1000 hPa LED enable
    
  if ( pressure_hPa >= 1005 && pressure_hPa < 1015 )
    return 4; // 1010 hPa LED enable
      
  if ( pressure_hPa > 1045 ) {
    if ( pressure_hPa > 1050 )
      blink_flag = 1; // 1050 hPa LED blink enable
    return 8; // 1050 hPa LED enable
  }
    
  if ( pressure_hPa >= 1035 )
    return 7; // 1040 hPa LED enable
    
  if ( pressure_hPa >= 1025 )
    return 6; // 1030 hPa LED enable
    
  if ( pressure_hPa >= 1015 )
    return 5; // 1020 hPa LED enable
}

void bootanimation( ) {
  digitalWrite( L_PWR, 1 );
  delay( 500 );
  
  temp_led.on( );
  delay( 500 );
  
  prs_led.on( );
  delay( 500 );

  prs_led.off( );
  delay( 500 );
  
  temp_led.off( );
  delay( 500 );

  for ( uint8_t i = 0; i < 15; i++ ) {
  digitalWrite( L_PWR, 0 );
  delay( 100 );

  digitalWrite( L_PWR, 1 );
  delay( 100 );
  }
}

void pressure_display( uint8_t range, bool blink_flag, bool blink_state ) {
  uint8_t blink_led = 9 * ( range != 0 && range != 8 ) + 8 * ( range == 8 );

  for ( uint8_t i = 0; i < 9; i++ ) 
    if ( i != range && i != blink_led && prs_led.get_pin( i ) )
      prs_led.set_pin( i , 0 ); 
  
  if ( blink_flag ) {
    prs_led.set_pin( blink_led, blink_state );    
  } else {
    prs_led.set_pin( range, 1 );
  }
}

void forecast_display( uint8_t range, uint8_t blink_mode,  bool blink_state ) {
  uint16_t led_indic_string = 0x0;

  for ( uint8_t i = 0; i < 9; i++ ) {
    if ( range <= 4 ) {
      
      if ( i <= 4 && i >= range ) 
        led_indic_string |= ( 1 << i );
    
    } else if ( range > 4 ) {
      
      if ( i >= 4 && i <= range )
        led_indic_string |= ( 1 << i );
    }
  }

  uint8_t blink_led = 9;
  switch ( blink_mode ) {
    case 1:
      if ( range < 4 )
        blink_led = range - 1;
      else 
        blink_led = range + 1;
      break;
    
    case 2:
      blink_led = range - 1;
      break;
    
    case 3:
      blink_led = range + 1;
      break;

    case 4:
      if ( blink_state ) {
        led_indic_string |= ( 1 << 3 ) | ( 1 << 5 );
      }
      break;

    case 5:
      if ( blink_state ) {
        led_indic_string = 0x0;
        led_indic_string |= ( 1 << 4 );
      }
      break;
  }

  if ( blink_led < 9 && blink_state )
    led_indic_string |= ( 1 << blink_led );
  else
    led_indic_string &= ~( 1 << blink_led );

  if ( !_forecast_actual_flag && !blink_state )
     led_indic_string &= ~( 1 << 4 );
     
  prs_led.set_byte( 0, ( led_indic_string & ( 0xFF ) ) );
  prs_led.set_byte( 1, ( ( led_indic_string >> 8 ) & ( 0xFF ) ) );
  
}

void temp_display( uint8_t range, bool blink_flag, bool blink_state ) {
  uint8_t blink_led = 7 * ( range != 0 && range != 6 ) + 6 * ( range == 6 );

  for ( uint8_t i = 0; i < 7; i++ ) 
    if ( i != range && i != blink_led && temp_led.get_pin( i ) )
      temp_led.set_pin( i , 0 ); 
  
  if ( blink_flag ) {
    temp_led.set_pin( blink_led, blink_state );    
  } else {
    temp_led.set_pin( range, 1 );
  }   
}

void history_display( bool blink_state ) {
  temp_led.set_pin( 0, 1 );
  temp_led.set_pin( 2, 1 );
  temp_led.set_pin( 4, 1 );
  temp_led.set_pin( 6, 1 );
}

void led_display( ) {
  static bool leds_disabled = 0;
  static bool blink_led_state = 1;  // flag used for display blinking
  static uint64_t blink_time_counter = millis( );

  if ( temprary_touch_awake ) // if it was temprary awaiking
    if ( millis( ) - temprary_touch_awake_counter >= 10000 ) // and 7 seconds left
    {
      display_mode = 0; // switching to the  current data fisplay
      prs_disp_mode = 0;  // switching pressure display mode to the pressure mode
    
      temprary_touch_awake = 0; // disabling temprary awaiking mode
    }

  // return to current data display & current pressure display:
  if ( display_return_mode == 1 ) { // from forecast display
    if ( millis( ) - display_return_counter >= 5000 ) // if 5 seconds left
    {                                                 // returning
      display_mode = 0; // switching to the  current data fisplay
      prs_disp_mode = 0;  // switching pressure display mode to the pressure mode
      
      display_return_counter = 0;
    }
  } else if ( display_return_mode == 2 ) {  //  from 1h ago data display
    if ( millis( ) - display_return_counter >= 7500 ) // if 7,5 seconds left
    {                                                 // returning
      display_mode = 0; // switching to the  current data fisplay
      prs_disp_mode = 0;  // switching pressure display mode to the pressure mode
      
      display_return_counter = 0;
    }
  }
      
  if ( millis( ) - blink_time_counter >= 800 ) {  // if it's time to blink 
    blink_time_counter = millis( ); // saving current time
    blink_led_state = !blink_led_state; // inverting flag

    // night mode enable indication:
    if ( button_blink_counter == L_PWR_BTN_BLINK_NUMBER )  
      if ( !night_mode_enable ) 
        digitalWrite( L_PWR, blink_led_state );
      else 
        digitalWrite( L_PWR, 1 );
  }

  // PWR LED blink algorithm
  if ( button_blink_counter < L_PWR_BTN_BLINK_NUMBER ) 
    if ( millis( ) - button_blink_counter_time > 100 ) {
      digitalWrite( L_PWR, !digitalRead( L_PWR ) );
      button_blink_counter_time = millis( );
      button_blink_counter++;
    }

  // Algorithm of automatic switching LED indication while night mode enabled/disabled
  if ( night_mode_enable && ( temprary_touch_awake || !night_mode_state ) && leds_disabled ) 
    leds_disabled = 0;
  else if ( !night_mode_enable && leds_disabled )
    leds_disabled = 0;
  else if ( night_mode_enable && !( temprary_touch_awake || !night_mode_state ) && !leds_disabled ) {
    leds_disabled = 1;
    temp_led.off( );
    prs_led.off( );
  }

  if ( !leds_disabled ) // if LED indication flag - "enabled"
    switch ( display_mode ) { // switching between displays
      case 0: // current data display
      
        if ( prs_disp_mode ) 
          forecast_display( _forecast_range, _forecast_range_blink_mode, blink_led_state );
        else
          pressure_display( _pressure_range, _pressure_range_blink_flag,  blink_led_state );

        temp_display( _temp_range, _temp_range_blink_flag, blink_led_state );
             
        break;
        
      case 1: // 1h ago data display
      
        if ( prs_disp_mode ) 
          forecast_display( _forecast_h_ago_range, _forecast_h_ago_range_blink_mode, blink_led_state );
        else
          pressure_display( _pressure_h_ago_range, _pressure_h_ago_range_blink_flag, blink_led_state );
        
        history_display( blink_led_state );
        break;
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

  delay( 1500 );
  
  bootanimation( );
  
  avr_pressure.init( round( bmp.readPressure( ) ) );
  avr_luminosity.init( analogRead( A7 ) );
  
  pressure = ( uint32_t )( avr_pressure.get_result( ) );  // saving current pressure 
  temperature = round( bmp.readTemperature( ) - 2.5 );    // saving current temperature
  luminosity = avr_luminosity.get_result( );              // saving current luminosity

  forecast.begin( pressure );

  _pressure_range = pressure_range( round( pressure * Pa_to_hPa ), _pressure_range_blink_flag ); // updating pressure range
  _temp_range = temp_range( temperature, _temp_range_blink_flag );  // updating temperature range
  
  _forecast_actual_flag = 0; //forecast.is_update( );

  _forecast_range = forecast_range( 15, _forecast_range_blink_mode );

    // System test using Serial
  //Serial.begin( 9600 ); 
  //serial_test_func( );
  //serial_flag_test_func( );
  //Serial.println( "BOOT COMPLETE" );
  //Serial.println( "------------------" );
}

void loop( ) {
  button.loop_func( ); 
  
  luminosity_upd( luminosity ); //updating luminosity
  
  // nigth mode switcher with gisteresis:
  if ( night_mode_enable ) {
    if ( luminosity < 95 )  // if luminosity is low
      night_mode_state = 1;
    else if ( luminosity > 127 )  // if luminosity is high
      night_mode_state = 0;
  } else if ( night_mode_state ) {
    night_mode_state = 0;
  }

  static uint64_t data_upd_counter = millis( ); // data update time counter  
  static uint64_t forecast_upd_counter = millis( ); // data update time counter

  
  if ( millis( ) - data_upd_counter  >= DATA_UPDATE_INTERVAL * 60000 ) { // if it's time to update data 
    pressure = pressure_upd( );  // updating pressure
    _pressure_range = pressure_range( round( pressure * Pa_to_hPa ), _pressure_range_blink_flag ); // updating pressure range
    
    temperature = round( bmp.readTemperature( ) - 2.5 ); // updating temperature
    _temp_range = temp_range( temperature, _temp_range_blink_flag );  // updating temperature range

    
    if ( millis( ) - forecast_upd_counter >= FORECAST_UPDATE_INTERVAL * 60000 ) {  // if it's time to update forecast
      forecast.set( pressure ); // updating forecast
      _forecast_actual_flag = forecast.is_update( );  // updating forecast actuality flag
      _forecast_range = forecast_range( forecast.get_forecast( ), _forecast_range_blink_mode ); // updating forecast range
      forecast_upd_counter = millis( );

      //Serial.println( "-----------Forecast update-----------" );
    }
    data_upd_counter = millis( ); // saving current time
    
    //serial_test_func( );
  }  
  
  led_display( ); // LED display func
}
