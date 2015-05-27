
/*
* Arduino Solar charge controller for 6 volt lead acid batteries.
* 
* This is the firmware to control my 6V solar charge controller
* The charge controller firmware monitors the pv voltage, converter
* current, load current, and battery voltage and adjusts the PWM 
* output to keep the battery charging power at an optimum level
* depending on the terminal voltage of the battery.
* 
* The buck converter in the charger hardware works in both continuous
* and discontinuous mode. Continuous mode is used at higher charge
* currents and discontinuous mode is used when charge currents are low.
* 
* The charger has 3 stages. BULK, ABSORB and FLOAT. The voltage 
* thresholds which control transition to each mode are adjusted based
* on temperature. A single LM335 temperature sensor provides battery
* temperature input to the control algorithm.
* 
* The BULK charge state tries to dump as much energy into the battery 
* as possible. The battery terminal voltage is monitored and when it 
* reaches the programmed threshold. The PV voltage is also monitored 
* every 30 seconds and if it deviates 5 % from the last voltage noted
* at maximum power, a scan will be done to determine the the point where
* the solar panel is delivering maximum power. 
* 
* The ABSORB charge state charges the battery at 1/10C to the gassing
* voltage. Once this voltage is reached, the charger changes to the FLOAT
* state. 
* 
* The FLOAT state keeps the battery terminal voltage at a low voltage
* to prevent battery gassing. If load currents get too high, the charger
* will transition out of this state into ABSORB or BULK depending on the
* current demand from the load.
* 
* When a minimum power conversion threshold is reached or the PV voltage
* drops below what is required for useable energy harvesting, the 
* charger will go to sleep. In this state, the charger will monitor the
* PV voltage periodically and make a decision to wake up or not.
* 
* There are several I2C commands to get data from the charger, perform
* calibration, and enable some loads to be switched on and off.
* 
* A calibration procedure should be performed after firmware is loaded
* to null out the tolerances of the  voltage sense resistors. See the
* I2C commands for details. Calibration values are stored in EEPROM
* so this procedure only needs to be done once. A semi-precision 6 volt
* source should be used to power the board during the calibration procedure.
* One can make such a source using an LM317 and a 3 1/2 digit DMM which
* is good enough to get a successful calibration result.
* 
* This code was tested on an Arduino mini pro 328. With suitable
* modifications, it should run on any 5 volt Arduino, but make sure
* the Arduino you are using has a voltage regulator rated for 1% load
* regulation so that the analog sense voltages will be accurate. Arduinos
* using the MIC5205 5V regulator are preferred.
*/

#include <EEPROM.h>
#include <Wire.h>
#include "TimerOne.h"


#define TWI_ADDRESS 0x08    // I2C address

/*
* Pin definitions
*/

// Analog / I2C
#define BVSENSEPIN 0
#define PVSENSEPIN 1
#define BISENSEPIN 2
#define LISENSEPIN 3
#define I2C_DATA 4
#define I2C_CLOCK 5
#define TSENSEPIN 6
#define PISENSEPIN 7    // Reserved for testing. PV input current (external sensor)

// Digital

#define DATA_READY 8
#define PVENAPIN 9
#define PROFILEPIN 10
#define PWMPIN 11
#define LOADENAPIN 12
#define LEDPIN 13

/*
* Constants
*/

#define CELLS 3
#define BATTERY_DISCHARGED_MILLIVOLTS CELLS * 1800     // Battery completely discharged
#define END_BULK_CHARGE_MILLIVOLTS CELLS * 2310        // End bulk charge at 293K mv
#define BATTERY_END_ABSORB_MILLIVOLTS CELLS * 2400     // Absorbtion stage end 293K
#define FLOAT_HOLD_MV CELLS * 2280                     // Float holding voltage 293K
#define BATTERY_GASSING_MILLIVOLTS CELLS * 2415        // Gassing at 293K mv

#define MIN_CONV_POWER 10                              // Need to see this minimum power to go into bulk mode from scan
#define ABSORB_TARGET_CURRENT 450                      // Target current in absorb mode (Tailor to battery C/10).
#define BATTERY_TEMPCO CELLS * -2                      // mV per deg. K for battery
#define ROOM_TEMP_K 293                                // Room temp in Kelvin
#define SYSTEM_OVERTEMP_LIMIT 338                      // Battery Overtemp cutout

#define BULK_RESCAN_TIME 3000                          // Time between rescans (10 ms ticks)
#define BULK_POWER_DIP_TIME 5000                       // Time to wait to validate a power dip in bulk charging mode
#define BULK_TO_ABSORB_TIME 30000                      // Time to wait while checking battery voltage stays >= the end bulk charge voltage
#define ABSORB_WAIT_TIME 2000                          // Time to wait in absorb state before going to previous or next state
#define FLOAT_EXIT_TIME 5000                           // Time to wait in float state before going to previous state
#define SWITCH_ON_TIME 5000                            // Number of milliseconds pv voltage needs to be above threshold switch on
#define SWITCH_OFF_TIME 5000                           // Number of milliseconds pv voltage needs to be under threshold to switch off
#define MIN_POWER_WAIT_TIME 12000                      // Number of 10 ms ticks to wait if converter does not produce the minimum power
#define SCAN_WAIT_TIME 2000                            // Time to wait before starting scan after setting pwm to 0
#define SCAN_TIMER_INCREMENT_FAST 20                   // Time in milliseconds to wait before incrementing pwm value, fast speed
#define SCAN_TIMER_INCREMENT_SLOW 100                  // Time in milliseconds to wait before incrementing pwm value, slow speed

#define SWITCH_ON_MILLIVOLTS 7500                      // Threshold to switch converter on from sleep mode

#define SLEEP_HYST_MV 50                               // Voltage hysteresis to switch in and out of sleep mode
#define FLOAT_HYST 50                                  // Voltage hysteresis used during float charging stage
#define ABSORB_HYST 5                                  // Current hysteresis used in absorb charging state


#define CONVERTER_PWM_CLIP 0xF0                        // Clip at current limit

#define ANALOG_FULL_SCALE 5000						             // Full scale voltage of an analog input

#define PV_CAL_VOLTAGE 6000						                 // PV Calibration test voltage in mv
#define BATT_CAL_VOLTAGE 6000                          // Battery calibration test voltage in mv


#define EEPROM_CALIB_ADDR 0xE0						             // Offset into EEPROM for calibration data
#define EEPROM_CALIB_SIG 0x5AA5						             // Calibration signature
#define CAL_UPPER_LIMIT 5200                           // Upper cal limit
#define CAL_LOWER_LIMIT 4800                           // Lower cal limit

#define EEPROM_CONFIG_ADDR 0x00                        // Offset into EEPROM for configuration data
#define EEPROM_CONFIG_SIG 0x55AA                       // Config signature

#define LED_FAST_BLINK 12                              // 240 ms blink period
#define LED_VERY_FAST_BLINK 7                          // 140 ms blink period

#define CALIB_DWELL_TIME 200                           // Time to wait between increment/decrement of calibration value
#define CALIB_V_HYST 5                                 // Hysteresis around calibration target

#define DESIGNER_ID "HWSTAR"                           // Designer handle
#define PROJECT_ID "66-000101"                         // 66-000101 printed circuit board
#define MAJOR_VERSION 0                                // Major version number
#define MINOR_VERSION 0                                // Minor version number

// Supported I2C commands
enum {CMD_NOP=0, CMD_CALIB_ENTER=1, CMD_CALIB_WRITE=2, CMD_CALIB_EXIT=3, 
  CMD_CALIB_PV_VOLTS=4, CMD_CALIB_BATT_VOLTS=5, CMD_CALIB_RETURN_STATE=6,
  CMD_CALIB_RETURN_VALUES=7, CMD_LOAD_ENABLE=8, CMD_LOAD_DISABLE=9,
  CMD_RETURN_SENSOR_VALUES=10, CMD_RETURN_CHARGE_MODE=11,
  CMD_RETURN_CONV_INFO=12, CMD_RESET_ENERGY = 13, CMD_RESET_CHARGE = 14, 
  CMD_RESET_DISCHARGE = 15, CMD_GET_LOAD_ENABLE_STATE = 16, 
  CMD_CONV_ENABLE = 17, CMD_CONV_DISABLE = 18,
  CMD_GET_ID_INFO = 255};
  
// Calibration states
enum {CALIB_IDLE = 0, CALIB_PVV_START, CALIB_PVV_WAIT, 
  CALIB_BV_START, CALIB_BV_WAIT};
  
// LED commands
enum {LEDC_OFF = 0, LEDC_ON, LED_FLASH_FAST};

// LED states
enum {LEDS_OFF, LEDS_ON, LEDS_FF_ON, LEDS_FF_OFF, LEDS_FVF_ON, LEDS_FVF_OFF};

// Used by timer interrupt

typedef struct {
  unsigned acquire : 1;
  volatile uint8_t ticks;
  volatile uint16_t scan;
  volatile uint16_t charge;
  volatile uint16_t charge10;
  volatile uint16_t fgload;
} timer_t;

// A place to store inputs and filtered versions of inputs

typedef struct {
  uint32_t pv_mv;
  uint32_t pv_mv_filt;
  uint32_t batt_mv;
  uint32_t batt_mv_filt;
  uint32_t conv_ma;
  uint32_t conv_ma_filt;
  uint32_t load_ma;
  uint32_t load_ma_filt;
  uint32_t pv_ma; // Testing only
  uint32_t pv_ma_filt; // Testing only
  int32_t batt_ma_filt;
  uint32_t conv_power_mw;
  uint32_t load_power_mw;
  uint32_t batt_power_mw;
  uint64_t conv_energy;
  uint64_t battery_charge;
  uint64_t battery_discharge;
  uint16_t battery_temp;
  uint16_t battery_temp_filt;

} sensor_values_t;

typedef struct {
  uint16_t pv_mv;
  uint16_t batt_mv;
  uint16_t conv_ma;
  uint16_t load_ma;
  uint16_t battery_temp_k;
  uint16_t conv_energy_mwh;
  uint16_t batt_charge_mah;
  uint16_t batt_discharge_mah;
  uint16_t pv_ma; // Testing only. Requires external sensor
} sensor_info_t;

// A place to store private converter variables

typedef struct {
  uint8_t slow_scan_stop_pwm;
} converter_private_t;


// A place to store public converter variables

typedef struct {
  uint8_t state;
  uint8_t servocurrentstate;
  uint8_t pwm;
  uint8_t pwm_max_power;
  uint8_t calibrate;
  uint8_t system_load_enabled;
  uint32_t max_power;
  int16_t tempoffset;
  uint16_t end_bulk_mv;
  uint16_t end_absorb_mv;
  uint16_t gassing_mv;
  uint16_t float_hold_mv;
  uint16_t max_power_mv;
  uint16_t fgload;
} converter_t;

typedef struct {
  uint8_t state;
  uint16_t pv_mv;
  uint16_t batt_mv;
} calib_t;

  
//Buffer type 

typedef struct {
  uint8_t command;
  uint8_t length;
  uint8_t buffer[32];
} i2c_buffer_t;


//I2C variables

typedef struct {
  volatile uint8_t cmd_received;
  volatile uint8_t command;
  i2c_buffer_t tx;
  
} i2c_t;


// EEPROM calibration layout

typedef struct {
  uint16_t sig;
  uint16_t batt_mv;
  uint16_t pv_mv;
} eeprom_calib_t;


// EEPROM configuration layout

typedef struct {
  uint16_t sig;
  uint8_t i2c_load_enabled;
} eeprom_config_t;
  
//LED variables 

typedef struct {
  uint8_t state;
  uint8_t timer;
} led_t;

// Identification

typedef struct {
  char designer[10];
  char project[10];
  uint16_t minor_version;
  uint16_t major_version;
} id_t;


/*
* Variable definitions
*/

static timer_t timer;
static sensor_values_t sensor_values;
static converter_private_t converter_private;
static converter_t converter;
static i2c_t i2c;
static calib_t calib;
static eeprom_calib_t eeprom_calib;
static eeprom_config_t eeprom_config;  
static led_t led;
static uint8_t debug_level = 5;





/*
* Timer 1 ISR
*/

void isr_timer1()
{
  //digitalWrite(PROFILEPIN, true);
  if(timer.charge)
    timer.charge--;
    
  if(timer.scan)
    timer.scan--;
    
  timer.fgload++;
  
  switch(led.state){
      case LEDS_OFF:
        digitalWrite(LEDPIN, false);
        break;
        
      case LEDS_ON:
        digitalWrite(LEDPIN, true);
        break;
        
      case LEDS_FF_OFF:
        if(!led.timer){
          led.timer = LED_FAST_BLINK;
          digitalWrite(LEDPIN, true);
          led.state = LEDS_FF_ON;
        }
        break;
      
      case LEDS_FF_ON:
       if(!led.timer){
          led.timer = LED_FAST_BLINK;
          digitalWrite(LEDPIN, false);
          led.state = LEDS_FF_OFF;
        }
        break;

     case LEDS_FVF_OFF:
        if(!led.timer){
          led.timer = LED_VERY_FAST_BLINK;
          digitalWrite(LEDPIN, true);
          led.state = LEDS_FVF_ON;
        }
        break;
      
      case LEDS_FVF_ON:
       if(!led.timer){
          led.timer = LED_VERY_FAST_BLINK;
          digitalWrite(LEDPIN, false);
          led.state = LEDS_FVF_OFF;
        }
        break;
  }
  
  // Tell foreground to re-acquire sensor values every 10 mSec.
  if(timer.ticks >= 9){
    //digitalWrite(PROFILEPIN, true);
    if(timer.charge10)
      timer.charge10--;
    timer.acquire = true;
    timer.ticks = 0;
    if(led.timer)
      led.timer--;
    //digitalWrite(PROFILEPIN, false);
  }
  else
    timer.ticks++;
  //digitalWrite(PROFILEPIN, false);
}


/*
* I2C Receive event
*/

void i2c_receive(int count)
{
  // Note the command, and set the commeand received flag
  i2c.command = Wire.read();
  i2c.cmd_received = true;
}

/*
* Print a debug message to the serial port
*/

void debug(uint8_t level, const char *format, ...)
{
  static char buffer[64];
  va_list ap;
  va_start(ap, format);
  if(level < debug_level){
    vsnprintf(buffer, sizeof(buffer), format, ap);
    Serial.write(buffer);
    Serial.write("\r\n");
  }
  va_end(ap);
}


/*
* I2C Transmit event
*/

void i2c_transmit()
{
  // Send the length and what is in the TX buffer
  Wire.write((uint8_t *) &i2c.tx, i2c.tx.length + 2);
  digitalWrite(DATA_READY, false); 
}



/*
* Set the pwm frequency for a specific pin
*/


void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = ((TCCR0B & 0b11111000) | mode);
    } else {
      TCCR1B = ((TCCR1B & 0b11111000) | mode);
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = ((TCCR2B & 0b11111000) | mode);
  }
}


/*
* Read multiple bytes from EEPROM
*/

void eeprom_read(void *buffer, uint16_t eeaddr, uint16_t count)
{
  uint8_t *p = (uint8_t *) buffer;
  uint16_t i;
  for(i = 0; i < count; i++)
    p[i] = EEPROM.read(eeaddr + i);
}

/*
* Write multiple bytes from EEPROM
*/

void eeprom_write(void *buffer, uint16_t eeaddr, uint16_t count)
{
  uint8_t *p = (uint8_t *) buffer;
  uint16_t i;
  for(i = 0; i < count; i++)
    EEPROM.write(eeaddr + i, p[i]);
}



/*
* Read analog channel and return value in millivolts
*/

uint32_t read_millivolts(uint8_t analogpin, uint16_t vcalib, uint8_t scale)
{
  uint32_t raw = analogRead(analogpin);
  return (raw * vcalib * scale)/1024;
  
}

/*
* Read analog channel and return value in milliamps
*/

uint32_t read_milliamps(uint8_t analogpin, uint16_t icalib, uint16_t idenom)
{
    return ((((uint32_t)analogRead(analogpin)) * icalib)/ idenom);
}

/*
 * Read temperature in Kelvin
 */

uint8_t read_tempk(uint8_t analogpin, uint16_t tcalib, uint16_t *result)
{

  uint32_t kvolts;
  
  if(!result)
    return false;
  
  kvolts = read_millivolts(analogpin, tcalib, 1);
  if((kvolts < 1000))
    return false;
  *result =  (uint16_t) kvolts/10;
  return true;
}
  


/*
* Read timer and return its value
*/

uint16_t read_timer(volatile uint16_t *ptimer)
{
  uint16_t res;
  noInterrupts();
  res = *ptimer;
  interrupts(); 
  return res;
}

/*
* Set timer
*/

void set_timer(volatile uint16_t *ptimer, uint16_t value)
{
  noInterrupts();
  *ptimer = value;
  interrupts(); 
}


/*
* Set pwm value to a range between 0 and the clip value
*/

uint8_t converter_pwm_set(uint8_t newval)
{
  if(!newval){
    converter.pwm = newval;
    return true;
  }
  else if(newval <= CONVERTER_PWM_CLIP){
    converter.pwm = newval;
    return false;
  }
  else 
    return true;
}
   

/*
* Initialization
*/

void setup()
{
  // Serial port setup
  Serial.begin(9600);
  
  // GPIO Setup
  pinMode(PWMPIN, OUTPUT);   // sets the PWM pin as output
  pinMode(LEDPIN, OUTPUT);   // sets the LED pin as output
  digitalWrite(LEDPIN, false);
  pinMode(LOADENAPIN, OUTPUT); // sets the LOAD enable pin as an output
  digitalWrite(LOADENAPIN, false);
  pinMode(PVENAPIN, OUTPUT); // sets the PV enable pin as output
  digitalWrite(PVENAPIN, false); 
  pinMode(PROFILEPIN, OUTPUT);   // sets the PROFILE pin as output
  
  // I2C Setup
  Wire.begin(TWI_ADDRESS);
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_transmit);
  
  // Read config from EEPROM
  eeprom_read(&eeprom_config, EEPROM_CONFIG_ADDR, sizeof(eeprom_config));
  if(EEPROM_CONFIG_SIG != eeprom_config.sig){
     // Bad signature, write default config data to EEPROM
     debug(0, "\r\nInvalid calibration signature\r\n");
     eeprom_config.sig = EEPROM_CONFIG_SIG;
     eeprom_config.i2c_load_enabled = false;
     eeprom_write(&eeprom_config, EEPROM_CONFIG_ADDR, sizeof(eeprom_config));
  }
  
  
  // Read Calibration constants from EEPROM
  eeprom_read(&eeprom_calib, EEPROM_CALIB_ADDR, sizeof(eeprom_calib));
  if(EEPROM_CALIB_SIG != eeprom_calib.sig){
     debug(0, "\r\nInvalid calibration signature\r\n");
     eeprom_calib.pv_mv = 5046; // Bad signature, install appx. values
     eeprom_calib.batt_mv = 5077; 
  }
 
  // Initialize 1 mSec interrupt source
  Timer1.initialize(1000);
  Timer1.attachInterrupt(isr_timer1);
  
  // Set the pwm frequency
  setPwmFrequency(PWMPIN, 1);

}

/*
* Read sensors and perform filtering. 
* Called from main loop.
*/

void update_values(void)
{
  digitalWrite(PROFILEPIN, true);
  
  // Use cal values if performing calibration
  uint16_t batt_mv = (calib.batt_mv) ? calib.batt_mv : eeprom_calib.batt_mv;
  uint16_t pv_mv = (calib.pv_mv) ? calib.pv_mv : eeprom_calib.pv_mv;
  // Get raw sensor values
  sensor_values.batt_mv = read_millivolts(BVSENSEPIN, batt_mv, 2);
  sensor_values.conv_ma = read_milliamps(BISENSEPIN, ANALOG_FULL_SCALE, 3994);
  sensor_values.pv_mv = read_millivolts(PVSENSEPIN, pv_mv, 3);
  sensor_values.load_ma = read_milliamps(LISENSEPIN, ANALOG_FULL_SCALE, 3994);
  sensor_values.pv_ma = read_milliamps(PISENSEPIN, ANALOG_FULL_SCALE, 3994); // External sensor, testing only!
  
  if(!read_tempk(TSENSEPIN, 5000, &sensor_values.battery_temp))
     sensor_values.battery_temp = ROOM_TEMP_K;    // Sensor out of range. Use room temp.
 
  // Filter sensor values
  sensor_values.batt_mv_filt = ((sensor_values.batt_mv_filt * 15) + sensor_values.batt_mv) >> 4;
  sensor_values.conv_ma_filt = ((sensor_values.conv_ma_filt * 15) + sensor_values.conv_ma) >> 4; 
  sensor_values.pv_mv_filt = ((sensor_values.pv_mv_filt * 15) + sensor_values.pv_mv) >> 4;
  sensor_values.load_ma_filt = ((sensor_values.load_ma_filt * 15) + sensor_values.load_ma) >> 4; 
  sensor_values.battery_temp_filt = ((sensor_values.battery_temp_filt * 15) + sensor_values.battery_temp) >> 4;
  sensor_values.pv_ma_filt = ((sensor_values.pv_ma_filt * 15) + sensor_values.pv_ma) >> 4; // External sensor, testing only!
  
 
  // Derive battery milliamps
  sensor_values.batt_ma_filt = sensor_values.conv_ma_filt - sensor_values.load_ma_filt;
  
  // Calculate converter power in milliwatts
  sensor_values.conv_power_mw = (sensor_values.conv_ma_filt * sensor_values.batt_mv_filt)/1000;
  
  // Calculate load power in milliwatts
  sensor_values.load_power_mw = (sensor_values.load_ma_filt * sensor_values.batt_mv_filt)/1000;
  
  // Calculate battery power in milliwatts
  uint16_t batt_ma_abs = sensor_values.batt_ma_filt;
  if(sensor_values.batt_ma_filt < 0)
    batt_ma_abs = sensor_values.batt_ma_filt * -1;

  sensor_values.batt_power_mw = (batt_ma_abs * sensor_values.batt_mv_filt)/1000;
  
  // Integrate power to get energy
  sensor_values.conv_energy += sensor_values.conv_power_mw;
  
  // Integrate battery milliamps to get maH
  if(sensor_values.batt_ma_filt < 0)
    sensor_values.battery_discharge += batt_ma_abs;
  else
    sensor_values.battery_charge += batt_ma_abs;
  
  digitalWrite(PROFILEPIN, false);
 
}

/* 
* Calibration state machine
*/


void do_calib(void)
{

  
  switch(calib.state){
     default:
       break;
     
     case CALIB_IDLE:
       break;
       
       
     case CALIB_PVV_START:
       // Calibrate PV Voltage
       led.state = LEDS_FF_ON; // Flash LED
       //eeprom_calib.pv_mv = ANALOG_FULL_SCALE;
       calib.pv_mv = ANALOG_FULL_SCALE;
       set_timer(&timer.charge, CALIB_DWELL_TIME);
       calib.state = CALIB_PVV_WAIT;
       break;
       
     case CALIB_PVV_WAIT:
       if(!read_timer(&timer.charge)){
         uint16_t val = sensor_values.pv_mv_filt;
         if(val > PV_CAL_VOLTAGE + CALIB_V_HYST){
           calib.pv_mv--;
           if(calib.pv_mv < CAL_LOWER_LIMIT){
            led.state = LEDS_OFF;
            calib.state = CALIB_IDLE;
            debug(0, "\n Error: PV lower limit reached!");
            break;
           }
           set_timer(&timer.charge, CALIB_DWELL_TIME);
     
         }
         else if( val < PV_CAL_VOLTAGE - CALIB_V_HYST){
           calib.pv_mv++;
           if(calib.pv_mv > CAL_UPPER_LIMIT){
            led.state = LEDS_OFF;
            calib.state = CALIB_IDLE;
            debug(0, "\n Error: PV upper limit reached!");
            break;
           }
           set_timer(&timer.charge, CALIB_DWELL_TIME);
         }
         else{
           led.state = LEDS_OFF;
           debug(0, "\n*** PV Calibration value = %u ***\n", calib.pv_mv);
           calib.state = CALIB_IDLE;
         }
      }
      break;
      
    case CALIB_BV_START:
       // Calibrate battery voltage
       led.state = LEDS_FF_ON;
       //eeprom_calib.batt_mv = ANALOG_FULL_SCALE;
       calib.batt_mv = ANALOG_FULL_SCALE;
       set_timer(&timer.charge, CALIB_DWELL_TIME);
       calib.state = CALIB_BV_WAIT;
       break;  
      
    case CALIB_BV_WAIT:
       if(!read_timer(&timer.charge)){
         uint16_t val = sensor_values.batt_mv_filt;
         if(val > BATT_CAL_VOLTAGE + 5){
           calib.batt_mv--;
           if(calib.batt_mv < CAL_LOWER_LIMIT){
            led.state = LEDS_OFF;
            calib.state = CALIB_IDLE;
            debug(0, "\n Error: BV lower limit reached!");
            break;
           }

           set_timer(&timer.charge, 200);
     
         }
         else if( val < BATT_CAL_VOLTAGE - 5){
           calib.batt_mv++;
           if(calib.batt_mv > CAL_UPPER_LIMIT){
            led.state = LEDS_OFF;
            calib.state = CALIB_IDLE;
            debug(0, "\n Error: BV upper limit reached!");
            break;
           }
           set_timer(&timer.charge, 200);
         }
         else{
           led.state = LEDS_OFF;
           debug(0, "\n*** BV Calibration value = %u ***\n", calib.batt_mv);
           calib.state = CALIB_IDLE;
         }
      }
      break;
  }
  
}



/*
* Converter control state machine
*/

// Converter states

enum {CONVSTATE_INIT=0, CONVSTATE_OFF, CONVSTATE_SLEEP, CONVSTATE_WAKEUP, 
    CONVSTATE_SCAN_START, CONVSTATE_SCAN_WAIT_FAST, CONVSTATE_SCAN_WAIT_SLOW,
    CONVSTATE_SCAN_FAST, CONVSTATE_SCAN_SLOW, CONVSTATE_MIN_POWER_WAIT, 
    CONVSTATE_BULK, CONVSTATE_BULK_POWER_DIP, CONVSTATE_BULK_ABSORB,
    CONVSTATE_ABSORB, CONVSTATE_ABSORB_FLOAT, CONVSTATE_FLOAT, 
    CONVSTATE_FLOAT_EXIT,CONVSTATE_DISABLED, CONVSTATE_OVERTEMP
};

// Current control states

enum {SRVCS_START = 0, SRVCS_UNDERVOLT, SRVCS_OVERVOLT};


// State machine code. Called from main loop

void converter_ctrl_loop(void)
{
  uint8_t stop = false;
  
  converter.tempoffset = (sensor_values.battery_temp - ROOM_TEMP_K) *
  BATTERY_TEMPCO;
  
  
  // Set values for termination voltages based on temperature
  converter.end_bulk_mv = END_BULK_CHARGE_MILLIVOLTS + converter.tempoffset;
  converter.end_absorb_mv = BATTERY_END_ABSORB_MILLIVOLTS + converter.tempoffset;
  converter.gassing_mv = BATTERY_GASSING_MILLIVOLTS + converter.tempoffset;
  converter.float_hold_mv = FLOAT_HOLD_MV + converter.tempoffset;
  
  switch(converter.state)
  {
    case CONVSTATE_INIT:
      converter_pwm_set(0);
      converter.state = CONVSTATE_OFF;
      break;
      
      
    case CONVSTATE_OFF:
    case CONVSTATE_OVERTEMP:
      led.state = LEDS_OFF;
      converter_pwm_set(0);
      converter.system_load_enabled = true; // Enable load
      converter.state = CONVSTATE_SLEEP;
      break;
    
    case CONVSTATE_SLEEP:
      digitalWrite(PVENAPIN, false);
      if(sensor_values.pv_mv_filt >= SWITCH_ON_MILLIVOLTS + SLEEP_HYST_MV){
        set_timer(&timer.charge, SWITCH_ON_TIME);
        converter.state = CONVSTATE_WAKEUP;
      }
      break;
      
    case CONVSTATE_WAKEUP:
      if(sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS - SLEEP_HYST_MV){
        set_timer(&timer.charge, 0);
        converter_pwm_set(0);
        converter.state = CONVSTATE_SLEEP;
        break;
      }
      if(!read_timer(&timer.charge)){
        digitalWrite(PVENAPIN, true);
        converter.state = CONVSTATE_SCAN_START;
        break;
      }
      break;
      
    case CONVSTATE_SCAN_START:
      converter_pwm_set(0);
      converter.pwm_max_power = 0;
      converter.max_power = 0;
      set_timer(&timer.scan, 0);
      set_timer(&timer.charge, SCAN_WAIT_TIME);
      converter.state = CONVSTATE_SCAN_WAIT_FAST;
      break;
      
    case CONVSTATE_SCAN_WAIT_FAST:
      if(!read_timer(&timer.charge))
        converter.state = CONVSTATE_SCAN_FAST;
      break;
      
    case CONVSTATE_SCAN_WAIT_SLOW:
      if(!read_timer(&timer.charge)){
        converter.state = CONVSTATE_SCAN_SLOW;
      }
      break;
    
      
    case CONVSTATE_SCAN_FAST:
    case CONVSTATE_SCAN_SLOW:
      if(!read_timer(&timer.scan)){
          if(sensor_values.conv_power_mw > converter.max_power){
            // We noted an increase in power
            // Save pwm power, and voltage observed
            converter.max_power = sensor_values.conv_power_mw;
            converter.pwm_max_power = converter.pwm;
            converter.max_power_mv = sensor_values.pv_mv_filt;
          }
          // Increase PWM duty cycle by one count, but  
          // cut scan short if gassing voltage is reached,
          // or if in slow scan mode, and stop pwm value is reached
          stop = converter_pwm_set(converter.pwm + 1);
          if(CONVSTATE_SCAN_SLOW == converter.state){
            stop |= (converter.pwm >= converter_private.slow_scan_stop_pwm);
          }
          if(stop ||
            (sensor_values.batt_mv_filt > converter.gassing_mv)){
            // At max duty cycle, stop the scan
            // Ensure it is over the minimum power
            if(converter.pwm_max_power < MIN_CONV_POWER){
              // Wait a prescribed amount of time before doing another scan
              converter_pwm_set(0);
              set_timer(&timer.charge10, MIN_POWER_WAIT_TIME);
              converter.state = CONVSTATE_MIN_POWER_WAIT;
              break;
            }
            else{
              if(converter.state == CONVSTATE_SCAN_SLOW){
                  converter_pwm_set(converter.pwm_max_power);
                  converter.state = CONVSTATE_BULK;
                  set_timer(&timer.charge10, BULK_RESCAN_TIME);
              }
              else{
                  uint16_t stop_pwm = 0;
                  // Start a slower scan at 90 to 110 percent
                  // of the fast scan peak pwm value
                  // This ensures an accurate detection of the peak value.
                  converter_pwm_set((converter.pwm_max_power > 2) ?
                  ((converter.pwm_max_power * 9)/10) :
                  0);
                  stop_pwm = (((uint16_t) converter.pwm_max_power) * 11)/10;
                  if(stop_pwm > CONVERTER_PWM_CLIP)
                    stop_pwm = CONVERTER_PWM_CLIP;
                  converter_private.slow_scan_stop_pwm = 
                  (uint8_t) stop_pwm;

                  // Reset the other peak vars
                  converter.pwm_max_power = 0;
                  converter.max_power = 0;
                  converter.max_power_mv = 0;
                  // Wait for voltages to stabilize
                  set_timer(&timer.charge, SCAN_WAIT_TIME);
                  converter.state = CONVSTATE_SCAN_WAIT_SLOW;
            }
            break;
          }
        }
        set_timer(&timer.scan, (converter.state == CONVSTATE_SCAN_FAST) ?
        SCAN_TIMER_INCREMENT_FAST : SCAN_TIMER_INCREMENT_SLOW);    
      }
      break;
      
    case CONVSTATE_MIN_POWER_WAIT:
      // Wait before starting another scan
      if(!read_timer(&timer.charge10))
        converter.state = CONVSTATE_OFF;
      break;
      
      if(sensor_values.pv_mv_filt >= SWITCH_ON_MILLIVOLTS + SLEEP_HYST_MV){
        // PV voltage rose back above the theshold. Redo the scan.
        set_timer(&timer.charge, 0);
        converter.state = CONVSTATE_SCAN_START;
      }
      break;
      
      
    case CONVSTATE_BULK:
    case CONVSTATE_BULK_POWER_DIP:
      // See if the scan timer expired
      if(!read_timer(&timer.charge10)){
        // Check to see if the pv mv value at the scan is gtr or less
        // than the current value by  5%. If it is, do a rescan.
        uint16_t delta_mv = (converter.max_power_mv >= sensor_values.pv_mv_filt)?
          converter.max_power_mv - sensor_values.pv_mv_filt :
          sensor_values.pv_mv_filt - converter.max_power_mv;
          if(converter.max_power_mv / delta_mv <= 20){
            converter.state = CONVSTATE_SCAN_START;
          }
          else
              set_timer(&timer.charge10, BULK_RESCAN_TIME);
        break;
      }

      if(CONVSTATE_BULK == converter.state){
        // If the pv voltage dips start a timer
        if((sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS - SLEEP_HYST_MV)){
          set_timer(&timer.charge, BULK_POWER_DIP_TIME);
          converter.state = CONVSTATE_BULK_POWER_DIP;
        }
      }
      else{
        if(!read_timer(&timer.charge)){
          if((sensor_values.pv_mv_filt > SWITCH_ON_MILLIVOLTS + SLEEP_HYST_MV)){
            converter.state = CONVSTATE_BULK;
          }
          else{
            // Power consistently below 90% after a time delay. Do a rescan.
            converter.state = CONVSTATE_SCAN_START;
            break;
          }
        }
      }
 
      converter_pwm_set(converter.pwm_max_power);
      if((sensor_values.batt_mv_filt >= converter.end_bulk_mv) || (sensor_values.batt_mv_filt > converter.gassing_mv)){
          // Exit bulk charging and go to absorbtion charging mode
          set_timer(&timer.charge, BULK_TO_ABSORB_TIME);
          converter.state = CONVSTATE_BULK_ABSORB;
      }    
      break;   
     
    case CONVSTATE_BULK_ABSORB:
      // Hold in bulk for a set time before transferring to absorb.
      // Unless voltage or power dips.
      // If the gassing voltage is reached, terminate early and go to absorb state.
      converter_pwm_set(converter.pwm_max_power);
      if((!read_timer(&timer.charge)) || (sensor_values.batt_mv_filt > converter.gassing_mv)){
        converter.state = CONVSTATE_ABSORB;
      }
      else if((sensor_values.conv_power_mw < (converter.pwm_max_power * 9)/10) || 
        (sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS - SLEEP_HYST_MV)){
        converter.state = CONVSTATE_SCAN_START;
      }  
      break;
      
     
    case CONVSTATE_ABSORB:

      switch(converter.servocurrentstate){
        case SRVCS_START:
          // See if battery voltage decreased to go to the previous state
          if(sensor_values.batt_mv_filt <= converter.end_bulk_mv ){
            set_timer(&timer.charge, ABSORB_WAIT_TIME);
            converter.servocurrentstate = SRVCS_UNDERVOLT;
            break;
          }
          // See if battery voltage increased to go to the next state
          else if(sensor_values.batt_mv_filt >= converter.end_absorb_mv){
            set_timer(&timer.charge, ABSORB_WAIT_TIME);
            converter.servocurrentstate = SRVCS_OVERVOLT;
            break;
          } 
          break;
          
        case SRVCS_UNDERVOLT:
          // If greater than the previous state termination voltage, stop timing the transition
          // and return to the starting current state. 
          if(sensor_values.batt_mv_filt > converter.end_bulk_mv ){
            converter.servocurrentstate = SRVCS_START;
            break;
          }
          if(!read_timer(&timer.charge)){
            // Timer expired, go to the previous charging state
            converter.servocurrentstate = SRVCS_START;
            converter.state = CONVSTATE_SCAN_START;
            break;
          }
          break;
        
        case SRVCS_OVERVOLT:
          // If less than the next state termination voltage, stop timing the transition,
          // and return to the starting current state.
          if(sensor_values.batt_mv_filt < converter.end_absorb_mv ){
            converter.servocurrentstate = SRVCS_START;
            break;
          }
          if(!read_timer(&timer.charge)){
            // Timer expired, go to the next charging state
            converter.servocurrentstate = SRVCS_START;
            converter.state = CONVSTATE_ABSORB_FLOAT;
            break;
          }
          break;
      }
      
      // Servo to current unless voltage at terminal value. 
      if(sensor_values.batt_mv_filt < converter.gassing_mv){
        if((sensor_values.batt_ma_filt > (ABSORB_TARGET_CURRENT + ABSORB_HYST))) 
          converter_pwm_set(converter.pwm - 1);
        else if(sensor_values.batt_ma_filt < (ABSORB_TARGET_CURRENT - ABSORB_HYST)) 
          converter_pwm_set(converter.pwm + 1);
      }
      else{
        converter_pwm_set(converter.pwm - 1);
      }
      break;
      
    case CONVSTATE_ABSORB_FLOAT:
      // Taper down to float voltage
      converter_pwm_set(0);
      if(sensor_values.batt_mv_filt < converter.float_hold_mv + FLOAT_HYST){
        converter.state = CONVSTATE_FLOAT;
      }
      break;
      
    
    case CONVSTATE_FLOAT:
    case CONVSTATE_FLOAT_EXIT:
      // Hold at float charge voltage
      if(CONVSTATE_FLOAT == converter.state){
        if((sensor_values.batt_mv_filt < converter.float_hold_mv - (FLOAT_HYST * 2)||
          sensor_values.batt_ma_filt > ABSORB_TARGET_CURRENT)){
          converter.state = CONVSTATE_FLOAT_EXIT;
          timer.charge = FLOAT_EXIT_TIME;
        }
      }
      else{
        if(sensor_values.batt_mv_filt >= converter.float_hold_mv - (FLOAT_HYST * 2)&&
          (sensor_values.batt_ma_filt < ABSORB_TARGET_CURRENT)){
          converter.state = CONVSTATE_FLOAT;
          break;
        }
        if(!timer.charge){
          converter.state = CONVSTATE_ABSORB;
          break;
        }
      }
    
      if(sensor_values.batt_mv_filt < (converter.float_hold_mv - FLOAT_HYST))
        converter_pwm_set(converter.pwm + 1);
      else if(sensor_values.batt_mv_filt > (converter.float_hold_mv + FLOAT_HYST))
        converter_pwm_set(converter.pwm - 1);
      break; 
  
      case CONVSTATE_DISABLED:
        // Disable converter and enable load at system level
        // Used for testing
        digitalWrite(PVENAPIN, false);
        converter_pwm_set(0);
        converter.system_load_enabled = true; // Enable load
        break;
 
    default:
      break;
  }  
  // Update the pwm hardware with the most recent calculated value
  analogWrite(PWMPIN, converter.pwm);
}    

/*
* Main loop
*/

void loop()
{
  static uint8_t ticks;
  uint16_t *values;
  sensor_info_t *si;
  id_t *id;
  
  // Update load enabled state
  digitalWrite(LOADENAPIN, eeprom_config.i2c_load_enabled && 
  converter.system_load_enabled && 
  !converter.calibrate);
  
  // I2C handler
  
  if(i2c.cmd_received){ // Process command
    i2c.cmd_received = false;
    // Zero out the request buffer
    memset(i2c.tx.buffer,0,sizeof(i2c.tx.buffer));
    // Act on command
    i2c.tx.command = i2c.command;
    switch(i2c.command){
      default:
        i2c.tx.length = 0;
        break;
    
      case CMD_CALIB_ENTER:
        // Enter calibration
        converter.state = CONVSTATE_INIT;
        converter_pwm_set(0);
        analogWrite(PWMPIN, converter.pwm);
        converter.calibrate = true;
        break;
      
      case CMD_CALIB_PV_VOLTS:
        // Start PV voltage calibration  
	      if(converter.calibrate && (CALIB_IDLE == calib.state)){
           calib.state = CALIB_PVV_START;
        }
        break;
	   
      
      case CMD_CALIB_BATT_VOLTS:
        // Start Battery voltage calibration
	      if(converter.calibrate && (CALIB_IDLE == calib.state)){
           calib.state = CALIB_BV_START;
        }
        break;  
       
      
      case CMD_CALIB_RETURN_STATE:
        // Return calibration state
        i2c.tx.buffer[0] = calib.state;
        i2c.tx.length = 1;
        digitalWrite(DATA_READY, true);
        break;
      
      
      case CMD_CALIB_RETURN_VALUES:
        // Return calibration values
        values = (uint16_t *) i2c.tx.buffer;
        i2c.tx.length = 4;
        values[0] = (calib.pv_mv) ? calib.pv_mv : eeprom_calib.pv_mv;
        values[1] = (calib.batt_mv) ? calib.batt_mv : eeprom_calib.batt_mv;
        digitalWrite(DATA_READY, true); 
        break;
        
      
      case CMD_CALIB_WRITE:
        // Write calibration to EEPROM if values within limits
        if((calib.pv_mv >= CAL_LOWER_LIMIT) &&
           (calib.pv_mv <= CAL_UPPER_LIMIT) &&
           (calib.batt_mv >= CAL_LOWER_LIMIT) &&
           (calib.batt_mv <= CAL_UPPER_LIMIT)){
          eeprom_calib.sig = EEPROM_CALIB_SIG;
          eeprom_calib.pv_mv = calib.pv_mv;
          eeprom_calib.batt_mv = calib.batt_mv;
          eeprom_write(&eeprom_calib, EEPROM_CALIB_ADDR, sizeof(eeprom_calib));
          calib.batt_mv = calib.pv_mv = 0;
        }
        break;        
      
     
      case CMD_CALIB_EXIT:
         // Exit Calibration
        calib.pv_mv = calib.batt_mv = 0;
        converter.calibrate = false;
        break;
        
      
      case CMD_LOAD_ENABLE:
        // Enable switched load  
        eeprom_config.i2c_load_enabled = true;
        eeprom_write(&eeprom_config, EEPROM_CONFIG_ADDR, sizeof(eeprom_config));
        break;
      
        
      case CMD_LOAD_DISABLE:
        // Disable switched load  
        eeprom_config.i2c_load_enabled = false;
        eeprom_write(&eeprom_config, EEPROM_CONFIG_ADDR, sizeof(eeprom_config));     
        break;
        
            
      case CMD_GET_LOAD_ENABLE_STATE:
        // Return load enabled state
        i2c.tx.buffer[0] = eeprom_config.i2c_load_enabled;
        i2c.tx.length = 1;
        digitalWrite(DATA_READY, true);
        break;
      
     
      case CMD_RETURN_SENSOR_VALUES:
        // Return sensor values  
        si = (sensor_info_t *) i2c.tx.buffer;
        i2c.tx.length = sizeof(sensor_info_t);
        si->pv_mv = (uint16_t) sensor_values.pv_mv_filt;
        si->batt_mv = (uint16_t) sensor_values.batt_mv_filt;
        si->conv_ma = (uint16_t) sensor_values.conv_ma_filt;
        si->load_ma = (uint16_t) sensor_values.load_ma_filt;
        si->pv_ma = (uint16_t) sensor_values.pv_ma_filt; // Testing only. Requires external sensor
        si->battery_temp_k = (uint16_t) sensor_values.battery_temp;
        si->conv_energy_mwh = (uint16_t) ((sensor_values.conv_energy / ((uint64_t)3600 * 100)));
        si->batt_charge_mah = (uint16_t) ((sensor_values.battery_charge / ((uint64_t)3600 * 100)));
        si->batt_discharge_mah = (uint16_t) ((sensor_values.battery_discharge / ((uint64_t)3600 * 100)));
        digitalWrite(DATA_READY, true);
        break;
        
      
      case CMD_RETURN_CHARGE_MODE:
        // Return charge mode
        i2c.tx.length = 1;
        i2c.tx.buffer[0] = converter.state;
        digitalWrite(DATA_READY, true);
        break;
        
      case CMD_RETURN_CONV_INFO:
        // Return converter state, thresholds and other info
        i2c.tx.length = sizeof(converter);
        memcpy(i2c.tx.buffer, &converter, sizeof(converter));
        digitalWrite(DATA_READY, true);
        break;
        
      case CMD_RESET_ENERGY:
        // Reset energy integrator
        sensor_values.conv_energy = 0;
        break;
       
      case CMD_RESET_CHARGE:
        // Reset charge integrator
        sensor_values.battery_charge = 0;
        break;
        
      case CMD_RESET_DISCHARGE:
        // Reset discharge integrator
        sensor_values.battery_discharge = 0;
        break;
        
      case CMD_GET_ID_INFO:
        // Return id info
        i2c.tx.length = sizeof(id_t);
        id = (id_t *) i2c.tx.buffer;
        strcpy(id->designer, DESIGNER_ID);
        strcpy(id->project, PROJECT_ID);
        id->major_version = MAJOR_VERSION;
        id->minor_version = MINOR_VERSION;
        digitalWrite(DATA_READY, true);
        break;
        
      case CMD_CONV_DISABLE:
        // Disable converter for testing  
        converter.state = CONVSTATE_DISABLED;
        break;
        
      case CMD_CONV_ENABLE:
        // Disable converter for testing  
        // Restarts the converter in the OFF state.
        // This allows test programs to monitor the state transitions
        // to the ending charge state. This is a feature, not a bug.
        converter.state = CONVSTATE_OFF;
        break;

    }
  }
  
 
  if(!timer.acquire)
    return;
    
  // 10 milliseconds has elapsed. Time to acquire inputs and do calulations.
  
  set_timer(&timer.fgload, 0);
 
  
  timer.acquire = false;  
  
  // Everything below here runs every 10 mSec.
 
  update_values(); // Get the inputs and average them
  

  if(converter.calibrate){
    // Do calibration instead of control
    do_calib();
  }
  else{
    if(sensor_values.battery_temp_filt <= SYSTEM_OVERTEMP_LIMIT){
      converter_ctrl_loop(); // Run the control loop to adjust the PWM output as needed.
    }
    else{
      /*** OVERTEMP! Shut everything off, and keep it off as long as the overtemp condition exists ***/
      if(converter.state != CONVSTATE_OVERTEMP){
        converter_pwm_set(0);
        led.state = LEDS_FVF_ON;
        digitalWrite(PVENAPIN, true);
        converter.system_load_enabled = false;
        converter.state = CONVSTATE_OVERTEMP;
      }
    }
    

  }
 
  if(ticks++ >= 99){
  
    ticks = 0;
  
    
    // This is a debug aid. It prints out variables every second.
    //debug(0, "Battery Millivolts raw %u", sensor_values.batt_mv);
    //debug(0, "Battery Millivolts filtered %u", sensor_values.batt_mv_filt);
    //debug(0, "PV Millivolts raw %u", sensor_values.pv_mv);
    //debug(0, "PV Millivolts filtered %u", sensor_values.pv_mv_filt);
    //debug(0, "Converter Milliamps raw %u", sensor_values.conv_ma);
    //debug(0, "Converter Milliamps filtered %u", sensor_values.conv_ma_filt);
    //debug(0, "Load Milliamps filtered %u", sensor_values.load_ma_filt);
    //debug(0, "Battery Milliamps filtered %i", sensor_values.batt_ma_filt);
    //debug(0, "Converter power %u mW", sensor_values.conv_power_mw);
    //debug(0, "Load power %u mW", sensor_values.load_power_mw);
    //debug(0, "Battery power %u mW", sensor_values.batt_power_mw);
    //debug(0, "Converter state %u", converter.state);
    //debug(0, "Servo current state %u", converter.servocurrentstate);
    //debug(0, "Converter power point pwm %u", converter.pwm_max_power);
    //debug(0, "Converter pwm %u", converter.pwm);
    //debug(0, "Converter temperature offset %u", converter.tempoffset);
    //debug(0, "Charge timer: %u", read_timer(&timer.charge));
    //debug(0, "Converter Energy (mWh): %u", (uint32_t) ((sensor_values.conv_energy/((uint64_t)3600*100))));
    //debug(0, "Battery Charge (mAh): %u", (uint32_t) ((sensor_values.battery_charge/((uint64_t)3600*100))));
    //debug(0, "Raw battery temperature %u\n", sensor_values.battery_temp);
    //debug(0,"\r\n");
    
  }
  //delay(2);
  converter.fgload = read_timer(&timer.fgload);
  
}
