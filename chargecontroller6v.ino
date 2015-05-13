
/*
* Solar charge controller for 6 volt lead acid batteries
*/
#include <EEPROM.h>
#include <Wire.h>
#include "TimerOne.h"


#define TWI_ADDRESS 0x08    // I2C address

#define BVSENSEPIN 0
#define PVSENSEPIN 1
#define BISENSEPIN 2
#define LISENSEPIN 3
#define I2C_CLOCK 5
#define I2C_DATA 4
#define DATA_READY 8
#define LOADEN 9
#define PROFILEPIN 10
#define PWMPIN 11
#define LOADENAPIN 12
#define LED 13







#define CELLS 3
#define BATTERY_DISCHARGED_MILLIVOLTS CELLS * 1800     // Battery completely discharged
#define END_BULK_CHARGE_MILLIVOLTS CELLS * 2310        // End bulk charge at 293K mv
#define BATTERY_FLOAT_MILLIVOLTS CELLS * 2400          // Absorbtion stage end 293K
#define BATTERY_GASSING_MILLIVOLTS CELLS * 2415        // Gassing at 293K mv
#define BATTERY_MAH 1300                               // Battery capacity in mAH
#define BATTERY_FLOAT_CURRENT 30                       // Battery float current in mA
#define BATTERY_TEMPCO CELLS * -2                      // mV per deg. K for battery
#define ROOM_TEMP_K 293                                // Room temp in Kelvin


#define BULK_POWER_DIP_TIME 5000                       // Time to wait to validate a power dip in bulk charging mode
#define BULK_TO_ABSORB_TIME 30000                      // Time to wait while checking battery voltage stays >= the end bulk charge voltage
#define ABSORB_WAIT_TIME 2000                          // Time to wait in absorb state before going to previous or next state
#define FLOAT_WAIT_TIME 5000                           // Time to wait in float state before going to previous state
//#define SWITCH_ON_TIME 30000                         // Number of milliseconds pv voltage needs to be above threshold switch on
#define SWITCH_ON_TIME 5000
#define SWITCH_OFF_TIME 5000                           // Number of milliseconds pv voltage needs to be under threshold to switch off

#define SWITCH_ON_MILLIVOLTS 7500                      // Threshold to switch converter on from sleep mode

#define SLEEP_HYST_MV 50                               // Voltage hysteresis to switch in and out of sleep mode
#define HYST_SERVO_BY_CURRENT 100                      // Voltage hysteresis to switch to the next state in servo_by_current

#define CONVERTER_PWM_CLIP 0xF0                        // Clip at current limit

#define ANALOG_FULL_SCALE 5000						             // Full scale voltage of an analog input

#define PV_CAL_VOLTAGE 10200						               // PV Calibration test voltage in mv
#define BATT_CAL_VOLTAGE 7200                          // Battery calibration test voltage in mv


#define EEPROM_CALIB_ADDR 0xE0						             // Offset into EEPROM for calibration data
#define EEPROM_CALIB_SIG 0x5AA5						             // Calibration signature

#define LED_FAST_BLINK 125                             // 4Hz blink rate

#define CALIB_DWELL_TIME 200                           // Time to wait between increment/decrement of calibration value
#define CALIB_V_HYST 5                                 // Hysteresis around calibration target


enum {CMD_NOP = 0, CMD_CALIB_ENTER, CMD_CALIB_WRITE_EXIT, CMD_CALIB_EXIT, 
CMD_CALIB_PV_VOLTS, CMD_CALIB_BATT_VOLTS, CMD_CALIB_RETURN_STATE,
CMD_CALIB_RETURN_VALUES, CMD_LOAD_ENABLE, CMD_LOAD_DISABLE,
CMD_RETURN_SENSOR_VALUES, CMD_RETURN_CHARGE_MODE};
enum {CALIB_IDLE = 0, CALIB_PVV_START, CALIB_PVV_WAIT, CALIB_BV_START, CALIB_BV_WAIT};
enum {LEDC_OFF = 0, LEDC_ON, LED_FLASH_FAST};
enum {LEDS_OFF, LEDS_ON, LEDS_FF_ON, LEDS_FF_OFF};

// Used by timer interrupt

typedef struct {
  unsigned acquire : 1;
  volatile uint8_t ticks;
  volatile uint8_t scan;
  volatile uint16_t charge;
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
  int32_t batt_ma_filt;
  uint32_t conv_power_mw;
  uint32_t load_power_mw;
  uint32_t batt_power_mw;
  uint16_t battery_temp;
  uint16_t battery_temp_filt;
} sensor_values_t;

// A place to store converter variables

typedef struct {
  uint8_t state;
  uint8_t servocurrentstate;
  uint8_t pwm;
  uint8_t pwm_max_power;
  uint8_t calibrate;
  uint32_t max_power;
  int16_t tempoffset;
  uint16_t end_bulk_mv;
  uint16_t end_absorb_mv;
  uint16_t gassing_mv;
  
} converter_t;

typedef struct {
  uint8_t state;
  uint16_t value;

} calib_t;

  
/* Buffer type */
typedef struct {
  uint8_t command;
  uint8_t length;
  uint8_t buffer[24];
} i2c_buffer_t;


/* I2C variables */

typedef struct {
  volatile uint8_t cmd_received;
  volatile uint8_t command;
  i2c_buffer_t tx;
  
} i2c_t;


/* EEPROM calibration layout */
typedef struct {
  uint16_t sig;
  uint16_t batt_mv;
  uint16_t pv_mv;
} eeprom_calib_t;


/* LED variables */
typedef struct {
  uint8_t state;
  uint16_t timer;
} led_t;

/*
* Variable definitions
*/



static timer_t timer;
static sensor_values_t sensor_values;
static converter_t converter;
static i2c_t i2c;
static calib_t calib;
static eeprom_calib_t eeprom_calib;  
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
  
  switch(led.state){
      case LEDS_OFF:
        digitalWrite(LED, false);
        break;
        
      case LEDS_ON:
        digitalWrite(LED, true);
        break;
        
      case LEDS_FF_OFF:
        if(!led.timer){
          led.timer = LED_FAST_BLINK;
          digitalWrite(LED, true);
          led.state = LEDS_FF_ON;
        }
        break;
      
      case LEDS_FF_ON:
       if(!led.timer){
          led.timer = LED_FAST_BLINK;
          digitalWrite(LED, false);
          led.state = LEDS_FF_OFF;
        }
        break;
  }
  
  // Tell foreground to re-acquire sensor values every 10 mSec.
  if(timer.ticks >= 9){
    //digitalWrite(PROFILEPIN, true);
    timer.acquire = true;
    timer.ticks = 0;
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
* I2C Transmit event
*/

void i2c_transmit()
{
  // Send the length and what is in the TX buffer
  Wire.write((uint8_t *) &i2c.tx, i2c.tx.length + 2);
  digitalWrite(DATA_READY, false); 
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

uint32_t read_milliamps(uint8_t analogpin, uint16_t icalib)
{
    return ((((uint32_t)analogRead(analogpin)) * icalib)/4096);
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
  pinMode(LOADENAPIN, OUTPUT);
  digitalWrite(LOADENAPIN, false);
  pinMode(PROFILEPIN, OUTPUT);   // sets the PROFILE pin as output
  
  // I2C Setup
  Wire.begin(TWI_ADDRESS);
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_transmit);
  
  
  // TODO: Fetch these from NVRAM
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

  //calib.state = CALIB_BV_START; // DEBUG
  //converter.calibrate = true; // DEBUG
  
  
}

/*
* Read sensors and perform filtering
*/

void update_values(void)
{
  
  digitalWrite(PROFILEPIN, true);
  
  // Get raw sensor values
  sensor_values.batt_mv = read_millivolts(BVSENSEPIN, eeprom_calib.batt_mv, 2);
  sensor_values.conv_ma = read_milliamps(BISENSEPIN, ANALOG_FULL_SCALE);
  sensor_values.pv_mv = read_millivolts(PVSENSEPIN, eeprom_calib.pv_mv, 3);
  sensor_values.load_ma = read_milliamps(LISENSEPIN, ANALOG_FULL_SCALE);
  
  sensor_values.battery_temp = ROOM_TEMP_K;    //  FIXME: Hardcode  battery temp
 

  // Filter sensor values
  sensor_values.batt_mv_filt = ((sensor_values.batt_mv_filt * 15) + sensor_values.batt_mv) >> 4;
  sensor_values.conv_ma_filt = ((sensor_values.conv_ma_filt * 15) + sensor_values.conv_ma) >> 4; 
  sensor_values.pv_mv_filt = ((sensor_values.pv_mv_filt * 15) + sensor_values.pv_mv) >> 4;
  sensor_values.load_ma_filt = ((sensor_values.load_ma_filt * 15) + sensor_values.load_ma) >> 4; 
  sensor_values.battery_temp_filt = ((sensor_values.battery_temp_filt * 15) + sensor_values.battery_temp) >> 4; 
  
  // Derive battery milliamps
  sensor_values.batt_ma_filt = sensor_values.conv_ma_filt - sensor_values.load_ma_filt;
  
  
  // Calculate converter power in milliwatts
  sensor_values.conv_power_mw = (sensor_values.conv_ma_filt * sensor_values.batt_mv_filt)/1000;
  
  // Calculate load power in milliwatts
  sensor_values.load_power_mw = (sensor_values.load_ma_filt * sensor_values.batt_mv_filt)/1000;
  
   // Calculate battery power in milliwatts
  sensor_values.batt_power_mw = (sensor_values.batt_ma_filt * sensor_values.batt_mv_filt)/1000;
  
  digitalWrite(PROFILEPIN, false);
 
}


/*
* Servo by current control
*/

enum {SRVCS_START = 0, SRVCS_UNDERVOLT, SRVCS_OVERVOLT};

void servo_by_current(uint16_t current, uint8_t current_hyst, 
uint16_t termvoltsprev, uint16_t termvoltsnext, uint16_t termms, 
uint8_t normstate, uint8_t prevstate, uint8_t nextstate)
{
  

  switch(converter.servocurrentstate){
    case SRVCS_START:
      // See if battery voltage decreased to go to the previous state
      if(sensor_values.batt_mv_filt <= termvoltsprev - HYST_SERVO_BY_CURRENT){
        set_timer(&timer.charge, termms);
        converter.servocurrentstate = SRVCS_UNDERVOLT;
        break;
      }
      // See if battery voltage increased to go to the next state
      else if(sensor_values.batt_mv_filt >= termvoltsnext + HYST_SERVO_BY_CURRENT){
        set_timer(&timer.charge, termms);
        converter.servocurrentstate = SRVCS_OVERVOLT;
        break;
      } 
      break;
      
    case SRVCS_UNDERVOLT:
      // If greater than the previous state termination voltage, stop timing the transition
      // and return to the starting current state. 
      if(sensor_values.batt_mv_filt > termvoltsprev - HYST_SERVO_BY_CURRENT){
        converter.servocurrentstate = SRVCS_START;
        break;
      }
      if(!read_timer(&timer.charge)){
        // Timer expired, go to the previous charging state
        converter.servocurrentstate = SRVCS_START;
        converter.state = prevstate;
        break;
      }
      break;
    
    case SRVCS_OVERVOLT:
      // If less than the next state termination voltage, stop timing the transition,
      // and return to the starting current state.
      if(sensor_values.batt_mv_filt < termvoltsnext + HYST_SERVO_BY_CURRENT){
        converter.servocurrentstate = SRVCS_START;
        break;
      }
      if(!read_timer(&timer.charge)){
        // Timer expired, go to the next charging state
        converter.servocurrentstate = SRVCS_START;
        converter.state = nextstate;
        break;
      }
      break;
  }
  

  // Servo to current unless voltage at terminal value. 
  // If the voltage hits the terminal value + 100mV, back off on the pwm duty cycle.
  if((sensor_values.batt_ma_filt > (current + current_hyst))||(sensor_values.batt_mv_filt > (termvoltsnext)))
    converter_pwm_set(converter.pwm - 1);
  else if(sensor_values.batt_ma_filt < (current - current_hyst))
    converter_pwm_set(converter.pwm + 1);  
 
}


/* 
* Calibration code
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
       eeprom_calib.pv_mv = ANALOG_FULL_SCALE;
       set_timer(&timer.charge, CALIB_DWELL_TIME);
       calib.state = CALIB_PVV_WAIT;
       break;
       
     case CALIB_PVV_WAIT:
       if(!read_timer(&timer.charge)){
         uint16_t val = sensor_values.pv_mv_filt;
         if(val > PV_CAL_VOLTAGE + CALIB_V_HYST){
           eeprom_calib.pv_mv--;
           set_timer(&timer.charge, CALIB_DWELL_TIME);
     
         }
         else if( val < PV_CAL_VOLTAGE - CALIB_V_HYST){
           eeprom_calib.pv_mv++;
           set_timer(&timer.charge, CALIB_DWELL_TIME);
         }
         else{
           led.state = LEDS_OFF;
           debug(0, "\n*** PV Calibration value = %u ***\n", eeprom_calib.pv_mv);
           calib.state = CALIB_IDLE;
         }
      }
      break;
      
    case CALIB_BV_START:
       // Calibrate battery voltage
       led.state = LEDS_FF_ON;
       eeprom_calib.pv_mv = ANALOG_FULL_SCALE;
       set_timer(&timer.charge, CALIB_DWELL_TIME);
       calib.state = CALIB_BV_WAIT;
       break;  
      
    case CALIB_BV_WAIT:
       if(!read_timer(&timer.charge)){
         uint16_t val = sensor_values.batt_mv_filt;
         if(val > BATT_CAL_VOLTAGE + 5){
           eeprom_calib.batt_mv--;
           set_timer(&timer.charge, 200);
     
         }
         else if( val < BATT_CAL_VOLTAGE - 5){
           eeprom_calib.batt_mv++;
           set_timer(&timer.charge, 200);
         }
         else{
           led.state = LEDS_OFF;
           debug(0, "\n*** BV Calibration value = %u ***\n", eeprom_calib.batt_mv);
           calib.state = CALIB_IDLE;
         }
      }
      break;
  }
  
}



/*
* Converter control loop
*/


enum {CONVSTATE_INIT=0, CONVSTATE_OFF, CONVSTATE_SLEEP, CONVSTATE_WAKEUP, CONVSTATE_SCAN_START, CONVSTATE_SCAN, CONVSTATE_VOLTAGE_DIP, 
CONVSTATE_BULK, CONVSTATE_BULK_POWER_DIP, CONVSTATE_BULK_ABSORB, CONVSTATE_ABSORB, CONVSTATE_FLOAT};

void converter_ctrl_loop(void)
{
  
  converter.tempoffset = (sensor_values.battery_temp - ROOM_TEMP_K) *
  BATTERY_TEMPCO;
  
  
  // Set values for termination voltages based on temperature
  converter.end_bulk_mv = END_BULK_CHARGE_MILLIVOLTS + converter.tempoffset;
  converter.end_absorb_mv = BATTERY_FLOAT_MILLIVOLTS + converter.tempoffset;
  converter.gassing_mv = BATTERY_GASSING_MILLIVOLTS + converter.tempoffset;
  

  switch(converter.state)
  {
    case CONVSTATE_INIT:
      set_timer(&timer.charge, 0);
      timer.scan = 0;
      converter_pwm_set(0);
      converter.state = CONVSTATE_OFF;
      
    case CONVSTATE_OFF:
      converter_pwm_set(0);
      digitalWrite(LOADENAPIN, true); // Enable load
      converter.state = CONVSTATE_SLEEP;
      break;
    
    case CONVSTATE_SLEEP:
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
        converter.state = CONVSTATE_SCAN_START;
        break;
      }
      break;
      
    case CONVSTATE_SCAN_START:
      converter.max_power = 0;
      converter.pwm_max_power = 0;
      timer.scan = 0;
      converter.state = CONVSTATE_SCAN;
      break;
      
    
    case CONVSTATE_SCAN:
      if(sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS - SLEEP_HYST_MV){
         // If voltage dips under switch on threshold, go back to sleep for a time period.
         set_timer(&timer.charge, SWITCH_OFF_TIME);
         converter_pwm_set(0);
         converter.state = CONVSTATE_VOLTAGE_DIP;
      }
      else{
        if(!timer.scan){
          if(sensor_values.conv_power_mw > converter.max_power){
            // We noted an increase in power
            converter.max_power = sensor_values.conv_power_mw;
            converter.pwm_max_power = converter.pwm;
          }  
          if(converter_pwm_set(converter.pwm + 1)){
            // At max duty cycle, stop the scan
            converter.state = CONVSTATE_BULK;
            break;
          }
          if((converter.pwm_max_power > 0) &&
            (converter.pwm_max_power < converter.pwm) && 
            (sensor_values.conv_power_mw < (converter.max_power * 9)/ 10)){
              // Terminate early if we passed the peak
              converter.state = CONVSTATE_BULK;
              break;
          }
          timer.scan = 20;
        }    
      }
        
      break;
      
    case CONVSTATE_VOLTAGE_DIP:
      if(!read_timer(&timer.charge)){
        // Timer expired, turn converter off and wait
        converter.state = CONVSTATE_OFF;
        break;
      }
      if(sensor_values.pv_mv_filt >= SWITCH_ON_MILLIVOLTS + SLEEP_HYST_MV){
        // PV voltage rose back above the theshold. Redo the scan.
        set_timer(&timer.charge, 0);
        converter.state = CONVSTATE_SCAN_START;
      }
      break;
      
      
    case CONVSTATE_BULK:
    case CONVSTATE_BULK_POWER_DIP:
      if(CONVSTATE_BULK == converter.state){
        // If power dips below 90% of the scan value, or the pv voltage dips start a timer
        if((sensor_values.conv_power_mw < (converter.pwm_max_power * 9)/10) || 
        (sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS - SLEEP_HYST_MV)){
          set_timer(&timer.charge, BULK_POWER_DIP_TIME);
          converter.state = CONVSTATE_BULK_POWER_DIP;
        }
      }
      else{
        if(!read_timer(&timer.charge)){
          // Power consistently below 90% after a time delay. Do a rescan.
          converter.state = CONVSTATE_SCAN_START;
          break;
        }
        // If power goes back over 90% of previous scan value and the pv voltage is good, return to bulk state
        else if((sensor_values.conv_power_mw >= (converter.pwm_max_power * 9)/10) &&
          (sensor_values.pv_mv_filt >= SWITCH_ON_MILLIVOLTS - SLEEP_HYST_MV))
          converter.state = CONVSTATE_BULK;    
      }
      
      converter_pwm_set(converter.pwm_max_power);
      if(sensor_values.batt_mv_filt >= END_BULK_CHARGE_MILLIVOLTS){
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
      if((!read_timer(&timer.charge)) || (sensor_values.batt_mv_filt > converter.gassing_mv))
        converter.state = CONVSTATE_ABSORB;
      else if((sensor_values.conv_power_mw < (converter.pwm_max_power * 9)/10) || 
        (sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS - SLEEP_HYST_MV)){
        converter.state = CONVSTATE_BULK;
      }  
    
      break;
     
    case CONVSTATE_ABSORB:
      servo_by_current(BATTERY_MAH/10, 5,
      converter.end_bulk_mv, 
      converter.end_absorb_mv,
      ABSORB_WAIT_TIME,
      CONVSTATE_ABSORB,
      CONVSTATE_SCAN,
      CONVSTATE_FLOAT);
      break;
    
    case CONVSTATE_FLOAT:
      servo_by_current(BATTERY_FLOAT_CURRENT, 5,
      converter.end_absorb_mv, 
      converter.gassing_mv,
      FLOAT_WAIT_TIME,
      CONVSTATE_FLOAT,
      CONVSTATE_ABSORB,
      CONVSTATE_FLOAT);
   
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
  
  if(i2c.cmd_received){ // Process command
    // Zero out the request buffer
    memset(i2c.tx.buffer,0,sizeof(i2c.tx.buffer));
    // Act on command
    i2c.tx.command = i2c.command;
    switch(i2c.command){
      default:
        break;
      // Enter calibration
      case CMD_CALIB_ENTER:
        converter.state = CONVSTATE_INIT;
        digitalWrite(LOADENAPIN, false);
        converter_pwm_set(0);
        converter.calibrate = true;
        break;
      // Start PV voltage calibration  
      case CMD_CALIB_PV_VOLTS:
	    if(converter.calibrate && (CALIB_IDLE == calib.state)){
           calib.state = CALIB_PVV_START;
        }
        break;
	   
      // Start Battery voltage calibration
      case CMD_CALIB_BATT_VOLTS:
	    if(converter.calibrate && (CALIB_IDLE == calib.state)){
           calib.state = CALIB_BV_START;
        }
        break;  
       
      // Return calibration state
      case CMD_CALIB_RETURN_STATE:
        i2c.tx.buffer[0] = calib.state;
        i2c.tx.length = 1;
        digitalWrite(DATA_READY, true);
        break;
      
      // Return calibration values
      case CMD_CALIB_RETURN_VALUES:
        values = (uint16_t *) i2c.tx.buffer;
        i2c.tx.length = 4;
        values[0] = eeprom_calib.pv_mv;
        values[1] = eeprom_calib.batt_mv;
        digitalWrite(DATA_READY, true); 
        break;
        
      // Write calibration to EEPROM and exit
      case CMD_CALIB_WRITE_EXIT:
        eeprom_calib.sig = EEPROM_CALIB_SIG;
        eeprom_write(&eeprom_calib, EEPROM_CALIB_ADDR, sizeof(eeprom_calib));
        converter.calibrate = false;
        break;        
      
      // Exit Calibration
      case CMD_CALIB_EXIT:
        converter.calibrate = false;
        break;
        
      // Enable switched load  
      case CMD_LOAD_ENABLE:
        digitalWrite(LOADEN, true);
        break;
        
      // Disable switched load  
      case CMD_LOAD_DISABLE:
        digitalWrite(LOADEN, false);
        break;
      
      // Return sensor values  
      case CMD_RETURN_SENSOR_VALUES:
        values = (uint16_t *) i2c.tx.buffer;
        i2c.tx.length = 10;
        values[0] = (uint16_t) sensor_values.pv_mv_filt;
        values[1] = (uint16_t) sensor_values.batt_mv_filt;
        values[2] = (uint16_t) sensor_values.conv_ma_filt;
        values[3] = (uint16_t) sensor_values.load_ma_filt;
        values[4] = (uint16_t) sensor_values.battery_temp;
        digitalWrite(DATA_READY, true);
        break;
        
      // Return charge mode
      case CMD_RETURN_CHARGE_MODE:
        i2c.tx.length = 1;
        i2c.tx.buffer[0] = converter.state;
        digitalWrite(DATA_READY, true);
        break;
        
        
        
    }
  }
  
 
  if(!timer.acquire)
    return;
    
  // 10 milliseconds has elapsed. Time to acquire inputs and do calulations.
  
  timer.acquire = false;  
  
  // Everything below here runs every 10 mSec.
 
  update_values(); // Get the inputs and average them
  
  if(converter.calibrate){
    // Do calibration instead of control
    do_calib();
  }
  else{
    converter_ctrl_loop(); // Run the control loop to adjust the PWM output as needed.
  }
 
  if(ticks++ >= 99){
    // This is a debug aid. It prints out variables every second.
    ticks = 0;
    debug(0, "Battery Millivolts raw %u", sensor_values.batt_mv);
    debug(0, "Battery Millivolts filtered %u", sensor_values.batt_mv_filt);
    debug(0, "PV Millivolts raw %u", sensor_values.pv_mv);
    debug(0, "PV Millivolts filtered %u", sensor_values.pv_mv_filt);
    debug(0, "Converter Milliamps raw %u", sensor_values.conv_ma);
    debug(0, "Converter Milliamps filtered %u", sensor_values.conv_ma_filt);
    debug(0, "Load Milliamps filtered %u", sensor_values.load_ma_filt);
    debug(0, "Battery Milliamps filtered %i", sensor_values.batt_ma_filt);
    debug(0, "Converter power %u mW", sensor_values.conv_power_mw);
    //debug(0, "Load power %u mW", sensor_values.load_power_mw);
    //debug(0, "Battery power %u mW", sensor_values.batt_power_mw);
    debug(0, "Converter state %u", converter.state);
    //debug(0, "Servo current state %u", converter.servocurrentstate);
    debug(0, "Converter pwm %u", converter.pwm);
    //debug(0, "Converter temperature offset %u", converter.tempoffset);
    //debug(0, "Charge timer: %u", read_timer(&timer.charge));
    debug(0,"\r\n");
    
  }
}
