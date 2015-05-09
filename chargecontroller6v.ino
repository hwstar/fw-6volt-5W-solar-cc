
/*
* Solar charge controller for 6 volt lead acid batteries
*/


#include "TimerOne.h"



#define BVSENSEPIN 0
#define PVSENSEPIN 1
#define BISENSEPIN 2
#define LISENSEPIN 3
#define PWMPIN 11
#define LOADENAPIN 12
#define PROFILEPIN 10


#define CELLS 3
#define END_BULK_CHARGE_MILLIVOLTS CELLS * 2310        // End bulk charge at 20C mv
#define BATTERY_FLOAT_MILLIVOLTS CELLS * 2400          // Absorbtion stage end 20C
#define BATTERY_GASSING_MILLIVOLTS CELLS * 2415        // Gassing at 20C mv
#define BATTERY_MAH 1300                               // Battery capacity in mAH
#define BATTERY_FLOAT_CURRENT 30                       // Battery float current in mA
#define BATTERY_TEMPCO CELLS * -2                      // mV per deg. C for battery referenced to 20 deg. C


#define SWITCH_ON_MILLIVOLTS 7500                      // Threshold to switch converter on
#define SWITCH_HYST_MV 50
//#define SWITCH_ON_TIME 30000                         // Number of milliseconds pv voltage needs to be above switch on
#define SWITCH_ON_TIME 5000
#define SWITCH_OFF_TIME 5000                           // Number of milliseconds pv voltage needs to be under threshold to switch off
#define CONVERTER_PWM_CLIP 0xE0                        // Clip at current limit

#define ANALOG_FULL_SCALE 5000


typedef struct {
  unsigned acquire : 1;
  volatile uint8_t ticks;
  volatile uint8_t scan;
  volatile uint16_t charge;
} timer_t;

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
} sensor_values_t;

typedef struct {
  uint8_t state;
  uint8_t servocurrentstate;
  uint8_t pwm;
  uint8_t pwm_max_power;
  uint32_t max_power;
} converter_t;
  

timer_t timer;
sensor_values_t sensor_values;
converter_t converter;
  
uint8_t debug_level = 5;






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
    Serial.write("\n");
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
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
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
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

/*
* Read analog channel and return value in millivolts
*/

uint32_t read_millivolts(uint8_t analogpin, uint16_t vcalib, uint8_t divisor)
{
  // return (((uint32_t)analogRead(analogpin))*ANALOG_FULL_SCALE)/(1024/scale);
  uint32_t raw = analogRead(analogpin);
  return (raw * vcalib * divisor)/1024;
  
}

/*
* Read analog channel and return value in milliamps
*/

uint32_t read_milliamps(uint8_t analogpin)
{
    return ((((uint32_t)analogRead(analogpin))*ANALOG_FULL_SCALE)/4096);
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
  pinMode(PWMPIN, OUTPUT);   // sets the PWM pin as output
  pinMode(LOADENAPIN, OUTPUT);
  digitalWrite(LOADENAPIN, false);
  pinMode(PROFILEPIN, OUTPUT);   // sets the PROFILE pin as output

  
  
  Timer1.initialize(1000);
  Timer1.attachInterrupt(isr_timer1);
  
  
  setPwmFrequency(PWMPIN, 1);
}

/*
* Read sensors and perform filtering
*/

void update_values(void)
{
  
  digitalWrite(PROFILEPIN, true);
  
  // Get raw sensor values
  sensor_values.batt_mv = read_millivolts(BVSENSEPIN, 4900, 2);
  sensor_values.conv_ma = read_milliamps(BISENSEPIN);
  sensor_values.pv_mv = read_millivolts(PVSENSEPIN, 4877, 3);
  sensor_values.load_ma = read_milliamps(LISENSEPIN);
  
 

  // Filter sensor values
  sensor_values.batt_mv_filt = ((sensor_values.batt_mv_filt * 15) + sensor_values.batt_mv) >> 4;
  sensor_values.conv_ma_filt = ((sensor_values.conv_ma_filt * 15) + sensor_values.conv_ma) >> 4; 
  sensor_values.pv_mv_filt = ((sensor_values.pv_mv_filt * 15) + sensor_values.pv_mv) >> 4;
  sensor_values.load_ma_filt = ((sensor_values.load_ma_filt * 15) + sensor_values.load_ma) >> 4; 
  
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

void servo_by_current(uint16_t current, uint8_t current_hyst, uint16_t termvoltsprev, uint16_t termvoltsnext, uint16_t termms, uint8_t normstate, uint8_t prevstate, uint8_t nextstate)
{
  

  switch(converter.servocurrentstate){
    case SRVCS_START:
      // See if battery voltage decreased to go to the previous state
      if(sensor_values.batt_mv_filt <= termvoltsprev){
        set_timer(&timer.charge, termms);
        converter.servocurrentstate = SRVCS_UNDERVOLT;
        break;
      }
      // See if battery voltage increased to go to the next state
      else if(sensor_values.batt_mv_filt >= termvoltsnext){
        set_timer(&timer.charge, termms);
        converter.servocurrentstate = SRVCS_OVERVOLT;
        break;
      } 
      break;
      
    case SRVCS_UNDERVOLT:
      if(sensor_values.batt_mv_filt > termvoltsprev){
        converter.servocurrentstate = SRVCS_START;
        break;
      }
      if(!read_timer(&timer.charge)){
        converter.servocurrentstate = SRVCS_START;
        converter.state = prevstate;
        break;
      }
      break;
    
    case SRVCS_OVERVOLT:
      if(sensor_values.batt_mv_filt < termvoltsnext){
        converter.servocurrentstate = SRVCS_START;
        break;
      }
      if(!read_timer(&timer.charge)){
        converter.servocurrentstate = SRVCS_START;
        converter.state = nextstate;
        break;
      }
      break;
  }
  

  // Servo to current unless voltage at terminal value
  if((sensor_values.batt_ma_filt > (current + current_hyst))||(sensor_values.batt_mv_filt > (termvoltsnext)))
    converter_pwm_set(converter.pwm - 1);
  else if(sensor_values.batt_ma_filt < (current - current_hyst))
    converter_pwm_set(converter.pwm + 1);  
 
}


/*
* Converter control loop
*/


enum {CONVSTATE_INIT=0, CONVSTATE_OFF, CONVSTATE_SLEEP, CONVSTATE_WAKEUP, CONVSTATE_SCAN_START, CONVSTATE_SCAN, CONVSTATE_VOLTAGE_DIP, 
CONVSTATE_BULK, CONVSTATE_ABSORB, CONVSTATE_FLOAT};

void converter_ctrl_loop(void)
{
  switch(converter.state)
  {
    case CONVSTATE_INIT:
      converter.state = CONVSTATE_OFF;
      
    case CONVSTATE_OFF:
      converter_pwm_set(0);
      digitalWrite(LOADENAPIN, true); // Enable load
      converter.state = CONVSTATE_SLEEP;
      break;
    
    case CONVSTATE_SLEEP:
      if(sensor_values.pv_mv_filt >= SWITCH_ON_MILLIVOLTS+SWITCH_HYST_MV){
        set_timer(&timer.charge, SWITCH_ON_TIME);
        converter.state = CONVSTATE_WAKEUP;
      }
      break;
      
    case CONVSTATE_WAKEUP:
      if(sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS-SWITCH_HYST_MV){
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
      if(sensor_values.pv_mv_filt < SWITCH_ON_MILLIVOLTS-SWITCH_HYST_MV){
         set_timer(&timer.charge, SWITCH_OFF_TIME);
         converter_pwm_set(0);
         converter.state = CONVSTATE_VOLTAGE_DIP;
      }
      else{
        if(!timer.scan){
          if(sensor_values.conv_power_mw > converter.max_power){
            converter.max_power = sensor_values.conv_power_mw;
            converter.pwm_max_power = converter.pwm;
          }  
          if(converter_pwm_set(converter.pwm + 1)){
            converter.state = CONVSTATE_BULK;
            break;
          }
          timer.scan = 20;
        }    
      }
        
      break;
      
    case CONVSTATE_VOLTAGE_DIP:
      if(!read_timer(&timer.charge)){
        converter.state = CONVSTATE_OFF;
        break;
      }
      if(sensor_values.pv_mv_filt >= SWITCH_ON_MILLIVOLTS+SWITCH_HYST_MV){
        set_timer(&timer.charge, 0);
        converter.state = CONVSTATE_SCAN_START;
      }
      break;
      
      
    case CONVSTATE_BULK:
      converter_pwm_set(converter.pwm_max_power);
      if(sensor_values.batt_mv_filt >= END_BULK_CHARGE_MILLIVOLTS){
          converter.state = CONVSTATE_ABSORB;
      }    
      break;
     
     
     
    case CONVSTATE_ABSORB:
      servo_by_current(BATTERY_MAH/10, 5,
      END_BULK_CHARGE_MILLIVOLTS - 100, 
      BATTERY_FLOAT_MILLIVOLTS,
      2000,
      CONVSTATE_ABSORB,
      CONVSTATE_SCAN,
      CONVSTATE_FLOAT);
      break;
    
    case CONVSTATE_FLOAT:
      servo_by_current(BATTERY_FLOAT_CURRENT, 5,
      BATTERY_FLOAT_MILLIVOLTS - 100, 
      BATTERY_GASSING_MILLIVOLTS,
      5000,
      CONVSTATE_FLOAT,
      CONVSTATE_ABSORB,
      CONVSTATE_FLOAT);
   
      break; 
  
 
    default:
      break;
  }  
  analogWrite(PWMPIN, converter.pwm);
}    

/*
* Main loop
*/

void loop()
{
  static uint8_t ticks;
  
 
  if(!timer.acquire)
    return;
  timer.acquire = false;  
  
  // Everything below here runs every 10 mSec.
 
  update_values();
  converter_ctrl_loop();
 
  if(ticks++ >= 99){
    ticks = 0;
    //debug(0, "Battery Millivolts raw %u", sensor_values.batt_mv);
    debug(0, "Battery Millivolts filtered %u", sensor_values.batt_mv_filt);
    //debug(0, "PV Millivolts raw %u", sensor_values.pv_mv);
    debug(0, "PV Millivolts filtered %u", sensor_values.pv_mv_filt);
    //debug(0, "Converter Milliamps raw %u", sensor_values.conv_ma);
    debug(0, "Converter Milliamps filtered %u", sensor_values.conv_ma_filt);
    debug(0, "Load Milliamps filtered %u", sensor_values.load_ma_filt);
    debug(0, "Battery Milliamps filtered %i", sensor_values.batt_ma_filt);
    debug(0, "Converter power %u mW", sensor_values.conv_power_mw);
    //debug(0, "Load power %u mW", sensor_values.load_power_mw);
    //debug(0, "Battery power %u mW", sensor_values.batt_power_mw);
    debug(0, "Converter state %u", converter.state);
    debug(0, "Servo current state %u", converter.servocurrentstate);
    debug(0, "Converter pwm %u", converter.pwm);
    //debug(0, "Charge timer: %u", read_timer(&timer.charge));
    debug(0,"\n");
    
  }
}
