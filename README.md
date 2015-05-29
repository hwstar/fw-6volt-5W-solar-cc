**fw-6volt-5W-solar-cc**

*Introduction*

This is the firmware to control my 6V solar charge controller
(For the hardware, see my [6volt-5W-solar-cc project](https://github.com/hwstar/6volt-5W-solar-cc) )
The charge controller firmware monitors the PV voltage, converter
current, load current, and battery voltage and adjusts the PWM 
output to keep the battery charging power at an optimum level
depending on the terminal voltage of the battery.

The buck converter in the charger hardware works in both continuous
and discontinuous mode. Continuous mode is used at higher charge
currents and discontinuous mode is used when charge currents are low.

The charger has 3 stages. BULK, ABSORB and FLOAT. The voltage 
thresholds which control transition to each mode are adjusted based
on temperature. A single LM335 temperature sensor provides battery
temperature input to the control algorithm.
 
The BULK charge state tries to dump as much energy into the battery 
as possible. The battery terminal voltage is monitored and when it 
reaches the programmed threshold. The PV voltage is also monitored 
every 30 seconds and if it deviates 5 % from the last voltage noted
at maximum power, a scan will be done to determine the the point where
the solar panel is delivering maximum power. 

The ABSORB charge state charges the battery at 1/10C to the gassing
voltage. Once this voltage is reached, the charger changes to the FLOAT
state. 
 
The FLOAT state keeps the battery terminal voltage at a low voltage
to prevent battery gassing. If load currents get too high, the charger
will transition out of this state into ABSORB or BULK depending on the
current demand from the load.
 
When a minimum power conversion threshold is reached or the PV voltage
drops below what is required for useable energy harvesting, the 
charger will go to sleep. In this state, the charger will monitor the
PV voltage periodically and make a decision to wake up or not.

There are several I2C commands to get data from the charger, perform
calibration, and enable some loads to be switched on and off.

A calibration procedure should be performed after firmware is loaded
to null out the tolerances of the  voltage sense resistors. See the
I2C commands for details. Calibration values are stored in EEPROM
so this procedure only needs to be done once. A semi-precision 6 volt
source should be used to power the board during the calibration procedure.
One can make such a source using an LM317 and a 3 1/2 digit DMM which
is good enough to get a successful calibration result. I wrote
a dedicated Python/tkinter utility perform the calibration using a 
Buspirate. See [chargectrlr-python-buspirate](https://github.com/hwstar/chargectrlr-python-buspirate)
 
This code was tested on an Arduino mini pro 328. With suitable
modifications, it should run on any 5 volt Arduino, but make sure
the Arduino you are using has a voltage regulator rated for 1% load
regulation so that the analog sense voltages will be accurate. Arduinos
using the MIC5205 5V regulator are preferred.

*License*

GNU GPLv3

