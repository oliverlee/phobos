This project runs static simulator code. It transmits sensor values over serial
in ASCII and is used to verify torque sensor measurements and that the motor
drive position and steer encoder measurements are equal.

The following sensors are used:
 - steer encoder, TIM5, 115200 counts/rev with index
 - kistler steer torque voltage, ADC12, 10 kHz
 - kollmorgen actual position voltage, ADC13, 10 kHz

The main loop transmits the following at 1 kHz:
 - short gitsha1
 - kistler steer torque [N/m]
 - steer angle (encoder) [rad]
 - steer angle voltage (motor drive) [adccount_t]

The analog torque command to the motor drive is disabled. A drive parameter file
is included to redefine the analog output value. Analog output scaling is set to
4.5 deg/V. The project will run only if data is being read from the serial port
and if disconnected.
