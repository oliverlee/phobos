This project runs static simulator code. It transmits sensor values over serial
in ASCII.

The following sensors are used:
 - steer encoder, TIM5, 115200 counts/rev with index
 - rear wheel encoder, TIM3, 1152 counts/rev, velocity estimate updated every 1 ms
 - kistler steer torque, ADC12, 1 kHz

The following actuators are used:
 - handlebar, DAC1, __torque reference__, 20 Hz

The main loop transmits the following at 20 Hz:
 - short gitsha1
 - kistler steer torque [N/m]
 - kollmorgen actual torque [N/m]
 - steer angle [rad]
 - roller angle [rad]
 - bicycle forward velocity [m/s]
The handlebar torque is a sinusoid with amplitude 5 N/m and a period of 1
second.
