This project runs static simulator code. It transmits sensor values over serial
in ASCII.

The following sensors are used:
 - kistler steer torque voltage, ADC12, 8 kHz
 - kollmorgen actual torque voltage, ADC13, 8 kHz

The main loop transmits the following at 1 kHz:
 - short gitsha1
 - realtime count [rtcnt_t]
 - kistler steer torque voltage [adccount_t]
 - kollmorgen actual torque voltage [adccount_t]

The analog channel is sampled at 8 kHz. Voltage values are converted to
`adccount_t` which are in the range [0, 4096).
