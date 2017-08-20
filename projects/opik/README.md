This project runs static simulator code. It transmits sensor values over serial
in ASCII and simulates a spring.

The following sensors are used:
 - steer encoder, TIM5, 115200 counts/rev with index
 - kistler steer torque voltage, ADC12, 10 kHz
 - kollmorgen actual torque voltage, ADC13, 10 kHz

The main loop transmits the following at 1 kHz:
 - short gitsha1
 - kistler steer torque [N/m]
 - kollmorgen actual torque [N/m]
 - steer angle [rad]

A virtual torsional spring with spring constant k = 3.14 N-m/rad is used to
generate the kollmorgen commanded motor velocity. Values are transmitted over
serial synchronously. The project will run only if data is being read from the
serial port and if disconnected, the velocity command will be held at the most
recent value.
