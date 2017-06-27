This project runs static simulator code.
It simulates the Whipple bicycle model and interfaces with the
[phobos-visualizer](https://gitlab.com/bikelab/phobos-visualizer) Unity
environment.
COBS framed protobuf messages are transmitted over serial.

The following sensors are used:
 - steer encoder, TIM5, 115200 counts/rev with index
 - rear wheel encoder, TIM3, 1152 counts/rev, velocity estimate updated every 1 ms
 - kistler steer torque, ADC12, 10 kHz
 - kollmorgen actual torque, ADC13, 10 kHz

The following actuators are used:
 - binary __flimnap_kinematic__
    - handlebar, DAC1, __torque reference__, 1 kHz
 - binary __flimnap_whipple__
    - handlebar, DAC1, __velocity reference__, 1 kHz
 - binary __flimnap_zero_input__
    - handlebar, DAC1, __velocity reference__, 1 kHz

Simulation loop rate is 1 kHz. This project creates multiple binaries
differences in configurations.
