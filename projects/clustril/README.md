This project runs static simulator code.
It simulates the Whipple bicycle model at a constant speed of 5.0 m/s and tests
basic file creation and write functionality.
The SDIO peripheral is used to write to a micro SD card and this prevents usage
of serial over USB.

The following sensors are used:
 - steer encoder, TIM5, 115200 counts/rev with index
 - rear wheel encoder, TIM3, 1152 counts/rev, velocity estimate updated every 1 ms
 - kistler steer torque, ADC12, 1 kHz
 - kollmorgen actual torque, ADC13, 1 kHz

The following actuators are used:
 - handlebar, DAC1, __velocity reference__, 200 Hz

Simulation loop rate is 200 Hz. Observer type is Kalman. Haptic type is
HandlebarStatic.
