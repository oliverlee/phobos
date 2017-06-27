This project runs static simulator code. It transmits sensor values over serial
in ASCII.

The following sensors are used:
 - steer encoder, TIM5, 115200 counts/rev with index

The main loop transmits the following at 1 kHz:
 - short gitsha1
 - realtime counter [rtcnt_t]
 - steer encoder count
The realtime clock runs at 168 MHz for the STM32F405.
