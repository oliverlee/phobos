This demo activates 2 PWM channels 1/4 period of of phase and an incremental
encoder.

The incremental encoder has 1024 counts per revolution and is connected to TIM3
in encoder mode via pins PA0 and PA1 for channels A and B.

PWM is configured on drivers PWMD1 (GPIOA8) and PWMD4 (GPIOB6) with 1 MHz clock
frequency, 1 kHz period, and 50% duty cycle. PWM output is 1/4 period out of
phase in order to simulate encoder channels A and B.

The encoder count is transmitted over serial as ASCII at 10 Hz.
