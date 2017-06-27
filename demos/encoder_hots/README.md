Activates incremental encoder with 15200 counts per revolution using
Higher-Order Time-Sampling with polynomial order 2, encoder event buffer length
16, and skip order 0.

This demo reads an incremental encoder with 15200 counts per revolution using
Higher-Order Time-Sampling. The encoder is connected to TIM5 in encoder mode via
pins PA0 and PA1 for channels A and B. PA2 is used for encoder index channel Z.

The encoder count, velocity, and acceleration are transmitted over serial as
ASCII at roughly 10 Hz.
