This demo reads an incremental encoder with 15200 counts per revolution. The
encoder is connected to TIM5 in encoder mode via pins PA0 and PA1 for channels A
and B. PA2 is used for encoder index channel Z.

The encoder uses First-Order Adaptive Windowing (FOAW) for velocity estimation.
The encoder velocity estimate is updated at 1 kHz and has an allowed encoder
count error of 1 count.

The encoder count and velocity are transmitted over serial as ASCII at roughly
10 Hz.

This demo is identical to the encoder demo except it also enables the index
channel.
