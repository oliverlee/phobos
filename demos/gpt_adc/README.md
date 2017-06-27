This demo reads ADC channels ADC10, ADC11, ADC12 and uses GPT triggering with a
continuous conversion frequency of 1 kHz and the ADC conversion group is [ADC10,
ADC11, ADC12]. The ADC values are transmitted over serial in ASCII in units of
`adcsample`, a 12-bit unsigned value corresponding to a range of 0V to 3.3V.
ADC events are used to transmit values as soon as conversion is complete.
