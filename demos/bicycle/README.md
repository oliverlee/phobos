This demo simulates the Whipple bicycle model using the benchmark parameters. A
Kalman filter is used to compute the state estimate for timing purposes; no
noise is added to the output of the Whipple model before it is input to the
Kalman filter. The initial state is set to zero and a periodic pulse (5.0 sec period, 2.0 N-m amplitude, 20% duty cycle) is applied to steer torque.

The simulation runs at roughly 200 Hz. At the end of each simulation loop, the
following information is transmitted over serial as ASCII:
 - error between the estimate error for state and auxiliary state in SI units
 - model state update time in realtime counts
 - Kalman filter estimate update time in realtime counts
 - model auxiliary state update in realtime counts
