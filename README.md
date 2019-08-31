# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## P, I and D

This section explains the effect of the `P`, `I` and `D` parameter when used to set the steering angle of a car that should follow a defined trajectory:

- The `P` component in the PID controller sets the steering angle towards the desired position `proportionally` to the error. This means, the bigger the parameter, the more aggressive will the controller head to the goal.

- The `D` component in the PID controller sets the steering angle stands for derivative. Depending on how big the change (therefore derivative) of the error is, the steering is being damped/strengthened.

- The `I` component (integral) increases steering if the error persists over time. This is necessary if the steering is slightly skewed.

## Parameter Tuning

I started to manually tune `P`, then `D` then `I`, resulting in `0.03, 0.00001, 6` as PID values. These parameters were able to move the car along the track for a whole round.

I then implemented the twiddle algorithm that was presented in the class. The challenging part was to simulate the control flow that was done in the class with `run(robot, p)` within the simulator.

I first ran twiddle with `[1,1,1]` similar to the class but noticed that (as I started with parameters that seem to be ok) `1` was a bit much and decreased `dp` to `[0.1, 0.1, 0.1]`.

The final values are `[0.577838, 1e-05, 7.48411]`. Watching the car drive with this PID controller, it feels a bit shaky. I assume that the reason is the high `D` value. I think this could potentially be fixed by incorporating a very high steering value into the error but I haven't tried this out.
