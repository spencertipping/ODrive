# Motor simulator
This is a simple simulator that models the motor as an ideal three-phase system.
I'm using
[d/q](https://en.wikipedia.org/wiki/Direct-quadrature-zero_transformation) to
simulate the coil states, then back-converting to phase B and C shunt voltages,
adding error, and simulating the ODrive software's D/Q/0 transform on those
modified values.

The goal here is to end up with a controller that figures out the motor dynamics
well enough to reliably control it. I think in this case that breaks down to a
few unknowns:

1. Rotor static friction
2. Rotor dynamic friction
3. Windage losses
4. Rotor angular momentum
5. Flux linkage

I'm testing the algorithm by creating two simulated motors, one "actual" and one
"assumed," and seeing if I can get the assumed motor to converge to the actual
parameters.

## Simulator details
The [model](motor.h) contains a bunch of hardware parameters that are used to
generate simulated measurements, e.g. via the ADCs. The simulator works by
time-stepping and evaluates integrals using
[RK4](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods#The_Runge%E2%80%93Kutta_method).

Internally, the simulator maintains state about a few things at the physical
level:

1. The field currents
2. PWM duty cycles
3. Rotor position/velocity/forces

The simulator's time base is synchronized to the CPU clock.

## How we infer stuff
### Static friction
This one is probably the easiest to infer: when the ODrive spins up a brushless
motor, it applies 10A along the D axis (rotor-aligned). The assumption is that
this is sufficient to overcome any default loading.

So if we want to measure this, we would do a series of startups at increasing
binary-spaced intervals and continue searching until we see the flux move. Then
we have two ends of a binary split if we want to refine.

### Dynamic friction
Linear component of steady-state current?

### Windage losses
Quadratic component of steady-state current?

### Angular momentum
The motor's Kv constant converts current to torque, so we should be able to
measure angular momentum in two ways. First, apply some fixed current to the
quadrature axis (like 10A) for a short amount of time; then measure the rate of
rotor acceleration. Then drop all current and measure deceleration, which should
be a bit faster.
