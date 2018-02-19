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

## Static friction
This one is probably the easiest to infer: when the ODrive spins up a brushless
motor, it applies 10A along the D axis (rotor-aligned). The assumption is that
this is sufficient to overcome any default loading.

So if we want to measure this, we would do a series of startups at increasing
binary-spaced intervals and continue searching until we see the flux move. Then
we have two ends of a binary split if we want to refine.

## Dynamic friction
Linear component of steady-state current?

## Windage losses
Quadratic component of steady-state current?

## Angular momentum
The motor's Kv constant converts current to torque, so we should be able to
measure angular momentum in two ways. First, apply some fixed current to the
quadrature axis (like 10A) for a short amount of time; then measure the rate of
rotor acceleration. Then drop all current and measure deceleration, which should
be a bit faster.
