# Magnets and inductors
I don't really know how these things work, so I need to figure it out. I guess
let's start with the basics:

- A change in magnetic flux induces a voltage: Wb = V·s
- Inductance couples Δflux with current: L = Wb/A
- Flux and current aren't the same thing because current won't keep flowing if
  you wrap wire around a permanent magnet

Ok, so when an inductor creates a voltage it can be either because the current
is changing the flux (Wb = A·L) or because something else is -- for instance the
rotor magnets.

## Stator cores
Let's suppose we have just one coil in the motor:

```
             B   A
             |   |
             |   |
   S--0--N  =/////=

      ^        ^
      |        |
  rotor        stator
```

If we turn the rotor by 180°, then we'll obviously get a voltage between A and B
during the rotation. If the circuit is closed, the current induced in the coil
will oppose the change in flux, creating resistance. If the circuit is open,
there's no induced current and the flux can change unopposed.

The circuit-closed case is worth more discussion. Let's say the stator coil has
L = 1, A and B are connected with a 1Ω resistor, and the rotor is perfectly
coupled and is changing the stator flux by 1 Wb/s. Then we should see 1V of EMF,
and therefore a 1A current.

OK, what if the coil has no resistance at all? Then the EMF we get from any
magnetic movement would create an infinite current? That doesn't make sense.
What _might_ make sense, though, is if the current in the coil itself faced EMF
from trying to oppose the changing flux. This makes more sense if we look at the
infinitesimals:

```
ΔWb/Δt -> V -> A/Δt -> ΔWb/Δt -> -V · L
```

Or, in English, the flux change induces voltage, which becomes current, which
faces inductive voltage as it changes the flux in the other direction. Since the
same inductor is responsible for both changes and the circuit has no resistance,
the current will be whatever cancels the two voltages, in this case inversely
proportional to `L`.

If we remove the rotor from the motor entirely, we'll get a current in the
stator coil as the field normalizes to its default value.
