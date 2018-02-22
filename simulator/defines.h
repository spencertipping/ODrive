#ifndef DEFINES_H
#define DEFINES_H


#include <cassert>
#include <cmath>


#define must_be_finite(x) assert(std::isfinite(x))


// real = double for PC simulations, but on the board we probably want floats
#define real_mut double
#define real real_mut const
#define r(x) ((real) (x))

// Shorthand: "let x = 10;", for example
#define let auto const

// Unit suffixes
#define GIGA  (MEGA * KILO)
#define MEGA  (KILO * KILO)
#define KILO  1000

#define MILLI r(0.001)
#define MICRO (MILLI * MILLI)
#define NANO  (MICRO * MILLI)
#define PICO  (NANO  * MILLI)

// The circle constant: radians per turn (https://tauday.com/)
#define TAU r(2 * M_PI)


#endif
