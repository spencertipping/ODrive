#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


// FIXME: this oscillates right now, but it should be critically damped and
// should end up in a nonzero position.

int main()
{
  motor m(c580l);
  let v0 = 9.666667;                // $ units -t 580rpm turns/sec
  let dt = 100*NANO;

  cout << "time\t"
       << motor_header << "\t"
       << endl;

  real_mut t = 0;

  // Rotate the motor for one second to establish baseline current
  while (t < 1)
  {
    cout << t << "\t"
         << m << "\t"
         << endl;

    for (int i = 0; i < 1000; ++i, t += dt)
    {
      m.rotor_velocity = v0 * t;
      m.step(dt, 0, 0, 0);
    }
  }

  // Let the rotor coast and generate some power from the rotor momentum
  while (t < 1.1)
  {
    cout << t << "\t"
         << m << "\t"
         << endl;
    for (int i = 0; i < 1000; ++i, t += dt) m.step(dt, 0, 0, 0);
  }

  return 0;
}
