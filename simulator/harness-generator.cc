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

  // Rotate the motor by one revolution to establish baseline currents
  for (real_mut t = 0; t < 1 / v0; t += dt)
  {
    m.rotor_velocity = v0;
    m.step(dt, 0, 0, 0);
  }

  cout << "time\t"
       << motor_header << "\t"
       << endl;

  // Let the rotor coast and generate some power from the rotor momentum
  for (real_mut t = 0; t < 1;)
  {
    cout << t << "\t"
         << m << "\t"
         << endl;
    for (int i = 0; i < 100; ++i, t += dt) m.step(dt, 0, 0, 0);
  }

  return 0;
}
