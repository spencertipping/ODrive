#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  let v0 = 9.666667;                // $ units -t 580rpm turns/sec
  let dt = 100*NANO;

  cout << motor_header << endl;

  // Rotate the motor for one second to establish baseline current
  while (m.time < 1)
  {
    cout << m << endl;

    for (int i = 0; i < 1000; ++i)
    {
      m.rotor_velocity = v0 * m.time;
      m.step(dt, 0, 0, 0);
    }
  }

  // Let the rotor coast and generate some power from the rotor momentum
  while (m.time < 2)
  {
    cout << m << endl;
    for (int i = 0; i < 1000; ++i) m.step(dt, 0, 0, 0);
  }

  return 0;
}
