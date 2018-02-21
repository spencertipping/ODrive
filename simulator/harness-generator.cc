#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  m.rotor_velocity = 9.666667;          // $ units -t 580rpm turns/sec

  cout << "time\t"
       << motor_header << "\t"
       << endl;

  // Let the rotor coast and generate some power
  real const dt = 100*NANO;
  for (real t = 0; t < 1;)
  {
    cout << t << "\t"
         << m << "\t"
         << endl;
    for (int i = 0; i < 100; ++i, t += dt) m.step(dt, 0, 0, 0);
  }

  return 0;
}
