#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  m.rotor_velocity = 9.666667;          // $ units -t 580rpm turns/sec

  // Given the way back-EMF works, we should see peaks at ±1V per coil pair.

  // Measure the three phase back-EMF at each rotation point
  for (real t = 0; t < 1/m.rotor_velocity/7; t += 1 * MICRO)
    cout << t << "\t"
         << m.bemf_ab(t) << "\t"
         << m.bemf_ac(t) << "\t"
         << endl;

  return 0;
}
