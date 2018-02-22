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
  for (real_mut t = 0; t < 1/m.rotor_velocity/7; t += 1 * MICRO)
  {
    let p     = m.phase_at(t);
    let emf_a = m.p->alpha_emf(p, m.rotor_velocity);
    let emf_b = m.p->beta_emf (p, m.rotor_velocity);
    cout << t << "\t"
         << emf_a << "\t"
         << emf_b << "\t"
         << endl;
  }

  return 0;
}
