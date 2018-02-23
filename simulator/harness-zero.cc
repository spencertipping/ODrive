#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  m.rotor_position = 0;       // cog directly to zero

  // print header row
  cout << motor_header << "\t"
       << "mag_torque\t"
       << endl;

  real dt = 100*NANO;
  while (m.time < 1)
  {
    cout << m << "\t"
         << m.p->magnetic_torque(m.phase_at(0), m.ialpha, m.ibeta) << "\t"
         << endl;

    for (int i = 0; i < 100; ++i) m.step(dt, 1, 0, 0);
  }

  return 0;
}
