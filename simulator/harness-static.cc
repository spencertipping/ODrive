#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  m.rotor_position = 0.2;       // not evenly cogged; we want it to move

  // print header row
  cout << "time\t"
       << motor_header << "\t"
       << endl;

  real dt = 100*NANO;
  for (real_mut t = 0; t < 1;)
  {
    cout << t << "\t"
         << m << "\t"
         << endl;
    for (int i = 0; i < 100; ++i, t += dt) m.step(dt, 1, 0, 0);
  }

  return 0;
}
