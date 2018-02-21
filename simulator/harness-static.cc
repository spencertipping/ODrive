#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  m.rotor_position = 0.2;       // not evenly cogged; we want it to move

  // print header row
  cout << "time\t" << motor_header << "bemf_ab\t" << endl;

  // run for 100ms, 100ns at a time; then coast for 900ms
  real const dt = 100*NANO;
  for (real t = 0; t < 1;)
  {
    cout << t << "\t" << m << m.bemf_ab(0) << "\t" << endl;
    for (int i = 0; i < 100; ++i, t += dt) m.step(dt, t < 0.1, 0, 0);
  }

  return 0;
}
