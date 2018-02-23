#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  m.rotor_position = 0.2;       // not evenly cogged; we want it to move

  // print header row
  cout << motor_header << endl;

  real dt = 100*NANO;
  while (m.time < 1)
  {
    cout << m << endl;
    for (int i = 0; i < 100; ++i) m.step(dt, 1, 0, 0);
  }

  return 0;
}
