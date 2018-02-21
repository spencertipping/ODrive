#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  m.rotor_position = 0.1;       // not evenly cogged; we want it to move

  // print header row
  cout << "time\t" << motor_header;

  // run for 1 second, 1 microsecond at a time
  for (real t = 0; t < 1;)
  {
    cout << t << "\t" << m;
    for (int i = 0; i < 100; ++i, t += 1*MICRO) m.step(1 * MICRO, 12, 0, 0);
  }

  return 0;
}
