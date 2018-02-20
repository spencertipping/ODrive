#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m;
  m.rotor_position = 0.1;       // not evenly cogged; we want it to move
  m.drive(0.7, 0, 0);           // ab = ac, drive A high

  // print header row
  cout << motor_header;

  // run for 10ms of simulated time, 200 clock cycles between printouts
  while (m.time() < 0.01)
  {
    cout << m;
    for (int i = 0; i < 100; ++i) m.step();
  }

  return 0;
}
