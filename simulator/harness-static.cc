#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m;
  m.drive(0.7, 0, 0);           // ab = ac, drive A high

  // print header row
  cout << "t\trotor_pos\trotor_vel\tab_current\tac_current\tshunt_b\tshunt_c"
       << endl;

  // run for 10ms of simulated time
  while (m.time() < 0.01)
  {
    m.step();
    cout << m.time()         << "\t"
         << m.rotor_position << "\t"
         << m.rotor_velocity << "\t"
         << m.ab_current     << "\t"
         << m.ac_current     << "\t"
         << m.adc_shunt_b()  << "\t"
         << m.adc_shunt_c()  << "\t"
         << endl;
  }

  return 0;
}
