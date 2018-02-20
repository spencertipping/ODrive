#include <iostream>

#define DEBUG

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m;
  m.rotor_position = 0.4;       // not evenly cogged; we want it to move
  m.drive(0.7, 0, 0);           // ab = ac, drive A high

  // print header row
  cout << "t\trotor_pos\trotor_vel\tab_current\t"
       << "ac_current\tshunt_b\tshunt_c\temf_ab\t"
       << "windage\tfriction\tmagnetic_torque\n"
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
         << m.transient_emf_ab          << "\t"
         << m.transient_windage_torque  << "\t"
         << m.transient_friction_torque << "\t"
         << m.transient_magnetic_torque << "\t"
         << endl;
  }

  return 0;
}
