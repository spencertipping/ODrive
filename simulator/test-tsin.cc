#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m;
  for (double x = 0; x < TAU; x += 0.001)
    cout << x << "\t" << m.trapezoid(x) << "\t" << m.tsin(x) << endl;
  return 0;
}
