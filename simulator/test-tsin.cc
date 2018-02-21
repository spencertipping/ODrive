#include <iostream>

#include "motor.h"

using namespace std;
using namespace simulator;


int main()
{
  motor m(c580l);
  for (double x = 0; x < TAU; x += 0.001)
    cout << x << "\t" << m.p->trapezoid(x) << "\t" << m.p->tsin(x) << endl;
  return 0;
}
