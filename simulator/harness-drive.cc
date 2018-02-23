#include <iostream>

#include "motor.h"
#include "board.h"

using namespace std;
using namespace simulator;


int main()
{
  board_parameters const bp;
  motor m(c580l);
  board b(bp, m);

  cout << board_header << "\t" << motor_header << endl;

  // Create a field rotating at 10 turns/sec
  while (m.time < 2)
  {
    let alpha = cos(m.time * 10 * TAU * m.p->pole_pairs);
    let beta  = sin(m.time * 10 * TAU * m.p->pole_pairs);
    let va    = (m.p->a(alpha, beta) + 1) / 2;
    let vb    = (m.p->b(alpha, beta) + 1) / 2;
    let vc    = (m.p->c(alpha, beta) + 1) / 2;

    b.drive(va, vb, vc);
    b.step(1680);
    cout << b << "\t" << *b.m << endl;
  }

  return 0;
}
