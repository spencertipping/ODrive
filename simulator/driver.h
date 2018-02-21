#ifndef DRIVER_H
#define DRIVER_H

#include <iostream>

#include "motor.h"


namespace simulator
{

class driver
{
public:
  motor assumed;
  motor actual;
};


}

#endif
