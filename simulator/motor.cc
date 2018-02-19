#include <stdlib.h>

#include "motor.h"

namespace simulator
{


uint16_t motor::adc_shunt_b() const
{
  return adc_sample_of(ab_current * shunt_resistance * shunt_amp_gain * (1.0 + shunt_b_error));
}

uint16_t motor::adc_shunt_c() const
{
  return adc_sample_of(ac_current * shunt_resistance * shunt_amp_gain * (1.0 + shunt_c_error));
}


uint16_t motor::adc_sample_of(double const real_voltage) const
{
  double const with_error = real_voltage
                          + ((double) rand() / RAND_MAX - 0.5) * adc_jitter;
  uint16_t measured = (uint16_t) (with_error / v33_voltage * 4096 + 0.5);
  return measured < 0 ? 0 : measured > 4095 ? 4095 : measured;
}


// Time stepping
// There are two time-variant factors to consider. One is obviously the magnetic
// flux, which should correspond directly to the coil currents. The other is the
// PWM switching state.
//
// I assume an ideal power supply: the FETs have no resistance and there is zero
// impedance. I also assume that FET switching happens instantly.

void motor::step()
{
  uint64_t const dc = 2;                // Δ(cycles)
  double const   dt = time_at(dc);      // Δ(time)

  // Step 1: integrate coil currents. This involves two factors: the drive
  // voltages and the back-EMF from the rotor. Let's break this down.
  //
  //   L = V / (dI/dt)                  # definition of inductance
  //   EMF = flux_linkage * rad/sec     # definition of flux linkage
  //   Vohm = I * phase_resistance      # ohmic resistance
  //
  // We need to solve for dI/dt in terms of net terminal voltage:
  //
  //   net_v = drive - EMF - Vohm
  //   dI/dt = net_v / L

  // Intermediate states
  double const ab_drive_t0 = driven_ab_at(cycles);
  double const ab_drive_t1 = driven_ab_at(cycles + dc/2);
  double const ab_drive_t2 = driven_ab_at(cycles + dc);

  double const ac_drive_t0 = driven_ac_at(cycles);
  double const ac_drive_t1 = driven_ac_at(cycles + dc/2);
  double const ac_drive_t2 = driven_ac_at(cycles + dc);

  // RK4 factors
  double const di_ab_k1;  // TODO
}



}
