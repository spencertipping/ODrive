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
  uint64_t const dc = 2;                          // Δ(cycles)
  double const   dt = time_at(dc) - time_at(0);   // Δ(time)
  double const   t  = time_at(cycles);            // time

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

  // Back-EMF (assumed to be constant within each timestep)
  // NB: this math looks wrong, but it isn't: we actually observe a trapezoidal
  // waveform from the coils with a lot of motors, so using the trapezoidal sine
  // function directly against the quadrature should give us the unscaled EMF.
  double const rotor_turns = rotor_at(t);
  double const q_alpha     = tsin(rotor_turns);
  double const q_beta      = tsin(rotor_turns + 0.25);

  // TODO: the below dot products don't look right; how do we get 1.5α total?
  // Do we need to model emf_bc?
  double const emf_magnitude = rotor_velocity * TAU * flux_linkage;
  double const emf_ab =
    emf_mag * (q_alpha - (-0.5 * q_alpha + -sqrt(3)/2.0 * q_beta));
  double const emf_ac =
    emf_mag * (q_alpha - (-0.5 * q_alpha +  sqrt(3)/2.0 * q_beta));

  // Ohmic resistance (assumed to be constant per timestep)
  double const ab_ohmic = phase_resistance * ab_current;
  double const ac_ohmic = phase_resistance * ac_current;

  // RK4 factors
  double const ab_didt_k1 = ab_drive_t0 - emf_ab - ab_current * phase_resistance;
  double const ac_didt_k1 = ac_drive_t0 - emf_ac - ac_current * phase_resistance;
  double const ab_didt_k2 = ab_drive_t1 - emf_ab - (ab_current + dt * ab_didt_k1 / 2.0) * phase_resistance;
  double const ac_didt_k2 = ac_drive_t1 - emf_ac - (ac_current + dt * ac_didt_k1 / 2.0) * phase_resistance;
  double const ab_didt_k3 = ab_drive_t1 - emf_ab - (ab_current + dt * ab_didt_k2 / 2.0) * phase_resistance;
  double const ac_didt_k3 = ac_drive_t1 - emf_ac - (ac_current + dt * ac_didt_k2 / 2.0) * phase_resistance;
  double const ab_didt_k4 = ab_drive_t2 - emf_ab - (ab_current + dt * ab_didt_k3) * phase_resistance;
  double const ac_didt_k4 = ac_drive_t2 - emf_ac - (ac_current + dt * ac_didt_k3) * phase_resistance;

  // Calculate midpoints to apply as forces
  double const d_ab_current    = 1/phase_inductance * dt/6 * (ab_didt_k1 + 2*ab_didt_k2 + 2*ab_didt_k3 + ab_didt_k4);
  double const d_ac_current    = 1/phase_inductance * dt/6 * (ac_didt_k1 + 2*ac_didt_k2 + 2*ac_didt_k3 + ac_didt_k4);
  double const mean_ab_current = ab_current + d_ab_current/2;
  double const mean_ac_current = ac_current + d_ac_current/2;

  // Step 2: calculate net force acting on the rotor. This consists of cogging
  // torque (TODO), dynamic friction, and windage losses, plus the magnetic
  // field.
  double const windage_torque  = rotor_windage    * rotor_velocity * rotor_velocity;
  double const friction_torque = dynamic_friction * rotor_velocity;

  // The flux linkage works both ways: here we use it to convert from
  // quadrature-aligned amps to N·m.
  // TODO
}



}