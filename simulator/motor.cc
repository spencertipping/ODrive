#include <stdlib.h>

#include "motor.h"

namespace simulator
{


// std::ostream stuff
std::ostream &operator<<(std::ostream &os, motor_header_t const &h)
{
  return os << "cycles:#0\t"
            << "time:#1\t"
            << "rotor_position:#2\t"
            << "rotor_velocity:#3\t"
            << "ab_current:#4\t"
            << "ac_current:#5\t"
            << "a_pwm:#6\t"
            << "b_pwm:#7\t"
            << "c_pwm:#8\t"
# ifdef DEBUG
            << "transient_driven_ab:#9\t"
            << "transient_driven_ac:#10\t"
            << "transient_emf_ab:#11\t"
            << "transient_emf_ac:#12\t"
            << "transient_windage_torque:#13\t"
            << "transient_friction_torque:#14\t"
            << "transient_magnetic_torque:#15\t"
            << "transient_d_velocity:#16\t"
# endif
            << "v33:#17\t"
            << "vbus:#18\t"
            << "vbus_divider:#19\t"
            << "shunt_resistance:#20\t"
            << "flux_linkage:#21\t"
            << "pwm_cycle_time:#22\t"
            << "rotor_inertia:#23\t"
            << "trapezoidal_bias:#24\t"
            << "rotor_windage:#25\t"
            << "dynamic_friction:#26\t"
            << "cogging_torque:#27\t"
            << "phase_resistance:#28\t"
            << "phase_inductance:#29\t"
            << "shunt_amp_gain:#30\t"
            << "adc_jitter:#31\t"
            << "shunt_b_error:#32\t"
            << "shunt_c_error:#33\t"
            << "adc_shunt_b:#34\t"
            << "adc_shunt_c:#35\t"
            << "driven_va:#36\t"
            << "driven_vb:#37\t"
            << "driven_vc:#38\t"
            << "driven_ab:#39\t"
            << "driven_ac:#40\t"
            << std::endl;
}


std::ostream &operator<<(std::ostream &os, motor const &m)
{
  return os << m.cycles         << "\t"
            << m.time()         << "\t"
            << m.rotor_position << "\t"
            << m.rotor_velocity << "\t"
            << m.ab_current     << "\t"
            << m.ac_current     << "\t"
            << m.a_pwm          << "\t"
            << m.b_pwm          << "\t"
            << m.c_pwm          << "\t"
# ifdef DEBUG
            << m.transient_driven_ab << "\t"
            << m.transient_driven_ac << "\t"
            << m.transient_emf_ab    << "\t"
            << m.transient_emf_ac    << "\t"
            << m.transient_windage_torque  << "\t"
            << m.transient_friction_torque << "\t"
            << m.transient_magnetic_torque << "\t"
            << m.transient_d_velocity      << "\t"
# endif
            << m.v33              << "\t"
            << m.vbus             << "\t"
            << m.vbus_divider     << "\t"
            << m.shunt_resistance << "\t"
            << m.flux_linkage     << "\t"
            << m.pwm_cycle_time   << "\t"
            << m.rotor_inertia    << "\t"
            << m.trapezoidal_bias << "\t"
            << m.rotor_windage    << "\t"
            << m.dynamic_friction << "\t"
            << m.cogging_torque   << "\t"
            << m.phase_resistance << "\t"
            << m.phase_inductance << "\t"
            << m.shunt_amp_gain   << "\t"
            << m.adc_jitter       << "\t"
            << m.shunt_b_error    << "\t"
            << m.shunt_c_error    << "\t"
            << m.adc_shunt_b()    << "\t"
            << m.adc_shunt_c()    << "\t"
            << m.driven_va_at(m.cycles) << "\t"
            << m.driven_vb_at(m.cycles) << "\t"
            << m.driven_vc_at(m.cycles) << "\t"
            << m.driven_ab_at(m.cycles) << "\t"
            << m.driven_ac_at(m.cycles) << "\t"
            << std::endl;
}


// motor instance methods
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
                          + ((double) random() / RAND_MAX - 0.5) * adc_jitter;

  // NB: keep it signed until we clip; otherwise we'll introduce wrapping
  // errors, which the hardware certainly doesn't have.
  int16_t measured = (int16_t) (with_error / v33 * 4096 + 0.5);
  return (uint16_t) (measured < 0 ? 0 : measured > 4095 ? 4095 : measured);
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

# ifdef DEBUG
    transient_driven_ab = ab_drive_t0;
    transient_driven_ac = ac_drive_t0;
# endif

  // Back-EMF (assumed to be constant within each timestep)
  // NB: this math looks wrong, but it isn't: we actually observe a trapezoidal
  // waveform from the coils with a lot of motors, so using the trapezoidal sine
  // function directly against the quadrature should give us the unscaled EMF.
  //
  // NB: all currents are modeled in terms of inflow through a coil, which means
  // ab_current is inflow through A + outflow through B. Mathematically, then,
  // ab_current = a_current - b_current.
  double const rotor_turns = rotor_at(t);
  double const q_alpha_un  = tsin(rotor_turns);
  double const q_beta_un   = tsin(rotor_turns + 0.25);

  // Normalize the quadrature coordinates
  double const q_norm  = 1.0 / sqrt(q_alpha_un*q_alpha_un + q_beta_un*q_beta_un);
  double const q_alpha = q_alpha_un * q_norm;
  double const q_beta  = q_beta_un * q_norm;

  double const emf_mag = rotor_velocity /* turns/s */
                       * TAU            /* * rad/turn = rad/s */
                       * flux_linkage;  /* * V/(rad/s) = V */

  double const q_dot_a = q_alpha;
  double const q_dot_b = -0.5 * q_alpha - sqrt(3)/2 * q_beta;
  double const q_dot_c = -0.5 * q_alpha + sqrt(3)/2 * q_beta;
  double const emf_ab  = emf_mag * (q_dot_a - q_dot_b);
  double const emf_ac  = emf_mag * (q_dot_a - q_dot_c);

# ifdef DEBUG
    transient_emf_ab = emf_ab;
    transient_emf_ac = emf_ac;
# endif

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
  //
  // These two resistive factors always apply against the current velocity
  // direction, so they need to have the same sign as the velocity. We subtract
  // them in the net torque calculation below.
  double const windage_torque  = rotor_windage    * rotor_velocity * fabs(rotor_velocity);
  double const friction_torque = dynamic_friction * rotor_velocity;

  // The flux linkage works both ways: here we use it to convert from
  // quadrature-aligned amps to N·m.
  //
  // Sanity check on these definitions:
  //
  //   A + B + C = 0
  //
  //   A =  AB + AC
  //   B = -AB + BC
  //   C = -AC - BC
  //
  // We can set BC = 0 because coil A can act as a virtual join point, in this
  // case netting no current. So we have:
  //
  //   A = AB + AC
  //   B = -AB
  //   C = -AC

  double const a_current = mean_ab_current + mean_ac_current;
  double const b_current = -mean_ab_current;
  double const c_current = -mean_ac_current;

  double const alpha_current = a_current - 0.5 * (b_current + c_current);
  double const beta_current  = -sqrt(3) * b_current + sqrt(3) * c_current;

  double const magnetic_torque =
    (alpha_current * q_alpha + beta_current * q_beta)   /* quadrature A */
    * flux_linkage;                                     /* * N·m/A = N·m */

# ifdef DEBUG
    transient_windage_torque  = windage_torque;
    transient_friction_torque = friction_torque;
    transient_magnetic_torque = magnetic_torque;
# endif

  double const net_torque = magnetic_torque - windage_torque - friction_torque;

  // Now update the simulator state. These integrals are all evaluated using
  // Euler's method, except for the rotor position which is trapezoidal.
  double const d_velocity = net_torque      /* N·m = kg m²/s² */
                          * dt              /* * s = kg m²/s */
                          / rotor_inertia   /* / kg m² = 1/s (implied radians) */
                          / TAU;            /* / rad/turn = turns/s */

# ifdef DEBUG
    transient_d_velocity = d_velocity;
# endif

  rotor_position += dt * (rotor_velocity + d_velocity / 2);
  rotor_velocity += d_velocity;
  cycles         += dc;
  ab_current     += d_ab_current;
  ac_current     += d_ac_current;
}


}
