#ifndef MOTOR_H
#define MOTOR_H

#include <cmath>
#include <iostream>

#include <stdint.h>

#include "defines.h"


namespace simulator
{


class motor_parameters
{
public:
  real pole_pairs;                    // Hz/turn
  real rotor_inertia;                 // kg·m²
  real kt;                            // torque constant: N·m/A = V·sec/rad

  real phase_resistance;              // ohms
  real phase_inductance;              // henries
  real a_inductance_error;            // henries/henry
  real b_inductance_error;            // henries/henry
  real c_inductance_error;            // henries/henry
  real saliency;                      // D henries / Q henries
  real trapezoidal_bias;              // interpolation factor

  real hysteresis_loss;               // J/ΔA
  real windage_loss;                  // N·m/(turn/sec)²
  real friction_loss;                 // N·m/(turn/sec)
  real cogging_torque;                // N·m?


  motor_parameters(real const pole_pairs_,
                   real const rotor_weight_,    // g
                   real const rotor_radius_,    // mm
                   real const kv_,              // RPM/V
                   real const phase_resistance_,
                   real const phase_inductance_,
                   real const a_inductance_error_,
                   real const b_inductance_error_,
                   real const c_inductance_error_,
                   real const saliency_,
                   real const trapezoidal_bias_,
                   real const hysteresis_loss_,
                   real const windage_loss_,
                   real const friction_loss_,
                   real const cogging_torque_)
    : pole_pairs   (pole_pairs_),
      rotor_inertia(rotor_weight_ * MILLI * (rotor_radius_ * MILLI)
                                          * (rotor_radius_ * MILLI)),

      kt(9.5492966 / kv_),    // units -t 'V/rpm' 'N*m/A' -> 9.5492966

      phase_resistance(phase_resistance_),
      phase_inductance(phase_inductance_),
      a_inductance_error(a_inductance_error_),
      b_inductance_error(b_inductance_error_),
      c_inductance_error(c_inductance_error_),
      saliency(saliency_),
      trapezoidal_bias(trapezoidal_bias_),

      hysteresis_loss(hysteresis_loss_),
      windage_loss(windage_loss_),
      friction_loss(friction_loss_),
      cogging_torque(cogging_torque_) {}


  // Field geometry
  inline real trapezoid(real const turns)
  {
    let tmod = std::fmod(turns, r(1));
    return tmod < r(1.0/6) ?  tmod * 6
         : tmod < r(2.0/6) ?  1
         : tmod < r(3.0/6) ?  1 - (tmod - r(2.0/6)) * 6
         : tmod < r(4.0/6) ?  0 - (tmod - r(3.0/6)) * 6
         : tmod < r(5.0/6) ? -1
         :                   -1 + (tmod - r(5.0/6)) * 6;
  }

  inline real tsin(real const t)    // TODO: improve accuracy of this fn
  { return sin(t * TAU) * (1 - trapezoidal_bias)
         + trapezoid(t) * trapezoidal_bias; }


  // Stator geometry (normalized Clarke transform)
  inline real i_alpha(real const a, real const b, real const c) const
  { return r(2.0/3) * (a - (b + c) / 2); }

  inline real i_beta (real const a, real const b, real const c) const
  { return r(2.0/3) * (sqrt(r(3)) * b - sqrt(r(3)) * c); }


  // Rotor geometry
  // TODO: with trapezoidal sin(), these vector components may not norm to 1.
  // How much of a problem is this?
  inline real d_dot_alpha(real const p) const { return tsin(p);           }
  inline real d_dot_beta (real const p) const { return tsin(p + r(0.25)); }
  inline real q_dot_alpha(real const p) const { return tsin(p + r(0.25)); }
  inline real q_dot_beta (real const p) const { return tsin(p + r(0.5));  }

  inline real q_dot_a(real const p) const { return q_dot_alpha(p); }
  inline real q_dot_b(real const p) const { return -0.5 * q_dot_alpha(p) + sqrt(r(3))/2 * q_dot_beta(p); }
  inline real q_dot_c(real const p) const { return -0.5 * q_dot_alpha(p) - sqrt(r(3))/2 * q_dot_beta(p); }

  inline real magnetic_torque(real const p, real const ab, real const ac) const
  {
    let a = ab + ac;                    // A
    let b = -ab;                        // A
    let c = -ac;                        // A

    let iq = i_alpha(a, b, c) * q_dot_alpha(p)
           + i_beta(a, b, c)  * q_dot_beta(p);

    // This page explains the 3/4 factor; the gist is that it's a result of the
    // scale-invariant Clarke transform I use above.
    //
    // https://e2e.ti.com/support/microcontrollers/c2000/f/902/p/298101/1044389#1044389
    //
    // Torque (Newton-Meters) = Rotor Flux (Webers = Volt-seconds/radian)
    //                        * Iq (Amps)
    //                        * Rotor Magnet Poles
    //                        * (3/4)

    return kt * iq * pole_pairs * 2 * r(0.75);
  }
};


// Predefined motors
motor_parameters const turnigy_c580l(
  7,                                    // pole pairs
  100,                                  // rotor weight grams (my guess)
  12,                                   // rotor radius mm (my guess)
  580,                                  // RPM/V
  2 * MILLI,                            // phase resistance
  250 * MICRO,                          // phase inductance
  0, 0, 0,                              // phase inductance errors
  0,                                    // saliency
  r(0.8),                               // trapezoidal bias (TODO: measure)
  0,                                    // hysteresis loss
  0,                                    // windage loss
  r(10 * MICRO),                        // friction loss
  r(1 * MILLI));                        // cogging torque


// Time-variant motor state
class motor
{
public:
  real rotor_position = 0;              // absolute rotor turns
  real rotor_velocity = 0;              // turns/sec
  real ab_current     = 0;              // A
  real ac_current     = 0;              // A

  real stator_joules  = 0;              // J (electrical/core losses)
  real dynamic_joules = 0;              // J (friction losses)

  motor_parameters const *p;


  motor(motor_parameters const *const p_) : p(p_) {}


  // Coil properties
  inline real ia(void) const { return ab_current + ac_current; }
  inline real ib(void) const { return -ab_current; }
  inline real ic(void) const { return -ac_current; }


  // Rotor positioning
  inline real pos_at(real const t) const { return rotor_position + t*rotor_velocity; }


  // Back-EMF
  inline real emf_magnitude(void)   const { return rotor_velocity * TAU * kt; }
  inline real bemf_ab(real const t) const { return emf_magnitude() * (p->q_dot_a(pos_at(t)) - p->q_dot_b(pos_at(t))); }
  inline real bemf_ac(real const t) const { return emf_magnitude() * (p->q_dot_a(pos_at(t)) - p->q_dot_c(pos_at(t))); }


  // Time stepping: assume constant drive voltage over a timestep, and return
  // joules of mechanical output
  real step(real const dt, real const va, real const vb, real const vc)
  {
    let vab = va - vb;
    let vac = va - vc;

    // Integrate coil currents with RK4
    let ab_bemf_t0  = bemf_ab(0);
    let ac_bemf_t0  = bemf_ac(0);
    let ab_bemf_t12 = bemf_ab(dt/2);
    let ac_bemf_t12 = bemf_ac(dt/2);
    let ab_bemf_t1  = bemf_ab(dt);
    let ac_bemf_t1  = bemf_ac(dt);

    // TODO
  }
};


class motor_header_t {};
motor_header_t const motor_header;

std::ostream &operator<<(std::ostream &os, motor_header_t const &h);
std::ostream &operator<<(std::ostream &os, motor const &m);


}


#endif
