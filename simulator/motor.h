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
  std::string name;
  int  pole_pairs;                    // Hz/turn
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


  motor_parameters(std::string const name_,
                   int  const pole_pairs_,
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
    : name         (name_),
      pole_pairs   (pole_pairs_),
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
  inline real trapezoid(real const phase) const
  {
    let pmod = std::fmod(phase, r(1));
    return pmod < r(1.0/6) ?  pmod * 6
         : pmod < r(2.0/6) ?  1
         : pmod < r(3.0/6) ?  1 - (pmod - r(2.0/6)) * 6
         : pmod < r(4.0/6) ?  0 - (pmod - r(3.0/6)) * 6
         : pmod < r(5.0/6) ? -1
         :                   -1 + (pmod - r(5.0/6)) * 6;
  }

  inline real tsin(real const phase) const   // TODO: improve accuracy
  { return sin(phase * TAU) * (1 - trapezoidal_bias)
         + trapezoid(phase) * trapezoidal_bias; }


  // Stator geometry (normalized Clarke transform)
  inline real i_alpha(real const a, real const b, real const c) const
  { return r(2.0/3) * (a - (b + c) / 2); }

  inline real i_beta (real const a, real const b, real const c) const
  { return r(2.0/3) * (sqrt(r(3)) * b - sqrt(r(3)) * c); }


  // Rotor geometry
  // TODO: with trapezoidal sin(), these vector components may not norm to 1.
  // How much of a problem is this?
  inline real d_dot_alpha(real const phase) const { return tsin(phase);           }
  inline real d_dot_beta (real const phase) const { return tsin(phase + r(0.25)); }
  inline real q_dot_alpha(real const phase) const { return tsin(phase + r(0.25)); }
  inline real q_dot_beta (real const phase) const { return tsin(phase + r(0.5));  }

  inline real q_dot_a(real const phase) const { return q_dot_alpha(phase); }
  inline real q_dot_b(real const phase) const { return -0.5 * q_dot_alpha(phase) + sqrt(r(3))/2 * q_dot_beta(phase); }
  inline real q_dot_c(real const phase) const { return -0.5 * q_dot_alpha(phase) - sqrt(r(3))/2 * q_dot_beta(phase); }

  inline real magnetic_torque(real const phase, real const ab, real const ac) const
  {
    let a = ab + ac;                    // A
    let b = -ab;                        // A
    let c = -ac;                        // A

    let iq = i_alpha(a, b, c) * q_dot_alpha(phase)
           + i_beta(a, b, c)  * q_dot_beta(phase);

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
motor_parameters const c580l("turnigy C580L 580Kv",
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
  real iab            = 0;              // A
  real iac            = 0;              // A

  real stator_joules  = 0;              // J (electrical/core losses)
  real dynamic_joules = 0;              // J (friction losses)

  motor_parameters const *p;


  motor(motor_parameters const &p_) : p(&p_) {}


  // Coil properties
  inline real ia(void) const { return iab + iac; }
  inline real ib(void) const { return -iab; }
  inline real ic(void) const { return -iac; }


  // Rotor positioning
  inline real phase_at(real const t) const { return p->pole_pairs * (rotor_position + t*rotor_velocity); }


  // Back-EMF
  inline real emf_magnitude(void)   const { return rotor_velocity * TAU * p->kt * r(2.0/3); }
  inline real bemf_ab(real const t) const { return emf_magnitude() * (p->q_dot_a(phase_at(t)) - p->q_dot_b(phase_at(t))); }
  inline real bemf_ac(real const t) const { return emf_magnitude() * (p->q_dot_a(phase_at(t)) - p->q_dot_c(phase_at(t))); }


  // Time stepping
  void step(real const dt, real const va, real const vb, real const vc)
  {
    let vab = va - vb;
    let vac = va - vc;

    // Integrate coil currents with RK4
    let dt2         = dt/2;
    let ab_bemf_t0  = bemf_ab(0);
    let ac_bemf_t0  = bemf_ac(0);
    let ab_bemf_t12 = bemf_ab(dt2);
    let ac_bemf_t12 = bemf_ac(dt2);
    let ab_bemf_t1  = bemf_ab(dt);
    let ac_bemf_t1  = bemf_ac(dt);

    // RK4 terms: k1, k2, k3, and k4
    let r           = p->phase_resistance;
    let ab_didt_k1  = vab - ab_bemf_t0  - iab * r;
    let ac_didt_k1  = vac - ac_bemf_t0  - iac * r;
    let ab_didt_k2  = vab - ab_bemf_t12 - (iab + dt2*ab_didt_k1) * r;
    let ac_didt_k2  = vac - ac_bemf_t12 - (iac + dt2*ac_didt_k1) * r;
    let ab_didt_k3  = vab - ab_bemf_t12 - (iab + dt2*ab_didt_k2) * r;
    let ac_didt_k3  = vac - ac_bemf_t12 - (iac + dt2*ac_didt_k2) * r;
    let ab_didt_k4  = vab - ab_bemf_t1  - (iab + dt*ab_didt_k3)  * r;
    let ac_didt_k4  = vac - ac_bemf_t1  - (iac + dt*ac_didt_k3)  * r;

    // Calculate the numerical deltas, but don't apply them yet. We want to use
    // the midpoints for second-stage integrals.
    let ab_di   = 1/p->phase_inductance * dt/6 * (ab_didt_k1 + 2*ab_didt_k2 + 2*ab_didt_k3 + ab_didt_k4);
    let ac_di   = 1/p->phase_inductance * dt/6 * (ac_didt_k1 + 2*ac_didt_k2 + 2*ac_didt_k3 + ac_didt_k4);
    let mid_iab = iab + ab_di/2;
    let mid_iac = iac + ab_di/2;

    // Calculate core and electrical losses for this timestep.
    let mean_current     = mid_iab + mid_iac;
    let voltage_drop     = mean_current * r;
    let resistive_joules = mean_current * voltage_drop * dt;
    let core_joules      = fabs(ab_di) + fabs(ac_di);

    // Net rotor torque
    let windage_torque   = p->windage_loss  * rotor_velocity * fabs(rotor_velocity);
    let friction_torque  = p->friction_loss * rotor_velocity;
    let magnetic_torque  = p->magnetic_torque(phase_at(dt2), mid_iab, mid_iac);
    let net_torque       = magnetic_torque - windage_torque - friction_torque;

    // Integrate into velocity
    let acceleration = net_torque       // N·m = kg m²/s²
                     * dt               // kg m²/s
                     / p->rotor_inertia // 1[radian]/s
                     / TAU;             // turn/sec

    // Update state
    stator_joules  += resistive_joules + core_joules;
    dynamic_joules += fabs(windage_torque + friction_torque)        // N·m
                    * fabs((rotor_velocity + acceleration/2) * TAU) // N·m/s
                    * dt;                                           // N·m

    rotor_position += dt * (rotor_velocity + acceleration/2);
    rotor_velocity += acceleration;
    iab            += ab_di;
    iac            += ac_di;
  }
};


class motor_header_t {};
motor_header_t const motor_header;

std::ostream &operator<<(std::ostream &os, motor_header_t const &h);
std::ostream &operator<<(std::ostream &os, motor const &m);


// Debugging/logging
std::ostream &operator<<(std::ostream &os, motor_header_t const &h)
{
  return os << "name\t"
            << "position\t"
            << "velocity\t"
            << "iab\t"
            << "iac\t"
            << "stator_joules\t"
            << "dynamic_joules\t";
}

std::ostream &operator<<(std::ostream &os, motor const &m)
{
  return os << m.p->name        << "\t"
            << m.rotor_position << "\t"
            << m.rotor_velocity << "\t"
            << m.iab            << "\t"
            << m.iac            << "\t"
            << m.stator_joules  << "\t"
            << m.dynamic_joules << "\t";
}


}


#endif
