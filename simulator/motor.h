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


class motor
{
public:
  // Time-variant state
  double rotor_position   = 0;                // absolute rotor turns (τ)
  double rotor_velocity   = 0;                // τ/sec
  double ab_current       = 0;                // A
  double ac_current       = 0;                // A

  // Debugging
#ifdef DEBUG
  double transient_driven_ab = 0;
  double transient_driven_ac = 0;
  double transient_emf_ab    = 0;
  double transient_emf_ac    = 0;

  double transient_windage_torque = 0;
  double transient_friction_torque = 0;
  double transient_magnetic_torque = 0;

  double transient_d_velocity = 0;
#endif

  // NB: these parameters are unusual in that they don't follow the ideal
  // three-phase model; they instead model the common BLDC configuration that
  // involves multiple pole-cycles per revolution. I'm not modeling actual
  // revolutions anywhere in the code; these quantities are converted to the
  // ideal three-phase model.
  //
  // flux_linkage is measured in peak V / (rad/sec); this also serves as the
  // motor's torque constant (N·m / quadrature amp).
  double flux_linkage     = 5.51328895422 / (MOTOR_KV * MOTOR_POLE_PAIRS);

  double rotor_inertia    = ROTOR_INERTIA;    // kg·m²
  double trapezoidal_bias = 0.7;              // interpolation factor

  double rotor_windage    = 1e-4;             // N·m / (τ²/s)
  double dynamic_friction = 1e-5;             // N·m / (τ/s)
  double cogging_torque   = 0.01;             // N·m
  double phase_resistance = 0.2;              // ohms
  double phase_inductance = 0.0005;           // henries


  motor(void) {}


  // Time stepping
  void step(void);
  inline double time(void) const { return time_at(cycles); }

  // Hardware functions
  inline void drive(double const a, double const b, double const c)
  {
    a_pwm = (uint16_t) (a * PWM_CLOCKS);
    b_pwm = (uint16_t) (b * PWM_CLOCKS);
    c_pwm = (uint16_t) (c * PWM_CLOCKS);
  }

  uint16_t adc_shunt_b(void) const;
  uint16_t adc_shunt_c(void) const;

  // Internal functions
  uint16_t adc_sample_of(double real_voltage) const;


  inline double rotor_at    (double   const t)      const { return rotor_position + t*rotor_velocity; }
  inline double time_at     (uint64_t const cycles) const { return (double) cycles / CPU_HZ; }

  inline double driven_va_at(uint64_t const cycles) const { return (cycles & PWM_CLOCKS - 1) < a_pwm ? vbus : 0; }
  inline double driven_vb_at(uint64_t const cycles) const { return (cycles & PWM_CLOCKS - 1) < b_pwm ? vbus : 0; }
  inline double driven_vc_at(uint64_t const cycles) const { return (cycles & PWM_CLOCKS - 1) < c_pwm ? vbus : 0; }

  inline double driven_ab_at(uint64_t const cycles) const { return driven_va_at(cycles) - driven_vb_at(cycles); }
  inline double driven_ac_at(uint64_t const cycles) const { return driven_va_at(cycles) - driven_vc_at(cycles); }
};


class motor_header_t {};
motor_header_t const motor_header;

std::ostream &operator<<(std::ostream &os, motor_header_t const &h);
std::ostream &operator<<(std::ostream &os, motor const &m);


}


#endif
