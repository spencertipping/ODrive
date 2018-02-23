#ifndef MOTOR_H
#define MOTOR_H

#ifdef IO
# include <iostream>
#endif

#include "defines.h"


namespace simulator
{


constexpr static real sqrt3      = sqrt(r(3));
constexpr static real half_sqrt3 = sqrt3 / 2;
constexpr static real over_sqrt3 = 1 / sqrt3;

static inline constexpr int sgn(real x) { return (x > 0) - (x < 0); }


struct motor_parameters
{
  std::string name;
  int  pole_pairs;                    // Hz/turn
  real rotor_inertia;                 // kg·m²/radian²
  real kt;                            // torque constant: N·m/A = V·sec/rad

  real phase_resistance;              // ohms per coil
  real phase_inductance;              // henries per coil
  real saliency;                      // D henries / Q henries
  real trapezoidal_bias;              // interpolation factor

  real windage_loss;                  // (N·m/radian)/(turn/sec)
  real friction_torque;               // N·m/radian
  real cogging_torque;                // N·m/radian? (TODO)


  motor_parameters(std::string const name_,
                   int const pole_pairs_,
                   real rotor_weight_,    // g
                   real rotor_radius_,    // mm
                   real kv_,              // RPM/V
                   real phase_resistance_,
                   real phase_inductance_,
                   real saliency_,
                   real trapezoidal_bias_,
                   real windage_loss_,
                   real friction_torque_,
                   real cogging_torque_)

    : name         (name_),
      pole_pairs   (pole_pairs_),
      rotor_inertia(rotor_weight_ * MILLI * (rotor_radius_ * MILLI)
                                          * (rotor_radius_ * MILLI) / 2),

      kt(9.5492966 / kv_),    // $ units -t 'V/rpm' 'N*m/radian/A' -> 9.5492966

      phase_resistance(phase_resistance_),
      phase_inductance(phase_inductance_),
      saliency(saliency_),
      trapezoidal_bias(trapezoidal_bias_),

      windage_loss(windage_loss_),
      friction_torque(friction_torque_),
      cogging_torque(cogging_torque_) {}


  // Field geometry
  inline real trapezoid(real phase) const
  {
    let pmod = std::fmod(phase, r(1));
    return pmod < r(1.0/6) ?  pmod * 6
         : pmod < r(2.0/6) ?  1
         : pmod < r(3.0/6) ?  1 - (pmod - r(2.0/6)) * 6
         : pmod < r(4.0/6) ?  0 - (pmod - r(3.0/6)) * 6
         : pmod < r(5.0/6) ? -1
         :                   -1 + (pmod - r(5.0/6)) * 6;
  }

  inline real tsin(real phase) const    // TODO: improve accuracy
  { return sin(phase * TAU) * (1 - trapezoidal_bias)
         + trapezoid(phase) * trapezoidal_bias; }


  // Stator geometry (Clarke transform)
  constexpr inline real alpha(real b, real c) const { let a = -(b + c); return r(2.0/3) * (a - (b + c) / 2); }
  constexpr inline real beta (real b, real c) const { return r(2.0/3) * half_sqrt3 * (b - c); }

  constexpr inline real a(real alpha, real beta) const { return alpha; }
  constexpr inline real b(real alpha, real beta) const { return -alpha/2 + half_sqrt3*beta; }
  constexpr inline real c(real alpha, real beta) const { return -alpha/2 - half_sqrt3*beta; }


  // Rotor geometry
  inline real phase_norm(real phase) const
  {
    // A fix for cases where D and Q aren't unit vectors, which they won't be in
    // our trapezoidal-sine world.
    let x = tsin(phase);
    let y = tsin(phase + r(0.25));
    return sqrt(x*x + y*y);
  }

  inline real d_alpha(real phase) const { return tsin(phase - r(0.25)) / phase_norm(phase); }
  inline real d_beta (real phase) const { return tsin(phase)           / phase_norm(phase); }
  inline real q_alpha(real phase) const { return tsin(phase)           / phase_norm(phase); }
  inline real q_beta (real phase) const { return tsin(phase + r(0.25)) / phase_norm(phase); }


  // Stator electrical
  // We can coordinate-transform nonlinear elements like inductors because
  // inductance behaves like a constant factor in the current integral; that is,
  // we don't have voltage-dependent inductance.
  //
  // Saliency doesn't break this either; at any given moment in time, saliency
  // is also a linear factor that we can separate from the rest of the integral
  // and transform.
  //
  // C along alpha faces the following impedance, where Z is the impedance of a
  // single coil:
  //
  //     B
  //      \
  //       \
  //        .-----A
  //       /
  //     /
  //    C
  //
  //    |---|-----|
  //     Z/2   Z
  //
  // There are two Z/2 paths, so the collective impedance between the midpoint
  // and B/C is Z/4.
  //
  // Current along β is simpler: the collective impedance there ignores the
  // A<->midpoint connection, so it's just sqrt(3) * Z.

  inline real alpha_inductance(real phase) const
  { let q_l = phase_inductance * r(5.0/4);
    let d_l = saliency * q_l;
    return d_l * fabs(d_alpha(phase)) + q_l * fabs(q_alpha(phase)); }

  inline real beta_inductance(real phase) const
  { let q_l = sqrt3 * phase_inductance;
    let d_l = saliency * q_l;
    return d_l * fabs(d_beta(phase)) + q_l * fabs(q_beta(phase)); }

  inline real alpha_resistance(void) const { return r(5.0/4) * phase_resistance; }
  inline real beta_resistance (void) const { return sqrt3    * phase_resistance; }


  // Loss calculations
  inline real stator_power_alpha(real ialpha) const { return ialpha * (ialpha * alpha_resistance()); }
  inline real stator_power_beta (real ibeta)  const { return ibeta  * (ibeta  * beta_resistance()); }

  inline real stator_power(real ialpha, real ibeta) const
  {
    let palpha = stator_power_alpha(ialpha);
    let pbeta  = stator_power_beta(ibeta);
    return sqrt(palpha*palpha + pbeta*pbeta);
  }


  // Mechanical/electrical conversion
  // NB: kT is already defined in terms of real-world rotation, not phase
  // rotations. So we need to treat kT as though it's already multiplied by the
  // number of pole pairs.
  //
  // Back-EMF is caused by a change in flux, so we use the rotor quadrature.

  inline real emf_magnitude(real velocity) const { return velocity * TAU * kt; }
  inline real alpha_emf(real phase, real velocity) const { return emf_magnitude(velocity) * q_alpha(phase); }
  inline real beta_emf (real phase, real velocity) const { return emf_magnitude(velocity) * q_beta(phase); }

  inline real magnetic_torque(real phase, real ialpha, real ibeta) const
  {
    // NB: 3/2 factor to invert Clarke transform unit scaling. It's appropriate
    // here because the alpha vector is shorter than the true vector of current
    // moving along that axis.
    return kt * r(3.0/2) * (ialpha * q_alpha(phase) + ibeta * q_beta(phase));
  }
};


// Predefined motors
motor_parameters const c580l("turnigy C580L",
  // NB: the no-load current for this motor is 1.6A @20V = 32W @11600 RPM. This
  // means we have a baseline for friction+windage losses (I assume mostly
  // friction).
  //
  // $ units -t '(1.6A)^2 * 2mohm' W                      -> 0.00512
  // $ units -t '(32 - 0.00512)W/11600rpm' 'N*m/radian'   -> 0.026338672

  7,                                    // pole pairs
  100,                                  // rotor weight grams (my guess)
  12,                                   // rotor radius mm (my guess)
  580,                                  // RPM/V
  2 * MILLI,                            // phase resistance
  250 * MICRO,                          // phase inductance
  1,                                    // saliency
  r(0.8),                               // trapezoidal bias (TODO: measure)
  0,                                    // windage loss (TODO: measure?)
  r(0.026338672),                       // friction torque
  r(1 * MILLI));                        // cogging torque (TODO)


struct motor;
struct motor_load_sum;

struct motor_load
{
  virtual ~motor_load() {}

  // Returns torque at a specific moment in time. dt is provided only for
  // midpoint interpolation purposes; semantically the idea is to return torque
  // at a single moment in time. This function can query, but not modify, the
  // motor.
  //
  // If the load is resistive, then the torque direction should be positive. The
  // torque will be multiplied by the sign of the rotor velocity and then
  // subtracted.
  virtual real torque(real dt, motor const &m) const { return 0; }

  // Simulates driving the load, advancing time. This is called each time you
  // step a motor. The position is specified in turns. By default this function
  // does nothing. The motor's time will not have been advanced when this
  // function is called.
  virtual void drive(real dt, real dposition, motor const &m) {}

  motor_load &operator+(motor_load &rhs);
};

struct motor_load_sum : public motor_load
{
  motor_load *const lhs;
  motor_load *const rhs;

  motor_load_sum(motor_load &lhs_, motor_load &rhs_) : lhs(&lhs_), rhs(&rhs_) {}

  real torque(real dt, motor const &m) const
  {
    return lhs->torque(dt, m) + rhs->torque(dt, m);
  }

  void drive(real dt, real dposition, motor const &m)
  {
    lhs->drive(dt, dposition, m);
    rhs->drive(dt, dposition, m);
  }
};

motor_load &motor_load::operator+(motor_load &rhs)
{ return *new motor_load_sum(*this, rhs); }


// Time-variant motor state
struct motor
{
  real_mut time            = 0;         // motor time
  real_mut rotor_position  = 0;         // absolute rotor turns
  real_mut rotor_velocity  = 0;         // turns/sec
  real_mut ialpha          = 0;         // A
  real_mut ibeta           = 0;         // A
  real_mut stator_losses   = 0;         // J (mostly electrical heating)
  real_mut friction_losses = 0;         // J (windage + bearing friction)

  motor_parameters const *p;
  motor_load             *l;


  motor(motor_parameters const &p_) : p(&p_), l(nullptr) {}

  motor(motor_parameters const &p_,
        motor_load             &l_) : p(&p_), l(&l_) {}


  // Rotor positioning
  inline real phase_at(real dt) const { return p->pole_pairs * (rotor_position + dt*rotor_velocity); }


  // Terminal currents
  inline real ia(void) const { return p->a(ialpha, ibeta); }
  inline real ib(void) const { return p->b(ialpha, ibeta); }
  inline real ic(void) const { return p->c(ialpha, ibeta); }


  // Time stepping
  real di(            // NB: this function integrates current along one α/β axis
      real dt,
      real i,
      real v,
      real (motor_parameters:: *const emf_fn)        (real, real) const,
      real (motor_parameters:: *const inductance_fn) (real)       const,
      real (motor_parameters:: *const resistance_fn) (void)       const)
  const;

  void step(real dt, real va, real vb, real vc);
};


// Mechanical loads
struct motor_friction_load : public motor_load
{
  real f = 0;
  real torque(real dt, motor const &m) const { return f; }
};


struct motor_inertial_load : public motor_load
{
  real     angular_inertia = 0;                       // kg m²/radian²
  real_mut velocity        = 0;

  real torque(real dt, motor const &m) const
  {
    return (velocity - m.rotor_velocity) / dt * TAU   // radian/s²
         * angular_inertia;                           // N·m/radian
  }

  void drive(real dt, real dposition, motor const &m) { velocity = dposition / dt; }
};


#ifdef IO

// Debugging/logging
class motor_header_t {};
motor_header_t const motor_header;

std::ostream &operator<<(std::ostream &os, motor_header_t const &h);
std::ostream &operator<<(std::ostream &os, motor const &m);


std::ostream &operator<<(std::ostream &os, motor_header_t const &h)
{
  return os << "name\t"
            << "time\t"
            << "position\t"
            << "velocity\t"
            << "iα\t"
            << "iβ\t"
            << "stator_losses\t"
            << "friction_losses";
}

std::ostream &operator<<(std::ostream &os, motor const &m)
{
  return os << m.p->name         << "\t"
            << m.time            << "\t"
            << m.rotor_position  << "\t"
            << m.rotor_velocity  << "\t"
            << m.ialpha          << "\t"
            << m.ibeta           << "\t"
            << m.stator_losses   << "\t"
            << m.friction_losses;
}

#endif      // ifdef IO


// Motor implementation
real motor::di(
    real dt,
    real i,
    real v,
    real (motor_parameters:: *const emf_fn)        (real, real) const,
    real (motor_parameters:: *const inductance_fn) (real)       const,
    real (motor_parameters:: *const resistance_fn) (void)       const)
const
{
  let dt2 = dt/2;
  let p0  = phase_at(0);
  let p12 = phase_at(dt2);
  let p1  = phase_at(dt);

  // Integrate everything in α/β space.
  let emf_t0  = (p->*emf_fn)(p0,  rotor_velocity);
  let emf_t12 = (p->*emf_fn)(p12, rotor_velocity);
  let emf_t1  = (p->*emf_fn)(p1,  rotor_velocity);

  let l = (p->*inductance_fn)(p12);
  let r = (p->*resistance_fn)();

  // RK4 terms: k1, k2, k3, and k4
  let didt_k1 = v - emf_t0  - i                   * r;
  let didt_k2 = v - emf_t12 - (i + dt2 * didt_k1) * r;
  let didt_k3 = v - emf_t12 - (i + dt2 * didt_k2) * r;
  let didt_k4 = v - emf_t1  - (i + dt  * didt_k3) * r;

  return 1/l * dt/6 * (didt_k1 + 2*didt_k2 + 2*didt_k3 + didt_k4);
}


void motor::step(real dt, real va, real vb, real vc)
{
  let vab = va - vb;
  let vac = va - vc;

  let alpha_di = di(dt, ialpha, p->alpha(-vab, -vac),
                    &motor_parameters::alpha_emf,
                    &motor_parameters::alpha_inductance,
                    &motor_parameters::alpha_resistance);

  let beta_di  = di(dt, ibeta, p->beta(-vab, -vac),
                    &motor_parameters::beta_emf,
                    &motor_parameters::beta_inductance,
                    &motor_parameters::beta_resistance);

  let mid_ialpha = ialpha + alpha_di/2;
  let mid_ibeta  = ibeta  + beta_di/2;

  // Net rotor torque
  let friction_torque = p->friction_torque * sgn(rotor_velocity);
  let windage_torque  = p->windage_loss    * rotor_velocity;
  let magnetic_torque = p->magnetic_torque(phase_at(dt/2), mid_ialpha, mid_ibeta);
  let load_torque     = l ? l->torque(dt, *this) * sgn(rotor_velocity) : 0;
  let net_torque      = magnetic_torque
                      - windage_torque - friction_torque - load_torque;

  // Integrate into velocity and position
  let acceleration = net_torque       // N·m/rad = (kg m²/rad)/s²
                   * dt               // (kg m²/rad)/s
                   / p->rotor_inertia // rad/s
                   / TAU;             // turn/s

  let dposition = dt * (rotor_velocity + acceleration/2);

  // Update state
  stator_losses   += p->stator_power(mid_ialpha, mid_ibeta) * dt;
  friction_losses +=
      fabs(windage_torque + friction_torque)         // N·m/rad
    * fabs((rotor_velocity + acceleration/2) * TAU)  // N·m/s
    * dt;                                            // J

  if (l) l->drive(dt, dposition, *this);
  time           += dt;
  rotor_position += dposition;
  rotor_velocity += acceleration;
  ialpha         += alpha_di;
  ibeta          += beta_di;
}


}


#endif
