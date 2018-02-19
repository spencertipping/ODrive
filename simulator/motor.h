#ifndef MOTOR_H
#define MOTOR_H

#include <cmath>

#include <stdint.h>


namespace simulator
{

// Values from v3.3 board
#define CPU_HZ     168000000
#define PWM_CLOCKS 8192
#define PWM_CYCLE  (1.0 / ((double) CPU_HZ / PWM_CLOCKS))

// Remotely sane motor defaults
#define MOTOR_KV 580                // rpm/V
#define MOTOR_POLE_PAIRS 7

#define ROTOR_WEIGHT (100 / 1000.0) // kg
#define ROTOR_RADIUS (12  / 1000.0) // m
#define ROTOR_LENGTH (40  / 1000.0) // m

#define ROTOR_INERTIA \
  (0.5 * ROTOR_RADIUS*ROTOR_RADIUS * ROTOR_WEIGHT * ROTOR_LENGTH)

// NB: I use τ as a unit of measure equal to one turn.
#define TAU (2.0 * M_PI)            // radians per turn

#define RMS_TO_PEAK (sqrt(2))


class motor
{
public:
  // Time-variant state
  uint64_t cycles         = 0;                // CPU clock cycles
  double rotor_position   = 0;                // absolute rotor turns
  double rotor_velocity   = 0;                // τ/sec
  double ab_current       = 0;                // A
  double ac_current       = 0;                // A

  uint16_t a_pwm          = 4096;             // number of cycles for ON state
  uint16_t b_pwm          = 4096;             // number of cycles
  uint16_t c_pwm          = 4096;             // number of cycles

  // Hardware parameters
  double v33              = 3.3;              // V
  double vbus             = 12;               // V
  double vbus_divider     = 11;               // V
  double shunt_resistance = 675e-6;           // ohms (v3.3 board)

  // NB: these parameters are unusual in that they don't follow the ideal
  // three-phase model; they instead model the common BLDC configuration that
  // involves multiple pole-cycles per revolution. I'm not modeling actual
  // revolutions anywhere in the code; these quantities are converted to the
  // ideal three-phase model.
  //
  // flux_linkage is measured in peak V / (rad/sec); this also serves as the
  // motor's torque constant (N·m / quadrature amp).
  double flux_linkage     = 5.51328895422 / (MOTOR_KV * MOTOR_POLE_PAIRS);

  double pwm_cycle_time   = PWM_CYCLE;        // seconds
  double rotor_inertia    = ROTOR_INERTIA;    // kg·m²/sec
  double trapezoidal_bias = 0.7;              // interpolation factor

  double rotor_windage    = 1e-4;             // N·m / (τ²/s)
  double dynamic_friction = 1e-5;             // N·m / (τ/s)
  double cogging_torque   = 0.01;             // N·m
  double phase_resistance = 0.2;              // ohms
  double phase_inductance = 0.0005;           // henries
  double shunt_amp_gain   = 40.0;             // V / V

  // Error modeling
  double adc_jitter       = 0.01;   // expected error in volts per 3.3v reading
  double shunt_b_error    = 0;      // resistor bias: ohms/ohm
  double shunt_c_error    = 0;      // resistor bias: ohms/ohm


  motor(double rotor_inertia_,
        double phase_resistance_,
        double phase_inductance_)
    : rotor_inertia(rotor_inertia_),
      phase_resistance(phase_resistance_),
      phase_inductance(phase_inductance_) {}


  // Time stepping
  void step();
  inline double time() const { return time_at(cycles); }

  // Hardware functions
  inline void drive(double const a, double const b, double const c)
  {
    a_pwm = (uint16_t) (a * PWM_CLOCKS);
    b_pwm = (uint16_t) (b * PWM_CLOCKS);
    c_pwm = (uint16_t) (c * PWM_CLOCKS);
  }

  uint16_t adc_shunt_b() const;
  uint16_t adc_shunt_c() const;

  // Internal functions
  uint16_t adc_sample_of(double real_voltage) const;

  // (partially-)trapezoidal sine
  inline double trapezoid(double const t)
  {
    double const tmod = std::fmod(t, 1.0);
    return tmod   >= 3.0/6 ?
             tmod >= 5.0/6 ? ( tmod - 11.0/6) * 6.0
           : tmod >= 4.0/6 ? -1.0
           :                 (-tmod +  3.0/6) * 6.0
         :   tmod >= 2.0/6 ? (-tmod +  3.0/6) * 6.0
           : tmod >= 1.0/6 ? 1.0
           :                 ( tmod - 11.0/6) * 6.0;
  }

  inline double tsin(double const t)
  { return sin(t * TAU) * (1.0 - trapezoidal_bias)
         + trapezoid(t) * trapezoidal_bias; }

  inline double rotor_at    (double   const t)      const { return rotor_position + t*rotor_velocity; }
  inline double time_at     (uint64_t const cycles) const { return (double) cycles / CPU_HZ; }

  inline double driven_va_at(uint64_t const cycles) const { return (cycles & PWM_CLOCKS - 1) < a_pwm ? vbus : 0; }
  inline double driven_vb_at(uint64_t const cycles) const { return (cycles & PWM_CLOCKS - 1) < b_pwm ? vbus : 0; }
  inline double driven_vc_at(uint64_t const cycles) const { return (cycles & PWM_CLOCKS - 1) < c_pwm ? vbus : 0; }

  inline double driven_ab_at(uint64_t const cycles) const { return driven_va_at(cycles) - driven_vb_at(cycles); }
  inline double driven_ac_at(uint64_t const cycles) const { return driven_va_at(cycles) - driven_vc_at(cycles); }
};

}


#endif
