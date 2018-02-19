#ifndef MOTOR_H
#define MOTOR_H


#include <stdint.h>


namespace simulator
{

// Values from v3.3 board
#define CPU_HZ     168000000
#define PWM_CLOCKS 8192
#define PWM_CYCLE  (1.0 / ((double) CPU_HZ / PWM_CLOCKS))

// Remotely sane motor defaults
#define ROTOR_WEIGHT 100            // grams
#define ROTOR_RADIUS 12             // mm
#define ROTOR_LENGTH 40             // mm

#define ROTOR_INERTIA \
  (0.5 * ROTOR_RADIUS*ROTOR_RADIUS * ROTOR_WEIGHT * ROTOR_LENGTH)

// NB: I use τ as a unit of measure equal to one turn.
#define TAU (2.0 * M_PI)            // radians per turn

class motor
{
public:
  // Time-variant state
  double time             = 0;                // seconds
  double rotor_position   = 0;                // absolute rotor turns
  double rotor_velocity   = 0;                // τ/sec
  double ab_current       = 0;                // A
  double ac_current       = 0;                // A

  double a_pwm            = 0.5;              // duty cycle (0-1 range)
  double b_pwm            = 0.5;              // duty cycle (0-1 range)
  double c_pwm            = 0.5;              // duty cycle (0-1 range)

  // Hardware parameters
  double pwm_cycle_time   = PWM_CYCLE;        // seconds
  double rotor_inertia    = ROTOR_INERTIA;    // g·m·m/sec
  double rotor_windage    = 0;                // N·m / (τ²/s)
  double cogging_torque   = 0.01;             // N·m
  double dynamic_friction = 1e-5;             // N·m / (τ/s)
  double resistance       = 0.2;              // ohms
  double inductance       = 0.0005;           // henries
  double shunt_gain       = 40.0;             // V / V

  // Error modeling
  double adc_jitter       = 0.01;   // expected error in volts per 3.3v reading
  double shunt_b_error    = 0;      // resistor bias: ohms/ohm
  double shunt_c_error    = 0;      // resistor bias: ohms/ohm

  double ohms_b_per_a     = 1;      // ohms per ohm (coil B vs coil A)
  double ohms_c_per_a     = 1;      // ohms per ohm
  double henries_b_per_a  = 1;      // henries per henry
  double henries_c_per_a  = 1;      // henries per henry


  motor(double rotor_momentum_,
        double phase_resistance_,
        double phase_inductance_)

    : rotor_momentum(rotor_momentum_),
      resistance(phase_resistance_),
      inductance(phase_inductance_) {}


  // Hardware functions
  void drive(double a, double b, double c);
  uint16_t adc_shunt_b() const;
  uint16_t adc_shunt_c() const;

  // Time stepping
  void step(double dt, double quantum);
};

}


#endif
