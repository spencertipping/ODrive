#ifndef MOTOR_H
#define MOTOR_H


namespace simulator
{

// Values from v3.3 board
#define CPU_HZ     168000000
#define PWM_CLOCKS 8192

// NB: I use τ as a unit of measure equal to one turn.
#define TAU (2.0 * M_PI)            // radians per turn

class motor
{
public:
  double time;                      // seconds

  double rotor_turns;               // absolute rotor position in turns
  double rotor_velocity;            // τ/sec
  double rotor_momentum;            // g·m·m/sec
  double rotor_windage;             // τ/s² / (τ²/s)

  double static_friction;           // N·m
  double dynamic_friction;          // N·m / (τ/s)

  double resistance;                // ohms
  double inductance;                // henries

  // All coil points are always driven either high or low. For this reason, the
  // ODrive can't do coil voltage sensing like hobby ESCs; instead, it uses the
  // current shunt resistors.
  double pwm_hz;                    // hz
  double a_pwm;                     // duty cycle (0-1 range)
  double b_pwm;                     // duty cycle (0-1 range)
  double c_pwm;                     // duty cycle (0-1 range)

  // Error modeling
  double adc_jitter;                // expected error in volts per volt
  double shunt_b_error;             // resistor bias: ohms/ohm
  double shunt_c_error;             // resistor bias: ohms/ohm

  double ohms_b_per_a;              // ohms per ohm (coil B vs coil A)
  double ohms_c_per_a;              // ohms per ohm
  double henries_b_per_a;           // henries per henry
  double henries_c_per_a;           // henries per henry


  motor(double rotor_momentum_,
        double phase_resistance_,
        double phase_inductance_,

        double pwm_hz_          = (double) CPU_HZ / (PWM_CLOCKS * 2),
        double adc_jitter_      = 0,
        double shunt_b_error_   = 0,
        double shunt_c_error_   = 0,
        double ohms_b_per_a_    = 1,
        double ohms_c_per_a_    = 1,
        double henries_b_per_a_ = 1,
        double henries_c_per_a_ = 1)

    : rotor_momentum(rotor_momentum_),
      resistance(phase_resistance_),
      inductance(phase_inductance_),
      pwm_hz(pwm_hz_),
      adc_jitter(adc_jitter_),
      shunt_b_error(shunt_b_error_),
      shunt_c_error(shunt_c_error_),
      ohms_b_per_a(ohms_b_per_a_),
      ohms_c_per_a(ohms_c_per_a_),
      henries_b_per_a(henries_b_per_a_),
      henries_c_per_a(henries_c_per_a_) {}


  // Hardware functions
  void drive(double a, double b, double c);
  uint16_t adc_shunt_b() const;
  uint16_t adc_shunt_c() const;

  // Time stepping
  void step(double dt, double quantum);
};

}


#endif
