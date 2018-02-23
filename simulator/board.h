#ifndef BOARD_H
#define BOARD_H

#include <iostream>

#include <stdint.h>
#include <stdlib.h>

#include "defines.h"

#include "motor.h"


namespace simulator
{


struct board_parameters
{
  // NB: defaults below are from the v3.3 board. I guess a TODO would be to have
  // multiple constructors that set these up differently. I can't have a
  // constant instance because I want to be able to tweak board-specific things
  // like shunt resistor errors.
  uint64_t cpu_hz             = 168 * MEGA;       // clocks/sec
  uint64_t pwm_clocks         = 8192;             // clocks
  uint64_t adc_clocks         = 8192;             // clocks (TODO: verify)

  // NB: I'm not sure about the thermal resistance. There are two values given
  // in the datasheet, 134.8 and 47.5, depending on the mounting configuration.
  // I'm being pessimistic and assuming poor thermal conductivity.
  real fet_thermal_resistance = r(134.8);         // °C/W
  real fet_on_resistance      = r(1.6 * MILLI);   // ohms (2x 3.2mΩ parallel)
  real fet_switching_time     = r(20 * NANO);     // seconds

  // Shunt resistors and amplifiers
  real     shunt_resistance   = r(675 * MICRO);   // ohms
  real     shunt_amp_gain     = 40;               // V / V
  real_mut shunt_b_error      = 0;                // ohms/ohm
  real_mut shunt_c_error      = 0;                // ohms/ohm

  real b_shunt_rc_capacitance = r(2200 * PICO);   // F
  real c_shunt_rc_capacitance = r(2200 * PICO);   // F
  real b_shunt_rc_resistance  = 220;              // ohm
  real c_shunt_rc_resistance  = 220;              // ohm

  // Logic components
  real adc_jitter             = r(0.01);          // V / V expected error
  real vlogic                 = r(3.3);           // V
  real vbus                   = 12;               // V
  real vbus_divider           = 11;               // V / V


  board_parameters(void) {}


  inline real     time_at  (uint64_t const c) const { return (real) c / cpu_hz; }
  inline uint16_t pwm_clock(uint64_t const c) const { return c % pwm_clocks; }

  inline uint16_t adc_read(real const real_voltage) const
  {
    // A note about the 4* on the error term. The error is with respect to the
    // full ADC range, which is 0-3.3v, and has units expected(volts per volt).
    // So it's a fraction that we expect to be wrong:
    //
    //   E(|measured - real| / 3.3).
    //
    // If our adc jitter is 0.01 for a 1% expected error, then the range of
    // errors needs to be ±2%.

    let error    = 4 * adc_jitter * ((real) random() / RAND_MAX - r(0.5)) * vlogic;
    let measured = (int16_t) ((real_voltage + error) / vlogic * 4096 + r(0.5));
    return (uint16_t) (measured < 0 ? 0 : measured > 4095 ? 4095 : measured);
  }

  inline real fet_switching_joules(real const dv) const
  {
    // dv is the voltage delta across the FET; we'll assume zero impedance on
    // each side, so current is limited only by the ohmic resistance. This might
    // be realistic for the switching case if only because the wires and traces
    // have some stray capacitance -- but either way, this is a worst-case.
    //
    // This is a simplistic model: let's assume the FET sinks power at
    // half-conductivity for the switching duration.

    let current = dv * fet_on_resistance / 2;   // A
    let power   = dv * current;                 // W
    return fet_switching_time * power;          // J
  }
};


struct board
{
  uint64_t cycles = 0;                  // clocks: current time

  // 3-PWM timings for the DRV8301 (the hardware supports 6-PWM, but looks like
  // it's being used in 3-PWM with the timer complement lines)
  uint16_t a_pwm = PWM_CLOCKS / 2;      // clocks: duty cycle / PWM_CLOCKS
  uint16_t b_pwm = PWM_CLOCKS / 2;      // clocks: duty cycle / PWM_CLOCKS
  uint16_t c_pwm = PWM_CLOCKS / 2;      // clocks: duty cycle / PWM_CLOCKS

  // Total drive power consumption (not including the processor)
  real_mut power_consumed = 0;          // J

  // FET temperature modeling: measure joules dissipated per FET, then divide
  // over time to get wattage and therefore temperature. I don't want to model
  // the junction temperature directly because we don't know the thermal mass.
  real_mut fet_al_heat = 0;             // J
  real_mut fet_ah_heat = 0;             // J
  real_mut fet_bl_heat = 0;             // J
  real_mut fet_bl_heat = 0;             // J
  real_mut fet_ch_heat = 0;             // J
  real_mut fet_ch_heat = 0;             // J

  // TODO: brake resistor

  // 2200pF capacitors in the RC filter from the (amplified) current shunts
  real_mut b_shunt_capacitor_voltage = 0;   // V
  real_mut c_shunt_capacitor_voltage = 0;   // V

  // Hardware and configuration
  board_parameters const *p;
  motor                  *m;


  board(board_parameters const &p_,
        motor                  &m_) : p(&p_), m(&m_) {}


  // FET drives
  inline real driven_va_at(uint64_t const c) const { return p->pwm_clock(c) < a_pwm ? vbus : 0; }
  inline real driven_vb_at(uint64_t const c) const { return p->pwm_clock(c) < b_pwm ? vbus : 0; }
  inline real driven_vc_at(uint64_t const c) const { return p->pwm_clock(c) < c_pwm ? vbus : 0; }

  inline void drive(real const a, real const b, real const c)
  {
    a_pwm = (uint16_t) (a * PWM_CLOCKS);
    b_pwm = (uint16_t) (b * PWM_CLOCKS);
    c_pwm = (uint16_t) (c * PWM_CLOCKS);
  }

  void step(uint64_t const dc)
  {
    let dt = p->t_at(dc) - p->t_at(0);

    // TODO: piecewise simulation for different FET states

    // TODO: joule heating for FETs, simulate shunt amp RC filters
    m->step(dt, TODO);
    cycles += dc;
  }


  // ADC readings
  inline real amplified_shunt_b(void) const { return m->ib() * p->shunt_resistance * p->shunt_amp_gain; }
  inline real amplified_shunt_c(void) const { return m->ic() * p->shunt_resistance * p->shunt_amp_gain; }

  inline uint16_t adc_shunt_b(void) const { return p->adc_read(amplified_shunt_b()); }
  inline uint16_t adc_shunt_c(void) const { return p->adc_read(amplified_shunt_c()); }


  // Thermal modeling (simplistic steady-state linear)
  inline real fet_temperature(real const joules) const
  { return cycles ? joules / p->time_at(cycles) * p->fet_thermal_resistance : 0; }
};


class board_header_t {};
board_header_t const board_header;

std::ostream &operator<<(std::ostream &os, board_header_t const &h);
std::ostream &operator<<(std::ostream &os, board const &h);


}


#endif
