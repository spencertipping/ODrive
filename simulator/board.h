#ifndef BOARD_H
#define BOARD_H

#ifdef IO
# include <iostream>
#endif

#include <stdint.h>

#ifdef MODEL_ERRORS
# include <stdlib.h>
#endif

#include "defines.h"
#include "motor.h"


namespace simulator
{


inline real uniform_error(real magnitude)
{
# ifdef MODEL_ERRORS
  return 2 * magnitude * ((real) random() / RAND_MAX - r(0.5));
# else
  return 0;
# endif
}


struct board_parameters
{
  // NB: defaults below are from the v3.3 board. I guess a TODO would be to have
  // multiple constructors that set these up differently. I can't have a
  // constant instance because I want to be able to tweak board-specific things
  // like shunt resistor errors.
  uint64_t const cpu_hz       = 168 * MEGA;       // clocks/sec
  uint64_t const pwm_clocks   = 8192;             // clocks
  uint64_t const adc_clocks   = 8192;             // clocks (TODO: verify)

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

  real_mut b_shunt_rc_capacitance = r(2200 * PICO);   // F
  real_mut c_shunt_rc_capacitance = r(2200 * PICO);   // F
  real_mut b_shunt_rc_resistance  = 22;               // ohm
  real_mut c_shunt_rc_resistance  = 22;               // ohm

  // Logic components
  real_mut adc_jitter             = r(0.01);          // V / V expected error
  real_mut vlogic                 = r(3.3);           // V
  real_mut vbus                   = 12;               // V
  real_mut vbus_divider           = 11;               // V / V


  board_parameters(void) {}


  constexpr inline real     time_at  (uint64_t const c) const { return (real) c / cpu_hz; }
  constexpr inline uint16_t pwm_clock(uint64_t const c) const { return c % pwm_clocks; }

  inline uint16_t adc_read(real real_voltage) const
  {
    // NB: 2x for the error factor because the ADC jitter is the _expected_
    // error, not the _range_ of errors.

    let error    = uniform_error(2 * adc_jitter * vlogic);
    let measured = (int16_t) ((real_voltage + error) / vlogic * 4096 + r(0.5));
    return (uint16_t) (measured < 0 ? 0 : measured > 4095 ? 4095 : measured);
  }

  inline real fet_switching_joules(void) const
  {
    // dv is the voltage delta across the FET; we'll assume zero impedance on
    // each side, so current is limited only by the ohmic resistance. This might
    // be realistic for the switching case if only because the wires and traces
    // have some stray capacitance -- but either way, this is a worst-case.
    //
    // This is a simplistic model: let's assume the FET sinks power at
    // half-conductivity for the switching duration.
    //
    // There are always two FETs involved in switching a coil, so the voltage
    // drop across each one is half the bus. I realize that the coils themselves
    // create back-EMF that we're ignoring, so this is a pessimistic upper
    // bound.

    let dv      = vbus / 2;
    let current = dv * fet_on_resistance / 2;   // A
    let power   = dv * current;                 // W
    return fet_switching_time * power;          // J
  }

  inline real fet_resistive_joules(real dt, real i) const
  {
    return i * i*fet_on_resistance * dt;
  }
};


struct board
{
  uint64_t cycles = 0;                  // clocks: current time

  // 3-PWM timings for the DRV8301 (the hardware supports 6-PWM, but looks like
  // it's being used in 3-PWM with the timer complement lines)
  uint16_t a_pwm = 0;                   // clocks: duty cycle / PWM_CLOCKS
  uint16_t b_pwm = 0;                   // clocks: duty cycle / PWM_CLOCKS
  uint16_t c_pwm = 0;                   // clocks: duty cycle / PWM_CLOCKS

  // 2200pF capacitors in the RC filter from the (amplified) current shunts
  real_mut b_shunt_capacitor_voltage = 0;   // V
  real_mut c_shunt_capacitor_voltage = 0;   // V

  // Total drive power consumption (not including the processor)
  real_mut power_consumed = 0;          // J

  // FET temperature modeling: measure joules dissipated per FET, then divide
  // over time to get wattage and therefore temperature. I don't want to model
  // the junction temperature directly because we don't know the thermal mass.
  real_mut fet_al_heat = 0;             // J
  real_mut fet_ah_heat = 0;             // J
  real_mut fet_bl_heat = 0;             // J
  real_mut fet_bh_heat = 0;             // J
  real_mut fet_cl_heat = 0;             // J
  real_mut fet_ch_heat = 0;             // J

  // TODO: brake resistor

  board_parameters const *p;
  motor                  *m;


  board(board_parameters const &p_,
        motor                  &m_) : p(&p_), m(&m_) {}


  // FET drives
  inline real driven_va_at(uint64_t const c) const { return p->pwm_clock(c) < a_pwm ? p->vbus : 0; }
  inline real driven_vb_at(uint64_t const c) const { return p->pwm_clock(c) < b_pwm ? p->vbus : 0; }
  inline real driven_vc_at(uint64_t const c) const { return p->pwm_clock(c) < c_pwm ? p->vbus : 0; }

  inline void drive(real a, real b, real c)
  {
    a_pwm = (uint16_t) (a * p->pwm_clocks);
    b_pwm = (uint16_t) (b * p->pwm_clocks);
    c_pwm = (uint16_t) (c * p->pwm_clocks);
  }

  int64_t next_transition(void) const;


  // ADC readings
  inline real amplified_shunt_b(void) const { return m->ib() * p->shunt_resistance * p->shunt_amp_gain; }
  inline real amplified_shunt_c(void) const { return m->ic() * p->shunt_resistance * p->shunt_amp_gain; }

  inline uint16_t adc_shunt_b(void) const { return p->adc_read(amplified_shunt_b()); }
  inline uint16_t adc_shunt_c(void) const { return p->adc_read(amplified_shunt_c()); }


  // Thermal modeling (simplistic steady-state linear)
  inline real fet_temperature(real joules) const
  { return cycles ? joules / p->time_at(cycles) * p->fet_thermal_resistance : 0; }


  // Time stepping
  void step_piece(uint64_t const dc);
  void step(uint64_t const dc);
};


#ifdef IO

class board_header_t {};
board_header_t const board_header;

std::ostream &operator<<(std::ostream &os, board_header_t const &h);
std::ostream &operator<<(std::ostream &os, board const &h);


std::ostream &operator<<(std::ostream &os, board_header_t const &h)
{
  return os << "cycles\t"
            << "a_pwm\t"
            << "b_pwm\t"
            << "c_pwm\t"
            << "va\t"
            << "vb\t"
            << "vc\t"
            << "b_shunt_capacitor_voltage\t"
            << "c_shunt_capacitor_voltage\t"
            << "power_consumed\t"
            << "fet_al_heat\t"
            << "fet_ah_heat\t"
            << "fet_bl_heat\t"
            << "fet_bh_heat\t"
            << "fet_cl_heat\t"
            << "fet_ch_heat";
}

std::ostream &operator<<(std::ostream &os, board const &b)
{
  return os << b.cycles                    << "\t"
            << b.a_pwm                     << "\t"
            << b.b_pwm                     << "\t"
            << b.c_pwm                     << "\t"
            << b.driven_va_at(b.cycles)    << "\t"
            << b.driven_vb_at(b.cycles)    << "\t"
            << b.driven_vc_at(b.cycles)    << "\t"
            << b.b_shunt_capacitor_voltage << "\t"
            << b.c_shunt_capacitor_voltage << "\t"
            << b.power_consumed            << "\t"
            << b.fet_al_heat               << "\t"
            << b.fet_ah_heat               << "\t"
            << b.fet_bl_heat               << "\t"
            << b.fet_bh_heat               << "\t"
            << b.fet_cl_heat               << "\t"
            << b.fet_ch_heat;
}

#endif      // ifdef IO


// Board implementation
int64_t board::next_transition(void) const
{
  // Return the absolute time (in cycles) at the next transition point. This
  // function always returns a number in the future; that is, its return value
  // will be higher than the board's current cycle time.
  let base = p->pwm_clock(cycles);
  if (base == p->pwm_clocks - 1) return cycles + 1;

  int64_t next = p->pwm_clocks - 1;
  if (a_pwm > base && a_pwm < next) next = a_pwm;
  if (b_pwm > base && b_pwm < next) next = b_pwm;
  if (c_pwm > base && c_pwm < next) next = c_pwm;
  return cycles - base + next;
}


void board::step_piece(uint64_t const dc)
{
  // Advance time with the assumption of local continuity; that is, all FET
  // drive voltages will be constant during this timestep.
  let dt  = p->time_at(dc) - p->time_at(0);
  let va  = driven_va_at(cycles);
  let vb  = driven_vb_at(cycles);
  let vc  = driven_vc_at(cycles);
  let ib0 = m->ib();
  let ic0 = m->ic();
  let c   = p->pwm_clock(cycles);

  m->step(dt, va, vb, vc);
  cycles += dc;

  // Accumulate joule heating across active FETs, both from running and from
  // switching. I'm not modeling dead-time because it's on the order of 50ns,
  // which is just eight clocks. (Also, current is still flowing while the FETs
  // are off; it's just going through the body diodes instead of the switching
  // element.)
  let ib = (m->ib() + ib0) / 2;
  let ic = (m->ic() + ic0) / 2;
  let ia = -(ib + ic);

  let aj = p->fet_resistive_joules(dt, ia) + (c == 0 || c == a_pwm ? p->fet_switching_joules() : 0);
  let bj = p->fet_resistive_joules(dt, ib) + (c == 0 || c == b_pwm ? p->fet_switching_joules() : 0);
  let cj = p->fet_resistive_joules(dt, ic) + (c == 0 || c == c_pwm ? p->fet_switching_joules() : 0);
  (va ? fet_ah_heat : fet_al_heat) += aj;
  (vb ? fet_bh_heat : fet_bl_heat) += bj;
  (vc ? fet_ch_heat : fet_cl_heat) += cj;

  // Total wattage = vbus * I(bus -> gnd)
  let ibusgnd = (va ? fabs(ia) : 0) + (vb ? fabs(ib) : 0) + (vc ? fabs(ic) : 0);
  power_consumed += p->vbus * ibusgnd * dt;

  // Now let's model the current measurement stuff. I'm assuming infinite
  // bandwidth from the DRV8301 shunt amp (relative to the RC filter
  // downstream).
  let shunt_b_v = p->shunt_amp_gain * (vb ? 0 : p->shunt_resistance * (1 + p->shunt_b_error) * -ib);
  let shunt_c_v = p->shunt_amp_gain * (vc ? 0 : p->shunt_resistance * (1 + p->shunt_c_error) * -ic);

  // RC filter (trapezoidal integration)
  // The current into the capacitors is limited by the resistors in the RC
  // filter, so we just need to calculate the voltage across those resistors.
  let shunt_qb = dt * (shunt_b_v - b_shunt_capacitor_voltage) / p->b_shunt_rc_resistance;
  let shunt_qc = dt * (shunt_c_v - c_shunt_capacitor_voltage) / p->c_shunt_rc_resistance;
  let b_cap_dv = shunt_qb / p->b_shunt_rc_capacitance;
  let c_cap_dv = shunt_qc / p->c_shunt_rc_capacitance;

  // Now commit the voltage differences based on the charge rate we'd get at the
  // midpoint. Because resistors are linear, we can just commit half of the
  // delta. Clip the capacitor voltage to prevent overshooting in either
  // direction.
  b_shunt_capacitor_voltage = max(r(0), min(shunt_b_v, b_shunt_capacitor_voltage + b_cap_dv/2));
  c_shunt_capacitor_voltage = max(r(0), min(shunt_c_v, c_shunt_capacitor_voltage + c_cap_dv/2));
}


void board::step(uint64_t const dc)
{
  for (uint64_t stepped = 0; stepped < dc;)
  {
    let step = min(dc - stepped, next_transition() - cycles);
    step_piece(step);
    stepped += step;
  }
}


}


#endif
