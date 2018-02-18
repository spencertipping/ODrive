/* Includes ------------------------------------------------------------------*/

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#define ARM_MATH_CM4
#include <arm_math.h>

#include <low_level.h>

#include <cmsis_os.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.h>

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;

// TODO stick parameter into struct
#define POLE_PAIRS 7 // This value is correct for N5065 motors and Turnigy SK3 series.

#if HW_VERSION_MAJOR == 3
#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
#define SHUNT_RESISTANCE (500e-6f)
#endif
#endif

// TODO: Migrate to C++, clearly we are actually doing object oriented code here...
// TODO: For nice encapsulation, consider not having the motor objects public

// NOTE: for gimbal motors, all units of A are instead V.
// example: vel_gain is [V/(count/s)] instead of [A/(count/s)]
// example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
Motor_t motors[] = {
    {
        // M0
        .control_mode = CTRL_MODE_VELOCITY_CONTROL,  //see: Motor_control_mode_t
        .enable_step_dir = false,                    //auto enabled after calibration
        .counts_per_step = 2.0f,
        .error = ERROR_NO_ERROR,
        .vel_setpoint = 400.0f,
        .vel_gain = 15.0f / 200.0f,      // [A/(rad/s)] <sensorless example>
        .vel_integrator_gain = 0.0f,     // [A/(rad/s * s)] <sensorless example>
        .vel_integrator_current = 0.0f,  // [A]
        .vel_limit = 20000.0f,           // [counts/s]
        .current_setpoint = 0.0f,        // [A]
        .calibration_current = 10.0f,    // [A]
        .resistance_calib_max_voltage = 1.0f, // [V]
        .phase_inductance = 0.0f,        // to be set by measure_phase_inductance
        .phase_resistance = 0.0f,        // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        // .enable_control = true,
        // .do_calibration = true,
        // .calibration_ok = false,
        .motor_timer = &htim1,
        .next_timings = {TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2},
        .control_deadline = TIM_1_8_PERIOD_CLOCKS,
        .last_cpu_time = 0,
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M0_nCS_GPIO_Port,
            .nCSgpioNumber = M0_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false,
        },
        // .gate_driver_regs Init by DRV8301_setup
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
        .phase_current_rev_gain = 0.0f,                // to be set by DRV8301_setup
        .current_control = {
            // Read out max_allowed_current to see max supported value for current_lim.
            // You can change DRV8301_ShuntAmpGain to get a different range.
            // .current_lim = 75.0f, //[A]
            .current_lim = 10.0f,  //[A]
            .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f,
            .final_v_alpha = 0.0f,
            .final_v_beta = 0.0f,
            .Iq_setpoint = 0.0f,
            .Iq_measured = 0.0f,
            .max_allowed_current = 0.0f,
        },
        .sensorless = {
            .phase = 0.0f,                        // [rad]
            .pll_pos = 0.0f,                      // [rad]
            .pll_vel = 0.0f,                      // [rad/s]
            .pll_kp = 0.0f,                       // [rad/s / rad]
            .pll_ki = 0.0f,                       // [(rad/s^2) / rad]
            .observer_gain = 1000.0f,             // [rad/s]
            .flux_state = {0.0f, 0.0f},           // [Vs]
            .V_alpha_beta_memory = {0.0f, 0.0f},  // [V]
            .pm_flux_linkage = 0.0013579529f,     // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
            .estimator_good = false,
            .spin_up_current = 10.0f,        // [A]
            .spin_up_acceleration = 400.0f,  // [rad/s^2]
            .spin_up_target_vel = 400.0f,    // [rad/s]
        },
        .loop_counter = 0,
        .timing_log_index = 0,
        .timing_log = {0},
    },
    {                                             // M1
        .control_mode = CTRL_MODE_VELOCITY_CONTROL,  //see: Motor_control_mode_t
        .enable_step_dir = false,                    //auto enabled after calibration
        .counts_per_step = 2.0f,
        .error = ERROR_NO_ERROR,
        .vel_setpoint = 0.0f,
        .vel_gain = 5.0f / 10000.0f,             // [A/(counts/s)]
        .vel_integrator_gain = 10.0f / 10000.0f,  // [A/(counts/s * s)]
        .vel_integrator_current = 0.0f,           // [A]
        .vel_limit = 20000.0f,                    // [counts/s]
        .current_setpoint = 0.0f,                 // [A]
        .calibration_current = 10.0f,             // [A]
        .resistance_calib_max_voltage = 1.0f, // [V]
        .phase_inductance = 0.0f,                 // to be set by measure_phase_inductance
        .phase_resistance = 0.0f,                 // to be set by measure_phase_resistance
        .motor_thread = 0,
        .thread_ready = false,
        // .enable_control = true,
        // .do_calibration = true,
        // .calibration_ok = false,
        .motor_timer = &htim8,
        .next_timings = {TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2, TIM_1_8_PERIOD_CLOCKS / 2},
        .control_deadline = (3 * TIM_1_8_PERIOD_CLOCKS) / 2,
        .last_cpu_time = 0,
        .current_meas = {0.0f, 0.0f},
        .DC_calib = {0.0f, 0.0f},
        .gate_driver = {
            .spiHandle = &hspi3,
            // Note: this board has the EN_Gate pin shared!
            .EngpioHandle = EN_GATE_GPIO_Port,
            .EngpioNumber = EN_GATE_Pin,
            .nCSgpioHandle = M1_nCS_GPIO_Port,
            .nCSgpioNumber = M1_nCS_Pin,
            .RxTimeOut = false,
            .enableTimeOut = false,
        },
        // .gate_driver_regs Init by DRV8301_setup
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
        .phase_current_rev_gain = 0.0f,                // to be set by DRV8301_setup
        .current_control = {
            // Read out max_allowed_current to see max supported value for current_lim.
            // You can change DRV8301_ShuntAmpGain to get a different range.
            // .current_lim = 75.0f, //[A]
            .current_lim = 10.0f,  //[A]
            .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
            .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
            .v_current_control_integral_d = 0.0f,
            .v_current_control_integral_q = 0.0f,
            .Ibus = 0.0f,
            .final_v_alpha = 0.0f,
            .final_v_beta = 0.0f,
            .Iq_setpoint = 0.0f,
            .Iq_measured = 0.0f,
            .max_allowed_current = 0.0f,
        },
        .sensorless = {
            .phase = 0.0f,                        // [rad]
            .pll_pos = 0.0f,                      // [rad]
            .pll_vel = 0.0f,                      // [rad/s]
            .pll_kp = 0.0f,                       // [rad/s / rad]
            .pll_ki = 0.0f,                       // [(rad/s^2) / rad]
            .observer_gain = 1000.0f,             // [rad/s]
            .flux_state = {0.0f, 0.0f},           // [Vs]
            .V_alpha_beta_memory = {0.0f, 0.0f},  // [V]
            .pm_flux_linkage = 1.58e-3f,          // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }
            .estimator_good = false,
            .spin_up_current = 10.0f,        // [A]
            .spin_up_acceleration = 400.0f,  // [rad/s^2]
            .spin_up_target_vel = 400.0f,    // [rad/s]
        },
        .loop_counter = 0,
        .timing_log_index = 0,
        .timing_log = {0},
    }
};
const size_t num_motors = sizeof(motors) / sizeof(motors[0]);

/* Private constant data -----------------------------------------------------*/
static const float one_by_sqrt3 = 0.57735026919f;
static const float sqrt3_by_2 = 0.86602540378f;
static const float current_meas_period = CURRENT_MEAS_PERIOD;
static const int current_meas_hz = CURRENT_MEAS_HZ;

/* Private variables ---------------------------------------------------------*/
static float brake_resistance = 0.47f;  // [ohm]

/* Function implementations --------------------------------------------------*/

//--------------------------------
// Command Handling
//--------------------------------

void set_vel_setpoint(Motor_t* motor, float vel_setpoint, float current_feed_forward) {
    motor->vel_setpoint = vel_setpoint;
    motor->current_setpoint = current_feed_forward;
    motor->control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", motor->vel_setpoint, motor->current_setpoint);
#endif
}

void set_current_setpoint(Motor_t* motor, float current_setpoint) {
    motor->current_setpoint = current_setpoint;
    motor->control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", motor->current_setpoint);
#endif
}

//--------------------------------
// Utility
//--------------------------------

uint16_t check_timing(Motor_t* motor) {
    TIM_HandleTypeDef* htim = motor->motor_timer;
    uint16_t timing = htim->Instance->CNT;
    bool down = htim->Instance->CR1 & TIM_CR1_DIR;
    if (down) {
        uint16_t delta = TIM_1_8_PERIOD_CLOCKS - timing;
        timing = TIM_1_8_PERIOD_CLOCKS + delta;
    }

    if (++(motor->timing_log_index) == TIMING_LOG_SIZE) {
        motor->timing_log_index = 0;
    }
    motor->timing_log[motor->timing_log_index] = timing;

    return timing;
}

void global_fault(int error) {
    // Disable motors NOW!
    for (int i = 0; i < num_motors; ++i) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motors[i].motor_timer);
    }
    // Set fault codes, etc.
    for (int i = 0; i < num_motors; ++i) {
        motors[i].error = error;
        *(motors[i].axis_legacy.enable_control) = false;
    }
    // disable brake resistor
    set_brake_current(0.0f);
}

float phase_current_from_adcval(Motor_t* motor, uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * motor->phase_current_rev_gain;
    float current = shunt_volt * motor->shunt_conductance;
    return current;
}

//--------------------------------
// Initalisation
//--------------------------------

// Initalises the low level motor control and then starts the motor control threads
void init_motor_control() {
    // Init gate drivers
    DRV8301_setup(&motors[0]);
    DRV8301_setup(&motors[1]);

    // Start PWM and enable adc interrupts/callbacks
    start_adc_pwm();

    // Start Encoders
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    //TODO: Enable index on only one channel

    // Wait for current sense calibration to converge
    // TODO make timing a function of calibration filter tau
    osDelay(1500);
}

// Set up the gate drivers
void DRV8301_setup(Motor_t* motor) {
    DRV8301_Obj* gate_driver = &motor->gate_driver;
    DRV_SPI_8301_Vars_t* local_regs = &motor->gate_driver_regs;

    DRV8301_enable(gate_driver);
    DRV8301_setupSpi(gate_driver, local_regs);

    // TODO we can use reporting only if we actually wire up the nOCTW pin
    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A
    local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_40VpV;
    // local_regs->Ctrl_Reg_2.GAIN = DRV8301_ShuntAmpGain_20VpV;

    switch (local_regs->Ctrl_Reg_2.GAIN) {
        case DRV8301_ShuntAmpGain_10VpV:
            motor->phase_current_rev_gain = 1.0f / 10.0f;
            break;
        case DRV8301_ShuntAmpGain_20VpV:
            motor->phase_current_rev_gain = 1.0f / 20.0f;
            break;
        case DRV8301_ShuntAmpGain_40VpV:
            motor->phase_current_rev_gain = 1.0f / 40.0f;
            break;
        case DRV8301_ShuntAmpGain_80VpV:
            motor->phase_current_rev_gain = 1.0f / 80.0f;
            break;
    }

    float margin = 0.90f;
    float max_input = margin * 0.3f * motor->shunt_conductance;
    float max_swing = margin * 1.6f * motor->shunt_conductance * motor->phase_current_rev_gain;
    motor->current_control.max_allowed_current = MACRO_MIN(max_input, max_swing);

    local_regs->SndCmd = true;
    DRV8301_writeData(gate_driver, local_regs);
    local_regs->RcvCmd = true;
    DRV8301_readData(gate_driver, local_regs);
}

void start_adc_pwm() {
    // Enable ADC and interrupts
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    // Warp field stabilize.
    osDelay(2);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);
    __HAL_ADC_ENABLE_IT(&hadc3, ADC_IT_EOC);

    // Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();

    start_pwm(&htim1);
    start_pwm(&htim8);
    // TODO: explain why this offset
    sync_timers(&htim1, &htim8, TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128);

    // Motor output starts in the disabled state
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    // Start brake resistor PWM in floating output configuration
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void start_pwm(TIM_HandleTypeDef* htim) {
    // Init PWM
    int half_load = TIM_1_8_PERIOD_CLOCKS / 2;
    htim->Instance->CCR1 = half_load;
    htim->Instance->CCR2 = half_load;
    htim->Instance->CCR3 = half_load;

    // This hardware obfustication layer really is getting on my nerves
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);

    htim->Instance->CCR4 = 1;
    HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);
}

void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
                 uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset) {
    // Store intial timer configs
    uint16_t MOE_store_a = htim_a->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t MOE_store_b = htim_b->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t CR2_store = htim_a->Instance->CR2;
    uint16_t SMCR_store = htim_b->Instance->SMCR;
    // Turn off output
    htim_a->Instance->BDTR &= ~(TIM_BDTR_MOE);
    htim_b->Instance->BDTR &= ~(TIM_BDTR_MOE);
    // Disable both timer counters
    htim_a->Instance->CR1 &= ~TIM_CR1_CEN;
    htim_b->Instance->CR1 &= ~TIM_CR1_CEN;
    // Set first timer to send TRGO on counter enable
    htim_a->Instance->CR2 &= ~TIM_CR2_MMS;
    htim_a->Instance->CR2 |= TIM_TRGO_ENABLE;
    // Set Trigger Source of second timer to the TRGO of the first timer
    htim_b->Instance->SMCR &= ~TIM_SMCR_TS;
    htim_b->Instance->SMCR |= TIM_CLOCKSOURCE_ITRx;
    // Set 2nd timer to start on trigger
    htim_b->Instance->SMCR &= ~TIM_SMCR_SMS;
    htim_b->Instance->SMCR |= TIM_SLAVEMODE_TRIGGER;
    // Dir bit is read only in center aligned mode, so we clear the mode for now
    uint16_t CMS_store_a = htim_a->Instance->CR1 & TIM_CR1_CMS;
    uint16_t CMS_store_b = htim_b->Instance->CR1 & TIM_CR1_CMS;
    htim_a->Instance->CR1 &= ~TIM_CR1_CMS;
    htim_b->Instance->CR1 &= ~TIM_CR1_CMS;
    // Set both timers to up-counting state
    htim_a->Instance->CR1 &= ~TIM_CR1_DIR;
    htim_b->Instance->CR1 &= ~TIM_CR1_DIR;
    // Restore center aligned mode
    htim_a->Instance->CR1 |= CMS_store_a;
    htim_b->Instance->CR1 |= CMS_store_b;
    // set counter offset
    htim_a->Instance->CNT = count_offset;
    htim_b->Instance->CNT = 0;
    // Start Timer a
    htim_a->Instance->CR1 |= (TIM_CR1_CEN);
    // Restore timer configs
    htim_a->Instance->CR2 = CR2_store;
    htim_b->Instance->SMCR = SMCR_store;
    // restore output
    htim_a->Instance->BDTR |= MOE_store_a;
    htim_b->Instance->BDTR |= MOE_store_b;
}

//--------------------------------
// IRQ Callbacks
//--------------------------------

void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
    static const float voltage_scale = 3.3f * VBUS_S_DIVIDER_RATIO / (float)(1 << 12);
    // Only one conversion in sequence, so only rank1
    uint32_t ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    vbus_voltage = ADCValue * voltage_scale;
}

// This is the callback from the ADC that we expect after the PWM has triggered an ADC conversion.
// TODO: Document how the phasing is done, link to timing diagram
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected) {
#define calib_tau 0.2f  //@TOTO make more easily configurable
    static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

    // Ensure ADCs are expected ones to simplify the logic below
    if (!(hadc == &hadc2 || hadc == &hadc3)) {
        global_fault(ERROR_ADC_FAILED);
        return;
    };

    // Motor 0 is on Timer 1, which triggers ADC 2 and 3 on an injected conversion
    // Motor 1 is on Timer 8, which triggers ADC 2 and 3 on a regular conversion
    // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
    // If we are counting down, we just sampled in SVM vector 7, with zero current
    Motor_t* motor = injected ? &motors[0] : &motors[1];
    bool counting_down = motor->motor_timer->Instance->CR1 & TIM_CR1_DIR;

    bool current_meas_not_DC_CAL;
    if (motor == &motors[1] && counting_down) {
        // We are measuring M1 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Load next timings for M0 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[0].motor_timer->Instance->CCR1 = motors[0].next_timings[0];
            motors[0].motor_timer->Instance->CCR2 = motors[0].next_timings[1];
            motors[0].motor_timer->Instance->CCR3 = motors[0].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor);

    } else if (motor == &motors[0] && !counting_down) {
        // We are measuring M0 current here
        current_meas_not_DC_CAL = true;
        // Load next timings for M1 (only once is sufficient)
        if (hadc == &hadc2) {
            motors[1].motor_timer->Instance->CCR1 = motors[1].next_timings[0];
            motors[1].motor_timer->Instance->CCR2 = motors[1].next_timings[1];
            motors[1].motor_timer->Instance->CCR3 = motors[1].next_timings[2];
        }
        // Check the timing of the sequencing
        check_timing(motor);

    } else if (motor == &motors[1] && !counting_down) {
        // We are measuring M1 current here
        current_meas_not_DC_CAL = true;
        // Check the timing of the sequencing
        check_timing(motor);

    } else if (motor == &motors[0] && counting_down) {
        // We are measuring M0 DC_CAL here
        current_meas_not_DC_CAL = false;
        // Check the timing of the sequencing
        check_timing(motor);

    } else {
        global_fault(ERROR_PWM_SRC_FAIL);
        return;
    }

    uint32_t ADCValue;
    if (injected) {
        ADCValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } else {
        ADCValue = HAL_ADC_GetValue(hadc);
    }
    float current = phase_current_from_adcval(motor, ADCValue);

    if (current_meas_not_DC_CAL) {
        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // We dispatch the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we store the value from ADC2 and signal the thread that the
        // measurement is ready when we receive the ADC3 measurement

        // return or continue
        if (hadc == &hadc2) {
            motor->current_meas.phB = current - motor->DC_calib.phB;
            return;
        } else {
            motor->current_meas.phC = current - motor->DC_calib.phC;
        }
        // Trigger motor thread
        if (motor->thread_ready)
            osSignalSet(motor->motor_thread, M_SIGNAL_PH_CURRENT_MEAS);
    } else {
        // DC_CAL measurement
        if (hadc == &hadc2) {
            motor->DC_calib.phB += (current - motor->DC_calib.phB) * calib_filter_k;
        } else {
            motor->DC_calib.phC += (current - motor->DC_calib.phC) * calib_filter_k;
        }
    }
}

//--------------------------------
// Measurement and calibration
//--------------------------------

// TODO check Ibeta balance to verify good motor connection
bool measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage) {
    static const float kI = 10.0f;                                 // [(V/s)/A]
    static const int num_test_cycles = 3.0f / CURRENT_MEAS_PERIOD; // Test runs for 3s
    float test_voltage = 0.0f;
    for (int i = 0; i < num_test_cycles; ++i) {
        osEvent evt = osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT);
        if (evt.status != osEventSignal) {
            if (!motor->error) motor->error = ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT;
            return false;
        }
        float Ialpha = -(motor->current_meas.phB + motor->current_meas.phC);
        test_voltage += (kI * current_meas_period) * (test_current - Ialpha);
        if (test_voltage > max_voltage) test_voltage = max_voltage;
        if (test_voltage < -max_voltage) test_voltage = -max_voltage;

        // Test voltage along phase A
        queue_voltage_timings(motor, test_voltage, 0.0f);

        // Check we meet deadlines after queueing
        motor->last_cpu_time = check_timing(motor);
        if (!(motor->last_cpu_time < motor->control_deadline)) {
            if (!motor->error) motor->error = ERROR_PHASE_RESISTANCE_TIMING;
            return false;
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float R = test_voltage / test_current;
    motor->phase_resistance = R;
    if (fabs(test_voltage) == fabs(max_voltage) || R < 0.01f || R > 1.0f) {
        if (!motor->error) motor->error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE;
        return false;
    }
    return true;
}

bool measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;

    for (int t = 0; t < num_cycles; ++t) {
        for (int i = 0; i < 2; ++i) {
            if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
                if (!motor->error) motor->error = ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT;
                return false;
            }
            Ialphas[i] += -motor->current_meas.phB - motor->current_meas.phC;

            // Test voltage along phase A
            queue_voltage_timings(motor, test_voltages[i], 0.0f);

            // Check we meet deadlines after queueing
            motor->last_cpu_time = check_timing(motor);
            if (!(motor->last_cpu_time < motor->control_deadline)) {
                if (!motor->error) motor->error = ERROR_PHASE_INDUCTANCE_TIMING;
                return false;
            }
        }
    }

    // De-energize motor
    queue_voltage_timings(motor, 0.0f, 0.0f);

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (current_meas_period * (float)num_cycles);
    float L = v_L / dI_by_dt;

    motor->phase_inductance = L;
    // TODO arbitrary values set for now
    if (L < 1e-6f || L > 500e-6f) {
        if (!motor->error) motor->error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE;
        return false;
    }
    return true;
}

bool motor_calibration(Motor_t* motor) {
    motor->error = ERROR_NO_ERROR;

    float R_calib_max_voltage = motor->resistance_calib_max_voltage;
    if (!measure_phase_resistance(motor, motor->calibration_current, R_calib_max_voltage))
        return false;
    if (!measure_phase_inductance(motor, -R_calib_max_voltage, R_calib_max_voltage))
        return false;

    // Calculate current control gains
    float current_control_bandwidth = 1000.0f;  // [rad/s]
    motor->current_control.p_gain = current_control_bandwidth * motor->phase_inductance;
    float plant_pole = motor->phase_resistance / motor->phase_inductance;
    motor->current_control.i_gain = plant_pole * motor->current_control.p_gain;

    return true;
}

//--------------------------------
// Main motor control
//--------------------------------

void update_rotor(Motor_t* const motor)
{
  // Algorithm based on paper: Sensorless Control of Surface-Mount Permanent-Magnet Synchronous Motors Based on a Nonlinear Observer
  // http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
  // In particular, equation 8 (and by extension eqn 4 and 6).

  // The V_alpha_beta applied immedietly prior to the current measurement associated with this cycle
  // is the one computed two cycles ago. To get the correct measurement, it was stored twice:
  // once by final_v_alpha/final_v_beta in the current control reporting, and once by V_alpha_beta_memory.

  //for convenience
  Sensorless_t* const sensorless = &motor->sensorless;

  // Up front: if stuff is NaN, reset it to a sane state
#define reset_if_nan(v) if (!isfinite(v)) (v) = 0;
  reset_if_nan(motor->sensorless.flux_state[0]);
  reset_if_nan(motor->sensorless.flux_state[1]);
  reset_if_nan(motor->sensorless.V_alpha_beta_memory[0]);
  reset_if_nan(motor->sensorless.V_alpha_beta_memory[1]);
#undef reset_if_nan

  // Clarke transform
  float const ialpha = -motor->current_meas.phB - motor->current_meas.phC;
  float const ibeta  = one_by_sqrt3 * (motor->current_meas.phB - motor->current_meas.phC);

  if (!isfinite(ialpha) || !isfinite(ibeta))
  {
    global_fault(ERROR_NAN_AT_STAGE1);
    return;
  }

  // Equations 4-8 in the paper
  float const ya = -motor->phase_resistance * ialpha + sensorless->V_alpha_beta_memory[0];
  float const yb = -motor->phase_resistance * ibeta  + sensorless->V_alpha_beta_memory[1];

  if (!isfinite(ya) || !isfinite(yb))
  {
    global_fault(ERROR_NAN_AT_STAGE2);
    return;
  }

  sensorless->flux_state[0] += ya * current_meas_period;
  sensorless->flux_state[1] += yb * current_meas_period;
  float eta_a = sensorless->flux_state[0] - motor->phase_inductance * ialpha;
  float eta_b = sensorless->flux_state[1] - motor->phase_inductance * ibeta;

  if (!isfinite(eta_a) || !isfinite(eta_b))
  {
    global_fault(ERROR_NAN_AT_STAGE3);
    return;
  }

  // Non-linear observer (see paper eqn 8):
  float const pm_flux_sqr = sensorless->pm_flux_linkage * sensorless->pm_flux_linkage;
  float const est_pm_flux_sqr = eta_a*eta_a + eta_b*eta_b;
  float const bandwidth_factor = 1.0f / pm_flux_sqr;
  float const eta_factor = 0.5f * (sensorless->observer_gain * bandwidth_factor) * (pm_flux_sqr - est_pm_flux_sqr);

  if (!isfinite(bandwidth_factor) || !isfinite(eta_factor))
  {
    global_fault(ERROR_NAN_AT_STAGE4);
    return;
  }

  static float eta_factor_avg_test = 0.0f;
  eta_factor_avg_test += 0.001f * (eta_factor - eta_factor_avg_test);

  // add observer action to flux estimate dynamics
  sensorless->flux_state[0] += eta_factor * eta_a * current_meas_period;
  sensorless->flux_state[1] += eta_factor * eta_b * current_meas_period;
  eta_a += sensorless->flux_state[0] - motor->phase_inductance * ialpha;
  eta_b += sensorless->flux_state[1] - motor->phase_inductance * ibeta;

  if (   !isfinite(motor->current_control.final_v_alpha)
      || !isfinite(motor->current_control.final_v_beta))
  {
    global_fault(ERROR_NAN_AT_CURRENTCONTROL);
    return;
  }

  // Flux state estimation done, store V_alpha_beta for next timestep
  sensorless->V_alpha_beta_memory[0] = motor->current_control.final_v_alpha;
  sensorless->V_alpha_beta_memory[1] = motor->current_control.final_v_beta;

  // PLL
  // predict PLL phase with velocity
  sensorless->pll_pos = wrap_pm_pi(sensorless->pll_pos + current_meas_period * sensorless->pll_vel);

  float const newphase = fast_atan2(eta_b, eta_a);
  if (isfinite(newphase)) sensorless->phase = newphase;

  float delta_phase = wrap_pm_pi(sensorless->phase - sensorless->pll_pos);
  sensorless->pll_pos = wrap_pm_pi(sensorless->pll_pos + current_meas_period * sensorless->pll_kp * delta_phase);
  sensorless->pll_vel += current_meas_period * sensorless->pll_ki * delta_phase;
}

float get_rotor_phase(Motor_t* motor) {
    return motor->sensorless.phase;
}

float get_pll_vel(Motor_t* motor) {
    return motor->sensorless.pll_vel;
}

bool spin_up_timestep(Motor_t* motor, float phase, float I_mag) {
    // wait for new timestep
    if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
        if (!motor->error) motor->error = ERROR_SPIN_UP_TIMEOUT;
        return false;
    }
    // run estimator
    update_rotor(motor);
    // override the phase during spinup
    motor->sensorless.phase = phase;
    // run current control (with the phase override)
    FOC_current(motor, I_mag, 0.0f);

    return true;
}

bool spin_up_sensorless(Motor_t* motor) {
    static const float ramp_up_time = 0.4f;
    static const float ramp_up_distance = 4 * M_PI;
    float ramp_step = current_meas_period / ramp_up_time;

    float phase = 0.0f;
    float vel = ramp_up_distance / ramp_up_time;
    float I_mag = 0.0f;

    // spiral up current
    for (float x = 0.0f; x < 1.0f; x += ramp_step) {
        phase = wrap_pm_pi(ramp_up_distance * x);
        I_mag = motor->sensorless.spin_up_current * x;
        if (!spin_up_timestep(motor, phase, I_mag))
            return false;
    }

    // accelerate
    while (vel < motor->sensorless.spin_up_target_vel) {
        vel += motor->sensorless.spin_up_acceleration * current_meas_period;
        phase = wrap_pm_pi(phase + vel * current_meas_period);
        if (!spin_up_timestep(motor, phase, motor->sensorless.spin_up_current))
            return false;
    }

    // test keep spinning
    /*while (true) {
        phase = wrap_pm_pi(phase + vel * current_meas_period);
        if(!spin_up_timestep(motor, phase, motor->sensorless.spin_up_current))
            return false;
    }*/

    return true;

    // TODO: check pll vel (abs ratio, 0.8)
}

void update_brake_current() {
    float Ibus_sum = 0.0f;
    for (int i = 0; i < num_motors; ++i) {
        Ibus_sum += motors[i].current_control.Ibus;
    }
    // Note: set_brake_current will clip negative values to 0.0f
    set_brake_current(-Ibus_sum);
}

void set_brake_current(float brake_current) {
    if (brake_current < 0.0f) brake_current = 0.0f;
    float brake_duty = brake_current * brake_resistance / vbus_voltage;

    // Duty limit at 90% to allow bootstrap caps to charge
    if (brake_duty > 0.9f) brake_duty = 0.9f;
    int high_on = TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty);
    int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
    if (low_off < 0) low_off = 0;

    // Safe update of low and high side timings
    // To avoid race condition, first reset timings to safe state
    // ch3 is low side, ch4 is high side
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    htim2.Instance->CCR3 = low_off;
    htim2.Instance->CCR4 = high_on;
}

void queue_modulation_timings(Motor_t* motor, float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    SVM(mod_alpha, mod_beta, &tA, &tB, &tC);
    motor->next_timings[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    motor->next_timings[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
}

void queue_voltage_timings(Motor_t* motor, float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    queue_modulation_timings(motor, mod_alpha, mod_beta);
}

bool FOC_voltage(Motor_t* motor, float v_d, float v_q) {
    float phase = get_rotor_phase(motor);
    float c = arm_cos_f32(phase);
    float s = arm_sin_f32(phase);
    float v_alpha = c*v_d - s*v_q;
    float v_beta  = c*v_q + s*v_d;
    queue_voltage_timings(motor, v_alpha, v_beta);

    // Check we meet deadlines after queueing
    if (!(check_timing(motor) < motor->control_deadline)) {
        if (!motor->error) motor->error = ERROR_FOC_VOLTAGE_TIMING;
        return false;
    }
    return true;
}

bool FOC_current(Motor_t* motor, float Id_des, float Iq_des) {
    Current_control_t* ictrl = &motor->current_control;

    if (!isfinite(Id_des) || !isfinite(Iq_des))
    {
      global_fault(ERROR_NAN_IQID);
      return false;
    }

    // For Reporting
    ictrl->Iq_setpoint = Iq_des;

    // Clarke transform
    float Ialpha = -motor->current_meas.phB - motor->current_meas.phC;
    float Ibeta = one_by_sqrt3 * (motor->current_meas.phB - motor->current_meas.phC);

    if (!isfinite(Ialpha) || !isfinite(Ibeta))
    {
      global_fault(ERROR_NAN_IAB);
      return false;
    }

    // Park transform
    float phase = get_rotor_phase(motor);
    float c = arm_cos_f32(phase);
    float s = arm_sin_f32(phase);
    float Id = c * Ialpha + s * Ibeta;
    float Iq = c * Ibeta - s * Ialpha;
    ictrl->Iq_measured = Iq;

    if (!isfinite(phase))
    {
      global_fault(ERROR_NAN_PHASE);
      return false;
    }

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = ictrl->v_current_control_integral_d + Ierr_d * ictrl->p_gain;
    float Vq = ictrl->v_current_control_integral_q + Ierr_q * ictrl->p_gain;

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);

    if (!isfinite(mod_scalefactor))
    {
      global_fault(ERROR_NAN_MODSCALE);
      return false;
    }

    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        ictrl->v_current_control_integral_d *= 0.99f;
        ictrl->v_current_control_integral_q *= 0.99f;
    } else {
        ictrl->v_current_control_integral_d += Ierr_d * (ictrl->i_gain * current_meas_period);
        ictrl->v_current_control_integral_q += Ierr_q * (ictrl->i_gain * current_meas_period);
    }

    // Compute estimated bus current
    ictrl->Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float mod_alpha = c * mod_d - s * mod_q;
    float mod_beta = c * mod_q + s * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl->final_v_alpha = mod_to_V * mod_alpha;
    ictrl->final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    queue_modulation_timings(motor, mod_alpha, mod_beta);

    // Check we meet deadlines after queueing
    motor->last_cpu_time = check_timing(motor);
    if (!(motor->last_cpu_time < motor->control_deadline)) {
        if (!motor->error) motor->error = ERROR_FOC_TIMING;
        return false;
    }
    return true;
}

//Returns true if the fault line is asserted
bool check_DRV_fault(Motor_t* motor) {
    //TODO: make this pin configurable per motor ch
    GPIO_PinState nFAULT_state = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);
    return (nFAULT_state == GPIO_PIN_RESET) ? true : false;
}

void control_motor_loop(Motor_t* motor) {
    while (*(motor->axis_legacy.enable_control)) {
        if (osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status != osEventSignal) {
            if (!motor->error) motor->error = ERROR_FOC_MEASUREMENT_TIMEOUT;
            break;
        }
        if (check_DRV_fault(motor)) {
            if (!motor->error) motor->error = ERROR_DRV_FAULT;
            break;
        }
        update_rotor(motor);

        // Position control
        float vel_des = motor->vel_setpoint;

        // Velocity limiting
        float vel_lim = motor->vel_limit;
        if (vel_des >  vel_lim) vel_des =  vel_lim;
        if (vel_des < -vel_lim) vel_des = -vel_lim;

        // Velocity control
        float Iq = motor->current_setpoint;

        float v_err = vel_des - get_pll_vel(motor);
        if (motor->control_mode >= CTRL_MODE_VELOCITY_CONTROL)
            Iq += motor->vel_gain * v_err;

        // Velocity integral action before limiting
        Iq += motor->vel_integrator_current;

        // Current limiting
        float Ilim = MACRO_MIN(motor->current_control.current_lim, motor->current_control.max_allowed_current);
        bool limited = false;
        if (Iq > Ilim) {
            limited = true;
            Iq = Ilim;
        }
        if (Iq < -Ilim) {
            limited = true;
            Iq = -Ilim;
        }

        // Velocity integrator (behaviour dependent on limiting)
        if (motor->control_mode < CTRL_MODE_VELOCITY_CONTROL) {
            // reset integral if not in use
            motor->vel_integrator_current = 0.0f;
        } else {
            if (limited) {
                // TODO make decayfactor configurable
                motor->vel_integrator_current *= 0.99f;
            } else {
                motor->vel_integrator_current += (motor->vel_integrator_gain * current_meas_period) * v_err;
            }
        }

        // Execute current command
        if(!FOC_current(motor, 0.0f, Iq)){
            break; // in case of error exit loop, motor->error has been set by FOC_current
        }

        update_brake_current();
        ++(motor->loop_counter);
    }

    //We are exiting control, reset Ibus, and update brake current
    motor->current_control.Ibus = 0.0f;
    update_brake_current();
}
