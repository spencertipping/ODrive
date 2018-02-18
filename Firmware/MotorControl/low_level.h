/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include "drv8301.h"

//default timeout waiting for phase measurement signals
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms]

/* Exported types ------------------------------------------------------------*/
typedef enum {
    M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
} Motor_thread_signals_t;

typedef enum {
    ERROR_NO_ERROR,
    ERROR_PHASE_RESISTANCE_TIMING,
    ERROR_PHASE_RESISTANCE_MEASUREMENT_TIMEOUT,
    ERROR_PHASE_RESISTANCE_OUT_OF_RANGE,
    ERROR_PHASE_INDUCTANCE_TIMING,
    ERROR_PHASE_INDUCTANCE_MEASUREMENT_TIMEOUT,
    ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE,
    ERROR_ADC_FAILED,
    ERROR_CALIBRATION_TIMING,
    ERROR_FOC_TIMING,
    ERROR_FOC_MEASUREMENT_TIMEOUT,
    ERROR_SCAN_MOTOR_TIMING,
    ERROR_FOC_VOLTAGE_TIMING,
    ERROR_GATEDRIVER_INVALID_GAIN,
    ERROR_PWM_SRC_FAIL,
    ERROR_UNEXPECTED_STEP_SRC,
    ERROR_SPIN_UP_TIMEOUT,
    ERROR_DRV_FAULT,
    ERROR_NOT_IMPLEMENTED_MOTOR_TYPE,
} Error_t;

// Note: these should be sorted from lowest level of control to
// highest level of control, to allow "<" style comparisons.
typedef enum {
    CTRL_MODE_CURRENT_CONTROL,
    CTRL_MODE_VELOCITY_CONTROL,
} Motor_control_mode_t;

typedef struct {
    float phB;
    float phC;
} Iph_BC_t;

typedef struct {
    float current_lim; // [A]
    float p_gain; // [V/A]
    float i_gain; // [V/As]
    float v_current_control_integral_d; // [V]
    float v_current_control_integral_q; // [V]
    float Ibus; // DC bus current [A]
    // Voltage applied at end of cycle:
    float final_v_alpha; // [V]
    float final_v_beta; // [V]
    float Iq_setpoint;
    float Iq_measured;
    float max_allowed_current;
} Current_control_t;

typedef struct {
    float phase;
    float pll_pos;
    float pll_vel;
    float pll_kp;
    float pll_ki;
    float observer_gain; // [rad/s]
    float flux_state[2]; // [Vs]
    float V_alpha_beta_memory[2]; // [V]
    float pm_flux_linkage; // [V / (rad/s)]
    bool estimator_good;
    float spin_up_current; // [A]
    float spin_up_acceleration; // [rad/s^2]
    float spin_up_target_vel; // [rad/s]
} Sensorless_t;

typedef struct {
    bool* enable_control;
} Axis_legacy_t;

#define TIMING_LOG_SIZE 16
typedef struct {
    Axis_legacy_t axis_legacy;
    Motor_control_mode_t control_mode;
    bool enable_step_dir;
    float counts_per_step;
    Error_t error;
    float vel_setpoint;
    float vel_gain;
    float vel_integrator_gain;
    float vel_integrator_current;
    float vel_limit;
    float current_setpoint;
    float calibration_current;
    float resistance_calib_max_voltage;
    float phase_inductance;
    float phase_resistance;
    osThreadId motor_thread;
    bool thread_ready;
    // bool enable_control; // enable/disable via usb to start motor control. will be set to false again in case of errors.requires calibration_ok=true
    // bool do_calibration; //  trigger motor calibration. will be reset to false after self test
    // bool calibration_ok;
    TIM_HandleTypeDef* motor_timer;
    uint16_t next_timings[3];
    uint16_t control_deadline;
    uint16_t last_cpu_time;
    Iph_BC_t current_meas;
    Iph_BC_t DC_calib;
    DRV8301_Obj gate_driver;
    DRV_SPI_8301_Vars_t gate_driver_regs; //Local view of DRV registers
    float shunt_conductance;
    float phase_current_rev_gain; //Reverse gain for ADC to Amps
    Current_control_t current_control;
    Sensorless_t sensorless;
    uint32_t loop_counter;
    int timing_log_index;
    uint16_t timing_log[TIMING_LOG_SIZE];
    // Cache for remote procedure calls arguments
    struct {
        float pos_setpoint; 
        float vel_feed_forward;
        float current_feed_forward;
    } set_pos_setpoint_args;
    struct {
        float vel_setpoint;
        float current_feed_forward;
    } set_vel_setpoint_args;
    struct {
        float current_setpoint;
    } set_current_setpoint_args;
} Motor_t;

typedef struct{
        int type;
        int index;
} monitoring_slot;

/* Exported constants --------------------------------------------------------*/
extern const size_t num_motors;
extern const float elec_rad_per_enc;
/* Exported variables --------------------------------------------------------*/
extern float vbus_voltage;
extern Motor_t motors[];
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

//Note: to control without feed forward, set feed forward terms to 0.0f.
void set_vel_setpoint(Motor_t* motor, float vel_setpoint, float current_feed_forward);
void set_current_setpoint(Motor_t* motor, float current_setpoint);

void step_cb(uint16_t GPIO_Pin);
void pwm_trig_adc_cb(ADC_HandleTypeDef* hadc, bool injected);
void vbus_sense_adc_cb(ADC_HandleTypeDef* hadc, bool injected);

void safe_assert(int arg);
void init_motor_control();
void setEncoderCount(Motor_t* motor, uint32_t count);

bool anti_cogging_calibration(Motor_t* motor);

bool motor_calibration(Motor_t* motor);


//// Old private:
// Utility
uint16_t check_timing(Motor_t* motor);
void global_fault(int error);
float phase_current_from_adcval(Motor_t* motor, uint32_t ADCValue);
// Initalisation
void DRV8301_setup(Motor_t* motor);
void start_adc_pwm();
void start_pwm(TIM_HandleTypeDef* htim);
void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
        uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset);
// IRQ Callbacks (are all public)
// Measurement and calibrationa
bool measure_phase_resistance(Motor_t* motor, float test_current, float max_voltage);
bool measure_phase_inductance(Motor_t* motor, float voltage_low, float voltage_high);

// Test functions
void scan_motor_loop(Motor_t* motor, float omega, float voltage_magnitude);
// Main motor control
void update_rotor(Motor_t* motor);
float get_rotor_phase(Motor_t* motor);
float get_pll_vel(Motor_t* motor);
bool spin_up_sensorless(Motor_t* motor);
void update_brake_current();
void set_brake_current(float brake_current);
void queue_modulation_timings(Motor_t* motor, float mod_alpha, float mod_beta);
void queue_voltage_timings(Motor_t* motor, float v_alpha, float v_beta);
bool FOC_voltage(Motor_t* motor, float v_d, float v_q);
bool FOC_current(Motor_t* motor, float Id_des, float Iq_des);
void control_motor_loop(Motor_t* motor);

//motor thread moved to axis object
//void motor_thread(void const * argument);

#ifdef __cplusplus
}
#endif

#endif //__LOW_LEVEL_H
