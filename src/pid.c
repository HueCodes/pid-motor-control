/*
 * PID controller -- position form
 * - Derivative on measurement (not error) to avoid derivative kick
 * - First-order low-pass on derivative term (N=10)
 * - Integral clamping anti-windup
 */

#include "pid.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;

    pid->setpoint = 0;
    pid->integral = 0;
    pid->prev_measurement = 0;
    pid->output = 0;
    pid->deriv_filtered = 0;

    pid->output_min = -1.0f;
    pid->output_max =  1.0f;

    /* Derivative filter: alpha = Td / (Td + N*Ts), N=10 */
    float td = (ki > 0) ? (kd / kp) : 0;
    pid->deriv_filter_alpha = (td > 0) ? td / (td + 10.0f * dt) : 0;
}

void pid_set_output_limits(pid_controller_t *pid, float min, float max) {
    pid->output_min = min;
    pid->output_max = max;
}

void pid_set_setpoint(pid_controller_t *pid, float setpoint) {
    pid->setpoint = setpoint;
}

void pid_reset(pid_controller_t *pid) {
    pid->integral = 0;
    pid->prev_measurement = 0;
    pid->deriv_filtered = 0;
    pid->output = 0;
}

float pid_update(pid_controller_t *pid, float measurement) {
    float error = pid->setpoint - measurement;

    /* Proportional */
    float p_term = pid->kp * error;

    /* Integral with anti-windup (clamping) */
    float i_candidate = pid->integral + pid->ki * pid->dt * error;

    /* Derivative on measurement, filtered */
    float d_raw = pid->kd * (pid->prev_measurement - measurement) / pid->dt;
    pid->deriv_filtered = pid->deriv_filter_alpha * pid->deriv_filtered
                        + (1.0f - pid->deriv_filter_alpha) * d_raw;
    float d_term = pid->deriv_filtered;

    /* Compute unclamped output */
    float output = p_term + i_candidate + d_term;

    /* Anti-windup: only update integral if output is not saturated,
     * or if the error is driving it away from saturation */
    if (output > pid->output_max) {
        output = pid->output_max;
        if (error < 0) pid->integral = i_candidate;
    } else if (output < pid->output_min) {
        output = pid->output_min;
        if (error > 0) pid->integral = i_candidate;
    } else {
        pid->integral = i_candidate;
    }

    pid->prev_measurement = measurement;
    pid->output = output;
    return output;
}
