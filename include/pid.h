#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float prev_measurement;
    float output;

    float output_min;
    float output_max;

    /* Derivative filter coefficient (N=10 typical) */
    float deriv_filter_alpha;
    float deriv_filtered;

    float dt;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt);
void pid_set_output_limits(pid_controller_t *pid, float min, float max);
void pid_set_setpoint(pid_controller_t *pid, float setpoint);
void pid_reset(pid_controller_t *pid);

/* Returns control effort. measurement is the process variable (not error). */
float pid_update(pid_controller_t *pid, float measurement);

#endif
