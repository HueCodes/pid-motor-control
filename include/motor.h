#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_REVERSE,
    MOTOR_DIR_BRAKE,
    MOTOR_DIR_COAST
} motor_dir_t;

void motor_init(void);

/* duty: 0.0 to 1.0 */
void motor_set(float duty, motor_dir_t dir);

/* Convenience: signed effort -1.0 to 1.0, handles direction internally */
void motor_set_effort(float effort);

void motor_brake(void);
void motor_coast(void);

#endif
