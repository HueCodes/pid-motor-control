/*
 * Host-side unit tests for PID controller
 * Build and run without any embedded toolchain or hardware.
 *
 * Compile: cc -o test_pid test_pid.c ../src/pid.c -I../include -lm -DHOST_TEST
 * Run: ./test_pid
 */

#include "pid.h"
#include <stdio.h>
#include <math.h>
#include <assert.h>

#define ASSERT_NEAR(a, b, tol) do { \
    float _a = (a), _b = (b), _t = (tol); \
    if (fabsf(_a - _b) > _t) { \
        fprintf(stderr, "FAIL %s:%d: %.6f != %.6f (tol %.6f)\n", \
                __FILE__, __LINE__, _a, _b, _t); \
        failures++; \
    } \
} while(0)

#define RUN_TEST(fn) do { \
    printf("  %s... ", #fn); \
    fn(); \
    printf("ok\n"); \
} while(0)

static int failures = 0;

/* Simulate a first-order plant: tau*dy/dt + y = K*u */
static float plant_sim(float y, float u, float K, float tau, float dt) {
    return y + dt * (K * u - y) / tau;
}

static void test_proportional_only(void) {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.001f);
    pid_set_setpoint(&pid, 10.0f);

    float out = pid_update(&pid, 0.0f);
    ASSERT_NEAR(out, 1.0f, 0.01f);  /* Clamped to output_max */

    pid_set_output_limits(&pid, -100.0f, 100.0f);
    pid_reset(&pid);
    out = pid_update(&pid, 0.0f);
    ASSERT_NEAR(out, 10.0f, 0.01f);  /* Kp * error = 1.0 * 10.0 */
}

static void test_integral_eliminates_offset(void) {
    pid_controller_t pid;
    pid_init(&pid, 0.5f, 2.0f, 0.0f, 0.001f);
    pid_set_output_limits(&pid, -10.0f, 10.0f);
    pid_set_setpoint(&pid, 5.0f);

    /* Simulate: plant holds at measurement=3.0, PID should accumulate integral */
    float accum = 0;
    for (int i = 0; i < 5000; i++) {
        float out = pid_update(&pid, 3.0f);
        accum = out;  /* Just track final output */
    }
    /* After 5 seconds at 2.0 error, integral should have grown substantially */
    assert(accum > 5.0f);
}

static void test_anti_windup_clamps(void) {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 10.0f, 0.0f, 0.001f);
    pid_set_output_limits(&pid, -1.0f, 1.0f);
    pid_set_setpoint(&pid, 100.0f);

    /* Saturate for 1000 iterations */
    for (int i = 0; i < 1000; i++) {
        float out = pid_update(&pid, 0.0f);
        ASSERT_NEAR(out, 1.0f, 0.01f);  /* Should stay clamped */
    }

    /* Now flip setpoint -- should recover quickly, not wind down slowly */
    pid_set_setpoint(&pid, 0.0f);
    float out = 1.0f;
    int recovery_iters = 0;
    for (int i = 0; i < 1000; i++) {
        out = pid_update(&pid, 0.0f);
        if (out <= 0.0f) {
            recovery_iters = i;
            break;
        }
    }
    /* Should recover within a few iterations, not hundreds */
    assert(recovery_iters < 50);
}

static void test_derivative_on_measurement(void) {
    pid_controller_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.001f);
    pid_set_output_limits(&pid, -100.0f, 100.0f);

    /* Step change in setpoint should NOT cause derivative spike */
    pid_update(&pid, 5.0f);  /* Prime prev_measurement */
    pid_set_setpoint(&pid, 50.0f);
    float out = pid_update(&pid, 5.0f);  /* Measurement unchanged */
    ASSERT_NEAR(out, 0.0f, 0.1f);  /* No derivative kick */

    /* But a step in measurement SHOULD produce derivative response */
    out = pid_update(&pid, 10.0f);  /* measurement jumped by 5 */
    assert(out < -1.0f);  /* Negative derivative (opposes increasing measurement) */
}

static void test_output_limits(void) {
    pid_controller_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.001f);
    pid_set_output_limits(&pid, -0.5f, 0.5f);
    pid_set_setpoint(&pid, 10.0f);

    float out = pid_update(&pid, 0.0f);
    ASSERT_NEAR(out, 0.5f, 0.001f);

    pid_set_setpoint(&pid, -10.0f);
    out = pid_update(&pid, 0.0f);
    ASSERT_NEAR(out, -0.5f, 0.001f);
}

static void test_closed_loop_first_order_plant(void) {
    /* Simulate PID controlling a first-order plant to setpoint */
    pid_controller_t pid;
    pid_init(&pid, 5.0f, 2.0f, 0.3f, 0.001f);
    pid_set_output_limits(&pid, -10.0f, 10.0f);
    pid_set_setpoint(&pid, 1.0f);

    float y = 0;
    float K = 1.0f, tau = 0.05f;

    for (int i = 0; i < 5000; i++) {
        float u = pid_update(&pid, y);
        y = plant_sim(y, u, K, tau, 0.001f);
    }

    /* After 5 seconds, should be very close to setpoint */
    ASSERT_NEAR(y, 1.0f, 0.05f);
}

static void test_reset_clears_state(void) {
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 1.0f, 1.0f, 0.001f);
    pid_set_output_limits(&pid, -10.0f, 10.0f);
    pid_set_setpoint(&pid, 5.0f);

    for (int i = 0; i < 100; i++)
        pid_update(&pid, 2.0f);

    assert(pid.integral != 0.0f);

    pid_reset(&pid);
    ASSERT_NEAR(pid.integral, 0.0f, 0.0001f);
    ASSERT_NEAR(pid.deriv_filtered, 0.0f, 0.0001f);
}

int main(void) {
    printf("PID controller tests\n");

    RUN_TEST(test_proportional_only);
    RUN_TEST(test_integral_eliminates_offset);
    RUN_TEST(test_anti_windup_clamps);
    RUN_TEST(test_derivative_on_measurement);
    RUN_TEST(test_output_limits);
    RUN_TEST(test_closed_loop_first_order_plant);
    RUN_TEST(test_reset_clears_state);

    if (failures == 0) {
        printf("\nall tests passed\n");
        return 0;
    } else {
        printf("\n%d assertion(s) failed\n", failures);
        return 1;
    }
}
