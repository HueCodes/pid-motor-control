/*
 * PID Motor Control -- Main
 *
 * Closed-loop angle control: MPU-6050 measures tilt, PID drives motor
 * to maintain setpoint angle. UART CLI for real-time tuning.
 *
 * Control loop runs at 1 kHz in TIM2 ISR.
 * Main loop handles CLI input and OLED updates (~20 Hz).
 *
 * Hardware:
 *   STM32 Nucleo-F411RE
 *   TB6612FNG motor driver on PA5/PA6/PA8/PA9
 *   MPU-6050 IMU on I2C1 (PB8/PB9)
 *   SSD1306 OLED on I2C1 (shared bus, addr 0x3C)
 *   USART2 (PA2/PA3) -> ST-Link VCP
 */

#include "hal.h"
#include "pid.h"
#include "mpu6050.h"
#include "motor.h"
#include "cli.h"
#include "ssd1306.h"
#include "stm32f411xe.h"
#include <stdio.h>

/* Shared state between ISR and main */
static pid_controller_t pid;
static mpu6050_t imu;
static imu_orientation_t orient;
static cli_t cli;

static volatile float g_measurement;
static volatile float g_output;
static volatile uint32_t g_loop_count;

/* Step response logging */
static volatile int logging_active;
static volatile uint32_t log_start_ms;
#define LOG_DURATION_MS 2000

/* Complementary filter alpha (0.98 = trust gyro 98% short-term) */
#define COMP_ALPHA 0.98f

/* Control loop: 1 kHz, called from TIM2 ISR */
static void control_loop(void) {
    if (mpu6050_read(&imu) < 0)
        return;

    mpu6050_get_orientation(&imu, &orient, 0.001f, COMP_ALPHA);

    float measurement = orient.roll;
    float effort = pid_update(&pid, measurement);
    motor_set_effort(effort);

    g_measurement = measurement;
    g_output = effort;
    g_loop_count++;

    /* Step response CSV logging */
    if (logging_active) {
        uint32_t elapsed = millis() - log_start_ms;
        if (elapsed < LOG_DURATION_MS) {
            /* Print every 10th sample (100 Hz) to avoid UART saturation */
            if (g_loop_count % 10 == 0) {
                uart_printf("%lu,%.2f,%.2f,%.3f\n",
                           (unsigned long)elapsed,
                           pid.setpoint,
                           measurement,
                           effort);
            }
        } else {
            logging_active = 0;
            uart_printf("# log end\n");
        }
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        control_loop();
    }
}

static void display_update(void) {
    ssd1306_clear();

    ssd1306_draw_float(0,  0,  "Kp:", pid.kp);
    ssd1306_draw_float(64, 0,  "Ki:", pid.ki);
    ssd1306_draw_float(0,  10, "Kd:", pid.kd);

    ssd1306_draw_float(0,  22, "SP:", pid.setpoint);
    ssd1306_draw_float(0,  32, "PV:", g_measurement);

    float error = pid.setpoint - g_measurement;
    ssd1306_draw_float(0,  42, "E:", error);

    /* Output bar: center = 0, left = -1, right = +1 */
    float bar_frac = (g_output + 1.0f) / 2.0f;
    ssd1306_draw_string(0, 54, "Out:");
    ssd1306_draw_bar(30, 54, 96, bar_frac);

    ssd1306_update();
}

void main(void) {
    rcc_init();
    gpio_init();
    timer_init();
    i2c_init();
    uart_init();

    uart_printf("\n-- PID Motor Control --\n");
    uart_printf("type 'help' for commands\n\n");

    /* Init IMU */
    if (mpu6050_init(&imu) < 0) {
        uart_printf("MPU-6050 init failed\n");
        while (1);
    }

    uart_printf("calibrating gyro (hold still)...\n");
    mpu6050_calibrate_gyro(&imu, 1000);
    uart_printf("done. bias: x=%.2f y=%.2f z=%.2f deg/s\n",
               imu.gyro_bias_x, imu.gyro_bias_y, imu.gyro_bias_z);

    ssd1306_init();
    motor_init();

    /* PID: conservative starting gains, 1 kHz loop */
    pid_init(&pid, 2.0f, 0.5f, 0.1f, 0.001f);
    pid_set_output_limits(&pid, -1.0f, 1.0f);
    pid_set_setpoint(&pid, 0.0f);

    cli_init(&cli, &pid);

    uart_printf("control loop running at 1 kHz\n> ");

    uint32_t last_display = 0;

    while (1) {
        /* Process CLI input */
        uint8_t c;
        while (uart_read_byte(&c)) {
            cli_process_byte(&cli, (char)c);

            /* Handle step command: set new setpoint and start logging */
            if (cli.step_pending) {
                cli.step_pending = 0;
                pid_set_setpoint(&pid, cli.step_target);
                pid_reset(&pid);
                log_start_ms = millis();
                logging_active = 1;
                uart_printf("# time_ms,setpoint,measurement,output\n");
            }
        }

        /* Update OLED at ~20 Hz */
        uint32_t now = millis();
        if (now - last_display >= 50) {
            last_display = now;
            display_update();
        }
    }
}
