/*
 * MPU-6050 6-axis IMU driver
 * Reads accel (+-2g) and gyro (+-250 deg/s)
 * Complementary filter for roll/pitch estimation
 */

#include "mpu6050.h"
#include "hal.h"
#include <math.h>

/* Scale factors for default ranges */
#define ACCEL_SCALE  (1.0f / 16384.0f)   /* +-2g  -> 16384 LSB/g */
#define GYRO_SCALE   (1.0f / 131.0f)     /* +-250 -> 131 LSB/deg/s */

int mpu6050_init(mpu6050_t *imu) {
    uint8_t who;
    if (i2c_read_reg(MPU6050_ADDR, MPU6050_WHO_AM_I, &who, 1) < 0)
        return -1;
    if (who != 0x68)
        return -2;

    /* Wake up (clear sleep bit), use PLL with X-axis gyro reference */
    i2c_write_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);
    delay_ms(10);

    /* Sample rate = 1 kHz / (1 + SMPLRT_DIV) = 1 kHz */
    i2c_write_reg(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 0x00);

    /* DLPF bandwidth = 44 Hz (CONFIG = 3), balances noise vs latency */
    i2c_write_reg(MPU6050_ADDR, MPU6050_CONFIG, 0x03);

    /* Gyro: +-250 deg/s (default, most sensitive) */
    i2c_write_reg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, 0x00);

    /* Accel: +-2g (default, most sensitive) */
    i2c_write_reg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 0x00);

    imu->gyro_bias_x = 0;
    imu->gyro_bias_y = 0;
    imu->gyro_bias_z = 0;

    return 0;
}

int mpu6050_read(mpu6050_t *imu) {
    uint8_t buf[14];

    /* Burst read: ACCEL_XOUT_H through GYRO_ZOUT_L (14 bytes) */
    if (i2c_read_reg(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14) < 0)
        return -1;

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature, skip */
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    imu->accel_x = ax * ACCEL_SCALE;
    imu->accel_y = ay * ACCEL_SCALE;
    imu->accel_z = az * ACCEL_SCALE;

    imu->gyro_x = gx * GYRO_SCALE - imu->gyro_bias_x;
    imu->gyro_y = gy * GYRO_SCALE - imu->gyro_bias_y;
    imu->gyro_z = gz * GYRO_SCALE - imu->gyro_bias_z;

    return 0;
}

void mpu6050_calibrate_gyro(mpu6050_t *imu, uint32_t samples) {
    float sx = 0, sy = 0, sz = 0;
    imu->gyro_bias_x = 0;
    imu->gyro_bias_y = 0;
    imu->gyro_bias_z = 0;

    for (uint32_t i = 0; i < samples; i++) {
        mpu6050_read(imu);
        sx += imu->gyro_x;
        sy += imu->gyro_y;
        sz += imu->gyro_z;
        delay_ms(1);
    }

    imu->gyro_bias_x = sx / (float)samples;
    imu->gyro_bias_y = sy / (float)samples;
    imu->gyro_bias_z = sz / (float)samples;
}

static float comp_roll;
static float comp_pitch;
static int comp_initialized;

void mpu6050_get_orientation(mpu6050_t *imu, imu_orientation_t *orient, float dt, float alpha) {
    /* Accelerometer angles (only valid when not accelerating) */
    float accel_roll  = atan2f(imu->accel_y, imu->accel_z) * (180.0f / 3.14159265f);
    float accel_pitch = atan2f(-imu->accel_x,
                               sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z))
                        * (180.0f / 3.14159265f);

    if (!comp_initialized) {
        comp_roll  = accel_roll;
        comp_pitch = accel_pitch;
        comp_initialized = 1;
    } else {
        /* Complementary filter: trust gyro short-term, accel long-term */
        comp_roll  = alpha * (comp_roll  + imu->gyro_x * dt) + (1.0f - alpha) * accel_roll;
        comp_pitch = alpha * (comp_pitch + imu->gyro_y * dt) + (1.0f - alpha) * accel_pitch;
    }

    orient->roll  = comp_roll;
    orient->pitch = comp_pitch;
    orient->gyro_rate = imu->gyro_x;  /* Rate about roll axis for PID */
}
