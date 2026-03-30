#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

#define MPU6050_ADDR        0x68

/* Register addresses */
#define MPU6050_SMPLRT_DIV  0x19
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_WHO_AM_I    0x75

typedef struct {
    float accel_x, accel_y, accel_z;   /* g */
    float gyro_x, gyro_y, gyro_z;      /* deg/s */
    float gyro_bias_x, gyro_bias_y, gyro_bias_z;
} mpu6050_t;

typedef struct {
    float roll;     /* degrees, from complementary filter */
    float pitch;
    float gyro_rate; /* deg/s about the controlled axis */
} imu_orientation_t;

int  mpu6050_init(mpu6050_t *imu);
int  mpu6050_read(mpu6050_t *imu);
void mpu6050_calibrate_gyro(mpu6050_t *imu, uint32_t samples);
void mpu6050_get_orientation(mpu6050_t *imu, imu_orientation_t *orient, float dt, float alpha);

#endif
