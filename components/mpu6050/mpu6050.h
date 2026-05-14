#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define MPU6050_ADDR 0x68

typedef struct {
    float pitch;
    float roll;
    float yaw;
} hand_attitude_t;

esp_err_t mpu6050_init(int sda_pin, int scl_pin, uint32_t i2c_freq);
esp_err_t mpu6050_get_attitude_now(hand_attitude_t *attitude);
bool mpu6050_is_level(float pitch_thresh, float roll_thresh);
bool mpu6050_is_upright(float pitch_thresh);
void mpu6050_calibrate(void);
esp_err_t mpu6050_reset(void);

#endif /* MPU6050_H */
