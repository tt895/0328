#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// MPU6050 I2C地址
#define MPU6050_ADDR        0x68
#define MPU6050_ADDR_ALT    0x69  // AD0接高电平时

// 姿态数据结构
typedef struct {
    float pitch;    // 俯仰角（前后倾斜）-90~90
    float roll;     // 横滚角（左右倾斜）-90~90
    float yaw;      // 偏航角（水平旋转）0~360
} hand_attitude_t;

// 原始数据
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float temperature;
} mpu_raw_data_t;

/**
 * @brief 初始化MPU6050
 * @param sda_pin SDA引脚
 * @param scl_pin SCL引脚
 * @param i2c_freq I2C频率（Hz），建议100000（100kHz）以降低线路要求
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_init(int sda_pin, int scl_pin, uint32_t i2c_freq);

/**
 * @brief 读取原始数据
 * @param data 原始数据指针
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_read_raw(mpu_raw_data_t *data);

/**
 * @brief 计算姿态角（使用互补滤波）
 * @param attitude 姿态数据指针
 * @param dt 时间间隔（秒）
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_get_attitude(hand_attitude_t *attitude, float dt);

/**
 * @brief 快速获取当前姿态（内部自动计算dt）
 * @param attitude 姿态数据指针
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_get_attitude_now(hand_attitude_t *attitude);

/**
 * @brief 检查手腕是否水平（用于模式切换）
 * @param pitch_thresh 俯仰角阈值（度）
 * @param roll_thresh 横滚角阈值（度）
 * @return true 水平
 */
bool mpu6050_is_level(float pitch_thresh, float roll_thresh);

/**
 * @brief 检查手腕是否竖起（pitch > 阈值）
 * @param pitch_thresh 俯仰角阈值
 * @return true 竖起
 */
bool mpu6050_is_upright(float pitch_thresh);

/**
 * @brief 校准零点（设备静止时调用）
 */
void mpu6050_calibrate(void);

/**
 * @brief 反初始化，释放资源
 */
void mpu6050_deinit(void);

/**
 * @brief 软件复位MPU6050（不断电重启）
 * @return ESP_OK 成功
 */
esp_err_t mpu6050_reset(void);

#endif /* MPU6050_H */