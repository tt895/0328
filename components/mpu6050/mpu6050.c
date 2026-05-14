#include "mpu6050.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MPU6050";

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS  500

#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H  0x43
#define REG_WHO_AM_I     0x75

#define ALPHA 0.98f

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} mpu_raw_data_t;

static float gyro_offset[3] = {0};
static float accel_offset[3] = {0};
static float pitch = 0, roll = 0, yaw = 0;
static uint32_t last_update_time = 0;
static bool initialized = false;

static esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data)
{
    for (int retry = 0; retry < 3; retry++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (cmd == NULL) return ESP_ERR_NO_MEM;

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) return ESP_OK;

        ESP_LOGW(TAG, "I2C写入失败（尝试%d/3）, reg=0x%02X", retry + 1, reg);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) return ESP_ERR_INVALID_ARG;

    for (int retry = 0; retry < 3; retry++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (cmd == NULL) return ESP_ERR_NO_MEM;

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

        if (len > 1) {
            i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
        }
        i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) return ESP_OK;

        ESP_LOGW(TAG, "I2C读取失败（尝试%d/3）, reg=0x%02X", retry + 1, reg);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t mpu6050_read_raw(mpu_raw_data_t *data)
{
    if (!initialized || data == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t buffer[14];
    esp_err_t ret = mpu6050_read_bytes(REG_ACCEL_XOUT_H, buffer, 14);
    if (ret != ESP_OK) return ret;

    data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    data->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    data->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    data->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);

    return ESP_OK;
}

static esp_err_t mpu6050_get_attitude(hand_attitude_t *attitude, float dt)
{
    if (!initialized || attitude == NULL) return ESP_ERR_INVALID_STATE;

    mpu_raw_data_t raw;
    esp_err_t ret = mpu6050_read_raw(&raw);
    if (ret != ESP_OK) return ret;

    float ax = (raw.accel_x - accel_offset[0]) / 16384.0f;
    float ay = (raw.accel_y - accel_offset[1]) / 16384.0f;
    float az = (raw.accel_z - accel_offset[2]) / 16384.0f;

    float gx = (raw.gyro_x - gyro_offset[0]) / 131.0f;
    float gy = (raw.gyro_y - gyro_offset[1]) / 131.0f;
    float gz = (raw.gyro_z - gyro_offset[2]) / 131.0f;

    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / M_PI;
    float accel_roll  = atan2f(ay, az) * 180.0f / M_PI;

    pitch = ALPHA * (pitch + gx * dt) + (1 - ALPHA) * accel_pitch;
    roll  = ALPHA * (roll + gy * dt)  + (1 - ALPHA) * accel_roll;
    yaw  += gz * dt;

    while (yaw >= 360) yaw -= 360;
    while (yaw < 0)    yaw += 360;

    attitude->pitch = pitch;
    attitude->roll  = roll;
    attitude->yaw   = yaw;

    return ESP_OK;
}

esp_err_t mpu6050_init(int sda_pin, int scl_pin, uint32_t i2c_freq)
{
    if (initialized) return ESP_OK;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_freq,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    vTaskDelay(pdMS_TO_TICKS(100));

    ret = mpu6050_write_byte(REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        i2c_driver_delete(I2C_MASTER_NUM);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    mpu6050_write_byte(REG_GYRO_CONFIG, 0x00);
    mpu6050_write_byte(REG_ACCEL_CONFIG, 0x00);

    uint8_t who_am_i;
    ret = mpu6050_read_bytes(REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK || (who_am_i != 0x68 && who_am_i != 0x72)) {
        ESP_LOGW(TAG, "ID验证失败(0x%02X)，继续运行", who_am_i);
    } else {
        ESP_LOGI(TAG, "MPU6050 ID: 0x%02X", who_am_i);
    }

    initialized = true;
    ESP_LOGI(TAG, "MPU6050初始化成功");
    return ESP_OK;
}

esp_err_t mpu6050_get_attitude_now(hand_attitude_t *attitude)
{
    uint32_t now = xTaskGetTickCount();
    float dt = (now - last_update_time) / (float)configTICK_RATE_HZ;
    if (last_update_time == 0) dt = 0.02f;
    last_update_time = now;

    return mpu6050_get_attitude(attitude, dt);
}

bool mpu6050_is_level(float pitch_thresh, float roll_thresh)
{
    hand_attitude_t att;
    if (mpu6050_get_attitude_now(&att) != ESP_OK) return false;
    return (fabsf(att.pitch) < pitch_thresh) && (fabsf(att.roll) < roll_thresh);
}

bool mpu6050_is_upright(float pitch_thresh)
{
    hand_attitude_t att;
    if (mpu6050_get_attitude_now(&att) != ESP_OK) return false;
    return att.pitch > pitch_thresh;
}

void mpu6050_calibrate(void)
{
    if (!initialized) return;

    ESP_LOGI(TAG, "开始校准，请保持静止...");
    const int samples = 200;
    int32_t accel_sum[3] = {0};
    int32_t gyro_sum[3] = {0};

    for (int i = 0; i < samples; i++) {
        mpu_raw_data_t raw;
        if (mpu6050_read_raw(&raw) == ESP_OK) {
            accel_sum[0] += raw.accel_x;
            accel_sum[1] += raw.accel_y;
            accel_sum[2] += raw.accel_z - 16384;
            gyro_sum[0] += raw.gyro_x;
            gyro_sum[1] += raw.gyro_y;
            gyro_sum[2] += raw.gyro_z;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    accel_offset[0] = accel_sum[0] / samples;
    accel_offset[1] = accel_sum[1] / samples;
    accel_offset[2] = accel_sum[2] / samples;
    gyro_offset[0] = gyro_sum[0] / samples;
    gyro_offset[1] = gyro_sum[1] / samples;
    gyro_offset[2] = gyro_sum[2] / samples;

    ESP_LOGI(TAG, "校准完成");
}

esp_err_t mpu6050_reset(void)
{
    if (!initialized) return ESP_ERR_INVALID_STATE;

    ESP_LOGW(TAG, "MPU6050软件复位...");

    esp_err_t ret = mpu6050_write_byte(REG_PWR_MGMT_1, 0x80);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(100));

    ret = mpu6050_write_byte(REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(10));
    mpu6050_write_byte(REG_GYRO_CONFIG, 0x00);
    mpu6050_write_byte(REG_ACCEL_CONFIG, 0x00);

    pitch = roll = yaw = 0;
    last_update_time = 0;

    ESP_LOGI(TAG, "MPU6050软件复位完成");
    return ESP_OK;
}
