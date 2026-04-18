#include "vibration_motor.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "Vibration";

static int motor_gpio = -1;
static uint8_t current_intensity = 0;
static uint32_t vibration_end_time = 0;
static bool is_vibrating = false;

// LEDC配置
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY      1000

esp_err_t vibration_motor_init(int gpio_num)
{
    motor_gpio = gpio_num;
    
    // 配置定时器（ESP-IDF v5.x 新API）
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 配置通道（ESP-IDF v5.x 新API）
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "振动马达初始化完成，GPIO: %d", gpio_num);
    return ESP_OK;
}

void vibration_set(uint8_t intensity, uint32_t duration_ms)
{
    if (motor_gpio < 0) return;
    
    intensity = (intensity > 100) ? 100 : intensity;
    
    // 计算duty (8bit: 0-255)
    uint32_t duty = (intensity * 255) / 100;
    
    // ESP-IDF v5.x 新API：使用 ledc_set_duty 和 ledc_update_duty
    // 注意：需要指定 speed_mode 和 channel
    esp_err_t ret = ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置duty失败: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "更新duty失败: %s", esp_err_to_name(ret));
        return;
    }
    
    current_intensity = intensity;
    vibration_end_time = pdTICKS_TO_MS(xTaskGetTickCount()) + duration_ms;
    is_vibrating = true;
    
    ESP_LOGD(TAG, "振动启动: 强度=%d, 持续时间=%dms", intensity, duration_ms);
}

void vibration_trigger(vibration_pattern_t pattern)
{
    switch (pattern) {
        case VIB_OFF:
            vibration_stop();
            break;
        case VIB_SHORT:
            vibration_set(80, 100);
            break;
        case VIB_LONG:
            vibration_set(100, 500);
            break;
        case VIB_DOUBLE:
            vibration_set(80, 300);
            break;
        case VIB_PATTERN:
            break;
    }
}

void vibration_stop(void)
{
    if (motor_gpio < 0) return;
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
    
    is_vibrating = false;
    current_intensity = 0;
}

void vibration_update(void)
{
    if (!is_vibrating) return;
    
    uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
    if (now >= vibration_end_time) {
        vibration_stop();
    }
}

void vibration_feedback_grip(uint8_t grip_force)
{
    if (grip_force < 10) {
        vibration_stop();
    } else {
        uint8_t intensity = 30 + (grip_force * 50) / 100;
        vibration_set(intensity, 100);
    }
}