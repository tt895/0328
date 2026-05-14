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

// VIB_DOUBLE 状态机
static uint8_t double_tap_step = 0;
static uint32_t double_tap_next_time = 0;

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY  1000

static void set_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

esp_err_t vibration_motor_init(int gpio_num)
{
    motor_gpio = gpio_num;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

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

static void vibration_raw_set(uint8_t intensity, uint32_t duration_ms)
{
    if (motor_gpio < 0) return;
    intensity = (intensity > 100) ? 100 : intensity;
    uint32_t duty = (intensity * 255) / 100;
    set_duty(duty);
    current_intensity = intensity;
    vibration_end_time = pdTICKS_TO_MS(xTaskGetTickCount()) + duration_ms;
    is_vibrating = true;
}

void vibration_trigger(vibration_pattern_t pattern)
{
    switch (pattern) {
        case VIB_OFF:
            vibration_stop();
            break;
        case VIB_SHORT:
            vibration_raw_set(80, 100);
            break;
        case VIB_LONG:
            vibration_raw_set(100, 500);
            break;
        case VIB_DOUBLE:
            double_tap_step = 1;
            vibration_raw_set(80, 100);
            double_tap_next_time = vibration_end_time + 100;
            break;
    }
}

void vibration_stop(void)
{
    if (motor_gpio < 0) return;
    set_duty(0);
    is_vibrating = false;
    current_intensity = 0;
    double_tap_step = 0;
}

void vibration_update(void)
{
    if (!is_vibrating) return;

    uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

    if (now >= vibration_end_time) {
        // VIB_DOUBLE 状态机
        if (double_tap_step == 1 && now >= double_tap_next_time) {
            double_tap_step = 2;
            vibration_raw_set(80, 100);
            return;
        }
        vibration_stop();
    }
}
