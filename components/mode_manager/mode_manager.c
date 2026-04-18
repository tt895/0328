#include "mode_manager.h"
#include "bend_sensor.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <math.h>

static const char *TAG = "ModeManager";

static uint32_t hold_time_ms;
static float pitch_level_thresh;
static float pitch_upright_thresh;

static control_mode_t current_mode = MODE_CAR;
static switch_state_t switch_state = SWITCH_NONE;
static uint32_t state_enter_time = 0;
static float progress = 0.0f;

void mode_manager_init(uint32_t _hold_time_ms, 
                       float _pitch_level_thresh,
                       float _pitch_upright_thresh)
{
    hold_time_ms = _hold_time_ms;
    pitch_level_thresh = _pitch_level_thresh;
    pitch_upright_thresh = _pitch_upright_thresh;
    
    current_mode = MODE_CAR;  // 默认小车模式
    switch_state = SWITCH_NONE;
    progress = 0.0f;
    
    ESP_LOGI(TAG, "模式管理器初始化: 保持时间=%dms, 水平阈值=%.1f°, 竖起阈值=%.1f°",
             hold_time_ms, pitch_level_thresh, pitch_upright_thresh);
}

static bool check_switch_to_car(void)
{
    // 五指伸直（<10%）且手腕水平
    bool fingers_open = bend_sensor_all_open(10);
    bool level = mpu6050_is_level(pitch_level_thresh, pitch_level_thresh);
    
    return fingers_open && level;
}

static bool check_switch_to_arm(void)
{
    // 握拳且手腕竖起
    bool fist = bend_sensor_is_fist(60);
    bool upright = mpu6050_is_upright(pitch_upright_thresh);
    
    return fist && upright;
}

control_mode_t mode_manager_update(void)
{
    uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
    
    if (current_mode == MODE_SWITCHING) {
        // 切换完成后恢复
        return current_mode;
    }
    
    // 检测当前手势
    bool to_car = check_switch_to_car();
    bool to_arm = check_switch_to_arm();
    
    switch_state_t detected = SWITCH_NONE;
    if (to_car && current_mode != MODE_CAR) {
        detected = SWITCH_PENDING_CAR;
    } else if (to_arm && current_mode != MODE_ARM) {
        detected = SWITCH_PENDING_ARM;
    }
    
    if (detected != switch_state) {
        // 状态变化，重置计时
        switch_state = detected;
        state_enter_time = now;
        progress = 0.0f;
        
        if (detected != SWITCH_NONE) {
            ESP_LOGI(TAG, "开始检测切换: %s", 
                     detected == SWITCH_PENDING_CAR ? "CAR" : "ARM");
        }
    } else if (detected != SWITCH_NONE) {
        // 持续检测中，计算进度
        uint32_t elapsed = now - state_enter_time;
        progress = fminf(1.0f, elapsed / (float)hold_time_ms);
        
        if (elapsed >= hold_time_ms) {
            // 切换确认
            current_mode = (detected == SWITCH_PENDING_CAR) ? MODE_CAR : MODE_ARM;
            switch_state = SWITCH_NONE;
            progress = 0.0f;
            
            ESP_LOGI(TAG, "模式切换完成: %s", 
                     mode_manager_get_name(current_mode));
        }
    } else {
        progress = 0.0f;
    }
    
    return current_mode;
}

control_mode_t mode_manager_get_current(void)
{
    return current_mode;
}

float mode_manager_get_progress(void)
{
    return progress;
}

void mode_manager_set_mode(control_mode_t mode)
{
    current_mode = mode;
    switch_state = SWITCH_NONE;
    progress = 0.0f;
    ESP_LOGI(TAG, "强制设置模式: %s", mode_manager_get_name(mode));
}

const char* mode_manager_get_name(control_mode_t mode)
{
    switch (mode) {
        case MODE_CAR: return "CAR";
        case MODE_ARM: return "ARM";
        case MODE_SWITCHING: return "SWITCHING";
        default: return "UNKNOWN";
    }
}