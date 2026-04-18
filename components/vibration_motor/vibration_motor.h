#ifndef VIBRATION_MOTOR_H
#define VIBRATION_MOTOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// 振动模式
typedef enum {
    VIB_OFF = 0,
    VIB_SHORT,      // 短促提示（100ms）
    VIB_LONG,       // 长振动（500ms）
    VIB_DOUBLE,     // 双击
    VIB_PATTERN     // 自定义模式
} vibration_pattern_t;

/**
 * @brief 初始化振动马达
 * @param gpio_num GPIO引脚号
 * @return ESP_OK 成功
 */
esp_err_t vibration_motor_init(int gpio_num);

/**
 * @brief 触发振动
 * @param pattern 振动模式
 */
void vibration_trigger(vibration_pattern_t pattern);

/**
 * @brief 设置自定义振动（用于力反馈）
 * @param intensity 强度 0-100
 * @param duration_ms 持续时间（毫秒）
 */
void vibration_set(uint8_t intensity, uint32_t duration_ms);

/**
 * @brief 停止振动
 */
void vibration_stop(void);

/**
 * @brief 更新振动状态（在任务循环中调用）
 */
void vibration_update(void);

/**
 * @brief 根据抓取力反馈振动（第三阶段功能）
 * @param grip_force 抓取力 0-100
 */
void vibration_feedback_grip(uint8_t grip_force);

#endif /* VIBRATION_MOTOR_H */