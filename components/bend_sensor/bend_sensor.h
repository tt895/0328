#ifndef BEND_SENSOR_H
#define BEND_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define FINGER_COUNT        5
#define ADC_THRESHOLD       15      // 死区阈值

// 手指名称枚举
typedef enum {
    FINGER_THUMB = 0,
    FINGER_INDEX,
    FINGER_MIDDLE,
    FINGER_RING,
    FINGER_PINKY
} finger_id_t;

// 手指数据结构
typedef struct {
    const char *name;
    uint8_t channel;        // ADC通道
    uint16_t min_val;       // 伸直校准值
    uint16_t max_val;       // 弯曲校准值
    uint16_t last_raw;      // 上次发送的原始值
    uint16_t current_raw;   // 当前原始值
    uint8_t bend_percent;   // 0-100
} finger_data_t;

// 全局手指数据（外部只读访问）
extern finger_data_t g_fingers[FINGER_COUNT];

/**
 * @brief 初始化弯曲传感器（ADC）
 * @return ESP_OK 成功
 */
esp_err_t bend_sensor_init(void);

/**
 * @brief 读取所有手指的ADC值
 * @return ESP_OK 成功
 */
esp_err_t bend_sensor_read_all(void);

/**
 * @brief 计算弯曲百分比（兼容性函数）
 * @param min_val 最小值（伸直）
 * @param max_val 最大值（弯曲）
 * @return 0-100的百分比
 */
uint8_t bend_sensor_calc_percent(uint16_t raw_val, uint16_t min_val, uint16_t max_val);

/**
 * @brief 检查是否有显著变化（超过死区）
 * @param threshold 变化阈值
 * @return true 有显著变化
 */
bool bend_sensor_has_change(uint16_t threshold);

/**
 * @brief 获取特定手指的弯曲百分比
 * @param id 手指ID
 * @return 0-100的百分比
 */
uint8_t bend_sensor_get_percent(finger_id_t id);

/**
 * @brief 检查是否所有手指都伸直（用于模式切换）
 * @param threshold 伸直阈值（百分比小于此值认为伸直）
 * @return true 所有手指伸直
 */
bool bend_sensor_all_open(uint8_t threshold);

/**
 * @brief 检查是否握拳（用于模式切换）
 * @param threshold 弯曲阈值（百分比大于此值认为弯曲）
 * @return true 食指/中指/无名指弯曲
 */
bool bend_sensor_is_fist(uint8_t threshold);

/**
 * @brief 重置基准值（重新校准）
 */
void bend_sensor_reset_baseline(void);

// ==================== 第二阶段：新增功能 ====================

/**
 * @brief 启用或禁用诊断模式
 * @param enable true启用，false禁用
 */
void bend_sensor_set_diagnostic(bool enable);

/**
 * @brief 手动校准手指
 * @param is_straight true为伸直校准，false为弯曲校准
 */
void bend_sensor_manual_calibrate(bool is_straight);

/**
 * @brief 获取当前校准状态（打印到日志）
 */
void bend_sensor_get_calibration_status(void);

#endif /* BEND_SENSOR_H */