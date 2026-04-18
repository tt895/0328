#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// 控制模式
typedef enum {
    MODE_CAR = 0,       // 小车模式：手腕倾斜控制移动
    MODE_ARM,           // 机械臂模式：手指弯曲控制关节
    MODE_SWITCHING      // 切换中（锁定状态）
} control_mode_t;

// 切换状态
typedef enum {
    SWITCH_NONE = 0,
    SWITCH_PENDING_CAR,
    SWITCH_PENDING_ARM
} switch_state_t;

/**
 * @brief 初始化模式管理器
 * @param hold_time_ms 切换保持时间（毫秒）
 * @param pitch_level_thresh 水平判定阈值（度）
 * @param pitch_upright_thresh 竖起判定阈值（度）
 */
void mode_manager_init(uint32_t hold_time_ms, 
                       float pitch_level_thresh,
                       float pitch_upright_thresh);

/**
 * @brief 更新模式检测（每帧调用）
 * @return 当前模式
 */
control_mode_t mode_manager_update(void);

/**
 * @brief 获取当前模式
 */
control_mode_t mode_manager_get_current(void);

/**
 * @brief 获取切换进度（0.0~1.0）
 */
float mode_manager_get_progress(void);

/**
 * @brief 强制设置模式（用于调试）
 */
void mode_manager_set_mode(control_mode_t mode);

/**
 * @brief 获取模式名称字符串
 */
const char* mode_manager_get_name(control_mode_t mode);

#endif /* MODE_MANAGER_H */