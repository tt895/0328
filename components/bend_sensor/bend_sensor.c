#include "bend_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include <string.h>
#include <math.h>

// 后面的代码保持不变...

static const char *TAG = "BendSensor";

static adc_oneshot_unit_handle_t adc_handle = NULL;

// ==================== 内部滤波结构 ====================
typedef struct {
    uint16_t filter_buffer[4];  // 4点滑动平均
    uint8_t filter_index;
    uint16_t filter_sum;
    uint8_t last_percent;       // 用于趋势检测
    uint16_t hist_min;          // 历史最小值（用于自动校准）
    uint16_t hist_max;          // 历史最大值（用于自动校准）
} filter_state_t;

static filter_state_t g_filter_state[FINGER_COUNT];
static bool g_diagnostic_mode = false;  // 诊断模式开关

// 手指配置（GPIO引脚对应ADC通道）
finger_data_t g_fingers[FINGER_COUNT] = {
    [FINGER_THUMB]  = {"thumb",  ADC_CHANNEL_1, 2000, 4100, 0, 0, 0},  // 伸直2800，弯曲4100
    [FINGER_INDEX]  = {"index",  ADC_CHANNEL_2, 2800, 4100, 0, 0, 0},  // 伸直2800，弯曲4100
    [FINGER_MIDDLE] = {"middle", ADC_CHANNEL_3, 2800, 4100, 0, 0, 0},  // 伸直2800，弯曲4100
    [FINGER_RING]   = {"ring",   ADC_CHANNEL_4, 2800, 4100, 0, 0, 0},  // 伸直2800，弯曲4100
    [FINGER_PINKY]  = {"pinky",  ADC_CHANNEL_5, 2000, 4100, 0, 0, 0},  // 伸直2800，弯曲4100
};

// ==================== 内部辅助函数 ====================

// 中值滤波（快速去除异常值）
static uint16_t median_filter(uint16_t a, uint16_t b, uint16_t c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

// 智能百分比计算（内部使用）
static uint8_t calculate_smart_percent(uint16_t raw_val, uint16_t min_val, uint16_t max_val, int finger_idx)
{
    // 防止除零和范围过小
    if (max_val <= min_val + 20) {
        return g_filter_state[finger_idx].last_percent;
    }
    
    // 计算原始百分比
   int raw_percent = ((int)max_val - (int)raw_val) * 100 / ((int)max_val - (int)min_val);
    
    // 限制范围
    if (raw_percent < 0) raw_percent = 0;
    if (raw_percent > 100) raw_percent = 100;
    
    // 软死区处理（避免0-2%和98-100%的微小抖动）
    if (raw_percent < 3) {
        raw_percent = 0;
    } else if (raw_percent > 97) {
        raw_percent = 100;
    }
    
    // 趋势检测：微小变化需要连续确认
    int diff = abs(raw_percent - (int)g_filter_state[finger_idx].last_percent);
    
    if (diff == 0) {
        return g_filter_state[finger_idx].last_percent;
    } else if (diff <= 2) {
        // 微小变化：积累机制
        static uint8_t trend_count[5] = {0};
        static int8_t trend_dir[5] = {0};
        
        int8_t dir = (raw_percent > (int)g_filter_state[finger_idx].last_percent) ? 1 : -1;
        
        if (dir == trend_dir[finger_idx]) {
            trend_count[finger_idx]++;
            if (trend_count[finger_idx] >= 2) {  // 连续2次同方向才更新
                g_filter_state[finger_idx].last_percent = raw_percent;
                trend_count[finger_idx] = 0;
            }
        } else {
            trend_dir[finger_idx] = dir;
            trend_count[finger_idx] = 1;
        }
    } else {
        // 显著变化：立即更新
        g_filter_state[finger_idx].last_percent = raw_percent;
    }
    
    return g_filter_state[finger_idx].last_percent;
}

// 自动校准函数
static void auto_calibrate_internal(void)
{
    static uint32_t last_calib_time = 0;
    static bool first_run = true;
    uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
    
    // 每10秒校准一次
    if (!first_run && (now - last_calib_time < 10000)) {
        return;
    }
    
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (first_run) {
            // 第一次运行：初始化历史极值
            g_filter_state[i].hist_min = g_fingers[i].current_raw;
            g_filter_state[i].hist_max = g_fingers[i].current_raw;
        } else {
            // 更新历史极值
            if (g_fingers[i].current_raw < g_filter_state[i].hist_min) {
                g_filter_state[i].hist_min = g_fingers[i].current_raw;
            }
            if (g_fingers[i].current_raw > g_filter_state[i].hist_max) {
                g_filter_state[i].hist_max = g_fingers[i].current_raw;
            }
            
            // 只有当范围足够大时才更新校准值（避免噪声影响）
            uint16_t range = g_filter_state[i].hist_max - g_filter_state[i].hist_min;
            if (range > 200) {  // 至少200个ADC单位的范围
                // 留出10%的边界
                g_fingers[i].min_val = g_filter_state[i].hist_min + range / 20;
                g_fingers[i].max_val = g_filter_state[i].hist_max - range / 20;
                
                // 确保最小值小于最大值
                if (g_fingers[i].min_val >= g_fingers[i].max_val) {
                    g_fingers[i].min_val = g_filter_state[i].hist_min;
                    g_fingers[i].max_val = g_filter_state[i].hist_max;
                }
                
                if (g_diagnostic_mode) {
                    ESP_LOGI(TAG, "%s 自动校准: min=%d, max=%d (范围=%d)", 
                             g_fingers[i].name, g_fingers[i].min_val, g_fingers[i].max_val, range);
                }
            }
        }
    }
    
    first_run = false;
    last_calib_time = now;
}

// ==================== 公共接口实现 ====================

esp_err_t bend_sensor_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    for (int i = 0; i < FINGER_COUNT; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, g_fingers[i].channel, &chan_cfg));
        ESP_LOGI(TAG, "配置通道: %s -> ADC1_CH%d", g_fingers[i].name, g_fingers[i].channel);
    }

    // 预热ADC（前几次读数可能不准）
    for (int warmup = 0; warmup < 5; warmup++) {
        for (int i = 0; i < FINGER_COUNT; i++) {
            int dummy;
            adc_oneshot_read(adc_handle, g_fingers[i].channel, &dummy);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // 初始读取一次
    bend_sensor_read_all();
    for (int i = 0; i < FINGER_COUNT; i++) {
        g_fingers[i].last_raw = g_fingers[i].current_raw;
    }

    ESP_LOGI(TAG, "弯曲传感器初始化完成（优化版）");
    return ESP_OK;
}

// 修改读取函数，增加三重滤波
esp_err_t bend_sensor_read_all(void)
{
    static int error_count = 0;
    static uint32_t diag_counter = 0;
    
    for (int i = 0; i < FINGER_COUNT; i++) {
        // 读取3次用于中值滤波
        uint16_t readings[3];
        for (int j = 0; j < 3; j++) {
            int raw_val;
            esp_err_t ret = adc_oneshot_read(adc_handle, g_fingers[i].channel, &raw_val);
            
            if (ret != ESP_OK) {
                readings[j] = g_fingers[i].current_raw;  // 失败时使用上次值
                if (error_count < 10) {  // 避免日志刷屏
                    ESP_LOGW(TAG, "%s 读取失败: %s", g_fingers[i].name, esp_err_to_name(ret));
                }
                error_count++;
            } else {
                readings[j] = (uint16_t)raw_val;
            }
        }
        
        // 1. 中值滤波去除异常值
        uint16_t median_val = median_filter(readings[0], readings[1], readings[2]);
        
        // 2. 滑动平均滤波
        g_filter_state[i].filter_sum -= g_filter_state[i].filter_buffer[g_filter_state[i].filter_index];
        g_filter_state[i].filter_buffer[g_filter_state[i].filter_index] = median_val;
        g_filter_state[i].filter_sum += median_val;
        g_filter_state[i].filter_index = (g_filter_state[i].filter_index + 1) & 0x03;  // 窗口大小4
        
        uint16_t filtered_val = g_filter_state[i].filter_sum >> 2;  // 除以4
        
        // 3. 更新原始值（保持原有字段）
        g_fingers[i].last_raw = g_fingers[i].current_raw;
        g_fingers[i].current_raw = filtered_val;
        
        // 4. 计算百分比（使用智能算法）
        g_fingers[i].bend_percent = calculate_smart_percent(
            filtered_val,
            g_fingers[i].min_val,
            g_fingers[i].max_val,
            i
        );
    }
    
    // 自动校准
    auto_calibrate_internal();
    
    // 诊断输出
    if (g_diagnostic_mode && (diag_counter++ % 10 == 0)) {
        uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
        ESP_LOGI(TAG, "DIAG: %lu, T:%d(%d%%), I:%d(%d%%), M:%d(%d%%), R:%d(%d%%), P:%d(%d%%)",
                 now,
                 g_fingers[0].current_raw, g_fingers[0].bend_percent,
                 g_fingers[1].current_raw, g_fingers[1].bend_percent,
                 g_fingers[2].current_raw, g_fingers[2].bend_percent,
                 g_fingers[3].current_raw, g_fingers[3].bend_percent,
                 g_fingers[4].current_raw, g_fingers[4].bend_percent);
    }
    
    if (error_count > 100) {
        ESP_LOGE(TAG, "ADC错误过多(%d)", error_count);
        error_count = 0;
    }
    
    return ESP_OK;
}

// 保持原有接口（兼容性）
uint8_t bend_sensor_calc_percent(uint16_t raw_val, uint16_t min_val, uint16_t max_val)
{
    if (raw_val <= min_val) return 0;
    if (raw_val >= max_val) return 100;
    
    int percent = (raw_val - min_val) * 100 / (max_val - min_val);
    if (percent < 5) percent = 0;
    return (uint8_t)(100 - percent);
}

bool bend_sensor_has_change(uint16_t threshold)
{
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (abs((int)g_fingers[i].current_raw - (int)g_fingers[i].last_raw) >= threshold) {
            return true;
        }
    }
    return false;
}

uint8_t bend_sensor_get_percent(finger_id_t id)
{
    if (id < FINGER_COUNT) {
        return g_fingers[id].bend_percent;
    }
    return 0;
}

bool bend_sensor_all_open(uint8_t threshold)
{
    // 由于滤波更稳定，可以使用更宽松的条件
    uint8_t adjusted_threshold = threshold > 5 ? threshold - 3 : threshold;
    
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (g_fingers[i].bend_percent > adjusted_threshold) {
            return false;
        }
    }
    return true;
}

bool bend_sensor_is_fist(uint8_t threshold)
{
    // 检查食指、中指、无名指是否弯曲（用于模式切换）
    return (g_fingers[FINGER_INDEX].bend_percent > threshold) &&
           (g_fingers[FINGER_MIDDLE].bend_percent > threshold) &&
           (g_fingers[FINGER_RING].bend_percent > threshold);
}

void bend_sensor_reset_baseline(void)
{
    for (int i = 0; i < FINGER_COUNT; i++) {
        g_fingers[i].last_raw = g_fingers[i].current_raw;
    }
    ESP_LOGI(TAG, "基准值已重置");
}

// ==================== 第二阶段：新增功能 ====================

// 启用/禁用诊断模式
void bend_sensor_set_diagnostic(bool enable)
{
    g_diagnostic_mode = enable;
    if (enable) {
        ESP_LOGI(TAG, "诊断模式启用");
        ESP_LOGI(TAG, "时间(ms),拇指(ADC,%),食指(ADC,%),中指(ADC,%),无名指(ADC,%),小指(ADC,%)");
    } else {
        ESP_LOGI(TAG, "诊断模式禁用");
    }
}

// 手动校准函数
void bend_sensor_manual_calibrate(bool is_straight)
{
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (is_straight) {
            // 伸直校准：当前值作为最小值，留出10%边界
            g_fingers[i].min_val = g_fingers[i].current_raw;
            g_filter_state[i].hist_min = g_fingers[i].current_raw;
            ESP_LOGI(TAG, "%s 手动伸直校准: min=%d", g_fingers[i].name, g_fingers[i].min_val);
        } else {
            // 弯曲校准：当前值作为最大值，留出10%边界
            g_fingers[i].max_val = g_fingers[i].current_raw;
            g_filter_state[i].hist_max = g_fingers[i].current_raw;
            ESP_LOGI(TAG, "%s 手动弯曲校准: max=%d", g_fingers[i].name, g_fingers[i].max_val);
        }
    }
    
    // 确保范围有效
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (g_fingers[i].min_val >= g_fingers[i].max_val) {
            g_fingers[i].max_val = g_fingers[i].min_val + 300;  // 默认300范围
            ESP_LOGW(TAG, "%s 校准范围无效，自动调整", g_fingers[i].name);
        }
    }
}

// 获取校准状态
void bend_sensor_get_calibration_status(void)
{
    for (int i = 0; i < FINGER_COUNT; i++) {
        ESP_LOGI(TAG, "%s: min=%d, max=%d, range=%d, current=%d, percent=%d%%",
                 g_fingers[i].name,
                 g_fingers[i].min_val,
                 g_fingers[i].max_val,
                 g_fingers[i].max_val - g_fingers[i].min_val,
                 g_fingers[i].current_raw,
                 g_fingers[i].bend_percent);
    }
}
