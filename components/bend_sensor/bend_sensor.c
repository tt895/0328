#include "bend_sensor.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "BendSensor";

static adc_oneshot_unit_handle_t adc_handle = NULL;

typedef struct {
    uint16_t filter_buffer[4];
    uint8_t filter_index;
    uint16_t filter_sum;
    uint8_t last_percent;
    uint16_t hist_min;
    uint16_t hist_max;
} filter_state_t;

static filter_state_t g_filter_state[FINGER_COUNT];
static bool g_diagnostic_mode = false;

finger_data_t g_fingers[FINGER_COUNT] = {
    [FINGER_THUMB]  = {"thumb",  ADC_CHANNEL_1, 2000, 4100, 0, 0, 0},
    [FINGER_INDEX]  = {"index",  ADC_CHANNEL_2, 2800, 4100, 0, 0, 0},
    [FINGER_MIDDLE] = {"middle", ADC_CHANNEL_3, 2800, 4100, 0, 0, 0},
    [FINGER_RING]   = {"ring",   ADC_CHANNEL_4, 2800, 4100, 0, 0, 0},
    [FINGER_PINKY]  = {"pinky",  ADC_CHANNEL_5, 2000, 4100, 0, 0, 0},
};

static uint16_t median_filter(uint16_t a, uint16_t b, uint16_t c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

static uint8_t calculate_smart_percent(uint16_t raw_val, uint16_t min_val, uint16_t max_val, int finger_idx)
{
    if (max_val <= min_val + 20) {
        return g_filter_state[finger_idx].last_percent;
    }

    int raw_percent = ((int)max_val - (int)raw_val) * 100 / ((int)max_val - (int)min_val);

    if (raw_percent < 0) raw_percent = 0;
    if (raw_percent > 100) raw_percent = 100;

    if (raw_percent < 3) {
        raw_percent = 0;
    } else if (raw_percent > 97) {
        raw_percent = 100;
    }

    int diff = abs(raw_percent - (int)g_filter_state[finger_idx].last_percent);

    if (diff == 0) {
        return g_filter_state[finger_idx].last_percent;
    } else if (diff <= 2) {
        static uint8_t trend_count[5] = {0};
        static int8_t trend_dir[5] = {0};

        int8_t dir = (raw_percent > (int)g_filter_state[finger_idx].last_percent) ? 1 : -1;

        if (dir == trend_dir[finger_idx]) {
            trend_count[finger_idx]++;
            if (trend_count[finger_idx] >= 2) {
                g_filter_state[finger_idx].last_percent = raw_percent;
                trend_count[finger_idx] = 0;
            }
        } else {
            trend_dir[finger_idx] = dir;
            trend_count[finger_idx] = 1;
        }
    } else {
        g_filter_state[finger_idx].last_percent = raw_percent;
    }

    return g_filter_state[finger_idx].last_percent;
}

static void auto_calibrate_internal(void)
{
    static uint32_t last_calib_time = 0;
    static bool first_run = true;
    uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

    if (!first_run && (now - last_calib_time < 10000)) {
        return;
    }

    for (int i = 0; i < FINGER_COUNT; i++) {
        if (first_run) {
            g_filter_state[i].hist_min = g_fingers[i].current_raw;
            g_filter_state[i].hist_max = g_fingers[i].current_raw;
        } else {
            if (g_fingers[i].current_raw < g_filter_state[i].hist_min) {
                g_filter_state[i].hist_min = g_fingers[i].current_raw;
            }
            if (g_fingers[i].current_raw > g_filter_state[i].hist_max) {
                g_filter_state[i].hist_max = g_fingers[i].current_raw;
            }

            uint16_t range = g_filter_state[i].hist_max - g_filter_state[i].hist_min;
            if (range > 200) {
                g_fingers[i].min_val = g_filter_state[i].hist_min + range / 20;
                g_fingers[i].max_val = g_filter_state[i].hist_max - range / 20;

                if (g_fingers[i].min_val >= g_fingers[i].max_val) {
                    g_fingers[i].min_val = g_filter_state[i].hist_min;
                    g_fingers[i].max_val = g_filter_state[i].hist_max;
                }

                if (g_diagnostic_mode) {
                    ESP_LOGI(TAG, "%s 自动校准: min=%d, max=%d (range=%d)",
                             g_fingers[i].name, g_fingers[i].min_val, g_fingers[i].max_val, range);
                }
            }
        }
    }

    first_run = false;
    last_calib_time = now;
}

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
    }

    for (int warmup = 0; warmup < 5; warmup++) {
        for (int i = 0; i < FINGER_COUNT; i++) {
            int dummy;
            adc_oneshot_read(adc_handle, g_fingers[i].channel, &dummy);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    bend_sensor_read_all();
    for (int i = 0; i < FINGER_COUNT; i++) {
        g_fingers[i].last_raw = g_fingers[i].current_raw;
    }

    ESP_LOGI(TAG, "弯曲传感器初始化完成");
    return ESP_OK;
}

esp_err_t bend_sensor_read_all(void)
{
    static int error_count = 0;
    static uint32_t diag_counter = 0;

    for (int i = 0; i < FINGER_COUNT; i++) {
        uint16_t readings[3];
        for (int j = 0; j < 3; j++) {
            int raw_val;
            esp_err_t ret = adc_oneshot_read(adc_handle, g_fingers[i].channel, &raw_val);

            if (ret != ESP_OK) {
                readings[j] = g_fingers[i].current_raw;
                if (error_count < 10) {
                    ESP_LOGW(TAG, "%s 读取失败: %s", g_fingers[i].name, esp_err_to_name(ret));
                }
                error_count++;
            } else {
                readings[j] = (uint16_t)raw_val;
            }
        }

        uint16_t median_val = median_filter(readings[0], readings[1], readings[2]);

        g_filter_state[i].filter_sum -= g_filter_state[i].filter_buffer[g_filter_state[i].filter_index];
        g_filter_state[i].filter_buffer[g_filter_state[i].filter_index] = median_val;
        g_filter_state[i].filter_sum += median_val;
        g_filter_state[i].filter_index = (g_filter_state[i].filter_index + 1) & 0x03;

        uint16_t filtered_val = g_filter_state[i].filter_sum >> 2;

        g_fingers[i].last_raw = g_fingers[i].current_raw;
        g_fingers[i].current_raw = filtered_val;

        g_fingers[i].bend_percent = calculate_smart_percent(
            filtered_val,
            g_fingers[i].min_val,
            g_fingers[i].max_val,
            i
        );
    }

    auto_calibrate_internal();

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

bool bend_sensor_all_open(uint8_t threshold)
{
    uint8_t adjusted = threshold > 5 ? threshold - 3 : threshold;
    for (int i = 0; i < FINGER_COUNT; i++) {
        if (g_fingers[i].bend_percent > adjusted) {
            return false;
        }
    }
    return true;
}

bool bend_sensor_is_fist(uint8_t threshold)
{
    return (g_fingers[FINGER_INDEX].bend_percent > threshold) &&
           (g_fingers[FINGER_MIDDLE].bend_percent > threshold) &&
           (g_fingers[FINGER_RING].bend_percent > threshold);
}

void bend_sensor_set_diagnostic(bool enable)
{
    g_diagnostic_mode = enable;
    ESP_LOGI(TAG, "诊断模式%s", enable ? "启用" : "禁用");
}
