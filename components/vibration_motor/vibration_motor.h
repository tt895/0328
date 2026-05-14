#ifndef VIBRATION_MOTOR_H
#define VIBRATION_MOTOR_H

#include <stdint.h>
#include "esp_err.h"

typedef enum {
    VIB_OFF = 0,
    VIB_SHORT,   // 短促提示（100ms）
    VIB_LONG,    // 长振动（500ms）
    VIB_DOUBLE   // 双击（100ms-100ms-100ms）
} vibration_pattern_t;

esp_err_t vibration_motor_init(int gpio_num);
void vibration_trigger(vibration_pattern_t pattern);
void vibration_stop(void);
void vibration_update(void);

#endif /* VIBRATION_MOTOR_H */
