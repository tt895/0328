#ifndef BEND_SENSOR_H
#define BEND_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define FINGER_COUNT 5

typedef enum {
    FINGER_THUMB = 0,
    FINGER_INDEX,
    FINGER_MIDDLE,
    FINGER_RING,
    FINGER_PINKY
} finger_id_t;

typedef struct {
    const char *name;
    uint8_t channel;
    uint16_t min_val;
    uint16_t max_val;
    uint16_t last_raw;
    uint16_t current_raw;
    uint8_t bend_percent;
} finger_data_t;

extern finger_data_t g_fingers[FINGER_COUNT];

esp_err_t bend_sensor_init(void);
esp_err_t bend_sensor_read_all(void);
bool bend_sensor_all_open(uint8_t threshold);
bool bend_sensor_is_fist(uint8_t threshold);
void bend_sensor_set_diagnostic(bool enable);

#endif /* BEND_SENSOR_H */
