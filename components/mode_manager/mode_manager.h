#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include <stdint.h>

typedef enum {
    MODE_CAR = 0,
    MODE_ARM,
    MODE_SWITCHING
} control_mode_t;

void mode_manager_init(uint32_t hold_time_ms,
                       float pitch_level_thresh,
                       float pitch_upright_thresh);
control_mode_t mode_manager_update(void);
float mode_manager_get_progress(void);

#endif /* MODE_MANAGER_H */
