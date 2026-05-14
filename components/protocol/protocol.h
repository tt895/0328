#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "mode_manager.h"

typedef struct {
    control_mode_t mode;
    float switch_progress;
    uint8_t finger_percent[5];
    float pitch;
    float roll;
    float yaw;
} gesture_data_packet_t;

// 二进制协议结构（15字节）
typedef struct __attribute__((packed)) {
    uint8_t header;       // 0xAA 帧头
    uint8_t mode;         // 0=CAR, 1=ARM, 2=SWITCH
    uint8_t progress;     // 0-100
    uint8_t fingers[5];   // 5指百分比 0-100
    int16_t pitch;        // 俯仰角 ×10 (-900~900)
    int16_t roll;         // 横滚角 ×10 (-900~900)
    int16_t yaw;          // 偏航角 ×10 (0~3600)
    uint8_t checksum;     // 校验和
} binary_packet_t;

int protocol_pack_binary(const gesture_data_packet_t *packet, uint8_t *buffer, size_t buf_len);
void protocol_init_packet(gesture_data_packet_t *packet);

#endif /* PROTOCOL_H */
