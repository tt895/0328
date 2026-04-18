// protocol.c - 简化版，修复缩进问题

#include "protocol.h"
#include "bend_sensor.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "Protocol";

int protocol_pack_text(const gesture_data_packet_t *packet, char *buffer, size_t buf_len)
{
    const char *mode_str = (packet->mode == MODE_CAR) ? "CAR" : 
                           (packet->mode == MODE_ARM) ? "ARM" : "SWITCH";
    
    int len = snprintf(buffer, buf_len,
        "[%s|%.0f]"
        "T:%d,%d;"
        "I:%d,%d;"
        "M:%d,%d;"
        "R:%d,%d;"
        "P:%d,%d;"
        "PIT:%.1f;ROL:%.1f;YAW:%.1f\n",
        mode_str,
        packet->switch_progress * 100,
        packet->finger_raw[0], packet->finger_percent[0],
        packet->finger_raw[1], packet->finger_percent[1],
        packet->finger_raw[2], packet->finger_percent[2],
        packet->finger_raw[3], packet->finger_percent[3],
        packet->finger_raw[4], packet->finger_percent[4],
        packet->pitch, packet->roll, packet->yaw
    );
    
    return (len < (int)buf_len) ? len : -1;
}

int protocol_pack_binary(const gesture_data_packet_t *packet, uint8_t *buffer, size_t buf_len)
{
    if (buf_len < sizeof(binary_packet_t)) {
        return -1;
    }
    
    binary_packet_t *bin = (binary_packet_t*)buffer;
    
    // 清零结构体，避免脏数据
    memset(bin, 0, sizeof(binary_packet_t));
    
    bin->header = 0xAA;
    bin->mode = (uint8_t)packet->mode;
    bin->progress = (uint8_t)(packet->switch_progress * 100);
    
    for (int i = 0; i < 5; i++) {
        bin->fingers[i] = packet->finger_percent[i];
    }
    
    bin->pitch = (int16_t)(packet->pitch * 10);
    bin->roll = (int16_t)(packet->roll * 10);
    bin->yaw = (int16_t)(packet->yaw * 10);
    
    // 计算校验和（除checksum字节外所有字节之和）
    bin->checksum = 0;
    uint8_t *p = (uint8_t*)bin;
    for (size_t i = 0; i < sizeof(binary_packet_t) - 1; i++) {
        bin->checksum += p[i];
    }
    
    return sizeof(binary_packet_t);
}

int protocol_parse_command(const char *data, size_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    
    if (strncmp(data, "CALIBRATE", 9) == 0) {
        ESP_LOGI(TAG, "收到校准命令");
        return 0;
    } else if (strncmp(data, "RESET", 5) == 0) {
        ESP_LOGI(TAG, "收到重置命令");
        return 0;
    }
    
    return -1;
}

void protocol_init_packet(gesture_data_packet_t *packet)
{
    memset(packet, 0, sizeof(gesture_data_packet_t));
    packet->mode = MODE_CAR;
}