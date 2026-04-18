#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "protocol.h"

#define ESPNOW_WIFI_CHANNEL    1
#define ESPNOW_SEND_RETRY      3
#define ESPNOW_SEND_DELAY_MS   5

typedef void (*espnow_recv_callback_t)(const uint8_t *mac_addr, const binary_packet_t *packet);

// 发送端初始化（自带 WiFi 初始化，用于独立项目如小车端）
esp_err_t espnow_comm_init_send(const uint8_t *peer_mac);

// 发送端初始化（使用外部已初始化的 WiFi，用于手势端共存 WiFi+ESP-NOW）
esp_err_t espnow_comm_init_send_with_external_wifi(const uint8_t *peer_mac);

// 接收端初始化（自带 WiFi）
esp_err_t espnow_comm_init_recv(espnow_recv_callback_t callback);

// 接收端初始化（使用外部已初始化的 WiFi）
esp_err_t espnow_comm_init_recv_with_external_wifi(espnow_recv_callback_t callback);

// 发送手势数据包
esp_err_t espnow_send_data(const binary_packet_t *packet);

// 获取本机 MAC 地址
void espnow_get_mac(uint8_t *mac);

#endif /* ESPNOW_COMM_H */
