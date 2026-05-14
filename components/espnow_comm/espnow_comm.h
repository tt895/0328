#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <stdint.h>
#include "esp_err.h"
#include "protocol.h"

#define ESPNOW_SEND_RETRY   3
#define ESPNOW_SEND_DELAY_MS 5
#define ESPNOW_QUEUE_SIZE   4
#define ESPNOW_TASK_STACK   3072
#define ESPNOW_TASK_PRIO    2

typedef void (*espnow_recv_callback_t)(const uint8_t *mac_addr, const binary_packet_t *packet);

esp_err_t espnow_comm_init_send_with_external_wifi(const uint8_t *peer_mac);
esp_err_t espnow_comm_init_recv_with_external_wifi(const uint8_t *peer_mac, espnow_recv_callback_t callback);
esp_err_t espnow_send_data_async(const binary_packet_t *packet);
uint32_t espnow_get_fail_count(void);
void espnow_get_mac(uint8_t *mac);

#endif /* ESPNOW_COMM_H */
