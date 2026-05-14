#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <stdint.h>
#include "esp_err.h"
#include "protocol.h"

#define ESPNOW_WIFI_CHANNEL 1
#define ESPNOW_SEND_RETRY   3
#define ESPNOW_SEND_DELAY_MS 5

typedef void (*espnow_recv_callback_t)(const uint8_t *mac_addr, const binary_packet_t *packet);

esp_err_t espnow_comm_init_send_with_external_wifi(const uint8_t *peer_mac);
esp_err_t espnow_comm_init_recv_with_external_wifi(espnow_recv_callback_t callback);
esp_err_t espnow_send_data(const binary_packet_t *packet);
void espnow_get_mac(uint8_t *mac);

#endif /* ESPNOW_COMM_H */
