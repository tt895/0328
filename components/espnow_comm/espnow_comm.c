#include "espnow_comm.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "ESPNowComm";

static SemaphoreHandle_t s_send_sem = NULL;
static espnow_recv_callback_t s_recv_callback = NULL;
static bool s_is_sender = false;

static void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (s_send_sem) {
        xSemaphoreGive(s_send_sem);
    }
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "发送失败");
    }
}

static void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (data == NULL || len != sizeof(binary_packet_t)) return;

    const binary_packet_t *packet = (const binary_packet_t *)data;

    if (packet->header != 0xAA) return;

    uint8_t checksum = 0;
    const uint8_t *p = data;
    for (size_t i = 0; i < sizeof(binary_packet_t) - 1; i++) {
        checksum += p[i];
    }
    if (checksum != packet->checksum) return;

    if (s_recv_callback) {
        s_recv_callback(info->src_addr, packet);
    }
}

esp_err_t espnow_comm_init_send_with_external_wifi(const uint8_t *peer_mac)
{
    s_is_sender = true;
    s_send_sem = xSemaphoreCreateBinary();

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "发送端初始化完成，目标: %02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5]);

    return ESP_OK;
}

esp_err_t espnow_comm_init_recv_with_external_wifi(espnow_recv_callback_t callback)
{
    s_is_sender = false;
    s_recv_callback = callback;

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    esp_now_peer_info_t peer_info = {};
    memset(peer_info.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "接收端初始化完成");
    return ESP_OK;
}

esp_err_t espnow_send_data(const binary_packet_t *packet)
{
    if (!s_is_sender) return ESP_ERR_INVALID_STATE;

    for (int i = 0; i < ESPNOW_SEND_RETRY; i++) {
        xSemaphoreTake(s_send_sem, 0);

        esp_err_t ret = esp_now_send(NULL, (uint8_t *)packet, sizeof(binary_packet_t));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_send失败 (重试%d/%d)", i + 1, ESPNOW_SEND_RETRY);
            vTaskDelay(pdMS_TO_TICKS(ESPNOW_SEND_DELAY_MS));
            continue;
        }

        if (xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "发送失败，达最大重试");
    return ESP_FAIL;
}

void espnow_get_mac(uint8_t *mac)
{
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
}
