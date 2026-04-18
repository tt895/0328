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
static bool s_wifi_external = false;

static void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (s_send_sem) {
        xSemaphoreGive(s_send_sem);
    }
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "发送成功");
    } else {
        ESP_LOGW(TAG, "发送失败");
    }
}

static void on_data_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (data == NULL || len != sizeof(binary_packet_t)) {
        ESP_LOGW(TAG, "无效数据长度: %d", len);
        return;
    }

    const binary_packet_t *packet = (const binary_packet_t *)data;
    
    if (packet->header != 0xAA) {
        ESP_LOGW(TAG, "无效帧头: 0x%02X", packet->header);
        return;
    }

    uint8_t checksum = 0;
    const uint8_t *p = data;
    for (size_t i = 0; i < sizeof(binary_packet_t) - 1; i++) {
        checksum += p[i];
    }
    if (checksum != packet->checksum) {
        ESP_LOGW(TAG, "校验和错误: 期望 0x%02X, 实际 0x%02X", packet->checksum, checksum);
        return;
    }

    if (s_recv_callback) {
        s_recv_callback(info->src_addr, packet);
    }
}

static esp_err_t wifi_init_sta(void)
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_start());
    return ESP_OK;
}

esp_err_t espnow_comm_init_send_with_external_wifi(const uint8_t *peer_mac)
{
    s_is_sender = true;
    s_send_sem = xSemaphoreCreateBinary();
    s_wifi_external = true;

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "发送端初始化完成（外部WiFi），目标 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5]);

    return ESP_OK;
}

esp_err_t espnow_comm_init_send(const uint8_t *peer_mac)
{
    s_is_sender = true;
    s_send_sem = xSemaphoreCreateBinary();
    s_wifi_external = false;

    ESP_ERROR_CHECK(wifi_init_sta());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "发送端初始化完成（自带WiFi），目标 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5]);

    return ESP_OK;
}

esp_err_t espnow_comm_init_recv_with_external_wifi(espnow_recv_callback_t callback)
{
    s_is_sender = false;
    s_recv_callback = callback;
    s_wifi_external = true;

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    esp_now_peer_info_t peer_info = {};
    memset(peer_info.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "接收端初始化完成（外部WiFi）");
    return ESP_OK;
}

esp_err_t espnow_comm_init_recv(espnow_recv_callback_t callback)
{
    s_is_sender = false;
    s_recv_callback = callback;
    s_wifi_external = false;

    ESP_ERROR_CHECK(wifi_init_sta());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    esp_now_peer_info_t peer_info = {};
    memset(peer_info.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_WIFI_CHANNEL;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "接收端初始化完成（自带WiFi）");
    return ESP_OK;
}

esp_err_t espnow_send_data(const binary_packet_t *packet)
{
    if (!s_is_sender) {
        ESP_LOGE(TAG, "当前不是发送端");
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < ESPNOW_SEND_RETRY; i++) {
        xSemaphoreTake(s_send_sem, 0);

        esp_err_t ret = esp_now_send(NULL, (uint8_t *)packet, sizeof(binary_packet_t));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "esp_now_send 失败: %s (重试 %d/%d)", esp_err_to_name(ret), i + 1, ESPNOW_SEND_RETRY);
            vTaskDelay(pdMS_TO_TICKS(ESPNOW_SEND_DELAY_MS));
            continue;
        }

        if (xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
            return ESP_OK;
        }
    }

    ESP_LOGE(TAG, "发送失败，已达最大重试次数");
    return ESP_FAIL;
}

void espnow_get_mac(uint8_t *mac)
{
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
}
