#include "espnow_comm.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "ESPNowComm";

static SemaphoreHandle_t s_send_sem = NULL;
static QueueHandle_t s_send_queue = NULL;
static TaskHandle_t s_send_task_handle = NULL;
static espnow_recv_callback_t s_recv_callback = NULL;
static bool s_is_sender = false;
static uint8_t s_peer_mac[6] = {0};
static uint8_t s_filter_mac[6] = {0};
static bool s_has_mac_filter = false;
static volatile uint32_t s_fail_count = 0;

static void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status)
{
    if (s_send_sem) {
        xSemaphoreGive(s_send_sem);
    }
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGD(TAG, "发送失败");
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

    if (s_has_mac_filter && memcmp(info->src_addr, s_filter_mac, 6) != 0) {
        ESP_LOGD(TAG, "忽略非目标源MAC");
        return;
    }

    if (s_recv_callback) {
        s_recv_callback(info->src_addr, packet);
    }
}

static void espnow_send_task(void *arg)
{
    binary_packet_t packet;

    while (1) {
        if (xQueueReceive(s_send_queue, &packet, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        bool sent_ok = false;
        for (int i = 0; i < ESPNOW_SEND_RETRY; i++) {
            xSemaphoreTake(s_send_sem, 0);

            esp_err_t ret = esp_now_send(s_peer_mac, (uint8_t *)&packet, sizeof(binary_packet_t));
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "esp_now_send失败 (重试%d/%d)", i + 1, ESPNOW_SEND_RETRY);
                vTaskDelay(pdMS_TO_TICKS(ESPNOW_SEND_DELAY_MS));
                continue;
            }

            if (xSemaphoreTake(s_send_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
                sent_ok = true;
                break;
            }
        }

        if (!sent_ok) {
            ESP_LOGE(TAG, "发送失败，达最大重试");
            s_fail_count++;
        }
    }
}

static uint8_t get_current_wifi_channel(void)
{
    uint8_t primary = 1;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);
    return primary;
}

esp_err_t espnow_comm_init_send_with_external_wifi(const uint8_t *peer_mac)
{
    s_is_sender = true;
    memcpy(s_peer_mac, peer_mac, 6);

    s_send_sem = xSemaphoreCreateBinary();
    if (s_send_sem == NULL) {
        ESP_LOGE(TAG, "创建信号量失败");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    uint8_t channel = get_current_wifi_channel();

    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = channel;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    s_send_queue = xQueueCreate(ESPNOW_QUEUE_SIZE, sizeof(binary_packet_t));
    if (s_send_queue == NULL) {
        ESP_LOGE(TAG, "创建发送队列失败");
        return ESP_FAIL;
    }

    BaseType_t task_ret = xTaskCreatePinnedToCore(
        espnow_send_task, "espnow_tx", ESPNOW_TASK_STACK,
        NULL, ESPNOW_TASK_PRIO, &s_send_task_handle, 1
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "创建发送任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "发送端初始化完成，通道: %d，目标: %02X:%02X:%02X:%02X:%02X:%02X",
             channel,
             peer_mac[0], peer_mac[1], peer_mac[2],
             peer_mac[3], peer_mac[4], peer_mac[5]);

    return ESP_OK;
}

esp_err_t espnow_comm_init_recv_with_external_wifi(const uint8_t *peer_mac, espnow_recv_callback_t callback)
{
    s_is_sender = false;
    s_recv_callback = callback;

    if (peer_mac != NULL) {
        memcpy(s_filter_mac, peer_mac, 6);
        s_has_mac_filter = true;
    }

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    uint8_t channel = get_current_wifi_channel();

    esp_now_peer_info_t peer_info = {};
    if (peer_mac != NULL) {
        memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    } else {
        memset(peer_info.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    }
    peer_info.channel = channel;
    peer_info.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    ESP_LOGI(TAG, "接收端初始化完成，通道: %d，MAC过滤: %s",
             channel, s_has_mac_filter ? "开启" : "关闭");
    return ESP_OK;
}

esp_err_t espnow_send_data_async(const binary_packet_t *packet)
{
    if (!s_is_sender || s_send_queue == NULL) return ESP_ERR_INVALID_STATE;

    if (xQueueSend(s_send_queue, packet, 0) != pdTRUE) {
        ESP_LOGD(TAG, "发送队列满，丢弃数据包");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

uint32_t espnow_get_fail_count(void)
{
    return s_fail_count;
}

void espnow_get_mac(uint8_t *mac)
{
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
}
