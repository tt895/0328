#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/socket.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "wifi_manager.h"
#include "bend_sensor.h"
#include "mpu6050.h"
#include "mode_manager.h"
#include "vibration_motor.h"
#include "protocol.h"
#include "espnow_comm.h"

#define WIFI_SSID       "tang."
#define WIFI_PASS       "13983039985"
#define TCP_PORT        8888

#define CAR_MAC_ADDR    {0xe0, 0x72, 0xa1, 0xf3, 0xb9, 0x80}

#define MPU_SDA_PIN     8
#define MPU_SCL_PIN     9
#define VIB_MOTOR_PIN   10

#define SEND_INTERVAL_MS    50
#define MODE_HOLD_TIME_MS   2000

static const char *TAG = "Main";
static volatile bool g_running = true;

void sensor_task(void *pvParameters)
{
    esp_task_wdt_delete(NULL);

    int client_sock = (intptr_t)pvParameters;
    ESP_LOGI(TAG, "传感器任务启动，socket=%d", client_sock);

    gesture_data_packet_t packet;
    protocol_init_packet(&packet);

    uint8_t tx_buffer[16];
    binary_packet_t espnow_packet;
    uint32_t last_send = 0;
    uint32_t frame_count = 0;
    uint32_t espnow_fail_count = 0;

    static int mpu_error_count = 0;
    static int mpu_reset_count = 0;
    const int MPU_ERROR_THRESHOLD = 50;

    static hand_attitude_t last_valid_attitude = {0.0f, 0.0f, 0.0f};

    uint8_t last_sent_percent[5] = {0};
    uint32_t last_send_time = 0;
    uint8_t unchanged_count[5] = {0};

    int flags = fcntl(client_sock, F_GETFL, 0);
    if (flags != -1) {
        fcntl(client_sock, F_SETFL, flags | O_NONBLOCK);
    }

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 1000;
    setsockopt(client_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    while (g_running) {
        uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());

        if (now - last_send < SEND_INTERVAL_MS) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        last_send = now;

        bend_sensor_read_all();

        hand_attitude_t attitude = {0.0f, 0.0f, 0.0f};
        esp_err_t mpu_ret = mpu6050_get_attitude_now(&attitude);

        if (mpu_ret != ESP_OK) {
            mpu_error_count++;
            ESP_LOGW(TAG, "MPU6050读取失败(%d/%d)", mpu_error_count, MPU_ERROR_THRESHOLD);

            attitude = last_valid_attitude;

            if (mpu_error_count >= MPU_ERROR_THRESHOLD) {
                if (mpu_reset_count < 3) {
                    ESP_LOGE(TAG, "MPU6050连续失败，软件复位(%d/3)", mpu_reset_count + 1);
                    mpu6050_reset();
                    mpu_reset_count++;
                    mpu_error_count = 0;
                    vibration_trigger(VIB_LONG);
                } else {
                    ESP_LOGE(TAG, "MPU6050可能损坏，切换纯手指模式");
                    attitude.pitch = 0;
                    attitude.roll = 0;
                    attitude.yaw = 0;
                }
            }
        } else {
            last_valid_attitude = attitude;
            if (mpu_error_count > 0) {
                ESP_LOGI(TAG, "MPU6050恢复连接");
                mpu_reset_count = 0;
            }
            mpu_error_count = 0;
        }

        control_mode_t mode = mode_manager_update();

        bool should_send = false;

        for (int i = 0; i < 5; i++) {
            int diff = abs((int)g_fingers[i].bend_percent - (int)last_sent_percent[i]);

            if (diff > 5) {
                should_send = true;
                last_sent_percent[i] = g_fingers[i].bend_percent;
                unchanged_count[i] = 0;
            } else if (diff > 0) {
                unchanged_count[i]++;
                if (unchanged_count[i] >= 3) {
                    should_send = true;
                    last_sent_percent[i] = g_fingers[i].bend_percent;
                    unchanged_count[i] = 0;
                }
            }
        }

        if (!should_send && (now - last_send_time > 100)) {
            should_send = true;
        }

        if (should_send) {
            packet.mode = mode;
            packet.switch_progress = mode_manager_get_progress();

            for (int i = 0; i < 5; i++) {
                packet.finger_percent[i] = g_fingers[i].bend_percent;
            }

            packet.pitch = attitude.pitch;
            packet.roll = attitude.roll;
            packet.yaw = attitude.yaw;

            // TCP -> Unity
            int pack_len = protocol_pack_binary(&packet, tx_buffer, sizeof(tx_buffer));
            if (pack_len > 0) {
                int sent = send(client_sock, tx_buffer, pack_len, MSG_DONTWAIT);
                if (sent < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        ESP_LOGD(TAG, "TCP缓冲区满，丢弃1帧");
                    } else if (errno == ECONNRESET || errno == EPIPE) {
                        ESP_LOGW(TAG, "连接被重置，准备重连");
                        break;
                    } else {
                        ESP_LOGW(TAG, "TCP发送错误: %d", errno);
                    }
                } else if (sent == pack_len) {
                    frame_count++;
                }
            }

            // ESP-NOW -> 小车
            memset(&espnow_packet, 0, sizeof(binary_packet_t));
            espnow_packet.header = 0xAA;
            espnow_packet.mode = (uint8_t)packet.mode;
            espnow_packet.progress = (uint8_t)(packet.switch_progress * 100);

            for (int i = 0; i < 5; i++) {
                espnow_packet.fingers[i] = packet.finger_percent[i];
            }

            espnow_packet.pitch = (int16_t)(packet.pitch * 10);
            espnow_packet.roll = (int16_t)(packet.roll * 10);
            espnow_packet.yaw = (int16_t)(packet.yaw * 10);

            uint8_t *p = (uint8_t *)&espnow_packet;
            espnow_packet.checksum = 0;
            for (size_t i = 0; i < sizeof(binary_packet_t) - 1; i++) {
                espnow_packet.checksum += p[i];
            }

            esp_err_t espnow_ret = espnow_send_data(&espnow_packet);
            if (espnow_ret != ESP_OK) {
                espnow_fail_count++;
            }

            last_send_time = now;
        }

        vibration_update();

        vTaskDelay(pdMS_TO_TICKS(10));

        if (frame_count % 100 == 0 && frame_count > 0) {
            ESP_LOGI(TAG, "已发送 %lu 帧 (ESP-NOW失败: %lu)", frame_count, espnow_fail_count);

            if (frame_count % 500 == 0) {
                ESP_LOGI(TAG, "手指: T=%d%% I=%d%% M=%d%% R=%d%% P=%d%%",
                         g_fingers[0].bend_percent, g_fingers[1].bend_percent,
                         g_fingers[2].bend_percent, g_fingers[3].bend_percent,
                         g_fingers[4].bend_percent);
            }
        }
    }

    wifi_manager_close_socket(client_sock);
    ESP_LOGI(TAG, "传感器任务结束，共发送 %lu 帧", frame_count);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "手势控制小车系统 [WiFi+ESP-NOW]");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(bend_sensor_init());
    bend_sensor_set_diagnostic(true);

    ret = mpu6050_init(MPU_SDA_PIN, MPU_SCL_PIN, 100000);
    if (ret == ESP_OK) {
        mpu6050_calibrate();
        ESP_LOGI(TAG, "MPU6050初始化成功");
        vibration_trigger(VIB_SHORT);
    } else {
        ESP_LOGW(TAG, "MPU6050初始化失败(%d)，纯手指模式", ret);
    }

    mode_manager_init(MODE_HOLD_TIME_MS, 15.0f, 45.0f);

    ESP_ERROR_CHECK(vibration_motor_init(VIB_MOTOR_PIN));
    vibration_trigger(VIB_SHORT);

    if (!wifi_manager_init_sta(WIFI_SSID, WIFI_PASS)) {
        ESP_LOGE(TAG, "WiFi初始化失败");
        return;
    }

    ESP_LOGI(TAG, "等待WiFi连接...");
    if (!wifi_manager_wait_connected(10000)) {
        ESP_LOGE(TAG, "WiFi连接超时");
        return;
    }

    const char *ip = wifi_manager_get_ip();
    if (ip) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "IP: %s  端口: %d", ip, TCP_PORT);
        ESP_LOGI(TAG, "MPU: SDA=%d SCL=%d", MPU_SDA_PIN, MPU_SCL_PIN);
        ESP_LOGI(TAG, "========================================");
    }

    uint8_t car_mac[6] = CAR_MAC_ADDR;

    uint8_t my_mac[6];
    espnow_get_mac(my_mac);
    ESP_LOGI(TAG, "本机MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             my_mac[0], my_mac[1], my_mac[2],
             my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "小车MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             car_mac[0], car_mac[1], car_mac[2],
             car_mac[3], car_mac[4], car_mac[5]);

    ret = espnow_comm_init_send_with_external_wifi(car_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW初始化失败");
        return;
    }

    ESP_LOGI(TAG, "WiFi+ESP-NOW已就绪，等待Unity连接...");
    vibration_trigger(VIB_LONG);

    int server_sock = wifi_manager_create_tcp_server(TCP_PORT);
    if (server_sock < 0) {
        ESP_LOGE(TAG, "创建TCP服务器失败");
        return;
    }

    while (1) {
        int client_sock = wifi_manager_wait_tcp_client(server_sock, 0);

        if (client_sock >= 0) {
            ESP_LOGI(TAG, "Unity已连接，启动传感器任务");
            vibration_trigger(VIB_DOUBLE);

            BaseType_t task_ret = xTaskCreatePinnedToCore(
                sensor_task, "sensor_task", 4096,
                (void *)(intptr_t)client_sock, 2, NULL, 1
            );

            if (task_ret != pdPASS) {
                ESP_LOGE(TAG, "创建任务失败");
                wifi_manager_close_socket(client_sock);
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
