#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <sys/socket.h> 

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化WiFi为STA模式
 * @param ssid WiFi名称
 * @param password WiFi密码
 * @return true 初始化成功，false 初始化失败
 */
bool wifi_manager_init_sta(const char* ssid, const char* password);

/**
 * @brief 等待WiFi连接成功
 * @param timeout_ms 超时时间（毫秒），0表示无限等待
 * @return true 连接成功，false 连接超时
 */
bool wifi_manager_wait_connected(uint32_t timeout_ms);

/**
 * @brief 获取IP地址
 * @return IP地址字符串，NULL表示未获取到IP
 */
const char* wifi_manager_get_ip(void);

/**
 * @brief 检查WiFi是否已连接
 * @return true 已连接，false 未连接
 */
bool wifi_manager_is_connected(void);

/**
 * @brief 创建TCP服务器
 * @param port 端口号
 * @return socket文件描述符，-1表示失败
 */
int wifi_manager_create_tcp_server(int port);

/**
 * @brief 等待TCP客户端连接
 * @param server_sock 服务器socket
 * @param timeout_ms 超时时间（毫秒），0表示无限等待
 * @return 客户端socket，-1表示失败
 */
int wifi_manager_wait_tcp_client(int server_sock, uint32_t timeout_ms);

/**
 * @brief 发送TCP数据
 * @param client_sock 客户端socket
 * @param data 数据指针
 * @param len 数据长度
 * @return 实际发送的字节数，-1表示失败
 */
int wifi_manager_send_tcp_data(int client_sock, const void* data, size_t len);

/**
 * @brief 接收TCP数据
 * @param client_sock 客户端socket
 * @param buffer 接收缓冲区
 * @param len 缓冲区长度
 * @return 实际接收的字节数，-1表示失败，0表示连接断开
 */
int wifi_manager_recv_tcp_data(int client_sock, void* buffer, size_t len);

/**
 * @brief 关闭socket连接
 * @param sock socket描述符
 */
void wifi_manager_close_socket(int sock);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */