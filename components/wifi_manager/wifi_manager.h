#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/socket.h>

bool wifi_manager_init_sta(const char *ssid, const char *password);
bool wifi_manager_wait_connected(uint32_t timeout_ms);
const char *wifi_manager_get_ip(void);
bool wifi_manager_is_connected(void);
int wifi_manager_create_tcp_server(int port);
int wifi_manager_wait_tcp_client(int server_sock, uint32_t timeout_ms);
void wifi_manager_close_socket(int sock);

#endif /* WIFI_MANAGER_H */
