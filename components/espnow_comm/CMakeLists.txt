idf_component_register(
    SRCS "espnow_comm.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash protocol
)
