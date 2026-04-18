idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES wifi_manager bend_sensor mpu6050 mode_manager 
                                vibration_motor protocol nvs_flash espnow_comm)