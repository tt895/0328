cmake_minimum_required(VERSION 3.16)

# 设置ESP-IDF路径（根据你的实际路径修改）
set(IDF_PATH "C:/esp/v5.5.3/esp-idf")

set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/components/wifi_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/components/bend_sensor
    ${CMAKE_CURRENT_SOURCE_DIR}/components/mpu6050
    ${CMAKE_CURRENT_SOURCE_DIR}/components/mode_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/components/vibration_motor
    ${CMAKE_CURRENT_SOURCE_DIR}/components/protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/components/espnow_comm
)

include(${IDF_PATH}/tools/cmake/project.cmake)
project(gestureCar)