idf_component_register(SRCS "vibration_motor.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver freertos)  # 添加 freertos 依赖