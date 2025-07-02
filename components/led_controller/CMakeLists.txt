idf_component_register(SRCS "led_controller.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos system_config
                       PRIV_REQUIRES esp_timer esp_common ) 