idf_component_register(SRCS "led_controller.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos system_config bluetooth_manager
                       PRIV_REQUIRES esp_timer esp_common ) 